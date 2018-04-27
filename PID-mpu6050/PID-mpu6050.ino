/**
   The software is provided "as is", without any warranty of any kind.
   Feel free to edit it if needed.

   @author tlorentzen <thomas@tlorentzen.net>
*/

// ---------------------------------------------------------------------------
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include "Wire.h"
#include <printf.h>
#include <nRF24L01.h>
#include <RF24.h>
#include <ESC.h>
#include <VL53L0X.h>
// ------------------- Define some constants for convenience -----------------

bool debug = false;

struct instruction {
  long throttle;
  byte yaw;
  byte pitch;
  byte roll;
  float kp = 0;
  double ki = 0;
  float kd = 0;
} data;

struct drone_feedback {
  float battery;
  byte  error = 0;
  float yaw_error = 0.0;
  float roll_error = 0.0;
  float pitch_error = 0.0;
  long  loop_time = 0;
  long  throttle = 0;
} feedback;

#define PI acos(-1)

#define YAW      0
#define PITCH    1
#define ROLL     2
#define THROTTLE 3

// ---------------- Receiver variables ---------------------------------------
// Received instructions formatted with good units, in that order : [Yaw, Pitch, Roll, Throttle]
float instruction[4];

// ----------------------- MPU variables -------------------------------------
float gyro_x_cal, gyro_y_cal, gyro_z_cal;
uint16_t packetSize;
uint16_t fifoCount;
uint8_t fifoBuffer[64];
Quaternion q;
VectorFloat gravity;
float ypr[3];

// ----------------------- Variables for servo signal generation -------------
unsigned long pulse_length_esc1 = 1000,
              pulse_length_esc2 = 1000,
              pulse_length_esc3 = 1000,
              pulse_length_esc4 = 1000;

// ------------- Global variables used for PID automation --------------------
float errors[3];                     // Measured errors (compared to instructions) : [Yaw, Pitch, Roll]
float error_sum[3]      = {0, 0, 0}; // Error sums (used for integral component) : [Yaw, Pitch, Roll]
float previous_error[3] = {0, 0, 0}; // Last errors (used for derivative component) : [Yaw, Pitch, Roll]
float measures[3]       = {0, 0, 0}; // Angular measures : [Yaw, Pitch, Roll]
// ---------------------------------------------------------------------------

// Initialize RF24 object (NRF24L01+)
RF24 radio(A0, 10);
VL53L0X lof;
MPU6050 mpu;

// Battery Voltage
#define battery_voltage_pin A2
const uint64_t pipeIn  = 0xE8E8F0F0E1LL;
const uint64_t pipeOut = 0xE7E8F0F0E1LL;
unsigned long lastRecievedTransmission = 0;
unsigned long lastFeedbackTransmission = 0;
unsigned long feedbackDelay = 1100; // 1 seconds
unsigned long lastTrasmissionTimeout = 1000; // 1 second

bool firstPackageRecieved = false;

ESC M1 (8, 1000, 2000, 500);
ESC M2 (9, 1000, 2000, 500);
ESC M3 (6, 1000, 2000, 500);
ESC M4 (7, 1000, 2000, 500);

int red_led = 5;
int green_led = A1;

unsigned long previousTime = 0;
unsigned long currentTime = 0;

/**
   Setup configuration
*/
void setup() {

  if (debug) {
    Serial.begin(9600);
    Serial.flush();
  }

  Wire.begin();
  pinMode(red_led, OUTPUT);
  pinMode(green_led, OUTPUT);
  digitalWrite(red_led, HIGH);
  digitalWrite(green_led, HIGH);

  writeDebugData("Initializing radio...");
  radio.begin();
  radio.setPALevel(RF24_PA_LOW);
  radio.setAutoAck(false);
  radio.setDataRate(RF24_250KBPS);
  radio.setChannel(108);
  radio.openReadingPipe(1, pipeIn);
  radio.openWritingPipe(pipeOut);
  radio.startListening();
  

  if (debug) {
    radio.printDetails();
  }

  writeDebugData("Done");

  mpu.initialize();
  mpu.dmpInitialize();
  mpu.setXGyroOffset(92);
  mpu.setYGyroOffset(34);
  mpu.setZGyroOffset(-14);
  mpu.setXAccelOffset(-2851);
  mpu.setYAccelOffset(2847);
  mpu.setZAccelOffset(907);
  mpu.setDMPEnabled(true);
  packetSize = mpu.dmpGetFIFOPacketSize();
  fifoCount = mpu.getFIFOCount();

  calibrateMpu9250();

  M1.arm();
  M2.arm();
  M3.arm();
  M4.arm();

  digitalWrite(red_led, LOW);
  digitalWrite(green_led, HIGH);
}

/**
   Main program loop
*/
void loop() {
  long start_of_loop = micros();
  // Keep track of time
  previousTime = currentTime;
  currentTime = millis();

  // Recieve controller transmissions if any.
  if (radio.available())
  {
    radio.read(&data, sizeof(data));
    lastRecievedTransmission = millis();
  }

  // 1. First, read angular values from MPU-6050
  readMPU();

  // 2. Then, translate received data into usable values
  getFlightInstruction();

  // 3. Calculate errors comparing received instruction with measures
  calculateErrors();

  // 4. Calculate motors speed with PID controller
  automation();

  // 5. Apply motors speed
  applyMotorSpeed();

  sendFeedback();

  //lof.readRangeSingleMillimeters();

  long end_of_loop = micros();
  feedback.loop_time = (end_of_loop-start_of_loop);
  writeDebugData(String(feedback.loop_time));
}

void readMPU()
{
  while (fifoCount < packetSize) {
      fifoCount = mpu.getFIFOCount();
  }

  if (fifoCount == 1024) {
      mpu.resetFIFO();
  }else{
    if (fifoCount % packetSize != 0) {
      mpu.resetFIFO();
    }else{
      while (fifoCount >= packetSize) {
          mpu.getFIFOBytes(fifoBuffer,packetSize);
          fifoCount -= packetSize;
      }

      mpu.dmpGetQuaternion(&q,fifoBuffer);
      mpu.dmpGetGravity(&gravity,&q);
      mpu.dmpGetYawPitchRoll(ypr,&q,&gravity);            

      measures[YAW]   = (ypr[0]*180/PI);
      measures[PITCH] = (ypr[2]*180/PI);
      measures[ROLL]  = (ypr[1]*180/PI);
    }
  }
}

/**
   Generate servo-signal on digital pins #4 #5 #6 #7 with a frequency of 250Hz (4ms period).
   Direct port manipulation is used for performances.

   This function might not take more than 2ms to run, which lets 2ms remaining to do other stuff.

   @see https://www.arduino.cc/en/Reference/PortManipulation

   @return void
*/
void calibrateMpu9250() {
  for (int cal_int = 0; cal_int < 2000; cal_int++) {
    readMPU();
    if(cal_int > 499){
      gyro_x_cal += measures[ROLL];
      gyro_y_cal += measures[PITCH];
      gyro_z_cal += measures[YAW];
    }
  }
  gyro_x_cal /= 1500;
  gyro_y_cal /= 1500;
  gyro_z_cal /= 1500;
}

/**
   Generate servo-signal on digital pins #4 #5 #6 #7 with a frequency of 250Hz (4ms period).
   Direct port manipulation is used for performances.

   This function might not take more than 2ms to run, which lets 2ms remaining to do other stuff.

   @see https://www.arduino.cc/en/Reference/PortManipulation

   @return void
*/
void applyMotorSpeed() {
  M1.speed(pulse_length_esc1);
  M2.speed(pulse_length_esc2);
  M3.speed(pulse_length_esc3);
  M4.speed(pulse_length_esc4);
}

/**
   Calculate motor speed for each motor of an X quadcopter depending on received instructions and measures from sensor
   by applying PID control.

   (A) (B)     x
     \ /     z ↑
      X       \|
     / \       +----→ y
   (C) (D)

   Motors A & D run clockwise.
   Motors B & C run counter-clockwise.

   Each motor output is considered as a servomotor. As a result, value range is about 1000µs to 2000µs

   @return void
*/
void automation() {

  float  Kp[3]       = {data.kp, 1.6, 1.6}; // P coefficients in that order : Yaw, Pitch, Roll //ku = 0.21
  double Ki[3]       = {data.ki, 0.000005, 0.000005};  // I coefficients in that order : Yaw, Pitch, Roll
  float  Kd[3]       = {data.kd, 85.0, 85.0};    // D coefficients in that order : Yaw, Pitch, Roll
  float  deltaErr[3] = {0, 0, 0};    // Error deltas in that order :  Yaw, Pitch, Roll
  float  yaw         = 0;
  float  pitch       = 0;
  float  roll        = 0;

  // Initialize motor commands with throttle
  pulse_length_esc1 = instruction[THROTTLE];
  pulse_length_esc2 = instruction[THROTTLE];
  pulse_length_esc3 = instruction[THROTTLE];
  pulse_length_esc4 = instruction[THROTTLE];

  // Do not calculate anything if throttle is 0
  if (instruction[THROTTLE] >= 1012) {
    
    // Calculate sum of errors : Integral coefficients
    error_sum[YAW] += errors[YAW];
    error_sum[PITCH] += errors[PITCH];
    error_sum[ROLL] += errors[ROLL];

    // Calculate error delta : Derivative coefficients
    deltaErr[YAW] = errors[YAW] - previous_error[YAW];
    deltaErr[PITCH] = errors[PITCH] - previous_error[PITCH];
    deltaErr[ROLL] = errors[ROLL] - previous_error[ROLL];

    // Save current error as previous_error for next time
    previous_error[YAW] = errors[YAW];
    previous_error[PITCH] = errors[PITCH];
    previous_error[ROLL] = errors[ROLL];

    yaw = (errors[YAW] * Kp[YAW]) + (error_sum[YAW] * Ki[YAW]) + (deltaErr[YAW] * Kd[YAW]);
    pitch = (errors[PITCH] * Kp[PITCH]) + (error_sum[PITCH] * Ki[PITCH]) + (deltaErr[PITCH] * Kd[PITCH]);
    roll = (errors[ROLL] * Kp[ROLL]) + (error_sum[ROLL] * Ki[ROLL]) + (deltaErr[ROLL] * Kd[ROLL]);

    // Yaw - Lacet (Z axis)
    pulse_length_esc1 -= yaw;
    pulse_length_esc4 -= yaw;
    pulse_length_esc3 += yaw;
    pulse_length_esc2 += yaw;

    // Pitch - Tangage (Y axis)
    pulse_length_esc1 += pitch;
    pulse_length_esc2 += pitch;
    pulse_length_esc3 -= pitch;
    pulse_length_esc4 -= pitch;

    // Roll - Roulis (X axis)
    pulse_length_esc1 -= roll;
    pulse_length_esc3 -= roll;
    pulse_length_esc2 += roll;
    pulse_length_esc4 += roll;
  }

  // Maximum value
  if (pulse_length_esc1 > 2000) pulse_length_esc1 = 2000;
  if (pulse_length_esc2 > 2000) pulse_length_esc2 = 2000;
  if (pulse_length_esc3 > 2000) pulse_length_esc3 = 2000;
  if (pulse_length_esc4 > 2000) pulse_length_esc4 = 2000;

  // Minimum value
  if (pulse_length_esc1 < 1000) pulse_length_esc1 = 1000;
  if (pulse_length_esc2 < 1000) pulse_length_esc2 = 1000;
  if (pulse_length_esc3 < 1000) pulse_length_esc3 = 1000;
  if (pulse_length_esc4 < 1000) pulse_length_esc4 = 1000;
}

/**
   Calculate errors of Yaw, Pitch & Roll: this is simply the difference between the measure and the command.

   @return void
*/
void calculateErrors() {
  errors[YAW]   = (measures[YAW]-gyro_z_cal)   - instruction[YAW];
  errors[PITCH] = (measures[PITCH]-gyro_y_cal) - instruction[PITCH];
  errors[ROLL]  = (measures[ROLL]-gyro_x_cal)  - instruction[ROLL];

  feedback.yaw_error = errors[YAW];
  feedback.roll_error = errors[ROLL];
  feedback.pitch_error = errors[PITCH];
}

/**
   Calculate real value of flight instructions from pulses length of each channel.

   - Roll     : from -33° to 33°
   - Pitch    : from -33° to 33°
   - Yaw      : from -180°/sec to 180°/sec
   - Throttle : from 1000µs to 2000µs

   @return void
*/
void getFlightInstruction() {
    instruction[YAW]      = map(data.yaw, 0, 250, -180, 180);
    instruction[PITCH]    = map(data.pitch, 0, 250, 20, -20);
    instruction[ROLL]     = map(data.roll, 0, 250, 20, -20);

    long mill = millis();
    if(mill-lastRecievedTransmission > lastTrasmissionTimeout && data.throttle > 0){
        instruction[THROTTLE] = 1450;
        data.throttle = 450;
    }else{
        instruction[THROTTLE] = map(data.throttle, 0, 1000, 1000, 2000);
    }

}

void sendFeedback() {
  if ((currentTime - lastFeedbackTransmission) > feedbackDelay) {
    feedback.battery  = read_battery_voltage();
    feedback.throttle = data.throttle;
    radio.stopListening();
    radio.write(&feedback, sizeof(feedback));
    lastFeedbackTransmission = currentTime;
    radio.startListening();
  }
}

/**
   Read current battery voltage level

   @return float
*/
float read_battery_voltage()
{
  return (analogRead(battery_voltage_pin) / 53.5);
}

/**
   Debugging method to write out to terminal if debugging is activated.

   @return void
*/
void writeDebugData(String text) {
  if (debug) {
    Serial.println(text);
  }
}
