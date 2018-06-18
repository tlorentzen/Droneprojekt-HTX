#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include "Wire.h"
#include <nRF24L01.h>
#include <RF24.h>
#include <ESC.h>

// Structs containing radio message structure for both instructions and feedback.
struct instruction {
  int throttle = 1000;
  int yaw      = 0;
  int pitch    = 0;
  int roll     = 0;
} data;

struct drone_feedback {
  float battery = 0.0;
  int  throttle = 1000;
} feedback;

#define PI acos(-1)
#define YAW      0
#define PITCH    1
#define ROLL     2
#define THROTTLE 3

// Controller input variables
// [Yaw, Pitch, Roll, Throttle]
int instruction[4];

// MPU6050 Variables
float gyro_x_cal, gyro_y_cal, gyro_z_cal;
uint16_t packetSize;
uint16_t fifoCount;
uint8_t fifoBuffer[64];
Quaternion q;
VectorFloat gravity;
float ypr[3];

// Pulselength variables for ESC output
unsigned long pulse_length_esc1 = 1000,
              pulse_length_esc2 = 1000,
              pulse_length_esc3 = 1000,
              pulse_length_esc4 = 1000;

// PID error calculation variables
float errors[3];                     // [Yaw, Pitch, Roll]
float error_sum[3]      = {0, 0, 0}; // [Yaw, Pitch, Roll]
float previous_error[3] = {0, 0, 0}; // [Yaw, Pitch, Roll]
float measures[3]       = {0, 0, 0}; // [Yaw, Pitch, Roll] in degrees

// Initialize RF24 object (NRF24L01+)
RF24 radio(A0, 10);
MPU6050 mpu;

// Battery Voltage
#define battery_voltage_pin A2
const uint64_t pipeIn  = 0xE8E8F0F0E1LL;
const uint64_t pipeOut = 0xE7E8F0F0E1LL;
unsigned long lastRecievedTransmission = 0;
unsigned long lastFeedbackTransmission = 0;
unsigned long feedbackDelay = 1100; // 1 seconds
unsigned long lastTrasmissionTimeout = 1000; // 1 second

ESC M1 (8, 1000, 2000, 500);
ESC M2 (9, 1000, 2000, 500);
ESC M3 (6, 1000, 2000, 500);
ESC M4 (7, 1000, 2000, 500);

#define RED_LED 5
#define GREEN_LED A1

unsigned long currentTime = 0;

/**
   Setup configuration
*/
void setup() 
{
  // Initialize I2C
  Wire.begin();

  // Setup pin modes
  pinMode(RED_LED, OUTPUT);
  pinMode(GREEN_LED, OUTPUT);

  // Prepare 
  digitalWrite(RED_LED, HIGH);
  digitalWrite(GREEN_LED, HIGH);

  // Initialize radio communikation (nRF24L01+)
  radio.begin();
  radio.setPALevel(RF24_PA_MAX);
  radio.setAutoAck(false);
  radio.setDataRate(RF24_250KBPS);
  radio.setChannel(108);
  radio.openReadingPipe(1, pipeIn);
  radio.openWritingPipe(pipeOut);
  radio.startListening();

  // Initialize MPU6050 (Gyroscope)
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
  fifoCount  = mpu.getFIFOCount();

  // Wait for MPU6050 DMP values to become stable and the calibrate errors.
  calibrateMpu6050();

  // Arm ESC controllers
  M1.arm();
  M2.arm();
  M3.arm();
  M4.arm();

  // Everything was initialized with no errors, turning of red led.  
  digitalWrite(RED_LED, LOW);
}

/**
   Main program loop
*/
void loop() 
{
  // Keep track of time
  currentTime = millis();
  
  // Recieve controller transmissions if any.
  recieveInstructions();

  // Read MPU6050 values
  readMPU();

  // Calculate motors speed with PID controller
  calculatePID();

  // Apply motors speed
  applyMotorSpeed();

  // Send feedback to controller
  sendFeedback();
}

/**
   Check the radio for any instructions and set new values 
   if any are available.

   @return void
*/
void recieveInstructions()
{
  if (radio.available()){
    radio.read(&data, sizeof(data));
    lastRecievedTransmission = millis();
    
    if(data.yaw > 180){
      data.yaw = 180;
    }else if(data.yaw < -180){
      data.yaw = -180;
    }

    if(data.pitch > 30){
      data.pitch = 30;
    }else if(data.pitch < -30){
      data.pitch = -30;
    }

    if(data.roll > 30){
      data.roll = 30;
    }else if(data.roll < -30){
      data.roll = -30;
    }

    instruction[YAW]      = data.yaw;
    instruction[PITCH]    = data.pitch;
    instruction[ROLL]     = data.roll;
    instruction[THROTTLE] = data.throttle;
  }

  /*
    Make sure that pitch, roll and yaw values are set 
    to zero for stabelizing if connection to controller is lost. 

    Also set throttle to 45% to make sure drone will land stedy on 
    the ground if connnection is lost.
    */
  long mill = millis();
  if((mill-lastRecievedTransmission) > lastTrasmissionTimeout && data.throttle > 1000){
    instruction[THROTTLE] = 1450;
    instruction[YAW]      = 0;
    instruction[PITCH]    = 0;
    instruction[ROLL]     = 0;
    data.throttle         = 1450;
  }
}

/**
   Read MPU values for PID stabelizing.
   This method sets x, y and z of the current posistion for PID stabelizing. 

   @return void
*/
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
   Find current stable MPU values for calculations of PID errors.

   @return void
*/
void calibrateMpu6050() {
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
   Apply calculated pulses to motors.

   @return void
*/
void applyMotorSpeed() {
  M1.speed(pulse_length_esc1);
  M2.speed(pulse_length_esc2);
  M3.speed(pulse_length_esc3);
  M4.speed(pulse_length_esc4);
}

/**
   Calculate motor speed by applying PID control.

   First PID errors are calculated based on MPU6050 messueres and controller inputs. 
   Then new pid values for each motor are calulated.
   Inspired by https://github.com/lobodol/drone.

   @return void
*/
void calculatePID() {

  float  Kp[3]       = {0.2, 1.6, 1.6}; // P coefficients in order : Yaw, Pitch, Roll //ku = 0.21
  double Ki[3]       = {0.0, 0.000005, 0.000005}; // I coefficients in order : Yaw, Pitch, Roll
  float  Kd[3]       = {20.0, 85.0, 85.0}; // D coefficients in order : Yaw, Pitch, Roll
  float  deltaErr[3] = {0, 0, 0}; // Error deltas in order :  Yaw, Pitch, Roll
  float  yaw         = 0;
  float  pitch       = 0;
  float  roll        = 0;

  // Calculate pid errors
  errors[YAW]   = (measures[YAW]-gyro_z_cal)   - instruction[YAW];
  errors[PITCH] = (measures[PITCH]-gyro_y_cal) - instruction[PITCH];
  errors[ROLL]  = (measures[ROLL]-gyro_x_cal)  - instruction[ROLL];
  
  // Initialize motor commands with throttle
  pulse_length_esc1 = instruction[THROTTLE];
  pulse_length_esc2 = instruction[THROTTLE];
  pulse_length_esc3 = instruction[THROTTLE];
  pulse_length_esc4 = instruction[THROTTLE];

  // Do not calculate anything if throttle is 0
  if (instruction[THROTTLE] >= 1012) {
    
    // Calculate sum of errors : Integral coefficients
    error_sum[YAW]   += errors[YAW];
    error_sum[PITCH] += errors[PITCH];
    error_sum[ROLL]  += errors[ROLL];

    // Calculate error delta : Derivative coefficients
    deltaErr[YAW]   = errors[YAW]   - previous_error[YAW];
    deltaErr[PITCH] = errors[PITCH] - previous_error[PITCH];
    deltaErr[ROLL]  = errors[ROLL]  - previous_error[ROLL];

    // Save current error as previous_error for next time
    previous_error[YAW]   = errors[YAW];
    previous_error[PITCH] = errors[PITCH];
    previous_error[ROLL]  = errors[ROLL];

    yaw   = (errors[YAW] * Kp[YAW]) + (error_sum[YAW] * Ki[YAW]) + (deltaErr[YAW] * Kd[YAW]);
    pitch = (errors[PITCH] * Kp[PITCH]) + (error_sum[PITCH] * Ki[PITCH]) + (deltaErr[PITCH] * Kd[PITCH]);
    roll  = (errors[ROLL] * Kp[ROLL]) + (error_sum[ROLL] * Ki[ROLL]) + (deltaErr[ROLL] * Kd[ROLL]);

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
   Send feedback to controller.

   - Battery voltage
   - Current throttle

   @return void
*/
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

