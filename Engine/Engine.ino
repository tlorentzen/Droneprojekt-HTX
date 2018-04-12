#include <PID_v1.h>
#include <printf.h>
#include <nRF24L01.h>
#include <RF24_config.h>
#include <RF24.h>
#include <MPU9250.h>
#include <ESC.h>
#include <Wire.h>

#define debug 0

// Initialize RF24 object (NRF24L01+)
RF24 radio(A0, 10);

// Battery Voltage
#define battery_voltage_pin A2

// Unique transmitting pipe ids
const uint64_t pipeIn  = 0xE8E8F0F0E1LL;
const uint64_t pipeOut = 0xE7E8F0F0E1LL;
unsigned long lastRecievedTransmission = 0;
unsigned long lastFeedbackTransmission = 0;
unsigned long feedbackDelay = 3000; // 3 seconds
unsigned long lastTrasmissionTimeout = 1000; // 1 second

// Initialize MPU9250 object (Gyro)
MPU9250 IMU(Wire, 0x68);

// Initialize ESC objects one for each engine.
ESC M1 (8, 1000, 2000, 500);
ESC M2 (7, 1000, 2000, 500);
ESC M3 (6, 1000, 2000, 500);
ESC M4 (9, 1000, 2000, 500);

// Predefined limits of pitch, roll and yaw.
#define PITCH_MIN -30
#define PITCH_MAX 30
#define ROLL_MIN -30
#define ROLL_MAX 30
#define YAW_MIN -180
#define YAW_MAX 180

struct instruction {
  byte throttle;
  byte yaw;
  byte pitch;
  byte roll;
  byte AUX1;
  byte AUX2;
};

struct drone_feedback {
  float battery;
  byte error = 0;
};

instruction data;
drone_feedback feedback;

unsigned long previousTime = 0;
unsigned long currentTime = 0;

float gyroY, gyroX, yaw;
float gyroXOffset, gyroYOffset;
boolean MPUOffsetRegistered = false;

int battery_voltage;

float pid_error_temp;
float pid_i_mem_roll, pid_roll_setpoint, gyro_roll_input, pid_output_roll, pid_last_roll_d_error;
float pid_i_mem_pitch, pid_pitch_setpoint, gyro_pitch_input, pid_output_pitch, pid_last_pitch_d_error;
float pid_i_mem_yaw, pid_yaw_setpoint, gyro_yaw_input, pid_output_yaw, pid_last_yaw_d_error;

float pid_p_gain_roll = 1.4;               //Gain setting for the roll P-controller (1.3)
float pid_i_gain_roll = 0.05;              //Gain setting for the roll I-controller (0.05)
float pid_d_gain_roll = 15;                //Gain setting for the roll D-controller (15)
int pid_max_roll = 400;                    //Maximum output of the PID-controller (+/-)

float pid_p_gain_pitch = pid_p_gain_roll;  //Gain setting for the pitch P-controller.
float pid_i_gain_pitch = pid_i_gain_roll;  //Gain setting for the pitch I-controller.
float pid_d_gain_pitch = pid_d_gain_roll;  //Gain setting for the pitch D-controller.
int pid_max_pitch = pid_max_roll;          //Maximum output of the PID-controller (+/-)

float pid_p_gain_yaw = 4.0;                //Gain setting for the pitch P-controller. //4.0
float pid_i_gain_yaw = 0.02;               //Gain setting for the pitch I-controller. //0.02
float pid_d_gain_yaw = 0.0;                //Gain setting for the pitch D-controller.
int pid_max_yaw = 400;                     //Maximum output of the PID-controller (+/-)

void setup() {

  /*
  // Setup connection to slave Atmega
  Wire.begin(8);                // join i2c bus with address #8
  Wire.onReceive(receiveEvent); // register event
  */
  Serial.begin(9600);
  Wire.begin();
  
  radio.begin();
  radio.setPALevel(RF24_PA_MIN);
  radio.setAutoAck(false);
  radio.setDataRate(RF24_250KBPS);
  radio.openReadingPipe(1, pipeIn);
  radio.openWritingPipe(pipeOut);
  radio.startListening();
  radio.printDetails();


  // Prepare MPU for communication
  int status = IMU.begin();
  if (status < 0) {
    Serial.println("IMU initialization unsuccessful");
    Serial.println("Check IMU wiring or try cycling power");
    Serial.print("Status: ");
    Serial.println(status);
    while(1) {}
  }

  // setting the accelerometer full scale range to +/-8G 
  IMU.setAccelRange(MPU9250::ACCEL_RANGE_8G);
  // setting the gyroscope full scale range to +/-500 deg/s
  IMU.setGyroRange(MPU9250::GYRO_RANGE_2000DPS);
  // setting DLPF bandwidth to 20 Hz
  IMU.setDlpfBandwidth(MPU9250::DLPF_BANDWIDTH_20HZ);
  // setting SRD to 19 for a 50 Hz update rate
  IMU.setSrd(19);

  // Arm ingines
  M1.arm();
  M2.arm();
  M3.arm();
  M4.arm();

  currentTime = millis();
  Serial.println("Ready!");
}
/*
void setSpeed(ESC M, int speed){
  int angle = map(speed, 0, 360, 0, 360); //Sets servo positions to different speeds ESC1.write(angle);
  ESC.write(angle);
}
**/

void loop() {

  // Set time variables.
  previousTime = currentTime;
  currentTime = millis();
  long timeDiffSec = (currentTime-previousTime)/1000; // Get degrees per second.
  
  // Read Gyroscope data and calculate position.
  IMU.readSensor();

  //supplementary filter, hvor orientationen bliver finpudset med dataene fra gyroskopet
  gyroX = ((atan(IMU.getAccelY_mss()/sqrt((IMU.getAccelX_mss()*IMU.getAccelX_mss())+(IMU.getAccelZ_mss()*IMU.getAccelZ_mss())))) + IMU.getGyroX_rads()*timeDiffSec)*(57.2957796);
  gyroY = ((atan(IMU.getAccelX_mss()/sqrt((IMU.getAccelY_mss()*IMU.getAccelY_mss())+(IMU.getAccelZ_mss()*IMU.getAccelZ_mss())))) + IMU.getGyroY_rads()*timeDiffSec)*(57.2957796);

  //Compasshedaing (YAW) udregnes medtilt-kompensationen
  yaw = atan2((-(IMU.getMagY_uT() *cos(gyroX)+IMU.getMagZ_uT()*sin(gyroX))), 
        IMU.getMagX_uT()*cos(gyroY)+IMU.getMagY_uT()*sin(gyroY)*sin(gyroX)+IMU.getMagZ_uT()*sin(gyroY)*cos(gyroX));

  // Save MPU readings as offset.
  if(MPUOffsetRegistered){
    gyroXOffset = gyroX;
    gyroYOffset = gyroY;
    MPUOffsetRegistered = true;
  }

  if(Serial.available()){
      int value = Serial.parseInt();
      Serial.println("Speed set to: "+String(value));
      throttle(value);
  }

  // Recieve controller transmissions if any.
  if (radio.available())
  {
    radio.read(&data, sizeof(data));
    Serial.println(data.throttle);
    throttle(data.throttle);
    lastRecievedTransmission = currentTime;
  }

  // Check if we lost connection to controller.
  if((currentTime - lastRecievedTransmission) >= lastTrasmissionTimeout){
      Serial.println("Connection lost...");
  }

  Serial.println("X: "+String(gyroX)+" Y: "+String(gyroY));
  
  sendFeedback();

  //delay(1000);
}

void throttle(int value)
{
  value = map(value, 0, 250, 1000, 2000);
  Serial.println("Throttle: "+String(value));

  M1.speed(value);
  M2.speed(value);
  M3.speed(value);
  M4.speed(value);
}

void sendFeedback(){
  if((currentTime-lastFeedbackTransmission) >= feedbackDelay){

    //TODO: Prepare date for transmission.
    feedback.battery = read_battery_voltage();

    radio.stopListening();
    //Serial.println(currentMicros);
    Serial.println("Sending feedback...");
    radio.write(&feedback, sizeof(feedback));
    lastFeedbackTransmission = currentTime;
    radio.startListening();
  }
}

void calculate_pid(){
  //Roll calculations
  pid_error_temp = gyro_roll_input - pid_roll_setpoint;
  pid_i_mem_roll += pid_i_gain_roll * pid_error_temp;
  if(pid_i_mem_roll > pid_max_roll)pid_i_mem_roll = pid_max_roll;
  else if(pid_i_mem_roll < pid_max_roll * -1)pid_i_mem_roll = pid_max_roll * -1;
  
  pid_output_roll = pid_p_gain_roll * pid_error_temp + pid_i_mem_roll + pid_d_gain_roll * (pid_error_temp - pid_last_roll_d_error);
  if(pid_output_roll > pid_max_roll)pid_output_roll = pid_max_roll;
  else if(pid_output_roll < pid_max_roll * -1)pid_output_roll = pid_max_roll * -1;
  
  pid_last_roll_d_error = pid_error_temp;
  
  //Pitch calculations
  pid_error_temp = gyro_pitch_input - pid_pitch_setpoint;
  pid_i_mem_pitch += pid_i_gain_pitch * pid_error_temp;
  if(pid_i_mem_pitch > pid_max_pitch)pid_i_mem_pitch = pid_max_pitch;
  else if(pid_i_mem_pitch < pid_max_pitch * -1)pid_i_mem_pitch = pid_max_pitch * -1;
  
  pid_output_pitch = pid_p_gain_pitch * pid_error_temp + pid_i_mem_pitch + pid_d_gain_pitch * (pid_error_temp - pid_last_pitch_d_error);
  if(pid_output_pitch > pid_max_pitch)pid_output_pitch = pid_max_pitch;
  else if(pid_output_pitch < pid_max_pitch * -1)pid_output_pitch = pid_max_pitch * -1;
    
  pid_last_pitch_d_error = pid_error_temp;
    
  //Yaw calculations
  pid_error_temp = gyro_yaw_input - pid_yaw_setpoint;
  pid_i_mem_yaw += pid_i_gain_yaw * pid_error_temp;
  if(pid_i_mem_yaw > pid_max_yaw)pid_i_mem_yaw = pid_max_yaw;
  else if(pid_i_mem_yaw < pid_max_yaw * -1)pid_i_mem_yaw = pid_max_yaw * -1;
  
  pid_output_yaw = pid_p_gain_yaw * pid_error_temp + pid_i_mem_yaw + pid_d_gain_yaw * (pid_error_temp - pid_last_yaw_d_error);
  if(pid_output_yaw > pid_max_yaw)pid_output_yaw = pid_max_yaw;
  else if(pid_output_yaw < pid_max_yaw * -1)pid_output_yaw = pid_max_yaw * -1;
    
  pid_last_yaw_d_error = pid_error_temp;
}

float read_battery_voltage()
{
  return (analogRead(battery_voltage_pin)/54.16);
}

