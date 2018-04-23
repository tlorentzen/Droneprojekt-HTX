#include <VL53L0X.h>

/**
 * The software is provided "as is", without any warranty of any kind.
 * Feel free to edit it if needed.
 * 
 * @author tlorentzen <thomas@tlorentzen.net>
 */

// ---------------------------------------------------------------------------
#include <Wire.h>
#include <printf.h>
#include <nRF24L01.h>
#include <RF24_config.h>
#include <RF24.h>
#include <ESC.h>
#include <MPU9250.h>
// ------------------- Define some constants for convenience -----------------

bool debug = false;

VL53L0X lof;

struct instruction {
  byte throttle;
  byte yaw;
  byte pitch;
  byte roll;
  float kp = 0;
  double ki = 0;
  float kd = 0;
} data;

struct drone_feedback {
  float battery;
  byte error = 0;
} feedback;

/*
#define CHANNEL1 0
#define CHANNEL2 1
#define CHANNEL3 2
#define CHANNEL4 3
*/
#define PI acos(-1) // 3.14.....

#define YAW      0
#define PITCH    1
#define ROLL     2
#define THROTTLE 3

#define X           0     // X axis
#define Y           1     // Y axis
#define Z           2     // Z axis
#define MPU_ADDRESS 0x68  // I2C address of the MPU-6050
#define SSF_GYRO    65.5  // Sensitivity Scale Factor of the gyro from datasheet
#define SSF_ACC     8.192 // Sensitivity Scale Factor of the accelerometer from datasheet
// ---------------- Receiver variables ---------------------------------------
// Received instructions formatted with good units, in that order : [Yaw, Pitch, Roll, Throttle]
float instruction[4];

// Previous state of each channel (HIGH or LOW)
volatile byte previous_state[4];

// Duration of the pulse on each channel of the receiver in µs (must be within 1000µs & 2000µs)
volatile unsigned int pulse_length[4] = {1500, 1500, 1000, 1500};

// Used to calculate pulse duration on each channel.
volatile unsigned long current_time;
volatile unsigned long timer[4]; // Timer of each channel.

// Used to configure which control (yaw, pitch, roll, throttle) is on which channel.
int mode_mapping[4];
// ----------------------- MPU variables -------------------------------------
int gyro_x, gyro_y, gyro_z;
long acc_x, acc_y, acc_z, acc_total_vector;
int temperature;
long gyro_x_cal, gyro_y_cal, gyro_z_cal;
float angle_pitch, angle_roll;
int angle_pitch_buffer, angle_roll_buffer;
boolean set_gyro_angles;
float angle_roll_acc, angle_pitch_acc;

float cx[30] = {0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0};
float cy[30] = {0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0};
float cz[30] = {0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0};
float xAvg = 0.0;
float yAvg = 0.0;
float zAvg = 0.0;
// ----------------------- Variables for servo signal generation -------------
unsigned long loop_timer;
unsigned long now, difference;
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
MPU9250 IMU(Wire,0x68);

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

int red_led = 5;
int green_led = A1;

unsigned long previousTime = 0;
unsigned long currentTime = 0;

float KKp = 0;
double KKi = 0;
float KKd = 0;

int i = 0;

/**
 * Setup configuration
 */
void setup() {

  
    //if(debug){
      Serial.begin(9600);
      Serial.flush();
    //}

    Serial.println("Test");
    Serial.println("Test 2");
    
    
    Wire.begin();
    //TWBR = 24; // 400kHz I2C clock (200kHz if CPU is 8MHz)

    pinMode(red_led, OUTPUT);
    pinMode(green_led, OUTPUT);

    digitalWrite(red_led, HIGH);
    digitalWrite(green_led, HIGH);

    writeDebugData("Initializing radio...");
    radio.begin();
    radio.setPALevel(RF24_PA_MIN);
    radio.setAutoAck(false);
    radio.setDataRate(RF24_250KBPS);
    radio.setChannel(108);
    radio.openReadingPipe(1, pipeIn);
    radio.openWritingPipe(pipeOut);
    radio.startListening();
    //radio.printDetails();
    writeDebugData("Done!");

    if (IMU.begin() < 0) {
        writeDebugData("IMU initialization unsuccessful");
        writeDebugData("Check IMU wiring or try cycling power");
        while(1) {}
    }else{
      // setting the accelerometer full scale range to +/-8G 
      IMU.setAccelRange(MPU9250::ACCEL_RANGE_8G);
      // setting the gyroscope full scale range to +/-500 deg/s
      IMU.setGyroRange(MPU9250::GYRO_RANGE_2000DPS);
      // setting DLPF bandwidth to 20 Hz
      IMU.setDlpfBandwidth(MPU9250::DLPF_BANDWIDTH_20HZ);
      // setting SRD to 19 for a 50 Hz update rate
      IMU.setSrd(19);

      calibrateMpu9250New();
    }

    //setupMpu6050Registers();

    //calibrateMpu9250();
    
    /*
    // Set pins 4 5 6 7 as outputs.
    DDRD |= B11110000;
    */

    
    // Initialize loop_timer.
    loop_timer = micros();
    /*
    configureChannelMapping();
    */

    M1.arm();
    M2.arm();
    M3.arm();
    M4.arm();
    
    //delay(1000);

    digitalWrite(red_led, LOW);
    digitalWrite(green_led, HIGH);
}


/**
 * Main program loop
 */
void loop() {
    
    previousTime = currentTime;
    currentTime = millis();

        int start_mil = millis();
    //Serial.println(String(start_mil));
    // Recieve controller transmissions if any.
    if (radio.available())
    {
      radio.read(&data, sizeof(data));
      //Serial.println(data.throttle);
      lastRecievedTransmission = currentTime;
    }
    
    
    // 1. First, read angular values from MPU-6050
    /*
    readSensor();
    convertRawValues();
    */
    readMPU();

    writeDebugData("ROLL-messured: "+String(measures[ROLL]));
    writeDebugData("PITCH-messured: "+String(measures[PITCH]));
    writeDebugData("YAW-messured: "+String(measures[YAW]));

    //Serial.println(measures[YAW]);

    //Serial.println("Battey voltage: "+String(read_battery_voltage()));
    
    
    // 2. Then, translate received data into usable values
    getFlightInstruction();
    instruction[YAW] = 0;
    

    // 3. Calculate errors comparing received instruction with measures
    calculateErrors();
  
    // 4. Calculate motors speed with PID controller
    automation();

    /*
    writeDebugData("#### Controller INPUT ####");
    writeDebugData("Pitch: "+String(instruction[PITCH]));
    writeDebugData("Roll: "+String(instruction[ROLL]));
    writeDebugData("yaw: "+String(instruction[YAW]));
    writeDebugData("Throttle: "+String(instruction[THROTTLE]));
    writeDebugData("");
    */
    
    
    // 5. Apply motors speed
    applyMotorSpeed();
    

    //Serial.println(String(measures[ROLL])+" - "+String(measures[PITCH]));
    //Serial.println();
    sendFeedback();
    //delay(500);

    int end_mil = millis();
    //Serial.println(String(end_mil));
    Serial.println(String(end_mil-start_mil));
}

void readMPU()
{

  
    IMU.readSensor();

    acc_x = IMU.getAccelX_mss();                                 //Add the low and high byte to the acc_x variable
    acc_y = IMU.getAccelY_mss();                                 //Add the low and high byte to the acc_y variable
    acc_z = IMU.getAccelZ_mss();                                  //Add the low and high byte to the acc_z variable
    temperature = IMU.getTemperature_C();                          
    gyro_x = IMU.getGyroX_rads();                                 //Add the low and high byte to the gyro_x variable
    gyro_y = IMU.getGyroY_rads();                                 //Add the low and high byte to the gyro_y variable
    gyro_z = IMU.getGyroZ_rads();

    float calcTime = ((currentTime-previousTime)/1000);
    float incY = 0;
    float incX = 0;
    float gyroY = 0;
    float gyroX = 0;
    float xh = 0;
    float yh = 0;
    float var_compass = 0;
  
    //Beregning for  hældningsvinklen for  x- and y-aksen.
    incY = atan(IMU.getAccelX_mss()/sqrt((IMU.getAccelY_mss()*IMU.getAccelY_mss())+(IMU.getAccelZ_mss()*IMU.getAccelZ_mss()))); 
    incX= atan(IMU.getAccelY_mss()/sqrt((IMU.getAccelX_mss()*IMU.getAccelX_mss())+(IMU.getAccelZ_mss()*IMU.getAccelZ_mss())));
  
    //supplementary filter, hvor orientationen bliver finpudset med dataene fra gyroskopet
    gyroY = incY + IMU.getGyroY_rads()*calcTime;
    gyroX = incX + IMU.getGyroX_rads()*calcTime;
  
    xh = IMU.getMagX_uT()*cos(IMU.getAccelY_mss())+IMU.getMagY_uT()*sin(IMU.getAccelY_mss())-IMU.getMagZ_uT()*cos(IMU.getAccelX_mss())*sin(IMU.getAccelY_mss());
    yh = IMU.getMagY_uT()*cos(IMU.getAccelX_mss())+IMU.getMagZ_uT()*sin(IMU.getAccelX_mss());
  
    var_compass = atan2((double)yh, (double)xh)*(180/PI)-90;
    if (var_compass>0){
      var_compass = var_compass - 360;
    }
    var_compass = 360+var_compass;
  
    gyroX = gyroX*180/PI;
    gyroY = gyroY*180/PI;
  
    //Serial.println(String(gyroX) + "\t" + String(gyroY)  + "\t" + String(var_compass));

    for (i=0; i<9; i++){
         cx[i] = cx[i+1];
         cy[i] = cy[i+1];
         cz[i] = cz[i+1];
    }

    cx[9] = gyroX;
    cy[9] = gyroY;
    cz[9] = var_compass;

    xAvg = 0.0;
    yAvg = 0.0;
    zAvg = 0.0;
    
    for (i=0; i<9; i++){
         xAvg = (xAvg+cx[i]);
         yAvg = (yAvg+cy[i]);
         zAvg = (zAvg+cz[i]);
    }

    measures[ROLL]  = (xAvg/10);   //Take 90% of the output pitch value and add 10% of the raw pitch value
    measures[PITCH] = (yAvg/10); 

    /*
    measures[ROLL] = gyroX;
    measures[PITCH] = gyroY;
    */
    measures[YAW] = (zAvg/10);
    /*
    Serial.println("ROLL-messured: "+String(measures[ROLL]));
    Serial.println("PITCH-messured: "+String(measures[PITCH]));
    Serial.println("YAW-messured: "+String(measures[YAW]));
    */

    
    //Serial.println();
}

void calibrateMpu9250New(){
    for (int cal_int = 0; cal_int < 1000; cal_int++) {                  //Run this code 2000 times
        readMPU();                                              //Read the raw acc and gyro data from the MPU-6050
        gyro_x_cal += gyro_x;                                              //Add the gyro x-axis offset to the gyro_x_cal variable
        gyro_y_cal += gyro_y;                                              //Add the gyro y-axis offset to the gyro_y_cal variable
        gyro_z_cal += gyro_z;                                              //Add the gyro z-axis offset to the gyro_z_cal variable
        delay(3);                                                          //Delay 3us to simulate the 250Hz program loop
    }
    gyro_x_cal /= 1000;                                                  //Divide the gyro_x_cal variable by 2000 to get the avarage offset
    gyro_y_cal /= 1000;                                                  //Divide the gyro_y_cal variable by 2000 to get the avarage offset
    gyro_z_cal /= 1000;
}

/**
 * Generate servo-signal on digital pins #4 #5 #6 #7 with a frequency of 250Hz (4ms period).
 * Direct port manipulation is used for performances.
 * 
 * This function might not take more than 2ms to run, which lets 2ms remaining to do other stuff.
 * 
 * @see https://www.arduino.cc/en/Reference/PortManipulation
 * 
 * @return void
 */
void applyMotorSpeed() {


 
    //while ((now = micros()) - loop_timer < 4000){

    loop_timer = now;
     /*
    //PORTD |= B11110000;

    //while (PORTD >= 16) {
    */
    now = micros();
    difference = now - loop_timer;

    //Serial.println("#### ESC OUTPUT ####");

    //if (difference >= pulse_length_esc1){
      writeDebugData("ESC 1: "+String(pulse_length_esc1));
      M1.speed(pulse_length_esc1);
    //}

    //if (difference >= pulse_length_esc2){
      writeDebugData("ESC 2: "+String(pulse_length_esc2));
      M2.speed(pulse_length_esc2);
    //}

    //if (difference >= pulse_length_esc3){
      writeDebugData("ESC 3: "+String(pulse_length_esc3));
      M3.speed(pulse_length_esc3);
    //}

    //if (difference >= pulse_length_esc4){
      writeDebugData("ESC 4: "+String(pulse_length_esc4));
      M4.speed(pulse_length_esc4);
    //}

    
        /*
        if (difference >= pulse_length_esc1) PORTD &= B11101111; // Pin 4
        if (difference >= pulse_length_esc2) PORTD &= B11011111; // pin 5
        if (difference >= pulse_length_esc3) PORTD &= B10111111; // pin 6
        if (difference >= pulse_length_esc4) PORTD &= B01111111; // pin 7
        */
    //}
}

/**
 * Calculate motor speed for each motor of an X quadcopter depending on received instructions and measures from sensor
 * by applying PID control.
 *
 * (A) (B)     x
 *   \ /     z ↑
 *    X       \|
 *   / \       +----→ y
 * (C) (D)
 *
 * Motors A & D run clockwise.
 * Motors B & C run counter-clockwise.
 *
 * Each motor output is considered as a servomotor. As a result, value range is about 1000µs to 2000µs
 * 
 * @return void
 */
void automation() {
  
    float  Kp[3]       = {0, data.kp, data.kp}; // P coefficients in that order : Yaw, Pitch, Roll //ku = 0.21
    double Ki[3]       = {0, data.ki, data.ki};  // I coefficients in that order : Yaw, Pitch, Roll
    float  Kd[3]       = {0, data.kd, data.kd};    // D coefficients in that order : Yaw, Pitch, Roll
    float deltaErr[3] = {0, 0, 0};    // Error deltas in that order :  Yaw, Pitch, Roll
    float yaw         = 0;
    float pitch       = 0;
    float roll        = 0;

    // Initialize motor commands with throttle
    pulse_length_esc1 = instruction[THROTTLE];
    pulse_length_esc2 = instruction[THROTTLE];
    pulse_length_esc3 = instruction[THROTTLE];
    pulse_length_esc4 = instruction[THROTTLE];

    // Do not calculate anything if throttle is 0
    if (instruction[THROTTLE] >= 1012) {

        writeDebugData("YAW: Error: "+String(errors[YAW]));
        writeDebugData("PITCH Error: "+String(errors[PITCH]));
        writeDebugData("ROLL error: "+String(errors[ROLL]));

        errors[YAW] = 0;
        /*
        errors[PITCH] = 0;
        errors[ROLL] = 0;
        */
      
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
 * Calculate errors of Yaw, Pitch & Roll: this is simply the difference between the measure and the command.
 *
 * @return void
 */
void calculateErrors() {
    errors[YAW]   = measures[YAW]   - instruction[YAW];
    errors[PITCH] = measures[PITCH] - instruction[PITCH];
    errors[ROLL]  = measures[ROLL]  - instruction[ROLL];
}

/**
 * Calculate real value of flight instructions from pulses length of each channel.
 *
 * - Roll     : from -33° to 33°
 * - Pitch    : from -33° to 33°
 * - Yaw      : from -180°/sec to 180°/sec
 * - Throttle : from 1000µs to 2000µs
 *
 * @return void
 */
void getFlightInstruction() {
    instruction[YAW]      = map(data.yaw, 0, 250, -180, 180);
    instruction[PITCH]    = map(data.pitch, 0, 250, -33, 33);
    instruction[ROLL]     = map(data.roll, 0, 250, -33, 33);
    instruction[THROTTLE] = map(data.throttle, 0, 250, 1000, 2000);
}

/**
 * Customize mapping of controls: set here which command is on which channel and call
 * this function in setup() routine.
 *
 * @return void
 */
 /*
void configureChannelMapping() {
    mode_mapping[YAW]      = CHANNEL4;
    mode_mapping[PITCH]    = CHANNEL2;
    mode_mapping[ROLL]     = CHANNEL1;
    mode_mapping[THROTTLE] = CHANNEL3;
}
*/

/**
 * This Interrupt Sub Routine is called each time input 8, 9, 10 or 11 changed state.
 * Read the receiver signals in order to get flight instructions.
 *
 * This routine must be as fast as possible to prevent main program to be messed up.
 * The trick here is to use port registers to read pin state.
 * Doing (PINB & B00000001) is the same as digitalRead(8) with the advantage of using less CPU loops.
 * It is less convenient but more efficient, which is the most important here.
 *
 * @see https://www.arduino.cc/en/Reference/PortManipulation
 */
 /*
ISR(PCINT0_vect) {
    current_time = micros();

    // Channel 1 -------------------------------------------------
    if (PINB & B00000001) {                                        // Is input 8 high ?
        if (previous_state[CHANNEL1] == LOW) {                     // Input 8 changed from 0 to 1 (rising edge)
            previous_state[CHANNEL1] = HIGH;                       // Save current state
            timer[CHANNEL1] = current_time;                        // Save current time
        }
    } else if (previous_state[CHANNEL1] == HIGH) {                 // Input 8 changed from 1 to 0 (falling edge)
        previous_state[CHANNEL1] = LOW;                            // Save current state
        pulse_length[CHANNEL1] = current_time - timer[CHANNEL1];   // Calculate pulse duration & save it
    }

    // Channel 2 -------------------------------------------------
    if (PINB & B00000010) {                                        // Is input 9 high ?
        if (previous_state[CHANNEL2] == LOW) {                     // Input 9 changed from 0 to 1 (rising edge)
            previous_state[CHANNEL2] = HIGH;                       // Save current state
            timer[CHANNEL2] = current_time;                        // Save current time
        }
    } else if (previous_state[CHANNEL2] == HIGH) {                 // Input 9 changed from 1 to 0 (falling edge)
        previous_state[CHANNEL2] = LOW;                            // Save current state
        pulse_length[CHANNEL2] = current_time - timer[CHANNEL2];   // Calculate pulse duration & save it
    }

    // Channel 3 -------------------------------------------------
    if (PINB & B00000100) {                                        // Is input 10 high ?
        if (previous_state[CHANNEL3] == LOW) {                     // Input 10 changed from 0 to 1 (rising edge)
            previous_state[CHANNEL3] = HIGH;                       // Save current state
            timer[CHANNEL3] = current_time;                        // Save current time
        }
    } else if (previous_state[CHANNEL3] == HIGH) {                 // Input 10 changed from 1 to 0 (falling edge)
        previous_state[CHANNEL3] = LOW;                            // Save current state
        pulse_length[CHANNEL3] = current_time - timer[CHANNEL3];   // Calculate pulse duration & save it
    }

    // Channel 4 -------------------------------------------------
    if (PINB & B00001000) {                                        // Is input 11 high ?
        if (previous_state[CHANNEL4] == LOW) {                     // Input 11 changed from 0 to 1 (rising edge)
            previous_state[CHANNEL4] = HIGH;                       // Save current state
            timer[CHANNEL4] = current_time;                        // Save current time
        }
    } else if (previous_state[CHANNEL4] == HIGH) {                 // Input 11 changed from 1 to 0 (falling edge)
        previous_state[CHANNEL4] = LOW;                            // Save current state
        pulse_length[CHANNEL4] = current_time - timer[CHANNEL4];   // Calculate pulse duration & save it
    }
}
*/
void sendFeedback(){
    if((currentTime-lastFeedbackTransmission) > feedbackDelay){
        //TODO: Prepare date for transmission.
        feedback.battery = read_battery_voltage();
        radio.stopListening();
        //Serial.println(currentMicros);
        writeDebugData("Sending feedback...");
        radio.write(&feedback, sizeof(feedback));
        lastFeedbackTransmission = currentTime;
        radio.startListening();
    }
}

float read_battery_voltage()
{
    return (analogRead(battery_voltage_pin)/53.5);
}

void writeDebugData(String text){
    if(debug){
        Serial.println(text);
    }
}


