#include <Wire.h>
#include <MPU9250.h>
#include <ESC.h>

// MPU
MPU9250 IMU(Wire,0x68);

// Motors
ESC M1 (8, 1000, 2000, 500);
ESC M2 (9, 1000, 2000, 500);
ESC M3 (6, 1000, 2000, 500);
ESC M4 (7, 1000, 2000, 500);

// Variables
String command = "";
int    value   = 0;

float x = 0.0;
float y = 0.0;
float z = 0.0;

int gyro_x, gyro_y, gyro_z;
long acc_x, acc_y, acc_z, acc_total_vector;
int temperature;
long gyro_x_cal, gyro_y_cal, gyro_z_cal;
float angle_pitch, angle_roll;
int angle_pitch_buffer, angle_roll_buffer;
boolean set_gyro_angles;
float angle_roll_acc, angle_pitch_acc;

unsigned long previousTime = 0;
unsigned long currentTime = 0;

float cx[5];
float cy[5];
float cz[5];

void setup() {
    // put your setup code here, to run once:
    Serial.begin(9600);
    Wire.begin();

    // Connect to MPU
    if (IMU.begin() < 0) {
        Serial.println("IMU initialization unsuccessful");
        Serial.println("Check IMU wiring or try cycling power");
        //while(1) {}
    }else{
        // setting the accelerometer full scale range to +/-8G 
        IMU.setAccelRange(MPU9250::ACCEL_RANGE_8G);
        // setting the gyroscope full scale range to +/-500 deg/s
        IMU.setGyroRange(MPU9250::GYRO_RANGE_2000DPS);
        // setting DLPF bandwidth to 20 Hz
        IMU.setDlpfBandwidth(MPU9250::DLPF_BANDWIDTH_20HZ);
        // setting SRD to 19 for a 50 Hz update rate
        IMU.setSrd(19);
    }

    // Arm motors
    M1.arm();
    M2.arm();
    M3.arm();
    M4.arm();
}

void loop() {

    readMPU();
    
    previousTime = currentTime;
    currentTime = millis();

    IMU.readSensor();
    
    // Read serial inputs
    if (Serial.available()) {
        String input = Serial.readString();

        if(input.indexOf("=") > 0){
            command = getValue(input, '=', 0);
            value   = getValue(input, '=', 1).toInt();
        }else{
            command = input;
            value   = 0;
        }

        command.toLowerCase();
        
        if(command == "stop"){
            M1.speed(1000);
            M2.speed(1000);
            M3.speed(1000);
            M4.speed(1000);
        }
    }

    // Read gyroscope data
    float gx = IMU.getGyroX_rads();
    float gy = IMU.getGyroY_rads();
    float gz = IMU.getGyroZ_rads(); 

    // Set motor values
    if(command == "m1"){
        M1.speed(value);
        M2.speed(1000);
        M3.speed(1000);
        M4.speed(1000);
    }else if(command == "m2"){
        M1.speed(1000);
        M2.speed(value);
        M3.speed(1000);
        M4.speed(1000);
    }else if(command == "m3"){
        M1.speed(1000);
        M2.speed(1000);
        M3.speed(value);
        M4.speed(1000);
    }else if(command == "m4"){
        M1.speed(1000);
        M2.speed(1000);
        M3.speed(1000);
        M4.speed(value);
    }else if(command == "all"){
        M1.speed(value);
        M2.speed(value);
        M3.speed(value);
        M4.speed(value);
    }
    
    // Write vibrations to serial (Serial plotter)
    Serial.println((x+y+z));
    Serial.print(" ");
    
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

    x = gyroX;
    y = gyroY;
    z = var_compass;

    memcpy(cx, &cx[1], sizeof(cx) - sizeof(float));
    cx[5] = x;

    memcpy(cy, &cy[1], sizeof(cy) - sizeof(float));
    cy[5] = y;

    memcpy(cz, &cz[1], sizeof(cz) - sizeof(float));
    cz[5] = z;

    float xcc = 0.0;
    float ycc = 0.0;
    float zcc = 0.0;
    
    for (int i=0; i <= sizeof(cx); i++){
      xcc += cx[i];
    }

    for (int i=0; i <= sizeof(cy); i++){
      ycc += cy[i];
    }

    for (int i=0; i <= sizeof(cz); i++){
      zcc += cz[i];
    }

    x = (xcc/sizeof(cx));
    y = (ycc/sizeof(cy));
    z = (zcc/sizeof(cz));
    
    /*
    measures[ROLL]  = gyroX * 0.9 + angle_pitch * 0.1;   //Take 90% of the output pitch value and add 10% of the raw pitch value
    measures[PITCH] = gyroY * 0.9 + angle_roll * 0.1; 
    */
    /*
    measures[ROLL] = gyroX;
    measures[PITCH] = gyroY;
    */

    /*
    Serial.println("ROLL-messured: "+String(measures[ROLL]));
    Serial.println("PITCH-messured: "+String(measures[PITCH]));
    Serial.println("YAW-messured: "+String(measures[YAW]));
    */
}

String getValue(String data, char separator, int index)
{
    int found = 0;
    int strIndex[] = {0, -1};
    int maxIndex = data.length()-1;
  
    for(int i=0; i<=maxIndex && found<=index; i++){
        if(data.charAt(i)==separator || i==maxIndex){
            found++;
            strIndex[0] = strIndex[1]+1;
            strIndex[1] = (i == maxIndex) ? i+1 : i;
        }
    }
  
    return found>index ? data.substring(strIndex[0], strIndex[1]) : "";
}
