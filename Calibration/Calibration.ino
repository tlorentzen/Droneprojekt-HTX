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

void setup() {
    // put your setup code here, to run once:
    Serial.begin(9600);
    Wire.begin();

    // Connect to MPU
    if (IMU.begin() < 0) {
        Serial.println("IMU initialization unsuccessful");
        Serial.println("Check IMU wiring or try cycling power");
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
    }

    // Arm motors
    M1.arm();
    M2.arm();
    M3.arm();
    M4.arm();
}

void loop() {

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

    // Read accelerometer data.
    float ax = IMU.getAccelX_mss();
    float ay = IMU.getAccelY_mss();
    float az = IMU.getAccelZ_mss();

    

    // Set motor values
    if(command == "m1"){
        M1.speed(value);
    }else if(command == "m2"){
        M2.speed(value);
    }else if(command == "m3"){
        M3.speed(value);
    }else if(command == "m4"){
        M4.speed(value);
    }else if(command == "all"){
        M1.speed(value);
        M2.speed(value);
        M3.speed(value);
        M4.speed(value);
    }else{
        M1.speed(1000);
        M2.speed(1000);
        M3.speed(1000);
        M4.speed(1000);
    }
    
    // Write vibrations to serial (Serial plotter)
    Serial.println((ax+ay+az));
    Serial.print(" ");
    
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
