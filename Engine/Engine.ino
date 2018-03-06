#include <Wire.h>
#include <Servo.h>

Servo M1, M2, M3, M4;

struct instruction {
  byte throttle; //We define each byte of data input, in this case just 6 channels
  byte yaw;
  byte pitch;
  byte roll;
  byte AUX1;
  byte AUX2;
};

instruction data;

void setup() {
  Wire.begin(8);                // join i2c bus with address #8
  Wire.onReceive(receiveEvent); // register event
  Serial.begin(9600);           // start serial for output

  M1.attach(8);  // BR
  M2.attach(7);  // FL
  M3.attach(10); // FR
  M4.attach(9);  // BL
  
}

void setSpeed(Servo ESC, int speed){
  int angle = map(speed, 0, 360, 0, 360); //Sets servo positions to different speeds ESC1.write(angle);
  ESC.write(angle);
}

void loop() {

  setSpeed(firstESC, value);
  
  if(Serial.available()){
    value = Serial.parseInt();
    Serial.println("Speed set to: "+String(value));
  } 
  
  delay(100);
}

// function that executes whenever data is received from master
// this function is registered as an event, see setup()
void receiveEvent(int howMany) {
  byte b[4];
  int x = 0;
  
  while (Wire.available()) { // loop through all but the last
    b[x] = Wire.read(); // receive byte as a character
    x++;
  }

  Serial.println(b[0]);
  Serial.println(b[1]);
  Serial.println(b[2]);
  Serial.println(b[3]);
}
