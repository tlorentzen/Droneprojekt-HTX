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

byte latest_throttle = 0;

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
  delay(10);
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

  throttle(b[0]);

}

void throttle(int value)
{
 
  Serial.println("Throttle: "+String(value));

  //if(latest_throttle != value){
    if(value > 72){
      value = 36;
      setSpeed(M1, 36);
      setSpeed(M2, 36);
      setSpeed(M3, 36);
      setSpeed(M4, 36);
    }else{
      setSpeed(M1, value);
      setSpeed(M2, value);
      setSpeed(M3, value);
      setSpeed(M4, value);
    }
  //}

  latest_throttle = value;
}

void yaw(int value)
{
  
}

void pitch(int value)
{
  /*
  if(value == 127){
    
  }else if(value > 127){
    
  }
  
  M1.attach(8);  // BR
  M2.attach(7);  // FL
  M3.attach(10); // FR
  M4.attach(9);  // BL

  // Forward
  setSpeed(M1, value);
  setSpeed(M4, value);

  setSpeed(M2, );
  
*/
  
    
}

void roll(int value)
{
  
}


