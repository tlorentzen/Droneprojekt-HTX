#include <ESC.h>
#include <Wire.h>

ESC M1 (8, 1000, 2000, 500);
ESC M2 (7, 1000, 2000, 500);
ESC M3 (10, 1000, 2000, 500);
ESC M4 (9, 1000, 2000, 500);

#define PITCH_MIN -30
#define PITCH_MAX 30
#define ROLL_MIN -30
#define ROLL_MAX 30
#define YAW_MIN -180
#define YAW_MAX 180

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

  /*
  Wire.begin(8);                // join i2c bus with address #8
  Wire.onReceive(receiveEvent); // register event
  */
  Serial.begin(9600);           // start serial for output
  
  M1.arm();
  M2.arm();
  M3.arm();
  M4.arm();

  Serial.println("Ready!");
}
/*
void setSpeed(ESC M, int speed){
  int angle = map(speed, 0, 360, 0, 360); //Sets servo positions to different speeds ESC1.write(angle);
  ESC.write(angle);
}
**/

void loop() {
  //delay(10);

  if(Serial.available()){
      int value = Serial.parseInt();
      Serial.println("Speed set to: "+String(value));
      throttle(value);

      
  }
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
  value = map(value, 0, 250, 1000, 2000);
  Serial.println("Throttle: "+String(value));

  M1.speed(value);
  M2.speed(value);
  M3.speed(value);
  M4.speed(value);
}

void yaw(int value)
{
  /*
  int lvalue = 125;
  int hvalue = 125;

  if(value > 125){
    // Right
    lvalue = value;
    hvalue = (125-(value-125));

    // Right engines
    setSpeed(M1, rvalue);
    setSpeed(M2, rvalue);
    // Left engines
    setSpeed(M3, lvalue);
    setSpeed(M4, lvalue);
    
  }else if(value < 125){
    // Left
    lvalue = value;
    rvalue = (125+(125-value));

    // Right engines
    setSpeed(M1, rvalue);
    setSpeed(M2, rvalue);
    // Left engines
    setSpeed(M3, lvalue);
    setSpeed(M4, lvalue);
  }else{
    // Zero out.
    // TODO??
  }
  */
}

void pitch(int value)
{

  int fvalue = 125;
  int bvalue = 125;

  if(value > 125){
    // Forward
    fvalue = (125-(value-125));
    bvalue = value;

    // Front engines
    M1.speed(fvalue);
    M3.speed(fvalue);
    // Back engines
    M2.speed(bvalue);
    M4.speed(bvalue);
    
  }else if(value < 125){
    // Backward
    fvalue = (125+(125-value));
    bvalue = value;

    // Front engines
    M1.speed(fvalue);
    M3.speed(fvalue);
    // Back engines
    M2.speed(bvalue);
    M4.speed(bvalue);
  }else{
    // Zero out.
    // TODO??
  }
}

void roll(int value)
{

  
  int lvalue = 125;
  int rvalue = 125;

  if(value > 125){
    // Right
    lvalue = value;
    rvalue = (125-(value-125));

    lvalue = map(lvalue, 0, 250, 1000, 2000);
    
    // Right engines
    M1.speed(rvalue);
    M4.speed(rvalue);
    // Left engines
    M2.speed(lvalue);
    M3.speed(lvalue);
    
  }else if(value < 125){
    // Left
    lvalue = value;
    rvalue = (125+(125-value));

    // Right engines
    M1.speed(rvalue);
    M4.speed(rvalue);
    // Left engines
    M2.speed(lvalue);
    M3.speed(lvalue);
  }else{
    // Zero out.
    // TODO??
  }
}


