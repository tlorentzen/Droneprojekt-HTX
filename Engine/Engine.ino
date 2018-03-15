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
    setSpeed(M1, fvalue);
    setSpeed(M3, fvalue);
    // Back engines
    setSpeed(M2, bvalue);
    setSpeed(M4, bvalue);
    
  }else if(value < 125){
    // Backward
    fvalue = (125+(125-value));
    bvalue = value;

    // Front engines
    setSpeed(M1, fvalue);
    setSpeed(M3, fvalue);
    // Back engines
    setSpeed(M2, bvalue);
    setSpeed(M4, bvalue);
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

    // Right engines
    setSpeed(M1, rvalue);
    setSpeed(M4, rvalue);
    // Left engines
    setSpeed(M2, lvalue);
    setSpeed(M3, lvalue);
    
  }else if(value < 125){
    // Left
    lvalue = value;
    rvalue = (125+(125-value));

    // Right engines
    setSpeed(M1, rvalue);
    setSpeed(M4, rvalue);
    // Left engines
    setSpeed(M2, lvalue);
    setSpeed(M3, lvalue);
  }else{
    // Zero out.
    // TODO??
  }
}


