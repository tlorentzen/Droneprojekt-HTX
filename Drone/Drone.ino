#include <Wire.h>
#include <SPI.h>
#include <printf.h>
#include <nRF24L01.h>
#include <RF24_config.h>
#include <RF24.h>

RF24 radio(A0, 8);

// Unique transmitting pipe ids
const uint64_t pipeIn  = 0xE8E8F0F0E1LL;
const uint64_t pipeOut = 0xE7E8F0F0E1LL;

struct instruction {
  byte throttle; //We define each byte of data input, in this case just 6 channels
  byte yaw;
  byte pitch;
  byte roll;
  byte AUX1;
  byte AUX2;
};

struct drone_feedback {
  byte battery;
  byte error = 0;
};

instruction data;
drone_feedback feedback;

unsigned long currentMicros = 0;
unsigned long lastLoopTime = 0;
unsigned long lastRecievedTransmission = 0;
unsigned long lastFeedbackTransmission = 0;
unsigned long  feedbackDelay = 3000000; // 3 seconds
unsigned long  lastTrasmissionTimeout = 1000000;

void setup()
{
  printf_begin();
  
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
}

void sendFeedback(){
  if((currentMicros-lastFeedbackTransmission) >= feedbackDelay){

    //TODO: Prepare date for transmission.
    feedback.battery = 100; // Dummy

    radio.stopListening();
    //Serial.println(currentMicros);
    Serial.println("Sending feedback...");
    radio.write(&feedback, sizeof(feedback));
    lastFeedbackTransmission = currentMicros;
    radio.startListening();
  }
}

void sendinstructions() {
  byte b[4];

  data.throttle = 10;
  data.yaw = 20;
  data.pitch = 30;
  data.roll = 40;

  b[0] = data.throttle;
  b[1] = data.yaw;
  b[2] = data.pitch;
  b[3] = data.roll;

  Wire.beginTransmission(8); // transmit to device #8
  Wire.write(b, sizeof(b));        // sends five bytes
  Wire.endTransmission();    // stop transmitting
}

void loop()
{
  currentMicros = micros();

  // Recieve controller transmissions if any.
  if (radio.available())
  {
    radio.read(&data, sizeof(data));
    Serial.println(data.throttle);
    lastRecievedTransmission = currentMicros;
  }

  // Check if we lost connection to controller.
  if((currentMicros - lastRecievedTransmission) >= lastTrasmissionTimeout){
      Serial.println("Connection lost...");
  }

  // Send feedback if nesecerry. 
  sendFeedback();

  sendinstructions();
  
  delay(500);
}
