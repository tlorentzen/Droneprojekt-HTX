#include <SPI.h>
#include <printf.h>
#include <nRF24L01.h>
#include <RF24_config.h>
#include <RF24.h>

RF24 radio(7, 8);

// Unique transmitting pipe ids
const uint64_t pipeOut = 0xE8E8F0F0E1LL;
const uint64_t pipeIn  = 0xE7E8F0F0E1LL;

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

void setup() {
  // put your setup code here, to run once:
  printf_begin();

  Serial.begin(9600);
  
  radio.begin();
  radio.setRetries(2, 2);
  radio.setPALevel(RF24_PA_LOW);
  radio.setAutoAck(false);
  radio.setDataRate(RF24_250KBPS);
  radio.openWritingPipe(pipeOut);
  radio.openReadingPipe(1, pipeIn);
  
  radio.startListening();

  radio.printDetails();

  data.throttle = 10;

}

void loop() {

  data.throttle++;

  if (radio.available())
  {
    radio.read(&feedback, sizeof(feedback));
    Serial.println("Battery: "+String(feedback.battery)+" %");
  }

  radio.stopListening();
  radio.write(&data, sizeof(data));
  radio.startListening();
  
  
  delay(500);
  
}
