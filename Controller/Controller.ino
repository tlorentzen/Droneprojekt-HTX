#include <SPI.h>
#include <printf.h>
#include <nRF24L01.h>
#include <RF24_config.h>
#include <RF24.h>

RF24 radio(7, 8);

int speeder = A0;

// Unique transmitting pipe ids
const uint64_t pipeOut = 0xE8E8F0F0E1LL;
const uint64_t pipeIn  = 0xE7E8F0F0E1LL;

struct instruction {
  byte throttle = 0;
  byte yaw;
  byte pitch = 127;
  byte roll = 127;
  byte AUX1;
  byte AUX2;
};

struct drone_feedback {
  byte battery;
  byte error = 0;
};

int value = 0;

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

  //data.throttle = 10;

  while(analogRead(speeder) > 0){
    delay(500);
    Serial.println("Please set throttle to zero!");
  }

}

void loop() {

  //data.throttle++;

  if (radio.available())
  {
    radio.read(&feedback, sizeof(feedback));
    Serial.println("Battery: "+String(feedback.battery)+" %");
  }

  int value = analogRead(speeder);
  value = map(value, 0, 1023, 0, 250);
  data.throttle = value;
  Serial.println(value);

  radio.stopListening();
  radio.write(&data, sizeof(data));
  radio.startListening();

  

  /*
  if(Serial.available()){
    value = Serial.parseInt();

    if(value > 72){
      value = 36;
    }
    
    data.throttle = value;
    Serial.println("Throttle set to: "+String(value));
  }
  */
  
  //delay(500);
  
}
