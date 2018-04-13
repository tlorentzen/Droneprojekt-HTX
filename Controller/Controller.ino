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
  float kp = 0;
  double ki = 0;
  float kd = 0;
};

struct drone_feedback {
  float battery;
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

  if (Serial.available()) {
      String input1 = Serial.readString();
      String command1 = getValue(input1,'=',0);
      double number1 = getValue(input1,'=',1).toDouble();

      if(command1 == "p"){
        data.kp = number1;
      }else if(command1 == "i"){
        data.ki = number1;
      }else if(command1 == "d"){
        data.kd = number1;
      }
      
      Serial.println(command1+"="+String(number1));
  }

  //data.throttle++;

  if (radio.available())
  {
    radio.read(&feedback, sizeof(feedback));
    Serial.println("Battery: "+String(feedback.battery)+" V");
  }

  int value = analogRead(speeder);
  value = map(value, 0, 1023, 0, 250);
  data.throttle = value;
  //Serial.println(value);

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
