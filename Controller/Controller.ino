#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <SPI.h>
#include <printf.h>
#include <nRF24L01.h>
#include <RF24_config.h>
#include <RF24.h>

//RF24 radio(9, 10);
RF24 radio(7, 8);
LiquidCrystal_I2C lcd(0x27, 16, 2);

// Unique transmitting pipe ids
const uint64_t pipeOut = 0xE8E8F0F0E1LL;
const uint64_t pipeIn  = 0xE7E8F0F0E1LL;

unsigned long displayUpdaterate = 500;
unsigned long previousTime = 0;
unsigned long currentTime = 0;
unsigned long displayLastUpdated = 0;

struct instruction {
    byte throttle = 0;
    byte yaw = 125;
    byte pitch = 125;
    byte roll = 125;
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

#define LED_RED    6
#define LED_YELLOW 8
#define LED_GREEN  7

#define INPUT_THROTTLE A0
#define INPUT_YAW A1
#define INPUT_ROLL A2
#define INPUT_PITCH A3

void setup() {
    Wire.begin();
    
    lcd.begin();
    //lcd.backlight();
    lcd.clear();
    
    // Setup ledpins
    pinMode(LED_RED, OUTPUT);
    pinMode(LED_YELLOW, OUTPUT);
    pinMode(LED_GREEN, OUTPUT);
  
    digitalWrite(LED_RED, HIGH);
    digitalWrite(LED_YELLOW, HIGH);
    digitalWrite(LED_GREEN, HIGH);
    
    // put your setup code here, to run once:
    printf_begin();
  
    Serial.begin(9600);

    lcd.print("Initializing!");
    lcd.setCursor(0,1);

    //10100101
    
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
    /*
    while(analogRead(speeder) > 0){
      delay(500);
      Serial.println("Please set throttle to zero!");
    }
    */
  
    
    digitalWrite(LED_RED, LOW);
    digitalWrite(LED_YELLOW, LOW);
    digitalWrite(LED_GREEN, LOW);
    delay(500);
    digitalWrite(LED_RED, HIGH);
    delay(500);
    digitalWrite(LED_YELLOW, HIGH);
    delay(500);
    digitalWrite(LED_GREEN, HIGH);
    delay(500);
    digitalWrite(LED_RED, LOW);
    delay(500);
    digitalWrite(LED_YELLOW, LOW);
    delay(500);
    digitalWrite(LED_GREEN, LOW);

    lcd.clear();
    lcd.print("Ready!");
    delay(1000);
    lcd.clear();

    lcd.print("Battery: 0.00v");
    lcd.setCursor(0,1);
    lcd.print("Throttle: 0 %");
}

void loop() {

    previousTime = currentTime;
    currentTime = millis();

    if (Serial.available()) {
        String input1   = Serial.readString();
        String command1 = getValue(input1,'=',0);
        double number1  = getValue(input1,'=',1).toDouble();
  
        if(command1 == "p"){
            data.kp = number1;
        }else if(command1 == "i"){
            data.ki = number1;
        }else if(command1 == "d"){
            data.kd = number1;
        }
        
        Serial.println(command1+"="+String(number1));
    }

    readControllerValues();
    
    getDroneFeedback();

    /*
    int value = analogRead(speeder);
    value = map(value, 0, 1023, 0, 250);
    data.throttle = value;
    //Serial.println(value);
    */
  
    sendInstructions();

    writeDisply();
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

void readControllerValues()
{ 
    data.yaw      = map(analogRead(INPUT_YAW), 0, 1023, 0, 250);
    data.roll     = map(analogRead(INPUT_ROLL), 0, 1023, 0, 250);
    data.pitch    = map(analogRead(INPUT_PITCH), 0, 1023, 0, 250);
    data.throttle = map(analogRead(INPUT_THROTTLE), 0, 1023, 0, 250);

    /*
    int throttle = map(analogRead(INPUT_THROTTLE), 0, 1023, -10, 10);

    if(throttle > 2 || throttle < -2){
        int throttle_out = ((int)data.throttle+throttle);

        if(throttle_out > 250){
            throttle_out = 250;
        }else if(throttle_out < 0){
            throttle_out = 0;
        }

        data.throttle = (byte)throttle_out;
    }  

    */
  
    //Serial.println("Throttle: "+String(throttle)+" - Data.throttle: "+String(data.throttle)+" YAW: "+String(data.yaw));
}

void sendInstructions()
{
    radio.stopListening();
    radio.write(&data, sizeof(data));
    radio.startListening();
}

void getDroneFeedback()
{
    if (radio.available())
    {
        radio.read(&feedback, sizeof(feedback));
        Serial.println("Battery: "+String(feedback.battery)+" V");
    }
}

void writeDisply()
{   
    if((currentTime-displayLastUpdated) > displayUpdaterate){

        lcd.setCursor(9,0);
        String battery_info = String(feedback.battery)+"v";
        lcd.print(battery_info);
        lcd.setCursor(10,1);
        String throttle_info = String(map((int)data.throttle, 0, 250, 0, 100))+" %";
        lcd.print(throttle_info);
        displayLastUpdated = currentTime;
    }
}

