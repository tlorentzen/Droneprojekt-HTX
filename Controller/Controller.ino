#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <SPI.h>
#include <printf.h>
#include <nRF24L01.h>
#include <RF24_config.h>
#include <RF24.h>

RF24 radio(9, 10);
//RF24 radio(7, 8);
LiquidCrystal_I2C lcd(0x27, 16, 2);

// Unique transmitting pipe ids
const uint64_t pipeOut = 0xE8E8F0F0E1LL;
const uint64_t pipeIn  = 0xE7E8F0F0E1LL;

unsigned long previousTime = 0;
unsigned long currentTime = 0;
unsigned long lastFeedbackResponse = 0;
unsigned int feedbackResponseTimeout = 2500;
bool firstPackageRecieved = false;

struct instruction {
    long throttle = 0;
    byte yaw = 125;
    byte pitch = 125;
    byte roll = 125;
    float kp = 0.2;
    double ki = 0;
    float kd = 20;
};

struct drone_feedback {
    float battery;
    byte  error = 0;
    float yaw_error = 0.0;
    float roll_error = 0.0;
    float pitch_error = 0.0;
    long  loop_time = 0;
    long  throttle = 0;
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

    radio.begin();
    radio.setPALevel(RF24_PA_MAX);
    radio.setAutoAck(false);
    radio.setDataRate(RF24_250KBPS);
    radio.setChannel(108);
    radio.openWritingPipe(pipeOut);
    radio.openReadingPipe(1, pipeIn);
    radio.startListening();
    radio.printDetails();

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

    sendInstructions();
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
    data.yaw   = map(analogRead(INPUT_YAW), 0, 1023, 0, 250);
    data.roll  = map(analogRead(INPUT_PITCH), 0, 1023, 0, 250);
    data.pitch = map(analogRead(INPUT_ROLL), 0, 1023, 0, 250);

    //Serial.println("Yaw: "+String(analogRead(INPUT_YAW)));

    if(data.yaw < 150 && data.yaw > 100){
        data.yaw = 125;
    }

    if(data.roll < 130 && data.roll > 120){
        data.roll = 125;
    }

    if(data.pitch < 130 && data.pitch > 120){
        data.pitch = 125;
    }

    int throttle = map(analogRead(INPUT_THROTTLE), 0, 1023, -1000, 1000);
    long throttle_out = 0;

    if(throttle > 5 || throttle < -5){

        // Positive throttle
        if(throttle > 5 && throttle < 300){
            throttle_out = ((int)data.throttle+1); // throttle_out = throttle_out * gain
        }else if(throttle >= 300 && throttle < 600){
            throttle_out = ((int)data.throttle+2);
        }else if(throttle >= 600 && throttle < 1000){
            throttle_out = ((int)data.throttle+2);
        }else if(throttle == 1000){
            throttle_out = ((int)data.throttle+5);
        }

        // Negative throttle
        if(throttle < -5 && throttle > -300){
            throttle_out = ((int)data.throttle-1);
        }else if(throttle <= -300 && throttle > -600){
            throttle_out = ((int)data.throttle-2);
        }else if(throttle <= -600 && throttle > -1000){
            throttle_out = ((int)data.throttle-2);
        }else if(throttle == -1000){
            throttle_out = ((int)data.throttle-5);
        }

        // Check for out of range values.
        if(throttle_out > 1000){
            throttle_out = 1000;
        }else if(throttle_out < 0){
            throttle_out = 0;
        }

        data.throttle = throttle_out;
    }

    // Write out current throttle level to display
    if(data.throttle < 100){
        writeDisply("", "Throttle: "+String(map((int)data.throttle, 0, 1000, 0, 100))+" %   ");
    }else if(data.throttle >= 100 && data.throttle < 1000){
        writeDisply("", "Throttle: "+String(map((int)data.throttle, 0, 1000, 0, 100))+" %  ");
    }else{
        writeDisply("", "Throttle: "+String(map((int)data.throttle, 0, 1000, 0, 100))+" % ");
    }
}

void sendInstructions()
{   
    if(firstPackageRecieved){
        radio.stopListening();
        radio.write(&data, sizeof(data));
        radio.startListening();
    }
}

void getDroneFeedback()
{
    if (radio.available()){
        lastFeedbackResponse = millis();
        radio.read(&feedback, sizeof(feedback));
        Serial.println("Battery: "+String(feedback.battery)+"v");
        Serial.println("P: "+String(feedback.pitch_error)+" R: "+String(feedback.roll_error)+" Y: "+String(feedback.yaw_error));
        setBatteryIndikator(feedback.battery);
        Serial.println("LoopTime: "+String(feedback.loop_time));

        if(!firstPackageRecieved){
            data.throttle = feedback.throttle;
            firstPackageRecieved = true;
        }
    }
}

void writeDisply(String l1, String l2)
{   
    if(l1 != ""){
        lcd.setCursor(0,0);
        lcd.print(l1);
    }

    if(l2 != ""){
        lcd.setCursor(0,1);
        lcd.print(l2);
    }
}

void setBatteryIndikator(float voltage)
{
    if(voltage < 9){
        // Battery voltage is at a state where nothing works.
        writeDisply("Battery: ???    ", "");
        digitalWrite(LED_RED, LOW);
        digitalWrite(LED_YELLOW, HIGH);
        digitalWrite(LED_GREEN, LOW);
    }else if(voltage < 13){
        // Battery voltage is at a state where landing is a good idea
        writeDisply("Battery: "+String(voltage)+"v! ", "");
        digitalWrite(LED_RED, HIGH);
        digitalWrite(LED_YELLOW, LOW);
        digitalWrite(LED_GREEN, LOW);
    }else{
        // Battery voltage is fine and normal flight capability should be espected 
        writeDisply("Battery: "+String(voltage)+"v  ", "");
        digitalWrite(LED_RED, LOW);
        digitalWrite(LED_YELLOW, LOW);
        digitalWrite(LED_GREEN, HIGH);
    }
}
