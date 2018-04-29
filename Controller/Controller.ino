#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>

// Structs containing radio message structure for both instructions and feedback.
struct instruction {
  int throttle = 1000;
  int yaw = 0;
  int pitch = 0;
  int roll = 0;
} data;

struct drone_feedback {
  float battery = 0.0;
  int  throttle = 0;
} feedback;

#define LED_RED    6
#define LED_YELLOW 8
#define LED_GREEN  7

#define INPUT_THROTTLE A0
#define INPUT_YAW A1
#define INPUT_ROLL A3
#define INPUT_PITCH A2

RF24 radio(9, 10);
LiquidCrystal_I2C lcd(0x27, 16, 2);

// Unique transmitting pipe ids
const uint64_t pipeOut = 0xE8E8F0F0E1LL;
const uint64_t pipeIn  = 0xE7E8F0F0E1LL;

unsigned long currentTime = 0;
bool firstPackageRecieved = false;

// Set controller limits
int pitchLimit = 20; // +- Degress
int rollLimit = 20; // +- Degress
int yawLimit = 180; // +- Degress

/**
   Setup configuration
*/
void setup() {
  // Initialize I2C
  Wire.begin();

  // Initialize LCD display
  lcd.begin();
  lcd.clear();

  // Setup ledpins
  pinMode(LED_RED, OUTPUT);
  pinMode(LED_YELLOW, OUTPUT);
  pinMode(LED_GREEN, OUTPUT);

  // Turn all leds on
  digitalWrite(LED_RED, HIGH);
  digitalWrite(LED_YELLOW, HIGH);
  digitalWrite(LED_GREEN, HIGH);

  lcd.print("Initializing!");
  lcd.setCursor(0,1);
  
  // Initialize radio communikation (nRF24L01+)
  radio.begin();
  radio.setPALevel(RF24_PA_MAX);
  radio.setAutoAck(false);
  radio.setDataRate(RF24_250KBPS);
  radio.setChannel(108);
  radio.openWritingPipe(pipeOut);
  radio.openReadingPipe(1, pipeIn);
  radio.startListening();

  // Initialized
  lcd.clear();
  lcd.print("Ready!");
  delay(1000);
  lcd.clear();

  // Turn leds off for now.
  digitalWrite(LED_RED, LOW);
  digitalWrite(LED_YELLOW, LOW);
  digitalWrite(LED_GREEN, LOW);
}

/**
   Main program loop
*/
void loop() {
  // Keep track of time
  currentTime = millis();

  // Read user inputs
  readControllerValues();

  // Send instructions to drone
  sendInstructions();

  // Recieve feedback from drone
  checkForDroneFeedback();
}

/**
   Read joystik values and converts them into values that the drone 
   understands. Then stores them in the instruction package. 

   @return void
*/
void readControllerValues()
{
  int input_yaw   = analogRead(INPUT_YAW);
  int input_pitch = analogRead(INPUT_PITCH);
  int input_roll  = analogRead(INPUT_ROLL);
  
  data.yaw   = map(input_yaw,   0, 1023,  yawLimit,   -yawLimit);
  data.roll  = map(input_roll, 0, 1023,   rollLimit,  -rollLimit);
  data.pitch = map(input_pitch,  0, 1023, pitchLimit, -pitchLimit);

  if (input_yaw < 530 && input_yaw > 490) {
    data.yaw = 0;
  }

  if (input_roll < 530 && input_roll > 490) {
    data.roll = 0;
  }

  if (input_pitch < 530 && input_pitch > 490) {
    data.pitch = 0;
  }

  int throttle = map(analogRead(INPUT_THROTTLE), 0, 1023, -1000, 1000);
  int throttle_out = 1000;

  if (throttle > 10 || throttle < -10) {

    // Positive throttle
    if (throttle > 10 && throttle < 300) {
      throttle_out = (data.throttle + 1);
    } else if (throttle >= 300 && throttle < 600) {
      throttle_out = (data.throttle + 2);
    } else if (throttle >= 600 && throttle < 1000) {
      throttle_out = (data.throttle + 2);
    } else if (throttle == 1000) {
      throttle_out = (data.throttle + 5);
    }

    // Negative throttle
    if (throttle < -10 && throttle > -300) {
      throttle_out = (data.throttle - 1);
    } else if (throttle <= -300 && throttle > -600) {
      throttle_out = (data.throttle - 2);
    } else if (throttle <= -600 && throttle > -1000) {
      throttle_out = (data.throttle - 2);
    } else if (throttle == -1000) {
      throttle_out = (data.throttle - 5);
    }

    // Check for out of range values.
    if (throttle_out > 2000) {
      throttle_out = 2000;
    } else if (throttle_out < 1000) {
      throttle_out = 1000;
    }

    data.throttle = throttle_out;
  }

  // Write out current throttle level to display
  if (data.throttle < 1100) {
    writeDisply("", "Throttle: " + String(map(data.throttle, 1000, 2000, 0, 100)) + " %   ");
  } else if (data.throttle >= 1100 && data.throttle < 2000) {
    writeDisply("", "Throttle: " + String(map(data.throttle, 1000, 2000, 0, 100)) + " %  ");
  } else {
    writeDisply("", "Throttle: " + String(map(data.throttle, 1000, 2000, 0, 100)) + " % ");
  }
}

/**
   Send instruction package to drone.

   @return void
*/
void sendInstructions()
{
  if (firstPackageRecieved) {
    radio.stopListening();
    radio.write(&data, sizeof(data));
    radio.startListening();
  }
}

/**
   Checks if there are any feedback from the drone, and set it in the feedback object.

   @return void
*/
void checkForDroneFeedback()
{
  if (radio.available()) {
    radio.read(&feedback, sizeof(feedback));
    setBatteryIndikator(feedback.battery);

    // if then controller got restareted or lost connection douing flight, 
    // wait to get the drones current throttle back before enabling control again.
    if (!firstPackageRecieved) {
      data.throttle = feedback.throttle;
      firstPackageRecieved = true;
    }
  }
}

/**
   Helper function to write display lines.

   @param l1 Content to write to first line on the display.
   @param l2 Content to write to first second on the display.
   @return void
*/
void writeDisply(String l1, String l2)
{
  if (l1 != "") {
    lcd.setCursor(0, 0);
    lcd.print(l1);
  }

  if (l2 != "") {
    lcd.setCursor(0, 1);
    lcd.print(l2);
  }
}

/**
   Set drone battery level in display

   @param voltage Current battery level in voltage.
   @return void
*/
void setBatteryIndikator(float voltage)
{
  if (voltage < 9) {
    // Battery voltage is at a state where nothing works.
    writeDisply("Battery: ???    ", "");
    digitalWrite(LED_RED, LOW);
    digitalWrite(LED_YELLOW, HIGH);
    digitalWrite(LED_GREEN, LOW);
  } else if (voltage < 14) {
    // Battery voltage is at a state where landing is a good idea
    writeDisply("Battery: " + String(voltage) + "v! ", "");
    digitalWrite(LED_RED, HIGH);
    digitalWrite(LED_YELLOW, LOW);
    digitalWrite(LED_GREEN, LOW);
  } else {
    // Battery voltage is fine and normal flight capability should be espected
    writeDisply("Battery: " + String(voltage) + "v  ", "");
    digitalWrite(LED_RED, LOW);
    digitalWrite(LED_YELLOW, LOW);
    digitalWrite(LED_GREEN, HIGH);
  }
}
