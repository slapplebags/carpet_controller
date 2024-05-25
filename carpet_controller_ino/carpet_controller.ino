#include <PID_v1.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include "Arduino.h"
#include "TFT_eSPI.h"
#include "pin_config.h"
#include <EEPROM.h>

struct DeviceSettings {
  double heSetpoint;
  double fanSetpoint;
  double coilSetpoint;
  double heMotorTime;
};

DeviceSettings settings; // Create an instance of DeviceSettings

bool lastLbuttonState = HIGH; // Assuming active low buttons
bool lastRbuttonState = HIGH;

const int LbuttonPin = 2; // The pin the button is connected to
const int RbuttonPin = 11; // The pin the button is connected to
const int CbuttonPin = 10; // The pin the button is connected to
volatile int loopState = 0;
volatile int buttonPressed = 0; // Flag to indicate button press
const long debounceDelay = 150; // Debounce delay in milliseconds
unsigned long lastDebounceTime = 0; // Last debounce check time
unsigned long motorStartTime = 0; 
enum MotorState {STOPPED, CLOCKWISE, COUNTERCLOCKWISE} motorState = STOPPED;
enum MotorState lastMotorState = STOPPED; // Add this to track the last non-stopped state
const int heaterPinCW = 43; // Heater control pin
const int heaterPinCCW = 44; // Heater control pin
const int fanPin = 17; // Heater control pin
const int coilPin = 21; // Heater control pin
const double thresholdCW = 0.8; // Threshold for CW close to 1
const double thresholdCCW = 0.2; // Threshold for CCW close to 0

double previousOutput = 0; // Initialize with 0 or an appropriate value.
double Setpoint = 80;
double heSetpoint, Input, Output, Input2, coilSetpoint, fanSetpoint, heMotorTime;

// PID tuning parameters - These need to be adjusted based on your setup and requirements
double Kp = 2.0, Ki = 5.0, Kd = 1.0;
PID myPID(&Input, &Output, &settings.heSetpoint, Kp, Ki, Kd, DIRECT);

TFT_eSPI tft = TFT_eSPI();
const int oneWireBus = 16;
OneWire oneWire(oneWireBus);
DallasTemperature sensors(&oneWire);

void IRAM_ATTR handleButtonPress() {
  unsigned long currentTime = millis();
    buttonPressed = 1; // Set flag to indicate the button was pressed
  }


// Addresses of 2 DS18B20s
//uint8_t sensor1[8] = { 0x28, 0xCA, 0x64, 0x43, 0xD4, 0xE1, 0x3C, 0x03 };
//uint8_t sensor2[8] = { 0x28, 0x75, 0x32, 0x43, 0xD4, 0xE1, 0x3C, 0x29 };
uint8_t sensor1[8] = { 0x28, 0xB9, 0x98, 0x43, 0xD4, 0xE1, 0x3C, 0x45 };
uint8_t sensor2[8] = { 0x28, 0xF5, 0x75, 0x43, 0xD4, 0xE1, 0x3C, 0x7B };

void setup() {
  Serial.begin(115200);
  sensors.begin();
  pinMode(PIN_POWER_ON, OUTPUT);
  pinMode(fanPin, OUTPUT);
  pinMode(coilPin, OUTPUT);
  pinMode(heaterPinCW, OUTPUT);
pinMode(heaterPinCCW, OUTPUT);
  digitalWrite(PIN_POWER_ON, HIGH);
  digitalWrite(fanPin, LOW);
  digitalWrite(coilPin, LOW);
  digitalWrite(heaterPinCW,LOW);
  digitalWrite(heaterPinCCW, LOW);
  tft.begin();
  tft.fillScreen(TFT_BLACK);
  tft.setRotation(3);
  EEPROM.begin(sizeof(DeviceSettings));
  
  pinMode(CbuttonPin, INPUT_PULLUP); // Initialize the button pin as input with pull-up resistor
  attachInterrupt(digitalPinToInterrupt(CbuttonPin), handleButtonPress, RISING); // Attach interrupt  

    EEPROM.get(0, settings);

  // If the EEPROM was never written, initialize with default values
  if (isnan(settings.heSetpoint)) { // Check if settings are uninitialized (NaN)
    settings.heSetpoint = 220; // Default heSetpoint
    settings.fanSetpoint = 217; // Default fanSetpoint
    settings.coilSetpoint = 240; // Default coilSetpoint
    settings.heMotorTime = 3000; // Default heMotorTime

    // Save the initialized settings back to EEPROM
    EEPROM.put(0, settings);
    EEPROM.commit(); // Ensure data is written to flash
  }

  myPID.SetMode(AUTOMATIC); // Turn PID on
  myPID.SetOutputLimits(0, 1); // Set PID output limits to match digital on/off state

}

void loop() {

  if (buttonPressed) {
    noInterrupts(); // Disable interrupts to safely check shared variables
    buttonPressed = 0; // Reset the flag
    loopState = (loopState + 1) % 5; // Cycle through 0, 1, 2
    interrupts(); // Re-enable interrupts
    Serial.print("Loop State Changed to: ");
    Serial.println(loopState);
        tft.fillScreen(TFT_BLACK);
  }
  sensors.requestTemperatures();
  Input = sensors.getTempF(sensor1); // Update current temperature from sensor
  Input2 = sensors.getTempF(sensor2);

  myPID.Compute();

  if (Input2 >= settings.fanSetpoint) {
//    Serial.println("fan on");
    digitalWrite(fanPin, HIGH);
  } else {
//    Serial.println("fan off");
    digitalWrite(fanPin, LOW);
  }

  if (Input2 >= settings.coilSetpoint) {
//    Serial.println("engine off");
    digitalWrite(coilPin, LOW);
  } else if (Input2 < settings.coilSetpoint) {
//    Serial.println("engine on");
    digitalWrite(coilPin, HIGH);
  }


    // Logic for changing motor direction based on PID output and previous state
    if (Output >= thresholdCW && motorState == STOPPED && lastMotorState != CLOCKWISE) {
        Serial.println("PID Output high, starting motor CW.");
        startMotorCW();
        motorState = CLOCKWISE;
        motorStartTime = millis(); // Capture the current time
    } 
    else if (Output <= thresholdCCW && motorState == STOPPED && lastMotorState != COUNTERCLOCKWISE) {
        Serial.println("PID Output low, starting motor CCW.");
        startMotorCCW();
        motorState = COUNTERCLOCKWISE;
        motorStartTime = millis(); // Capture the current time
    }

    // Check if the motor has run for the intended duration
    if ((motorState == CLOCKWISE || motorState == COUNTERCLOCKWISE) && 
        (millis() - motorStartTime > settings.heMotorTime)) {
        Serial.println("Motor run time elapsed. Stopping motor.");
        stopMotor();
    }

  switch (loopState) {
    case 0:
      loopOriginal();
      
      break;
    case 1:
      loopAlternate1();
              
      break;
    case 2:
      loopAlternate2();
              
      break;
    case 3:
      loopAlternate3();
              
      break;
    case 4:
      loopAlternate4();
              EEPROM.put(0, settings); // Optionally, save the new setting to EEPROM
              EEPROM.commit(); // Ensure data is written to flash
      break;           
  }

}

void loopOriginal() {
  char temperatureFString[10];
  int temperatureFInt = (int)Input; // Convert float to int for display
  itoa(temperatureFInt, temperatureFString, 10); // Convert int to string
  tft.setTextColor(TFT_GREEN, TFT_BLACK);
  tft.drawString("HexTmp:", 0, 0, 4);
  if (settings.heSetpoint < sensors.getTempF(sensor1)) { 
    tft.setTextColor(TFT_BLUE, TFT_BLACK);
  } else { 
    tft.setTextColor(TFT_RED, TFT_BLACK);
  }
  tft.drawString(temperatureFString, 0, 40, 6);
  char temperatureFString2[10];
  int temperatureFInt2 = (int)Input2; // Convert float to int for display
  itoa(temperatureFInt2, temperatureFString2, 10); // Convert int to string
  tft.setTextColor(TFT_GREEN, TFT_BLACK);
  tft.drawString("EngTmp:", 0, 85, 4);
    if (settings.coilSetpoint > sensors.getTempF(sensor2)) { 
    tft.setTextColor(TFT_BLUE, TFT_BLACK);
  } else { 
    tft.setTextColor(TFT_RED, TFT_BLACK);
  }
  tft.drawString(temperatureFString2, 0, 120, 6);

}

void loopAlternate1() {
    bool currentLbuttonState = digitalRead(LbuttonPin);
    bool currentRbuttonState = digitalRead(RbuttonPin);

    // Check for rising edge on Lbutton (button release if active low)
    if (lastRbuttonState == LOW && currentLbuttonState == HIGH) {
        // Lbutton was released, increment the setting
        settings.heSetpoint += 1.0;
    }
    // Check for rising edge on Rbutton (button release if active low)
    if (lastLbuttonState == LOW && currentRbuttonState == HIGH) {
        // Rbutton was released, decrement the setting
        settings.heSetpoint -= 1.0;
    }

    // Update the last button states for the next loop iteration
    lastLbuttonState = currentLbuttonState;
    lastRbuttonState = currentRbuttonState;

    // Display code
    tft.fillScreen(TFT_BLACK);
    tft.setTextColor(TFT_GREEN, TFT_BLACK);
    tft.drawString("HeX Set Temp", 0, 0, 4);
    tft.drawString(String(settings.heSetpoint, 1), 0, 40, 8); // Show current heSetpoint settings);
}

void loopAlternate2() {
    bool currentLbuttonState = digitalRead(LbuttonPin);
    bool currentRbuttonState = digitalRead(RbuttonPin);

    // Check for rising edge on Lbutton (button release if active low)
    if (lastRbuttonState == LOW && currentLbuttonState == HIGH) {
        // Lbutton was released, increment the setting
        settings.heMotorTime += 100.0;
    }
    // Check for rising edge on Rbutton (button release if active low)
    if (lastLbuttonState == LOW && currentRbuttonState == HIGH) {
        // Rbutton was released, decrement the setting
        settings.heMotorTime -= 100.0;
    }

    // Update the last button states for the next loop iteration
    lastLbuttonState = currentLbuttonState;
    lastRbuttonState = currentRbuttonState;

    // Display code
    tft.fillScreen(TFT_BLACK);
    tft.setTextColor(TFT_GREEN, TFT_BLACK);
    tft.drawString("Diverter Valve Runtime", 0, 0, 4);
    tft.drawString(String(settings.heMotorTime, 0), 0, 40, 8); // Show current heSetpoint settings);
}

void loopAlternate3() {
    bool currentLbuttonState = digitalRead(LbuttonPin);
    bool currentRbuttonState = digitalRead(RbuttonPin);

    // Check for rising edge on Lbutton (button release if active low)
    if (lastRbuttonState == LOW && currentLbuttonState == HIGH) {
        // Lbutton was released, increment the setting
        settings.fanSetpoint += 1.0;
    }
    // Check for rising edge on Rbutton (button release if active low)
    if (lastLbuttonState == LOW && currentRbuttonState == HIGH) {
        // Rbutton was released, decrement the setting
        settings.fanSetpoint -= 1.0;
    }

    // Update the last button states for the next loop iteration
    lastLbuttonState = currentLbuttonState;
    lastRbuttonState = currentRbuttonState;

    // Display code
    tft.fillScreen(TFT_BLACK);
    tft.setTextColor(TFT_GREEN, TFT_BLACK);
    tft.drawString("Fan Set Temp", 0, 0, 4);
    tft.drawString(String(settings.fanSetpoint, 1), 0, 40, 8); // Show current heSetpoint settings);
}

void loopAlternate4() {
    bool currentLbuttonState = digitalRead(LbuttonPin);
    bool currentRbuttonState = digitalRead(RbuttonPin);

    // Check for rising edge on Lbutton (button release if active low)
    if (lastRbuttonState == LOW && currentLbuttonState == HIGH) {
        // Lbutton was released, increment the setting
        settings.coilSetpoint += 1.0;
    }
    // Check for rising edge on Rbutton (button release if active low)
    if (lastLbuttonState == LOW && currentRbuttonState == HIGH) {
        // Rbutton was released, decrement the setting
        settings.coilSetpoint -= 1.0;
    }

    // Update the last button states for the next loop iteration
    lastLbuttonState = currentLbuttonState;
    lastRbuttonState = currentRbuttonState;

    // Display code
    tft.setTextColor(TFT_GREEN, TFT_BLACK);
    tft.drawString("E-Stop Set Temp", 0, 0, 4);
    tft.drawString(String(settings.coilSetpoint, 1), 0, 40, 8); // Show current heSetpoint settings);
}

void startMotorCW() {
  digitalWrite(heaterPinCW, HIGH); // Set pinA HIGH to drive the motor in one direction
  digitalWrite(heaterPinCCW, LOW);  // Ensure pinB is LOW
}

void startMotorCCW() {
  digitalWrite(heaterPinCW, LOW); // Set both pins LOW to stop the motor
  digitalWrite(heaterPinCCW, HIGH);
}

void stopMotor() {
    // Stop the motor and update lastMotorState before setting motorState to STOPPED
    digitalWrite(heaterPinCW, LOW); 
    digitalWrite(heaterPinCCW, LOW);

    // Only update lastMotorState if the motor was running (to remember the last direction)
    if (motorState == CLOCKWISE || motorState == COUNTERCLOCKWISE) {
        lastMotorState = motorState;
    }
    motorState = STOPPED; // Finally, set the motorState to STOPPED
}