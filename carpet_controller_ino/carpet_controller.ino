#include <PID_v1.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include "Arduino.h"
#include "TFT_eSPI.h"
#include "pin_config.h"
#include <EEPROM.h>
#include <math.h>

struct DeviceSettings {
  double heSetpoint;
  double fanSetpoint;
  double coilSetpoint;
  double heMotorTime;
};

DeviceSettings settings; // Create an instance of DeviceSettings

bool lastLbuttonState = HIGH; // Assuming active low buttons
bool lastRbuttonState = HIGH;

const int LbuttonPin = 13; // button pins (active-low with pullups)
const int RbuttonPin = 12;
const int CbuttonPin = 11;

volatile int loopState = 0;
volatile int buttonPressed = 0; // Flag to indicate button press
const long debounceDelay = 150; // Debounce delay in milliseconds
unsigned long lastDebounceTime = 0; // Last debounce check time
unsigned long motorStartTime = 0;

enum MotorState {STOPPED, CLOCKWISE, COUNTERCLOCKWISE} motorState = STOPPED;
enum MotorState lastMotorState = STOPPED; // Track last non-stopped state

const int heaterPinCW = 16; // Heater control / motor driver pins
const int heaterPinCCW = 21;
const int fanPin = 2;
const int coilPin = 44;
const int dumpPin = 1;      // NOTE: ensure this GPIO is safe on your board

const double thresholdCW = 0.8; // PID thresholds
const double thresholdCCW = 0.2;

double previousOutput = 0;
double Setpoint = 80;
double heSetpoint, Input, Output, Input2, coilSetpoint, fanSetpoint, heMotorTime;

// PID tuning parameters - adjust to your setup
double Kp = 2.0, Ki = 5.0, Kd = 1.0;
PID myPID(&Input, &Output, &settings.heSetpoint, Kp, Ki, Kd, DIRECT);

TFT_eSPI tft = TFT_eSPI();
const int oneWireBus = 17;
OneWire oneWire(oneWireBus);
DallasTemperature sensors(&oneWire);

// ---------- Clear-screen throttling (every 10 seconds) ----------
const unsigned long SCREEN_CLEAR_INTERVAL_MS = 10000;
unsigned long lastScreenClearMs = 0;
void maybeClearScreen() {
  unsigned long now = millis();
  if (now - lastScreenClearMs >= SCREEN_CLEAR_INTERVAL_MS) {
    tft.fillScreen(TFT_BLACK);
    lastScreenClearMs = now;
  }
}

// ---------- Dump control with hysteresis based on Hex temp ----------
const double HEX_DUMP_HYST = 2.0;  // °F below setpoint to turn back off
bool dumpIsHigh = false;
inline void updateDumpFromHexTemp(double hexF, double setF) {
  if (!dumpIsHigh && hexF >= setF) {
    digitalWrite(dumpPin, HIGH);
    dumpIsHigh = true;
  } else if (dumpIsHigh && hexF <= setF - HEX_DUMP_HYST) {
    digitalWrite(dumpPin, LOW);
    dumpIsHigh = false;
  }
}

void IRAM_ATTR handleButtonPress() {
  buttonPressed = 1; // Set flag to indicate the button was pressed
}

// Addresses of 2 DS18B20s
//uint8_t sensor1[8] = { 0x28, 0xCA, 0x64, 0x43, 0xD4, 0xE1, 0x3C, 0x03 };
//uint8_t sensor2[8] = { 0x28, 0x75, 0x32, 0x43, 0xD4, 0xE1, 0x3C, 0x29 };
uint8_t sensor1[8] = { 0x28, 0xA2, 0x0F, 0xB3, 0x10, 0x24, 0x08, 0x28 };
uint8_t sensor2[8] = { 0x28, 0x32, 0x1F, 0xBF, 0x10, 0x24, 0x07, 0xB6 };

void setup() {
  Serial.begin(115200);
  sensors.begin();

  pinMode(PIN_LCD_BL, OUTPUT);
  pinMode(PIN_POWER_ON, OUTPUT);
  pinMode(fanPin, OUTPUT);
  pinMode(coilPin, OUTPUT);
  pinMode(heaterPinCW, OUTPUT);
  pinMode(heaterPinCCW, OUTPUT);
  pinMode(dumpPin, OUTPUT);           // <-- REQUIRED
  digitalWrite(dumpPin, LOW);         // start low

  pinMode(LbuttonPin, INPUT_PULLUP);  // ensure defined states
  pinMode(RbuttonPin, INPUT_PULLUP);
  pinMode(CbuttonPin, INPUT_PULLUP);

  digitalWrite(PIN_POWER_ON, HIGH);
  digitalWrite(PIN_LCD_BL, HIGH);
  digitalWrite(fanPin, LOW);
  digitalWrite(coilPin, LOW);
  digitalWrite(heaterPinCW, LOW);
  digitalWrite(heaterPinCCW, LOW);

  tft.begin();
  tft.fillScreen(TFT_BLACK);      // initial clear
  tft.setRotation(3);

  EEPROM.begin(sizeof(DeviceSettings));

  attachInterrupt(digitalPinToInterrupt(CbuttonPin), handleButtonPress, RISING);

  EEPROM.get(0, settings);

  // If the EEPROM was never written, initialize with default values
  if (isnan(settings.heSetpoint)) { // Check if settings are uninitialized (NaN)
    settings.heSetpoint = 220; // Default heSetpoint
    settings.fanSetpoint = 217; // Default fanSetpoint
    settings.coilSetpoint = 240; // Default coilSetpoint
    settings.heMotorTime = 3000; // Default heMotorTime
    EEPROM.put(0, settings);
    EEPROM.commit();
  }

  myPID.SetMode(AUTOMATIC);   // Turn PID on
  myPID.SetOutputLimits(0, 1); // PID output 0..1
}

void loop() {
  if (buttonPressed) {
    noInterrupts();
    buttonPressed = 0;
    loopState = (loopState + 1) % 5; // Cycle through 0..4
    interrupts();
    Serial.print("Loop State Changed to: ");
    Serial.println(loopState);
  }

  sensors.requestTemperatures();
  Input  = sensors.getTempF(sensor1); // Hex temp
  Input2 = sensors.getTempF(sensor2); // Engine temp

  myPID.Compute();

  // Control fans/coil from Input2 thresholds
  digitalWrite(fanPin,  (Input2 >= settings.fanSetpoint)  ? HIGH : LOW);
  digitalWrite(coilPin, (Input2 >= settings.coilSetpoint) ? HIGH : LOW);

  // Control dump pin from Hex temp vs heSetpoint (with hysteresis)
  updateDumpFromHexTemp(Input, settings.heSetpoint);

  // Motor control based on PID Output
  if (Output >= thresholdCW && motorState == STOPPED && lastMotorState != CLOCKWISE) {
    Serial.println("PID Output high, starting motor CW.");
    startMotorCW();
    motorState = CLOCKWISE;
    motorStartTime = millis();
  } else if (Output <= thresholdCCW && motorState == STOPPED && lastMotorState != COUNTERCLOCKWISE) {
    Serial.println("PID Output low, starting motor CCW.");
    startMotorCCW();
    motorState = COUNTERCLOCKWISE;
    motorStartTime = millis();
  }

  // Stop motor after runtime
  if ((motorState == CLOCKWISE || motorState == COUNTERCLOCKWISE) &&
      (millis() - motorStartTime > settings.heMotorTime)) {
    Serial.println("Motor run time elapsed. Stopping motor.");
    stopMotor();
  }

  // UI pages
  switch (loopState) {
    case 0: loopOriginal();   break;
    case 1: loopAlternate1(); break;
    case 2: loopAlternate2(); break;
    case 3: loopAlternate3(); break;
    case 4:
      loopAlternate4();
      EEPROM.put(0, settings);
      EEPROM.commit();
      break;
  }
}

void loopOriginal() {
  maybeClearScreen();

  char temperatureFString[10];
  int temperatureFInt = (int)Input; // Hex temp
  itoa(temperatureFInt, temperatureFString, 10);

  tft.setTextColor(TFT_GREEN, TFT_BLACK);
  tft.drawString("HexTmp:", 0, 30, 4);
  if (settings.heSetpoint < Input) { 
    tft.setTextColor(TFT_BLUE, TFT_BLACK);
  } else { 
    tft.setTextColor(TFT_RED, TFT_BLACK);
  }
  tft.drawString(temperatureFString, 0, 60, 6);

  char temperatureFString2[10];
  int temperatureFInt2 = (int)Input2; // Engine temp
  itoa(temperatureFInt2, temperatureFString2, 10);

  tft.setTextColor(TFT_GREEN, TFT_BLACK);
  tft.drawString("EngTmp:", 0, 100, 4);
  if (settings.coilSetpoint > Input2) { 
    tft.setTextColor(TFT_BLUE, TFT_BLACK);
  } else { 
    tft.setTextColor(TFT_RED, TFT_BLACK);
  }
  tft.drawString(temperatureFString2, 0, 130, 6);
}

void loopAlternate1() {
  bool currentLbuttonState = digitalRead(LbuttonPin);
  bool currentRbuttonState = digitalRead(RbuttonPin);

  if (lastRbuttonState == LOW && currentLbuttonState == HIGH) {
    settings.heSetpoint += 1.0;
  }
  if (lastLbuttonState == LOW && currentRbuttonState == HIGH) {
    settings.heSetpoint -= 1.0;
  }

  lastLbuttonState = currentLbuttonState;
  lastRbuttonState = currentRbuttonState;

  maybeClearScreen();
  tft.setTextColor(TFT_GREEN, TFT_BLACK);
  tft.drawString("HeX Set Temp", 0, 50, 4);
  tft.drawString(String(settings.heSetpoint, 1), 0, 90, 8);
}

void loopAlternate2() {
  bool currentLbuttonState = digitalRead(LbuttonPin);
  bool currentRbuttonState = digitalRead(RbuttonPin);

  if (lastRbuttonState == LOW && currentLbuttonState == HIGH) {
    settings.heMotorTime += 100.0;
  }
  if (lastLbuttonState == LOW && currentRbuttonState == HIGH) {
    settings.heMotorTime -= 100.0;
  }

  lastLbuttonState = currentLbuttonState;
  lastRbuttonState = currentRbuttonState;

  maybeClearScreen();
  tft.setTextColor(TFT_GREEN, TFT_BLACK);
  tft.drawString("Diverter Valve Runtime", 0, 50, 4);
  tft.drawString(String(settings.heMotorTime, 0), 0, 90, 8);
}

void loopAlternate3() {
  bool currentLbuttonState = digitalRead(LbuttonPin);
  bool currentRbuttonState = digitalRead(RbuttonPin);

  if (lastRbuttonState == LOW && currentLbuttonState == HIGH) {
    settings.fanSetpoint += 1.0;
  }
  if (lastLbuttonState == LOW && currentRbuttonState == HIGH) {
    settings.fanSetpoint -= 1.0;
  }

  lastLbuttonState = currentLbuttonState;
  lastRbuttonState = currentRbuttonState;

  maybeClearScreen();
  tft.setTextColor(TFT_GREEN, TFT_BLACK);
  tft.drawString("Fan Set Temp", 0, 50, 4);
  tft.drawString(String(settings.fanSetpoint, 1), 0, 90, 8);
}

void loopAlternate4() {
  bool currentLbuttonState = digitalRead(LbuttonPin);
  bool currentRbuttonState = digitalRead(RbuttonPin);

  if (lastRbuttonState == LOW && currentLbuttonState == HIGH) {
    settings.coilSetpoint += 1.0;
  }
  if (lastLbuttonState == LOW && currentRbuttonState == HIGH) {
    settings.coilSetpoint -= 1.0;
  }

  lastLbuttonState = currentLbuttonState;
  lastRbuttonState = currentRbuttonState;

  maybeClearScreen();
  tft.setTextColor(TFT_GREEN, TFT_BLACK);
  tft.drawString("E-Stop Set Temp", 0, 50, 4);
  tft.drawString(String(settings.coilSetpoint, 1), 0, 90, 8);
}

void startMotorCW() {
  digitalWrite(heaterPinCW, HIGH); // Drive motor CW
  digitalWrite(heaterPinCCW, LOW);
}

void startMotorCCW() {
  digitalWrite(heaterPinCW, LOW);
  digitalWrite(heaterPinCCW, HIGH);
}

void stopMotor() {
  digitalWrite(heaterPinCW, LOW); 
  digitalWrite(heaterPinCCW, LOW);

  if (motorState == CLOCKWISE || motorState == COUNTERCLOCKWISE) {
    lastMotorState = motorState;
  }
  motorState = STOPPED;
}
