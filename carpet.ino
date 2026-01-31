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

DeviceSettings settings;

bool lastLbuttonState = HIGH;
bool lastRbuttonState = HIGH;

const int LbuttonPin = 13;
const int RbuttonPin = 12;
const int CbuttonPin = 11;

volatile int loopState = 0;
volatile int buttonPressed = 0;

unsigned long motorStartTime = 0;

// ---------------- Motor control ----------------
enum MotorState { STOPPED, CLOCKWISE, COUNTERCLOCKWISE };
MotorState motorState = STOPPED;

// Track valve position so we only move when changing state.
// This inherently prevents "same direction twice".
enum ValvePos { VALVE_UNKNOWN, VALVE_HEAT_CW, VALVE_COOL_CCW };
ValvePos valvePos = VALVE_UNKNOWN;

// Cooldown between moves (prevents immediate retrigger)
const unsigned long MOVE_COOLDOWN_MS = 1500;
unsigned long lastMoveEndMs = 0;

// Motor driver pins
const int heaterPinCW  = 16;
const int heaterPinCCW = 21;

const int fanPin  = 2;
const int coilPin = 44;
const int dumpPin = 1;  // unchanged

// ---------------- Hex temp hysteresis (your request) ----------------
const double HEX_TEMP_DEADBAND_F = 2.0;  // °F

// ---------------- PID (kept, but NOT used to command the 2-pos valve) ----------------
double Input, Output, Input2;
double Kp = 2.0, Ki = 5.0, Kd = 1.0;
PID myPID(&Input, &Output, &settings.heSetpoint, Kp, Ki, Kd, DIRECT);

// ---------------- Display / sensors ----------------
TFT_eSPI tft = TFT_eSPI();
const int oneWireBus = 17;
OneWire oneWire(oneWireBus);
DallasTemperature sensors(&oneWire);

// DS18B20 addresses
uint8_t sensor1[8] = { 0x28, 0xA2, 0x0F, 0xB3, 0x10, 0x24, 0x08, 0x28 };
uint8_t sensor2[8] = { 0x28, 0x32, 0x1F, 0xBF, 0x10, 0x24, 0x07, 0xB6 };

// ---------- Clear-screen throttling ----------
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
const double HEX_DUMP_HYST = 2.0;
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
  buttonPressed = 1;
}

// ---------------- Motor helpers ----------------
void startMotorCW() {
  digitalWrite(heaterPinCW, HIGH);
  digitalWrite(heaterPinCCW, LOW);
}

void startMotorCCW() {
  digitalWrite(heaterPinCW, LOW);
  digitalWrite(heaterPinCCW, HIGH);
}

void stopMotor() {
  digitalWrite(heaterPinCW, LOW);
  digitalWrite(heaterPinCCW, LOW);
  motorState = STOPPED;
}

void setup() {
  Serial.begin(115200);
  sensors.begin();

  pinMode(PIN_LCD_BL, OUTPUT);
  pinMode(PIN_POWER_ON, OUTPUT);

  pinMode(fanPin, OUTPUT);
  pinMode(coilPin, OUTPUT);

  pinMode(heaterPinCW, OUTPUT);
  pinMode(heaterPinCCW, OUTPUT);

  pinMode(dumpPin, OUTPUT);
  digitalWrite(dumpPin, LOW);

  pinMode(LbuttonPin, INPUT_PULLUP);
  pinMode(RbuttonPin, INPUT_PULLUP);
  pinMode(CbuttonPin, INPUT_PULLUP);

  digitalWrite(PIN_POWER_ON, HIGH);
  digitalWrite(PIN_LCD_BL, HIGH);

  stopMotor();

  tft.begin();
  tft.fillScreen(TFT_BLACK);
  tft.setRotation(3);

  EEPROM.begin(sizeof(DeviceSettings));
  EEPROM.get(0, settings);

  if (isnan(settings.heSetpoint)) {
    settings.heSetpoint   = 220;
    settings.fanSetpoint  = 217;
    settings.coilSetpoint = 240;
    settings.heMotorTime  = 3000; // 90° move time
    EEPROM.put(0, settings);
    EEPROM.commit();
  }

  attachInterrupt(digitalPinToInterrupt(CbuttonPin), handleButtonPress, RISING);

  myPID.SetMode(AUTOMATIC);
  myPID.SetOutputLimits(0, 1);

  lastMoveEndMs = millis();
}

void loop() {
  if (buttonPressed) {
    noInterrupts();
    buttonPressed = 0;
    loopState = (loopState + 1) % 5;
    interrupts();
    Serial.print("Loop State Changed to: ");
    Serial.println(loopState);
  }

  sensors.requestTemperatures();
  Input  = sensors.getTempF(sensor1); // Hex temp
  Input2 = sensors.getTempF(sensor2); // Engine temp

  // Still compute PID if you want to watch Output for tuning/debug
  myPID.Compute();

  // Fan/coil logic unchanged
  digitalWrite(fanPin,  (Input2 >= settings.fanSetpoint)  ? HIGH : LOW);
  digitalWrite(coilPin, (Input2 >= settings.coilSetpoint) ? HIGH : LOW);

  // Dump logic unchanged
  updateDumpFromHexTemp(Input, settings.heSetpoint);

  const unsigned long now = millis();

  // Stop motor after runtime
  if (motorState != STOPPED &&
      (now - motorStartTime) > (unsigned long)settings.heMotorTime) {
    Serial.println("Motor run time elapsed. Stopping motor.");
    stopMotor();
    lastMoveEndMs = now;
  }

  // ---------------- INVERTED heating/cooling logic ----------------
  // INVERTED per request:
  //  - Above band -> HEAT (CW)
  //  - Below band -> COOL (CCW)
  bool wantHeat = (Input >= settings.heSetpoint + HEX_TEMP_DEADBAND_F);
  bool wantCool = (Input <= settings.heSetpoint - HEX_TEMP_DEADBAND_F);

  // (Optional) debug; comment out once verified
  Serial.printf("Hex=%.1f Set=%.1f wantHeat=%d wantCool=%d valvePos=%d motor=%d\n",
                Input, settings.heSetpoint, wantHeat, wantCool, valvePos, motorState);

  // Start move only if stopped and cooldown elapsed
  if (motorState == STOPPED && (now - lastMoveEndMs) >= MOVE_COOLDOWN_MS) {
    if (wantCool && valvePos != VALVE_COOL_CCW) {
      // BELOW band => COOL => CCW
      Serial.println("Hex BELOW band -> CCW (COOL) [INVERTED]");
      startMotorCCW();
      motorState = COUNTERCLOCKWISE;
      motorStartTime = now;
      valvePos = VALVE_COOL_CCW;
    } else if (wantHeat && valvePos != VALVE_HEAT_CW) {
      // ABOVE band => HEAT => CW
      Serial.println("Hex ABOVE band -> CW (HEAT) [INVERTED]");
      startMotorCW();
      motorState = CLOCKWISE;
      motorStartTime = now;
      valvePos = VALVE_HEAT_CW;
    }
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

// ---------------- UI ----------------
void loopOriginal() {
  maybeClearScreen();

  // Hex temp
  tft.setTextColor(TFT_GREEN, TFT_BLACK);
  tft.drawString("HexTmp:", 0, 30, 4);

  // Original red/blue meaning (over/under setpoint)
  if (Input >= settings.heSetpoint) {
    tft.setTextColor(TFT_BLUE, TFT_BLACK);   // OVER setpoint
  } else {
    tft.setTextColor(TFT_RED, TFT_BLACK);    // UNDER setpoint
  }
  tft.drawString(String((int)Input), 0, 60, 6);

  // Engine temp
  tft.setTextColor(TFT_GREEN, TFT_BLACK);
  tft.drawString("EngTmp:", 0, 100, 4);

  if (Input2 >= settings.coilSetpoint) {
    tft.setTextColor(TFT_BLUE, TFT_BLACK);   // OVER setpoint
  } else {
    tft.setTextColor(TFT_RED, TFT_BLACK);    // UNDER setpoint
  }
  tft.drawString(String((int)Input2), 0, 130, 6);
}

void loopAlternate1() {
  bool currentLbuttonState = digitalRead(LbuttonPin);
  bool currentRbuttonState = digitalRead(RbuttonPin);

  if (lastRbuttonState == LOW && currentLbuttonState == HIGH) settings.heSetpoint += 1.0;
  if (lastLbuttonState == LOW && currentRbuttonState == HIGH) settings.heSetpoint -= 1.0;

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

  if (lastRbuttonState == LOW && currentLbuttonState == HIGH) settings.heMotorTime += 100.0;
  if (lastLbuttonState == LOW && currentRbuttonState == HIGH) settings.heMotorTime -= 100.0;

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

  if (lastRbuttonState == LOW && currentLbuttonState == HIGH) settings.fanSetpoint += 1.0;
  if (lastLbuttonState == LOW && currentRbuttonState == HIGH) settings.fanSetpoint -= 1.0;

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

  if (lastRbuttonState == LOW && currentLbuttonState == HIGH) settings.coilSetpoint += 1.0;
  if (lastLbuttonState == LOW && currentRbuttonState == HIGH) settings.coilSetpoint -= 1.0;

  lastLbuttonState = currentLbuttonState;
  lastRbuttonState = currentRbuttonState;

  maybeClearScreen();
  tft.setTextColor(TFT_GREEN, TFT_BLACK);
  tft.drawString("E-Stop Set Temp", 0, 50, 4);
  tft.drawString(String(settings.coilSetpoint, 1), 0, 90, 8);
}
