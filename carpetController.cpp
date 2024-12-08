#include "carpetController.h"

CarpetController::CarpetController()
  : _pid(&_input, &_output, &_settings.heSetpoint, Kp, Ki, Kd, DIRECT)
  , _oneWire(oneWireBus)
  , _sensors(_oneWire) {
}

void CarpetController::Setup() {
  _sensors.begin();

  pinMode(PIN_POWER_ON, OUTPUT);
  digitalWrite(PIN_POWER_ON, HIGH);

  pinMode(fanPin, OUTPUT);
  digitalWrite(fanPin, LOW);

  pinMode(coilPin, OUTPUT);
  digitalWrite(coilPin, LOW);

  pinMode(heaterPinCW, OUTPUT);
  digitalWrite(heaterPinCW,LOW);

  pinMode(heaterPinCCW, OUTPUT);
  digitalWrite(heaterPinCCW, LOW);

  readSettings();

  pinMode(CbuttonPin, INPUT_PULLUP); // Initialize the button pin as input with pull-up resistor
  _pid.SetMode(AUTOMATIC); // Turn PID on
  _pid.SetOutputLimits(0, 1); // Set PID output limits to match digital on/off state
}

void CarpetController::readSettings() {
  EEPROM.begin(sizeof(DeviceSettings));

  EEPROM.get(0, _settings);
  // If the EEPROM was never written, initialize with default values
  static_assert(std::numeric_limits<T>::has_quiet_NaN, "Platform doesn't have quiet NaN");
  if (std::isnan(_settings.heSetpoint)) { // Check if settings are uninitialized (NaN)
    _settings.heSetpoint = 220; // Default heSetpoint
    _settings.fanSetpoint = 217; // Default fanSetpoint
    _settings.coilSetpoint = 240; // Default coilSetpoint
    _settings.heMotorTime = 3000; // Default heMotorTime
    saveSettings();
  }
}

void CarpetController::handleSensors() {
  _sensors.requestTemperatures();
  _input = sensors.getTempF(sensor1); // Update current temperature from sensor
  _input2 = sensors.getTempF(sensor2);

  myPID.Compute();

  if (_input2 >= settings.fanSetpoint) {
    debugPrintln("fan on");
    digitalWrite(1, HIGH);
  } else {
    debugPrintln("fan off");
    digitalWrite(1, LOW);
  }

  if (_input2 >= settings.coilSetpoint) {
    debugPrintln("engine off");
    digitalWrite(coilPin, LOW);
  } else if (Input2 < settings.coilSetpoint) {
    debugPrintln("engine on");
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
}

void CarpetController::setTextColor(Display& tft) {
  if (_settings.heSetpoint < _sensors.getTempF(sensor1)) {
    tft.setTextColor(TFT_BLUE, TFT_BLACK);
  } else {
    tft.setTextColor(TFT_RED, TFT_BLACK);
  }
}

void CarpetController::saveSettings() {
    // Save the initialized settings back to EEPROM
    EEPROM.put(0, _settings);
    EEPROM.commit(); // Ensure data is written to flash
}

void CarpetController::adjustSetpoint(ButtonState current, double increment, double& value) {
    // Check for rising edge on Lbutton (button release if active low)
    if (_lastRbuttonState == LOW && current.left == HIGH) {
        // Lbutton was released, increment the setting
        value += increment;
    }
    // Check for rising edge on Rbutton (button release if active low)
    if (_lastLbuttonState == LOW && current.right == HIGH) {
        // Rbutton was released, decrement the setting
        value -= increment;
    }
    _state = current;
}

void CarpetController::adjustHeSetpoint(ButtonState current) {
  adjustSetpoint(current, 1.0, settings.heSetpoint);
}

void CarpetController::adjustMotorTime(ButtonState current) {
  adjustSetpoint(current, 100.0, settings.heMotorTime);
}

void CarpetController::adjustFanSetpoint(ButtonState current) {
  adjustSetpoint(current, 1.0, settings.fanSetpoint);
}

void CarpetController::adjustCoilSetpoint(ButtonState current) {
  adjustSetpoint(current, 1.0, settings.coilSetpoint);
}

void CarpetController::startMotorCW() {
  digitalWrite(heaterPinCW, HIGH); // Set pinA HIGH to drive the motor in one direction
  digitalWrite(heaterPinCCW, LOW);  // Ensure pinB is LOW
}

void CarpetController::startMotorCCW() {
  digitalWrite(heaterPinCW, LOW); // Set both pins LOW to stop the motor
  digitalWrite(heaterPinCCW, HIGH);
}

void CarpetController::stopMotor() {
    // Stop the motor and update lastMotorState before setting motorState to STOPPED
    digitalWrite(heaterPinCW, LOW);
    digitalWrite(heaterPinCCW, LOW);

    // Only update lastMotorState if the motor was running (to remember the last direction)
    if (motorState == CLOCKWISE || motorState == COUNTERCLOCKWISE) {
        lastMotorState = motorState;
    }
    motorState = STOPPED; // Finally, set the motorState to STOPPED
}
