#include "globals.h"

void IRAM_ATTR handleButtonPress() {
  unsigned long currentTime = millis();
  buttonPressed = 1; // Set flag to indicate the button was pressed
}

ButtonState readButtons() {
    return ButtonState{.left = digitalRead(LbuttonPin), .right = digitalRead(RbuttonPin)};
}

void setup() {
  Serial.begin(115200);

  carpetController.setup();

  display.setup();

  attachInterrupt(digitalPinToInterrupt(CbuttonPin), handleButtonPress, RISING); // Attach interrupt
}

void loop() {

  if (buttonPressed) {
    noInterrupts(); // Disable interrupts to safely check shared variables
    buttonPressed = 0; // Reset the flag
    loopState = (loopState + 1) % 5; // Cycle through 0, 1, 2
    interrupts(); // Re-enable interrupts
    Serial.print("Loop State Changed to: ");
    Serial.println(loopState);
    display.fillScreen(TFT_BLACK);
  }

  carpetController.handleSensors();

  switch (loopState) {
    case LoopState::eDisplayTemperature:
      displayTemperature();

      break;
    case LoopState::eAdjustHeSetPoint:
      adjustHeSetpoint();

      break;
    case LoopState::eAdjustMotorTime:
      adjustMotorTime();

      break;
    case LoopState::eAdjustFanSetpoint:
      adjustFanSetpoint();

      break;
    case LoopState::eAdjustCoilSetpoint:
      adjustCoilSetpoint();
      carpetController.saveSettings();
      break;
  }
}

void adjustHeSetpoint() {
    carpetController.adjustHeSetpoint(readButtons());
    display.displayText(carpetController.getHeSetpoint());
}

void adjustMotorTime() {
    carpetController.adjustMotorTime(readButtons());
    display.displayText(carpetController.getMotorTime());
}

void adjustFanSetpoint() {
    carpetController.adjustFanSetpoint(readButtons());
    display.displayText(carpetController.getFanSetpoint());
}

void adjustCoilSetpoint() {
    carpetController.adjustCoilSetpoint(readButtons());
    display.displayText(carpetController.getCoilSetpoint());
}

