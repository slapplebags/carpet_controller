#pragma once
#include <PID_v1.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include "Arduino.h"
#include "pin_config.h"
#include <EEPROM.h>
#include "display.h"
#include "carpetController.h"

const int LbuttonPin = 2; // The pin the button is connected to
const int RbuttonPin = 11; // The pin the button is connected to
const int CbuttonPin = 10; // The pin the button is connected to

enum class LoopState : int {
  eDisplayTemperature = 0,
  eAdjustHeSetPoint = 1,
  eAdjustMotorTime = 2,
  eAdjustFanSetpoint = 3,
  eAdjustCoilSetpoint = 4,
};

volatile LoopState loopState = LoopState::eDisplayTemperature;
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

// PID tuning parameters - These need to be adjusted based on your setup and requirements
double Kp = 2.0, Ki = 5.0, Kd = 1.0;
const int oneWireBus = 16;

// Addresses of 2 DS18B20s
const uint8_t sensor1[8] = { 0x28, 0xB9, 0x98, 0x43, 0xD4, 0xE1, 0x3C, 0x45 };
const uint8_t sensor2[8] = { 0x28, 0xF5, 0x75, 0x43, 0xD4, 0xE1, 0x3C, 0x7B };
CarpetController carpetController;
Display display;
