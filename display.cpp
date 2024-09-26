#include "display.h"

void Display::setup() {
  tft.begin();
  tft.fillScreen(TFT_BLACK);
  tft.setRotation(3);
}

void Display::displayTemperature(int temperature) {
  char temperatureFString[10];
  itoa(temperature, temperatureFString, 10); // Convert int to string
  tft.setTextColor(TFT_GREEN, TFT_BLACK);
  tft.drawString("HexTmp:", 0, 0, 4);
  carpetController.setBackgroundColor(tft);
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

void Display::displayText(double value, const char* text) {
    // Display code
    tft.fillScreen(TFT_BLACK);
    tft.setTextColor(TFT_GREEN, TFT_BLACK);
    tft.drawString("HeX Set Temp", 0, 0, 4);
    tft.drawString(String(value, 1), 0, 40, 8);
}

void Display::setTextColor(uint16_t foreground, uint16_t background) {
  tft.setTextColor(foreground, background);
}

void Display::fillScreen(uint32_t color) {
  tft.fillScreen(color);
}
