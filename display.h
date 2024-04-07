#pragma once
#include "TFT_eSPI.h"

class Display {
  public:
    void setup();

    void displayTemperature();

    void setTextColor(uint16_t foreground, uint16_t background);

    void displayText();

    void fillScreen(uint32_t color);
  private:
    TFT_eSPI tft = TFT_eSPI();
};

