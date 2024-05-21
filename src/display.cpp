#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include "display.h"

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

void setupDisplay() {
  // Initialize with the I2C addr 0x3C (for the 128x64)
  if(!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) { 
    Serial.println(F("SSD1306 allocation failed"));
  }

  // Clear the buffer
  display.clearDisplay();
  display.display();
}

// Custom function to display centered text in the upper half of the screen
void displayText(const String &text) {
  display.setTextSize(2);
  display.setTextColor(SSD1306_WHITE);

  int16_t x1, y1;
  uint16_t w, h;
  display.getTextBounds(text, 0, 0, &x1, &y1, &w, &h); // Calculate the bounds of the text

  int16_t x = (SCREEN_WIDTH - w) / 2;
  int16_t y = (SCREEN_HEIGHT / 4 - h / 2); // Position the text in the upper half

  display.setCursor(x, y);
  display.println(text);
}

// Adjusted function to accurately display 13 bars with gaps
void displayBoolArrayAsBars(const bool arr[], int size) {
  int totalGapWidth = size - 1; // Total width taken by gaps
  int totalBarWidth = SCREEN_WIDTH - totalGapWidth; // Total width available for bars
  int barWidth = totalBarWidth / (size-1); // Individual bar width
  int barHeight = SCREEN_HEIGHT / 2; // Bars take up the lower half of the screen
  int startY = SCREEN_HEIGHT / 2; // Starting Y-coordinate for the bars

  for (int i = 0; i < size; i++) {
    if (arr[i]) {
      int x = i * (barWidth + 1); // Position each bar with a gap
      display.fillRect(x, startY, barWidth, barHeight, SSD1306_WHITE);
    }
  }
}