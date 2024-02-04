#pragma once

// OLED display width and height, for a typical SSD1306 these are 128 x 64 pixels.
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 32

// Declaration for an SSD1306 display connected to I2C (SDA, SCL pins)
#define OLED_RESET    -1 // Reset pin # (or -1 if sharing Arduino reset pin)


void setupDisplay();
void displayBoolArrayAsBars(const bool arr[], int size);
void displayText(const String &text);