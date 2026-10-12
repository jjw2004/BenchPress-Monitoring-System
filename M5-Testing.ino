#include <M5Unified.h>

// Colours to cycle through
const uint16_t textColors[] = { WHITE, YELLOW, GREEN, CYAN, MAGENTA, ORANGE, RED };
const uint16_t bgColors[]   = { BLACK, NAVY, DARKGREEN, MAROON, PURPLE, DARKGREY };
const int numText = sizeof(textColors) / sizeof(textColors[0]);
const int numBg   = sizeof(bgColors) / sizeof(bgColors[0]);

int textIdx = 0;
int bgIdx = 0;

void redrawAll() {
  M5.Display.fillScreen(bgColors[bgIdx]);
  M5.Display.setTextColor(textColors[textIdx], bgColors[bgIdx]);
  M5.Display.setTextSize(3);
  M5.Display.setCursor(6, 4);
  M5.Display.print("M5 Testing");
  M5.Display.setTextSize(1);
  M5.Display.setCursor(6, 124);
  M5.Display.print("A: text colour   B: background");
}

void setup() {
  auto cfg = M5.config();
  M5.begin(cfg);
  M5.Display.setRotation(1);  // landscape, 240 x 135
  redrawAll();
}

void loop() {
  M5.update();

  // Big front "M5" button: change text colour
  if (M5.BtnA.wasPressed()) {
    textIdx = (textIdx + 1) % numText;
    M5.Speaker.tone(2000, 50);
    redrawAll();
  }

  // Side button: change background colour
  if (M5.BtnB.wasPressed()) {
    bgIdx = (bgIdx + 1) % numBg;
    M5.Speaker.tone(1000, 50);
    redrawAll();
  }

  // Gyroscope readings in degrees per second
  float gx, gy, gz;
  M5.Imu.getGyro(&gx, &gy, &gz);

  M5.Display.setTextSize(2);
  M5.Display.setTextColor(textColors[textIdx], bgColors[bgIdx]);
  M5.Display.setCursor(6, 40);
  M5.Display.printf("Gyro (deg/s)");
  M5.Display.setCursor(6, 62);
  M5.Display.printf("X %8.1f", gx);
  M5.Display.setCursor(6, 82);
  M5.Display.printf("Y %8.1f", gy);
  M5.Display.setCursor(6, 102);
  M5.Display.printf("Z %8.1f", gz);

  Serial.printf("%.1f,%.1f,%.1f\n", gx, gy, gz);
  delay(50);
}
