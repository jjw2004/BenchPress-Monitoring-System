// BarTone 3.1 - IMU stream for the laptop tracker (bar_tracker.py)
//
// This sketch does no tracking itself. It reads the IMU about 1000 times a
// second, averages the readings into 100 samples per second, and sends them
// over USB. bar_tracker.py on the laptop does all the maths (orientation,
// dead reckoning, rep detection) and draws the 3D position and bar path.
//
// How to use:
//   1. Strap the stick to the bar with its long side along the bar.
//   2. Plug it into the laptop and run:  python3 bar_tracker.py
//   3. Leave the bar in the rack for a second while the laptop calibrates.
//   4. Press A (front "M5" button) to tell the laptop a new set is starting.
//
// Each line sent over USB is:
//   D,<time us>,<gyro x>,<gyro y>,<gyro z>,<accel x>,<accel y>,<accel z>
//      gyro in deg/s, accel in g, both in the stick's own axes
//   EV,A    when button A is pressed

#include <M5Unified.h>

// We draw everything into this off-screen image first, then copy it to the
// screen in one go. That stops the screen flickering.
M5Canvas canvas(&M5.Display);

const uint32_t SAMPLE_US = 10000;  // send one averaged sample every 10 ms (100 per second)

// One averaged sample, passed from the sensor task to loop()
struct Packet {
  uint32_t t;       // when it was taken, microseconds since power-on
  float g[3];       // rotation, deg/s
  float a[3];       // acceleration, g
  bool buttonA;     // A was pressed during this sample
};

QueueHandle_t queue;           // samples waiting to be sent over USB
volatile uint32_t dropped = 0; // samples lost because the queue was full

// ---------- Sensor task: runs on core 0, reads the IMU ~1000 times a second ----------
// Buttons are read here too: on the M5StickC Plus the power chip and the IMU
// share the same wires (I2C), so only one core should talk to them.

void imuTask(void*) {
  float sumG[3] = { 0, 0, 0 }, sumA[3] = { 0, 0, 0 };
  int count = 0;
  bool buttonA = false;
  uint32_t next = micros() + SAMPLE_US;

  for (;;) {
    M5.update();  // read the buttons
    if (M5.BtnA.wasPressed()) {
      buttonA = true;
      M5.Speaker.tone(2000, 50);
    }

    M5.Imu.update();
    auto d = M5.Imu.getImuData();
    sumG[0] += d.gyro.x;  sumG[1] += d.gyro.y;  sumG[2] += d.gyro.z;
    sumA[0] += d.accel.x; sumA[1] += d.accel.y; sumA[2] += d.accel.z;
    count++;

    // Every 10 ms, send the average of all the readings since last time.
    // Averaging cuts noise and keeps the total movement exactly right.
    uint32_t now = micros();
    if ((int32_t)(now - next) >= 0) {
      Packet p;
      p.t = now;
      for (int i = 0; i < 3; i++) {
        p.g[i] = sumG[i] / count;
        p.a[i] = sumA[i] / count;
        sumG[i] = sumA[i] = 0;
      }
      p.buttonA = buttonA;
      if (xQueueSend(queue, &p, 0) != pdTRUE) dropped++;
      count = 0;
      buttonA = false;
      next += SAMPLE_US;
      if ((int32_t)(now - next) > 0) next = now + SAMPLE_US;  // fell behind: catch up
    }

    vTaskDelay(1);  // let other tasks run for a moment
  }
}

void setup() {
  auto cfg = M5.config();
  cfg.serial_baudrate = 115200;    // M5Unified only starts USB serial if this is set
  M5.begin(cfg);
  M5.Display.setRotation(1);       // landscape, 240 x 135 pixels
  M5.Imu.setCalibration(0, 0, 0);  // turn off M5Unified's own auto-calibration; the laptop does it

  canvas.createSprite(M5.Display.width(), M5.Display.height());
  queue = xQueueCreate(64, sizeof(Packet));  // room for 0.64 s of samples

  // Start the sensor task on core 0 (loop() runs on core 1)
  xTaskCreatePinnedToCore(imuTask, "imu", 4096, nullptr, 3, nullptr, 0);
}

// loop() sends the samples over USB, and updates the screen 4 times a second
void loop() {
  static uint32_t sentThisSecond = 0, rate = 0;
  static uint32_t lastSecond = millis(), lastDraw = 0;
  static Packet latest = {};

  Packet p;
  while (xQueueReceive(queue, &p, 0) == pdTRUE) {
    Serial.printf("D,%lu,%.2f,%.2f,%.2f,%.4f,%.4f,%.4f\n", (unsigned long)p.t,
                  p.g[0], p.g[1], p.g[2], p.a[0], p.a[1], p.a[2]);
    if (p.buttonA) Serial.println("EV,A");
    sentThisSecond++;
    latest = p;
  }

  uint32_t ms = millis();
  if (ms - lastSecond >= 1000) {
    rate = sentThisSecond;
    sentThisSecond = 0;
    lastSecond = ms;
  }

  if (ms - lastDraw >= 250) {
    lastDraw = ms;
    canvas.fillSprite(TFT_BLACK);
    canvas.setTextColor(TFT_YELLOW);
    canvas.setTextSize(2);
    canvas.setCursor(4, 4);
    canvas.print("BarTone Stream");

    canvas.setTextSize(1);
    canvas.setTextColor(TFT_WHITE);
    canvas.setCursor(4, 32);
    canvas.printf("Sending %lu samples/s", (unsigned long)rate);
    canvas.setCursor(4, 46);
    canvas.setTextColor(dropped ? TFT_RED : TFT_LIGHTGREY);
    canvas.printf("Lost: %lu", (unsigned long)dropped);

    // Live readings, handy for checking which axis is which
    canvas.setTextColor(TFT_CYAN);
    canvas.setCursor(4, 66);
    canvas.printf("accel x %+.2f  y %+.2f  z %+.2f g", latest.a[0], latest.a[1], latest.a[2]);
    canvas.setCursor(4, 80);
    canvas.printf("gyro  x %+5.0f  y %+5.0f  z %+5.0f", latest.g[0], latest.g[1], latest.g[2]);

    canvas.setTextColor(TFT_LIGHTGREY);
    canvas.setCursor(4, 104);
    canvas.print("Run bar_tracker.py on the laptop");
    canvas.setCursor(4, 118);
    canvas.print("A: new set");
    canvas.pushSprite(0, 0);
  }

  delay(2);
}
