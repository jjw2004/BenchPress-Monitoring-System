// BarTone - Orientation cube (factory style) with roll, pitch and yaw values
//
// A mix of M5Stack's factory IMU demo and BarTone_Orientation:
//   - The cube is drawn exactly like the factory demo: white wireframe,
//     red/green/blue axis lines, the same 3D viewing angle.
//   - But it moves using the gyro + accelerometer together (a "Mahony filter"),
//     so unlike the factory demo it can also show yaw (turning flat on a table).
//   - The right side shows the live values: roll (X), pitch (Y) and yaw (Z).
//
// How to use:
//   1. Turn it on and put it down flat. It calibrates the gyro for 1 s
//      (it starts again by itself if you move it).
//   2. Pick it up and tilt/turn it. The cube and numbers follow.
//
// Buttons:
//   A (front "M5"): set roll, pitch and yaw to 0 in the current position
//   B (side):       recalibrate the gyro (put it down still for 1 s)
//
// Yaw has no compass to correct it, so it slowly drifts. Press A to re-zero.

#include <M5Unified.h>

// We draw everything into this off-screen image first, then copy it to the
// screen in one go. That stops the screen flickering.
M5Canvas canvas(&M5.Display);

const float DEG2RAD = 0.0174533f;  // multiply degrees by this to get radians
const float RAD2DEG = 57.2958f;    // multiply radians by this to get degrees

// ---- Sensitivity: tweak these ----
const float GYRO_DEADBAND = 1.0f;  // deg/s, slower rotation than this is ignored as noise
const float KP = 0.5f;             // how fast gravity pulls roll/pitch back (lower = calmer)
const float SMOOTHING = 0.2f;      // display smoothing, 0..1 (the factory demo also used 0.2)

// ---- Calibration ----
const uint32_t CAL_US = 1000000;        // calibrate for 1 s
const float CAL_JUMP_ACCEL    = 0.10f;  // g, one reading jumping away from the average = moved
const float CAL_JUMP_GYRO     = 10.0f;  // deg/s
const float CAL_MAX_ACCEL_STD = 0.03f;  // g, overall wobble across the whole second
const float CAL_MAX_GYRO_STD  = 2.0f;   // deg/s

// The sensor is read in a separate task on the ESP32's other CPU core, so it
// never misses readings while the screen is being drawn. This struct is how
// that task passes its results to the drawing code in loop().
struct Shared {
  float q[4];        // orientation relative to the zero position (quaternion)
  bool calibrating;  // true while calibrating
  bool calMoved;     // true if calibration had to restart because the stick moved
  int progress;      // calibration progress, 0-100 %
};
Shared shared = { { 1, 0, 0, 0 }, true, false, 0 };
portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;  // lock so both cores don't touch `shared` at once

// =====================================================================
// Orientation maths
// =====================================================================
// Orientation is stored as a quaternion q[4] = {w, x, y, z}: a compact way to
// describe any 3D rotation without the problems angles have when you flip over.

// Scale q so its length is 1 (rounding errors slowly stretch it otherwise)
void normalize(float q[4]) {
  float n = sqrtf(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);
  for (int i = 0; i < 4; i++) q[i] /= n;
}

// Starting orientation from gravity alone: the rotation that turns the
// measured "down" direction into straight down (+Z = up)
void quatFromAccel(float q[4], float ax, float ay, float az) {
  float n = sqrtf(ax * ax + ay * ay + az * az);
  ax /= n; ay /= n; az /= n;
  if (az < -0.999f) {  // upside down: special case to avoid dividing by zero
    q[0] = 0; q[1] = 1; q[2] = 0; q[3] = 0;
    return;
  }
  q[0] = 1 + az; q[1] = ay; q[2] = -ax; q[3] = 0;
  normalize(q);
}

// Update the orientation using the gyro (Mahony filter).
// gx/gy/gz = rotation speed in rad/s, dt = time since last reading in seconds.
// kp > 0 also nudges it towards what gravity says is down, cancelling gyro drift
// in roll and pitch. (Yaw can't be corrected this way: gravity doesn't change
// when you turn flat on a table.)
void mahonyUpdate(float q[4], float gx, float gy, float gz,
                  float ax, float ay, float az, float kp, float dt) {
  float n = sqrtf(ax * ax + ay * ay + az * az);
  if (kp > 0 && n > 0) {
    ax /= n; ay /= n; az /= n;
    // Where we currently think "up" is, in the stick's own frame
    float vx = 2 * (q[1] * q[3] - q[0] * q[2]);
    float vy = 2 * (q[0] * q[1] + q[2] * q[3]);
    float vz = q[0] * q[0] - q[1] * q[1] - q[2] * q[2] + q[3] * q[3];
    // The difference between that and the measured gravity becomes a small correction
    gx += kp * (ay * vz - az * vy);
    gy += kp * (az * vx - ax * vz);
    gz += kp * (ax * vy - ay * vx);
  }
  // Rotate q by the gyro reading over this time step
  gx *= 0.5f * dt; gy *= 0.5f * dt; gz *= 0.5f * dt;
  float a = q[0], b = q[1], c = q[2], d = q[3];
  q[0] += -b * gx - c * gy - d * gz;
  q[1] +=  a * gx + c * gz - d * gy;
  q[2] +=  a * gy - b * gz + d * gx;
  q[3] +=  a * gz + b * gy - c * gx;
  normalize(q);
}

// out = conj(ref) * q: the rotation from the zero position (ref) to now (q)
void relativeQuat(float out[4], const float ref[4], const float q[4]) {
  float a0 = ref[0], a1 = -ref[1], a2 = -ref[2], a3 = -ref[3];
  out[0] = a0 * q[0] - a1 * q[1] - a2 * q[2] - a3 * q[3];
  out[1] = a0 * q[1] + a1 * q[0] + a2 * q[3] - a3 * q[2];
  out[2] = a0 * q[2] - a1 * q[3] + a2 * q[0] + a3 * q[1];
  out[3] = a0 * q[3] + a1 * q[2] - a2 * q[1] + a3 * q[0];
}

// =====================================================================
// Calibration: average 1 s of readings, start again if the stick moves
// =====================================================================

struct Calibrator {
  float sum[6], sq[6];  // running totals: accel x,y,z then gyro x,y,z
  int count;            // number of readings so far
  uint32_t startUs;     // when this calibration started

  void reset(uint32_t now) {
    memset(sum, 0, sizeof(sum));
    memset(sq, 0, sizeof(sq));
    count = 0;
    startUs = now;
  }
  float mean(int i) { return sum[i] / count; }
  float stdev(int i) {
    float m = mean(i);
    return sqrtf(fmaxf(sq[i] / count - m * m, 0));
  }
  // Add a reading. Returns false if it jumped too far from the average (stick moved).
  bool add(const float s[6]) {
    if (count > 10) {
      for (int i = 0; i < 6; i++) {
        float limit = i < 3 ? CAL_JUMP_ACCEL : CAL_JUMP_GYRO;
        if (fabsf(s[i] - mean(i)) > limit) return false;
      }
    }
    for (int i = 0; i < 6; i++) {
      sum[i] += s[i];
      sq[i] += s[i] * s[i];
    }
    count++;
    return true;
  }
  // Returns false if the readings wobbled too much overall
  bool steady() {
    for (int i = 0; i < 3; i++) if (stdev(i) > CAL_MAX_ACCEL_STD) return false;
    for (int i = 3; i < 6; i++) if (stdev(i) > CAL_MAX_GYRO_STD) return false;
    return true;
  }
};

// =====================================================================
// Sensor task: runs on core 0, reads the IMU ~1000 times a second
// =====================================================================

void imuTask(void*) {
  float q[4] = { 1, 0, 0, 0 };    // current orientation
  float ref[4] = { 1, 0, 0, 0 };  // orientation when A was last pressed (the "zero")
  float bias[3] = { 0, 0, 0 };    // gyro reading when still (subtracted from every reading)
  Calibrator cal;
  bool calibrating = true, calMoved = false;
  uint32_t lastUs = micros();
  cal.reset(lastUs);

  for (;;) {
    M5.update();  // read the buttons

    if (M5.BtnA.wasPressed() && !calibrating) {  // A: zero in the current position
      memcpy(ref, q, sizeof(q));
      M5.Speaker.tone(2000, 50);
    }
    if (M5.BtnB.wasPressed()) {  // B: recalibrate
      calibrating = true;
      calMoved = false;
      cal.reset(micros());
      M5.Speaker.tone(1000, 50);
    }

    // ---- Read the sensor ----
    M5.Imu.update();
    auto d = M5.Imu.getImuData();
    uint32_t now = micros();
    float dt = (now - lastUs) * 1e-6f;  // seconds since the last reading
    lastUs = now;
    if (dt > 0.05f) dt = 0.05f;         // guard against a long pause

    if (calibrating) {
      float s[6] = { d.accel.x, d.accel.y, d.accel.z, d.gyro.x, d.gyro.y, d.gyro.z };
      if (!cal.add(s)) {
        cal.reset(now);  // moved: start again
        calMoved = true;
      } else if (now - cal.startUs >= CAL_US) {
        if (!cal.steady()) {
          cal.reset(now);  // too wobbly overall: start again
          calMoved = true;
        } else {
          // Whatever the gyro read while still is its error: subtract it from now on
          for (int i = 0; i < 3; i++) bias[i] = cal.mean(3 + i);
          // Start the orientation from gravity, and call this position "zero"
          quatFromAccel(q, cal.mean(0), cal.mean(1), cal.mean(2));
          memcpy(ref, q, sizeof(q));
          calibrating = false;
          calMoved = false;
          M5.Speaker.tone(3000, 80);
        }
      }
    } else {
      float ax = d.accel.x, ay = d.accel.y, az = d.accel.z;                          // g
      float gx = d.gyro.x - bias[0], gy = d.gyro.y - bias[1], gz = d.gyro.z - bias[2];  // deg/s

      // Ignore tiny rotations (sensor noise), so the cube doesn't creep
      if (sqrtf(gx * gx + gy * gy + gz * gz) < GYRO_DEADBAND) gx = gy = gz = 0;

      // Only let gravity correct the tilt when it's feeling about 1 g,
      // i.e. not being shaken
      float an = sqrtf(ax * ax + ay * ay + az * az);
      float kp = fabsf(an - 1.0f) < 0.1f ? KP : 0.0f;
      mahonyUpdate(q, gx * DEG2RAD, gy * DEG2RAD, gz * DEG2RAD, ax, ay, az, kp, dt);
    }

    // ---- Hand the results to the drawing code ----
    float rel[4];
    relativeQuat(rel, ref, q);
    int progress = (now - cal.startUs) * 100 / CAL_US;
    portENTER_CRITICAL(&mux);
    memcpy(shared.q, rel, sizeof(rel));
    shared.calibrating = calibrating;
    shared.calMoved = calMoved;
    shared.progress = progress > 100 ? 100 : progress;
    portEXIT_CRITICAL(&mux);

    vTaskDelay(1);  // let other tasks run for a moment
  }
}

// =====================================================================
// Drawing the cube (factory demo style)
// =====================================================================

// Fixed viewing angles from the factory demo, used to draw 3D on the flat screen
const float sin_alpha = sinf(19.47f * DEG2RAD);
const float cos_alpha = cosf(19.47f * DEG2RAD);
const float sin_gamma = sinf(20.7f * DEG2RAD);
const float cos_gamma = cosf(20.7f * DEG2RAD);

// Where the middle of the cube goes on screen (left part, values on the right)
const int CUBE_X = 82;
const int CUBE_Y = 67;

// The 8 corners of a cube from -30 to +30 pixels (same size as the factory demo)
const float CORNERS[8][3] = {
  { -30, -30, -30 }, { 30, -30, -30 }, { 30, 30, -30 }, { -30, 30, -30 },
  { -30, -30,  30 }, { 30, -30,  30 }, { 30, 30,  30 }, { -30, 30,  30 },
};
// Which corners are joined by the 12 edges
const uint8_t EDGES[12][2] = {
  { 0, 1 }, { 1, 2 }, { 2, 3 }, { 3, 0 },  // bottom square
  { 4, 5 }, { 5, 6 }, { 6, 7 }, { 7, 4 },  // top square
  { 0, 4 }, { 1, 5 }, { 2, 6 }, { 3, 7 },  // vertical edges
};

// Rotate a 3D point by the rotation matrix R, then project it onto the
// screen with the factory demo's viewing angle
void toScreen(const float R[3][3], float x, float y, float z, int& sx, int& sy) {
  // Rotate
  float px = R[0][0] * x + R[0][1] * y + R[0][2] * z;
  float py = R[1][0] * x + R[1][1] * y + R[1][2] * z;
  float pz = R[2][0] * x + R[2][1] * y + R[2][2] * z;
  // Project (same formula as the factory point3Dto2D())
  float x2 = (px * cos_gamma) - (py * sin_gamma);
  float y2 = -(px * sin_gamma * sin_alpha) - (py * cos_gamma * sin_alpha) + (pz * cos_alpha);
  // Centre on screen, flip Y so up is up
  sx = CUBE_X + (int)x2;
  sy = CUBE_Y - (int)y2;
}

void drawCube(const float q[4]) {
  // Turn the quaternion into a 3x3 rotation matrix
  float w = q[0], x = q[1], y = q[2], z = q[3];
  float R[3][3] = {
    { 1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y) },
    { 2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x) },
    { 2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y) },
  };

  // White wireframe cube
  int px[8], py[8];
  for (int i = 0; i < 8; i++) toScreen(R, CORNERS[i][0], CORNERS[i][1], CORNERS[i][2], px[i], py[i]);
  for (auto& e : EDGES) canvas.drawLine(px[e[0]], py[e[0]], px[e[1]], py[e[1]], TFT_WHITE);

  // Axis lines from the centre, 60 pixels long like the factory demo:
  // X = red (roll), Y = green (pitch), Z = blue (yaw)
  int cx, cy, ex, ey;
  toScreen(R, 0, 0, 0, cx, cy);
  toScreen(R, 60, 0, 0, ex, ey); canvas.drawLine(cx, cy, ex, ey, TFT_RED);
  toScreen(R, 0, 60, 0, ex, ey); canvas.drawLine(cx, cy, ex, ey, TFT_GREEN);
  toScreen(R, 0, 0, 60, ex, ey); canvas.drawLine(cx, cy, ex, ey, TFT_BLUE);
}

// One value in the right-hand panel: coloured label, then the number
void drawValue(int y, const char* label, float value, uint16_t color) {
  canvas.setTextSize(1);
  canvas.setTextColor(color);
  canvas.setCursor(170, y);
  canvas.print(label);
  canvas.setTextSize(2);
  canvas.setTextColor(TFT_WHITE);
  canvas.setCursor(170, y + 10);
  canvas.printf("%+4.0f", value);
}

// Calibration screen: title, progress bar, and a warning if the stick moved
void drawCalibrating(int progress, bool moved) {
  canvas.setTextColor(TFT_YELLOW);
  canvas.setTextSize(2);
  canvas.setCursor(20, 25);
  canvas.print("Calibrating");
  canvas.setTextSize(1);
  canvas.setTextColor(TFT_LIGHTGREY);
  canvas.setCursor(20, 47);
  canvas.print("Put me down flat and keep still");
  canvas.drawRect(20, 62, 200, 12, TFT_DARKGREY);
  canvas.fillRect(22, 64, 196 * progress / 100, 8, TFT_YELLOW);
  if (moved) {
    canvas.setTextColor(TFT_RED);
    canvas.setCursor(20, 85);
    canvas.print("Moved! Starting again...");
  }
}

// =====================================================================

void setup() {
  auto cfg = M5.config();
  M5.begin(cfg);                   // starts the screen, IMU, buttons, buzzer and Serial (115200)
  M5.Display.setRotation(1);       // landscape, B button at the top
  M5.Imu.setCalibration(0, 0, 0);  // turn off M5Unified's own auto-calibration; we do our own
  canvas.createSprite(M5.Display.width(), M5.Display.height());

  // Start the sensor task on core 0 (loop() runs on core 1)
  xTaskCreatePinnedToCore(imuTask, "imu", 8192, nullptr, 3, nullptr, 0);
}

// loop() only draws the screen, about 50 times a second
void loop() {
  static float qs[4] = { 1, 0, 0, 0 };  // smoothed orientation for display

  // Take a copy of the latest results from the sensor task
  Shared s;
  portENTER_CRITICAL(&mux);
  s = shared;
  portEXIT_CRITICAL(&mux);

  canvas.fillSprite(TFT_BLACK);

  if (s.calibrating) {
    drawCalibrating(s.progress, s.calMoved);
    memcpy(qs, s.q, sizeof(qs));
  } else {
    // Smooth towards the latest orientation so small jitters don't show
    // (like the factory demo's 20% new / 80% old filter)
    float dot = qs[0] * s.q[0] + qs[1] * s.q[1] + qs[2] * s.q[2] + qs[3] * s.q[3];
    float sign = dot < 0 ? -1.0f : 1.0f;  // q and -q are the same rotation; pick the closer one
    for (int i = 0; i < 4; i++) qs[i] += SMOOTHING * (sign * s.q[i] - qs[i]);
    normalize(qs);

    // Quaternion -> roll, pitch, yaw in degrees
    const float* q = qs;
    float roll  = atan2f(2 * (q[0] * q[1] + q[2] * q[3]), 1 - 2 * (q[1] * q[1] + q[2] * q[2])) * RAD2DEG;
    float sp    = constrain(2 * (q[0] * q[2] - q[3] * q[1]), -1.0f, 1.0f);
    float pitch = asinf(sp) * RAD2DEG;
    float yaw   = atan2f(2 * (q[0] * q[3] + q[1] * q[2]), 1 - 2 * (q[2] * q[2] + q[3] * q[3])) * RAD2DEG;

    drawCube(q);

    // Right-hand panel, colours match the axis lines
    canvas.drawFastVLine(162, 0, 135, TFT_DARKGREY);
    drawValue(4,  "ROLL  X", roll,  TFT_RED);
    drawValue(38, "PITCH Y", pitch, TFT_GREEN);
    drawValue(72, "YAW   Z", yaw,   TFT_BLUE);
    canvas.setTextSize(1);
    canvas.setTextColor(TFT_LIGHTGREY);
    canvas.setCursor(170, 112);
    canvas.print("A: zero");
    canvas.setCursor(170, 123);
    canvas.print("B: recal");

    // Values to the Serial Plotter (Tools > Serial Plotter, 115200 baud)
    Serial.printf("%.1f,%.1f,%.1f\n", roll, pitch, yaw);
  }

  canvas.pushSprite(0, 0);  // copy the finished image to the screen
  delay(20);
}
