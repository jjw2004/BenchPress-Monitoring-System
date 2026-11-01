// BarTone 2.2 - Swing speed and distance meter (dead reckoning)
//
// How to use:
//   1. Turn it on and put it down flat on a table. It calibrates for 1 s.
//   2. Pick it up and press A (front "M5" button) when you're ready.
//   3. Hold still for half a second until it beeps, then swing.
//   4. The screen shows your speed right now, the fastest speed of that go,
//      and how far the stick moved during the latest swing.
//   5. Press B (side button) to reset back to the start screen.
//
// How it works (dead reckoning):
//   - The gyroscope measures how fast the stick is rotating, which lets us
//     keep track of which way it is facing.
//   - The accelerometer measures acceleration, but it always feels gravity too.
//     Since we know which way is down, we subtract gravity and are left with
//     just the acceleration from your swing.
//   - Adding up that acceleration over time gives the velocity (speed).
//   - Adding up the velocity over time gives the position: how far, and in
//     which direction, the stick has moved since the swing started.
//   - Small sensor errors add up over time, so velocity is reset to 0 whenever
//     the stick is held steady, and each swing is only measured for 0.7 s
//     (a real swing speeds up for less than that). Treat readings as good
//     estimates, not exact.

#include <M5Unified.h>

// We draw everything into this off-screen image first, then copy it to the
// screen in one go. That stops the screen flickering.
M5Canvas canvas(&M5.Display);

const float G = 9.80665f;          // 1 g in m/s^2
const float DEG2RAD = 0.0174533f;  // multiply degrees by this to get radians
const float MS_TO_KMH = 3.6f;      // multiply m/s by this to get km/h

// ---- Timing ----
const uint32_t STARTUP_CAL_US = 1000000;  // 1 s calibration on the table at power-on
const uint32_t GET_READY_US   = 500000;   // 0.5 s hold-still after pressing A

// ---- Sensitivity: tweak these ----
const float START_ACCEL    = 0.5f;   // g, a movement must be harder than this to count as a swing
const float ACCEL_DEADBAND = 0.05f;  // g, smaller accelerations are ignored as noise
const float GYRO_DEADBAND  = 1.0f;   // deg/s, slower rotation is ignored as noise
const float STEADY_ACCEL   = 0.10f;  // g, "steady" = total acceleration within this of 1 g...
const float STEADY_GYRO    = 20.0f;  // deg/s, ...and rotating slower than this...
const uint32_t STEADY_US   = 150000; // ...for 0.15 s. Then velocity is reset to 0.

// ---- Limits that keep the readings realistic ----
// Without these, shaking the stick makes the speed climb forever: tiny tilt
// errors let some of gravity sneak in, and 1 g adds ~35 km/h every second.
const uint32_t MAX_SWING_US = 700000;  // a swing or throw speeds up for well under 0.7 s,
                                       // so after that the swing is over and velocity resets
const float MAX_REAL_SPEED  = 45.0f;   // m/s (162 km/h), faster than any human throw. A swing
                                       // that goes past this is a sensor error and is thrown away.
const float MOVING_KP       = 0.3f;    // how gently gravity corrects the tilt while moving

// ---- Calibration limits: if the stick moves more than this, calibration restarts ----
// On the table (strict):
const float CAL_JUMP_ACCEL    = 0.10f;  // g, one reading jumping away from the average
const float CAL_JUMP_GYRO     = 10.0f;  // deg/s
const float CAL_MAX_ACCEL_STD = 0.03f;  // g, overall wobble across the whole second
const float CAL_MAX_GYRO_STD  = 2.0f;   // deg/s
// In your hand after pressing A (looser, because hands are never perfectly still):
const float READY_JUMP_ACCEL  = 0.15f;  // g
const float READY_JUMP_GYRO   = 30.0f;  // deg/s

// What the stick is currently doing
enum Mode {
  MODE_STARTUP,    // calibrating on the table after power-on
  MODE_IDLE,       // waiting for you to press A
  MODE_GET_READY,  // A pressed: hold still while it works out which way is down
  MODE_MEASURING,  // measuring your swing
};

// The sensor is read in a separate task running on the ESP32's other CPU core,
// so it never misses readings while the screen is being drawn. This struct is
// how that task passes its results to the screen-drawing code in loop().
struct Shared {
  Mode mode;
  bool calMoved;   // true if calibration had to restart because the stick moved
  int progress;    // calibration progress, 0-100 %
  bool moving;     // true while a swing is in progress
  bool glitch;     // true for a moment after an impossible reading was thrown away
  float speed;     // speed right now, m/s
  float maxSpeed;  // fastest speed since A was pressed, m/s
  float p[3];      // position relative to where the latest swing started, m (Z = up)
  float distance;  // straight-line distance moved in the latest swing, m
};
Shared shared = { MODE_STARTUP };
portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;  // lock so both cores don't touch `shared` at once

// ---------- Orientation maths ----------
// Orientation is stored as a quaternion q[4] = {w, x, y, z}: a compact way to
// describe any 3D rotation without the problems angles have when you flip over.

// Scale q so its length is 1 (rounding errors slowly stretch it otherwise)
void normalize(float q[4]) {
  float n = sqrtf(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);
  for (int i = 0; i < 4; i++) q[i] /= n;
}

// Work out the starting orientation from gravity alone: the rotation that
// turns the measured "down" direction into straight down (+Z = up).
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
// If kp > 0, it also nudges the orientation towards what gravity says is down,
// which cancels out slow gyro drift. We only do that when the stick is steady.
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

// Turn a vector measured by the stick (its own tilted axes) into room axes
// (Z = straight up), using the current orientation.
void toWorld(const float q[4], const float in[3], float out[3]) {
  float w = q[0], x = q[1], y = q[2], z = q[3];
  out[0] = (1 - 2 * (y * y + z * z)) * in[0] + 2 * (x * y - w * z) * in[1] + 2 * (x * z + w * y) * in[2];
  out[1] = 2 * (x * y + w * z) * in[0] + (1 - 2 * (x * x + z * z)) * in[1] + 2 * (y * z - w * x) * in[2];
  out[2] = 2 * (x * z - w * y) * in[0] + 2 * (y * z + w * x) * in[1] + (1 - 2 * (x * x + y * y)) * in[2];
}

// ---------- Calibration ----------
// Averages the sensor readings over a short time. It also keeps track of how
// much they wobble, so we can tell if the stick moved and start again.

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
  // Average of reading i
  float mean(int i) { return sum[i] / count; }
  // How much reading i wobbles (standard deviation)
  float stdev(int i) {
    float m = mean(i);
    return sqrtf(fmaxf(sq[i] / count - m * m, 0));
  }
  // Add a reading. Returns false if it jumped too far from the average (stick moved).
  bool add(const float s[6], float jumpAccel, float jumpGyro) {
    if (count > 10) {
      for (int i = 0; i < 6; i++) {
        float limit = i < 3 ? jumpAccel : jumpGyro;
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

// ---------- Sensor task: runs on core 0, reads the IMU ~1000 times a second ----------

void imuTask(void*) {
  float q[4] = { 1, 0, 0, 0 };  // current orientation
  float bias[3] = { 0, 0, 0 };  // gyro reading when perfectly still (subtracted from every reading)
  float gScale = 2.0f;          // turns raw accel readings into g (fine-tuned at startup)
  float v[3] = { 0, 0, 0 };     // velocity in room axes, m/s
  float maxSpeed = 0;           // fastest speed so far, m/s
  float p[3] = { 0, 0, 0 };     // position relative to where the swing started, m
  float distance = 0;           // straight-line distance moved in the latest swing, m
  float distBefore = 0;         // distance of the swing before this one (restored if this one was a glitch)
  Mode mode = MODE_STARTUP;
  bool calMoved = false;        // calibration restarted because the stick moved
  bool moving = false;          // a swing is in progress
  uint32_t swingStart = 0;      // when the current swing started
  float maxBefore = 0;          // fastest speed before this swing (restored if it was a glitch)
  uint32_t glitchTime = 0;      // when the last impossible reading was thrown away (0 = never)
  int progress = 0;
  Calibrator cal;
  uint32_t lastUs = micros();
  uint32_t steadySince = lastUs;
  cal.reset(lastUs);

  for (;;) {
    M5.update();  // read the buttons

    // ---- Read the sensor ----
    M5.Imu.update();
    auto d = M5.Imu.getImuData();
    uint32_t now = micros();
    float dt = (now - lastUs) * 1e-6f;  // seconds since the last reading
    lastUs = now;
    if (dt > 0.05f) dt = 0.05f;         // guard against a long pause

    float a[3] = { d.accel.x * gScale, d.accel.y * gScale, d.accel.z * gScale };  // acceleration, g
    float g[3] = { d.gyro.x - bias[0], d.gyro.y - bias[1], d.gyro.z - bias[2] };  // rotation, deg/s
    float s[6] = { a[0], a[1], a[2], g[0], g[1], g[2] };

    // ---- Buttons (ignored during the startup calibration) ----
    if (mode != MODE_STARTUP) {
      if (M5.BtnA.wasPressed()) {  // A: get ready to swing
        mode = MODE_GET_READY;
        calMoved = false;
        glitchTime = 0;
        cal.reset(now);
        M5.Speaker.tone(2000, 50);
      }
      if (M5.BtnB.wasPressed()) {  // B: reset back to the start screen
        mode = MODE_IDLE;
        for (int i = 0; i < 3; i++) { v[i] = 0; p[i] = 0; }
        maxSpeed = 0;
        distance = 0;
        moving = false;
        glitchTime = 0;
        M5.Speaker.tone(1000, 50);
      }
    }

    switch (mode) {

      // ---- Power-on: learn the gyro bias and the accel scale while on the table ----
      case MODE_STARTUP:
        if (!cal.add(s, CAL_JUMP_ACCEL, CAL_JUMP_GYRO)) {
          cal.reset(now);  // moved: start again
          calMoved = true;
        } else if (now - cal.startUs >= STARTUP_CAL_US) {
          if (!cal.steady()) {
            cal.reset(now);  // too wobbly overall: start again
            calMoved = true;
          } else {
            // Whatever the gyro read while still is its error, so subtract it from now on
            for (int i = 0; i < 3; i++) bias[i] = cal.mean(3 + i);
            // Gravity should read exactly 1 g; scale the accel so it does
            float m = sqrtf(cal.mean(0) * cal.mean(0) + cal.mean(1) * cal.mean(1) + cal.mean(2) * cal.mean(2));
            gScale /= m;
            mode = MODE_IDLE;
            calMoved = false;
            M5.Speaker.tone(3000, 80);
          }
        }
        progress = (now - cal.startUs) * 100 / STARTUP_CAL_US;
        break;

      // ---- Waiting for A ----
      case MODE_IDLE:
        break;

      // ---- A pressed: average gravity for 0.5 s to find which way is down ----
      case MODE_GET_READY:
        if (!cal.add(s, READY_JUMP_ACCEL, READY_JUMP_GYRO)) {
          cal.reset(now);  // moved: start again
          calMoved = true;
        } else if (now - cal.startUs >= GET_READY_US) {
          quatFromAccel(q, cal.mean(0), cal.mean(1), cal.mean(2));
          for (int i = 0; i < 3; i++) { v[i] = 0; p[i] = 0; }  // start from 0
          maxSpeed = 0;
          distance = 0;
          moving = false;
          steadySince = now;
          mode = MODE_MEASURING;
          calMoved = false;
          M5.Speaker.tone(3000, 150);  // "go" beep
        }
        progress = (now - cal.startUs) * 100 / GET_READY_US;
        break;

      // ---- Measuring the swing ----
      case MODE_MEASURING: {
        // Ignore tiny rotations (sensor noise)
        float gyroMag = sqrtf(g[0] * g[0] + g[1] * g[1] + g[2] * g[2]);
        if (gyroMag < GYRO_DEADBAND) g[0] = g[1] = g[2] = 0;
        float an = sqrtf(a[0] * a[0] + a[1] * a[1] + a[2] * a[2]);  // total acceleration, g

        // "Steady" = feeling about 1 g (just gravity) and barely rotating, for a short while
        bool steadyNow = fabsf(an - 1.0f) < STEADY_ACCEL && gyroMag < STEADY_GYRO;
        if (!steadyNow) steadySince = now;
        bool steady = (now - steadySince) > STEADY_US;

        // Update which way the stick is facing, using the gyro.
        // Gravity also corrects it, so the tilt doesn't slowly drift:
        //  - held steady: strong correction (it's only feeling gravity)
        //  - moving but feeling close to 1 g: gentle correction. When you shake it,
        //    the shaking averages out and what's left is gravity.
        //  - feeling far from 1 g (mid-swing): no correction, it's mostly the swing
        float kp = 0.0f;
        if (steady) kp = 2.0f;
        else if (fabsf(an - 1.0f) < 0.3f) kp = MOVING_KP;
        mahonyUpdate(q, g[0] * DEG2RAD, g[1] * DEG2RAD, g[2] * DEG2RAD,
                     a[0], a[1], a[2], kp, dt);

        // Convert acceleration to room axes and subtract gravity (1 g straight down)
        float w[3];
        toWorld(q, a, w);
        w[2] -= 1.0f;
        float wMag = sqrtf(w[0] * w[0] + w[1] * w[1] + w[2] * w[2]);  // swing acceleration, g

        if (steady) {
          // Held steady: it can't be moving, so reset velocity to stop errors building up
          moving = false;
          for (int i = 0; i < 3; i++) v[i] = 0;
        } else if (!moving && wMag > START_ACCEL) {
          // A hard enough movement: a new swing has started, from 0
          moving = true;
          swingStart = now;
          maxBefore = maxSpeed;
          distBefore = distance;
          for (int i = 0; i < 3; i++) { v[i] = 0; p[i] = 0; }  // velocity and position start from 0
        }

        // During a swing, add up acceleration x time to get velocity
        if (moving && wMag > ACCEL_DEADBAND) {
          for (int i = 0; i < 3; i++) v[i] += w[i] * G * dt;
        }

        // During a swing, add up velocity x time to get position (dead reckoning).
        // distance = straight-line distance from where the swing started.
        // Note: once the swing ends, position stays frozen at where it got to,
        // so the screen keeps showing that swing's distance.
        if (moving) {
          for (int i = 0; i < 3; i++) p[i] += v[i] * dt;
          distance = sqrtf(p[0] * p[0] + p[1] * p[1] + p[2] * p[2]);
        }

        // Speed is the length of the velocity vector (direction doesn't matter)
        float speed = sqrtf(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);

        if (moving && speed > MAX_REAL_SPEED) {
          // Impossible for a human: it's sensor error, so throw this swing away
          maxSpeed = maxBefore;
          distance = distBefore;
          moving = false;
          for (int i = 0; i < 3; i++) { v[i] = 0; p[i] = 0; }
          glitchTime = now | 1;  // (| 1 so it's never 0, which means "never")
        } else if (moving && now - swingStart > MAX_SWING_US) {
          // The swing is over. Reset so errors can't keep building up.
          // If you're still moving hard, a new swing starts straight away.
          moving = false;
          for (int i = 0; i < 3; i++) v[i] = 0;
        } else if (speed > maxSpeed) {
          maxSpeed = speed;
        }
        break;
      }
    }

    // ---- Hand the results to the screen code ----
    portENTER_CRITICAL(&mux);
    shared.mode = mode;
    shared.calMoved = calMoved;
    shared.progress = progress > 100 ? 100 : progress;
    shared.moving = moving;
    shared.glitch = glitchTime != 0 && now - glitchTime < 1500000;  // show for 1.5 s
    shared.speed = sqrtf(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);
    shared.maxSpeed = maxSpeed;
    memcpy(shared.p, p, sizeof(p));
    shared.distance = distance;
    portEXIT_CRITICAL(&mux);

    vTaskDelay(1);  // let other tasks run for a moment
  }
}

// ---------- Screen ----------

// Calibration screen: a title, a progress bar, and a warning if the stick moved
void drawProgress(const char* title, const char* subtitle, int progress, bool moved) {
  canvas.setTextColor(TFT_YELLOW);
  canvas.setTextSize(2);
  canvas.setCursor(20, 25);
  canvas.print(title);
  canvas.setTextSize(1);
  canvas.setTextColor(TFT_LIGHTGREY);
  canvas.setCursor(20, 47);
  canvas.print(subtitle);
  canvas.drawRect(20, 62, 200, 12, TFT_DARKGREY);
  canvas.fillRect(22, 64, 196 * progress / 100, 8, TFT_YELLOW);
  if (moved) {
    canvas.setTextColor(TFT_RED);
    canvas.setCursor(20, 85);
    canvas.print("Moved! Starting again...");
  }
}

void setup() {
  auto cfg = M5.config();
  M5.begin(cfg);
  M5.Display.setRotation(1);       // landscape, 240 x 135 pixels
  M5.Imu.setCalibration(0, 0, 0);  // turn off M5Unified's own auto-calibration; we do our own

  // The accelerometer's default range is +-8 g, which a fast swing can go past.
  // Switch it to +-16 g. M5Unified still assumes 8 g, so readings come out at
  // half size; gScale (2.0, then fine-tuned at startup) corrects that.
  M5.In_I2C.writeRegister8(0x68, 0x1C, 0x18, 400000);  // MPU6886 ACCEL_CONFIG register = +-16 g

  canvas.createSprite(M5.Display.width(), M5.Display.height());

  // Start the sensor task on core 0 (loop() runs on core 1)
  xTaskCreatePinnedToCore(imuTask, "imu", 8192, nullptr, 3, nullptr, 0);
}

// loop() only draws the screen, about 20 times a second
void loop() {
  // Take a copy of the latest results from the sensor task
  Shared s;
  portENTER_CRITICAL(&mux);
  s = shared;
  portEXIT_CRITICAL(&mux);

  canvas.fillSprite(TFT_BLACK);

  switch (s.mode) {
    case MODE_STARTUP:
      drawProgress("Calibrating", "Put me down flat on a table", s.progress, s.calMoved);
      break;

    case MODE_IDLE:
      canvas.setTextColor(TFT_WHITE);
      canvas.setTextSize(3);
      canvas.setCursor(20, 30);
      canvas.print("Press A");
      canvas.setTextSize(2);
      canvas.setCursor(20, 65);
      canvas.print("when ready");
      break;

    case MODE_GET_READY:
      drawProgress("Hold still...", "Swing after the beep", s.progress, s.calMoved);
      break;

    case MODE_MEASURING:
      // Speed right now
      canvas.setTextColor(TFT_LIGHTGREY);
      canvas.setTextSize(1);
      canvas.setCursor(4, 2);
      canvas.print("SPEED NOW");
      canvas.setTextColor(TFT_WHITE);
      canvas.setTextSize(4);
      canvas.setCursor(4, 12);
      canvas.printf("%.1f", s.speed * MS_TO_KMH);
      canvas.setTextSize(2);
      canvas.print(" km/h");

      // Fastest speed of this go, in km/h and m/s
      canvas.setTextColor(TFT_LIGHTGREY);
      canvas.setTextSize(1);
      canvas.setCursor(4, 50);
      canvas.print("FASTEST");
      canvas.setTextColor(TFT_GREEN);
      canvas.setTextSize(4);
      canvas.setCursor(4, 60);
      canvas.printf("%.1f", s.maxSpeed * MS_TO_KMH);
      canvas.setTextSize(2);
      canvas.print(" km/h");
      canvas.setCursor(4, 98);
      canvas.printf("%.2f m/s", s.maxSpeed);

      // Distance moved in the latest swing (dead-reckoned position)
      canvas.setTextColor(TFT_LIGHTGREY);
      canvas.setTextSize(1);
      canvas.setCursor(128, 102);
      canvas.print("MOVED");
      canvas.setTextColor(TFT_CYAN);
      canvas.setTextSize(2);
      canvas.setCursor(162, 98);
      canvas.printf("%.2fm", s.distance);

      // Status line at the bottom
      canvas.setTextSize(1);
      canvas.setCursor(4, 124);
      if (s.glitch) {
        canvas.setTextColor(TFT_RED);
        canvas.print("Too fast - sensor error, ignored");
      } else if (s.moving) {
        canvas.setTextColor(TFT_YELLOW);
        canvas.print("SWINGING...");
      } else {
        canvas.setTextColor(TFT_LIGHTGREY);
        canvas.print("Swing now!   A: again   B: reset");
      }

      // To the Serial Plotter (Tools > Serial Plotter, 115200 baud):
      // speed and fastest in km/h, then position X, Y, Z in cm. Watching the
      // X/Y/Z lines during a swing shows the path (trajectory) it estimated.
      Serial.printf("speed:%.2f,fastest:%.2f,x_cm:%.1f,y_cm:%.1f,z_cm:%.1f\n",
                    s.speed * MS_TO_KMH, s.maxSpeed * MS_TO_KMH,
                    s.p[0] * 100, s.p[1] * 100, s.p[2] * 100);
      break;
  }

  canvas.pushSprite(0, 0);  // copy the finished image to the screen
  delay(50);
}
