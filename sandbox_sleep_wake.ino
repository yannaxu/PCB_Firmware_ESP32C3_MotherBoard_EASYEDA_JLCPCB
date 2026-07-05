#include <Wire.h>
#include <U8g2lib.h>
#include <WiFi.h>
#include <esp_sleep.h>
#include <esp_wifi.h>
#include <esp_bt.h>
#include <math.h>

// Lock R0 normal orientation (SSD1306 pins on top)
U8G2_SSD1306_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, /* reset=*/ U8X8_PIN_NONE);

const int MPU_ADDR = 0x68;
const int FLASH_BTN_PIN = 1;       // physical button on GPIO1 (open-drain, active-low)

bool last_btn_state = HIGH;

// Sensor zero-offset calibration variables
float errorAccRoll = 0, errorAccPitch = 0;
float errorGyroX = 0, errorGyroY = 0;

float roll = 0, pitch = 0;
unsigned long lastTime = 0;
const float alpha = 0.985;          // complementary filter weight (trust gyro 98.5%)

// Platform border filling the 128x64 screen
const int BORDER_X1 = 2;
const int BORDER_Y1 = 2;
const int BORDER_X2 = 126;
const int BORDER_Y2 = 62;

// Boxes
const int NUM_BOXES = 30;
const int BOX_SIZE = 4;

struct PhysicsBox {
  float x;
  float y;
  float vx;
  float vy;
};

PhysicsBox boxes[NUM_BOXES];

// Physics engine parameters
const float ACCEL_SCALE = 0.02;
const float FRICTION = 0.88;

// --- Software wake-on-motion settings ---
// During sleep the ESP32 wakes briefly every SLEEP_POLL_US to read the accelerometer.
// If the acceleration vector differs from the resting baseline by more than
// WAKE_MOTION_THRESH (in g), it counts as motion and the device wakes fully.
const uint64_t SLEEP_POLL_US      = 150000;  // 150 ms light-sleep interval
const float    WAKE_MOTION_THRESH = 0.08;    // g; lower = more sensitive

// Function declarations
void calibrateMPU6050();
void enterSleep();
void writeReg(uint8_t reg, uint8_t val);
void readAccelG(float &ax, float &ay, float &az);

void setup() {
  Serial.begin(115200);

  // Aggressive power saving: radios off
  WiFi.mode(WIFI_OFF);
  esp_wifi_stop();
  esp_bt_controller_disable();
  esp_bt_controller_deinit();

  pinMode(FLASH_BTN_PIN, INPUT_PULLUP);

  Wire.begin(8, 9);
  Wire.setTimeOut(50);

  u8g2.begin();

  // Wake the MPU6050, all axes on (gyro is needed for the sandbox filter)
  writeReg(0x6B, 0x00);   // PWR_MGMT_1: wake up
  writeReg(0x6C, 0x00);   // PWR_MGMT_2: all axes enabled
  delay(50);

  // Flat-table calibration
  calibrateMPU6050();

  // Clear leftover memory
  memset(boxes, 0, sizeof(boxes));

  // Initial box layout
  int startX = 40;
  int startY = 20;
  for (int i = 0; i < NUM_BOXES; i++) {
    boxes[i].x = startX + (i % 6) * 8;
    boxes[i].y = startY + (i / 6) * 8;
  }

  lastTime = micros();
}

void loop() {
  // --- Button -> sleep (edge: released HIGH -> pressed LOW) ---
  int btn_reading = digitalRead(FLASH_BTN_PIN);
  if (btn_reading == LOW && last_btn_state == HIGH) {
    enterSleep();             // blocks here until motion wakes it, then resumes
    last_btn_state = HIGH;    // reset after returning from sleep
    return;                   // restart loop cleanly
  }
  last_btn_state = btn_reading;

  // --- Read MPU6050 raw data (normal sandbox operation) ---
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B);
  if (Wire.endTransmission(false) != 0) return;
  Wire.requestFrom((uint8_t)MPU_ADDR, (size_t)14, (bool)true);

  if (Wire.available() >= 14) {
    int16_t rawX = (Wire.read() << 8) | Wire.read();
    int16_t rawY = (Wire.read() << 8) | Wire.read();
    int16_t rawZ = (Wire.read() << 8) | Wire.read();
    Wire.read(); Wire.read(); // skip temperature
    int16_t rawGX = (Wire.read() << 8) | Wire.read();
    int16_t rawGY = (Wire.read() << 8) | Wire.read();

    float accX = rawX / 16384.0;
    float accY = rawY / 16384.0;
    float accZ = rawZ / 16384.0;
    float gyroX = (rawGX / 131.0) - errorGyroX;
    float gyroY = (rawGY / 131.0) - errorGyroY;

    unsigned long currentTime = micros();
    float dt = (currentTime - lastTime) / 1000000.0;
    lastTime = currentTime;

    // Complementary filter sensor fusion
    float rawAccRoll  = atan2(accY, sqrt(accX * accX + accZ * accZ)) * 180.0 / PI;
    float rawAccPitch = atan2(-accX, sqrt(accY * accY + accZ * accZ)) * 180.0 / PI;
    roll  = alpha * (roll + gyroX * dt) + (1.0 - alpha) * (rawAccRoll - errorAccRoll);
    pitch = alpha * (pitch + gyroY * dt) + (1.0 - alpha) * (rawAccPitch - errorAccPitch);

    // Negative sign fixes the 180-degree enclosure flip without polluting draw coords
    float ax = -roll * ACCEL_SCALE;
    float ay = pitch * ACCEL_SCALE;

    // === Stacking physics solver ===
    for (int k = 0; k < 3; k++) {
      for (int i = 0; i < NUM_BOXES; i++) {
        boxes[i].vx += ax;
        boxes[i].vy += ay;
        boxes[i].vx *= FRICTION;
        boxes[i].vy *= FRICTION;

        float nextX = boxes[i].x + boxes[i].vx;
        float nextY = boxes[i].y + boxes[i].vy;

        if (nextX < BORDER_X1) { nextX = BORDER_X1; boxes[i].vx = 0; }
        if (nextX > BORDER_X2 - BOX_SIZE) { nextX = BORDER_X2 - BOX_SIZE; boxes[i].vx = 0; }
        if (nextY < BORDER_Y1) { nextY = BORDER_Y1; boxes[i].vy = 0; }
        if (nextY > BORDER_Y2 - BOX_SIZE) { nextY = BORDER_Y2 - BOX_SIZE; boxes[i].vy = 0; }

        for (int j = 0; j < NUM_BOXES; j++) {
          if (i == j) continue;
          if (nextX < boxes[j].x + BOX_SIZE && nextX + BOX_SIZE > boxes[j].x &&
              nextY < boxes[j].y + BOX_SIZE && nextY + BOX_SIZE > boxes[j].y) {
            if (abs(boxes[i].vx) > abs(boxes[i].vy)) {
              nextX = (boxes[i].vx > 0) ? (boxes[j].x - BOX_SIZE) : (boxes[j].x + BOX_SIZE);
              boxes[i].vx = 0;
            } else {
              nextY = (boxes[i].vy > 0) ? (boxes[j].y - BOX_SIZE) : (boxes[j].y + BOX_SIZE);
              boxes[i].vy = 0;
            }
          }
        }

        boxes[i].x = nextX;
        boxes[i].y = nextY;
      }
    }

    // === Render ===
    u8g2.clearBuffer();
    u8g2.drawFrame(BORDER_X1 - 1, BORDER_Y1 - 1, (BORDER_X2 - BORDER_X1) + 2, (BORDER_Y2 - BORDER_Y1) + 2);
    for (int i = 0; i < NUM_BOXES; i++) {
      u8g2.drawBox((int)boxes[i].x, (int)boxes[i].y, BOX_SIZE, BOX_SIZE);
    }
    u8g2.sendBuffer();
  }
  delay(10);
}

// Write one byte to an MPU6050 register
void writeReg(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg);
  Wire.write(val);
  Wire.endTransmission();
}

// Read accelerometer in g (+/-2g range)
void readAccelG(float &ax, float &ay, float &az) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B);                                  // ACCEL_XOUT_H
  Wire.endTransmission(false);
  Wire.requestFrom((uint8_t)MPU_ADDR, (size_t)6, (bool)true);
  int16_t rx = (Wire.read() << 8) | Wire.read();
  int16_t ry = (Wire.read() << 8) | Wire.read();
  int16_t rz = (Wire.read() << 8) | Wire.read();
  ax = rx / 16384.0;
  ay = ry / 16384.0;
  az = rz / 16384.0;
}

// Calibration
void calibrateMPU6050() {
  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_ncenB08_tr);
  u8g2.drawStr(10, 20, "Place flat on table");
  u8g2.drawStr(10, 38, "Do NOT touch board");
  u8g2.drawStr(10, 55, "Calibrating...");
  u8g2.sendBuffer();

  long sumGX = 0, sumGY = 0;
  float sumAccRoll = 0, sumAccPitch = 0;
  int samples = 150;

  for (int i = 0; i < samples; i++) {
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(0x3B);
    Wire.endTransmission(false);
    Wire.requestFrom((uint8_t)MPU_ADDR, (size_t)14, (bool)true);
    if (Wire.available() >= 14) {
      int16_t rawX = (Wire.read() << 8) | Wire.read();
      int16_t rawY = (Wire.read() << 8) | Wire.read();
      int16_t rawZ = (Wire.read() << 8) | Wire.read();
      Wire.read(); Wire.read();
      int16_t rawGX = (Wire.read() << 8) | Wire.read();
      int16_t rawGY = (Wire.read() << 8) | Wire.read();
      float accX = rawX / 16384.0;
      float accY = rawY / 16384.0;
      float accZ = rawZ / 16384.0;
      sumAccRoll  += atan2(accY, sqrt(accX * accX + accZ * accZ)) * 180.0 / PI;
      sumAccPitch += atan2(-accX, sqrt(accY * accY + accZ * accZ)) * 180.0 / PI;
      sumGX += rawGX;
      sumGY += rawGY;
    }
    delay(10);
  }

  errorAccRoll  = sumAccRoll / (float)samples;
  errorAccPitch = sumAccPitch / (float)samples;
  errorGyroX = (sumGX / (float)samples) / 131.0;
  errorGyroY = (sumGY / (float)samples) / 131.0;
}

// Light sleep until the board is moved (software motion detection).
// Does NOT use the MPU hardware motion-detect interrupt, which this chip does not
// generate reliably. Instead it polls the accelerometer and compares against the
// resting baseline captured at sleep entry.
void enterSleep() {
  // 1) Show "Sleeping..." briefly
  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_ncenB14_tr);
  u8g2.drawStr(20, 38, "Sleeping...");
  u8g2.sendBuffer();
  delay(2000);

  // 2) Turn the OLED off
  u8g2.setPowerSave(1);

  // 3) Low power: keep accelerometer, turn gyro off (gyro not needed while asleep)
  writeReg(0x6C, 0x07);   // PWR_MGMT_2: STBY gyro X/Y/Z = 1, accel stays on
  delay(20);

  // 4) Capture the resting baseline acceleration
  float bx, by, bz;
  readAccelG(bx, by, bz);

  // 5) Light-sleep / sample loop until motion exceeds threshold
  while (true) {
    esp_sleep_enable_timer_wakeup(SLEEP_POLL_US);
    esp_light_sleep_start();          // CPU halts ~150 ms, RAM retained

    float ax, ay, az;
    readAccelG(ax, ay, az);
    float dx = ax - bx, dy = ay - by, dz = az - bz;
    float diff = sqrtf(dx * dx + dy * dy + dz * dz);

    if (diff > WAKE_MOTION_THRESH) break;   // moved -> wake up
  }

  // 6) Woke by motion: restore everything
  writeReg(0x6C, 0x00);   // gyro back on for the sandbox
  delay(20);
  u8g2.setPowerSave(0);   // OLED on
  lastTime = micros();    // reset dt so the filter does not jump
}
