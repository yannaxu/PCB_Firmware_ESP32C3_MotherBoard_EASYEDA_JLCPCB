#include <Wire.h>
#include <U8g2lib.h>
#include <math.h>

// SSD1306 128x64 OLED (normal orientation R0)
U8G2_SSD1306_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, /* reset=*/ U8X8_PIN_NONE);

const int MPU_ADDR = 0x68;

#define BUTTON_PIN  1          // GPIO1 <- open-drain button (active-low)

// Counters
volatile uint32_t btnCount  = 0;       // incremented in button ISR
volatile uint32_t lastBtnMs = 0;       // button debounce timestamp
uint32_t          mpuCount  = 0;       // software-detected motion count

// Software motion detection state
bool     wasMoving    = false;
uint32_t lastMotionMs = 0;

const float    MOTION_THRESH     = 0.25; // |accel - 1g| in g to count as motion
const uint32_t MOTION_REFRACTORY = 300;  // ms minimum gap between motion counts

void writeReg(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg);
  Wire.write(val);
  Wire.endTransmission();
}

// Read accelerometer (in g, +/-2g range)
void readAccel(float &ax, float &ay, float &az) {
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

void IRAM_ATTR buttonISR() {
  uint32_t now = millis();
  if (now - lastBtnMs > 200) {   // 200 ms software debounce
    btnCount++;
    lastBtnMs = now;
  }
}

void drawDisplay() {
  char buf[24];
  u8g2.clearBuffer();

  u8g2.setFont(u8g2_font_ncenB10_tr);
  u8g2.drawStr(6, 14, "Pulse Counter");
  u8g2.drawHLine(0, 18, 128);

  u8g2.setFont(u8g2_font_ncenB12_tr);
  snprintf(buf, sizeof(buf), "MPU : %lu", (unsigned long)mpuCount);
  u8g2.drawStr(6, 40, buf);

  snprintf(buf, sizeof(buf), "BTN : %lu", (unsigned long)btnCount);
  u8g2.drawStr(6, 62, buf);

  u8g2.sendBuffer();
}

void setup() {
  Serial.begin(115200);
  delay(100);

  Wire.begin(8, 9);          // SDA=GPIO8, SCL=GPIO9
  u8g2.begin();

  pinMode(BUTTON_PIN, INPUT_PULLUP);   // open-drain button needs pull-up

  writeReg(0x6B, 0x00);      // PWR_MGMT_1: wake up the MPU6050
  delay(50);

  // Button is active-low -> count on FALLING edge (hardware interrupt)
  attachInterrupt(digitalPinToInterrupt(BUTTON_PIN), buttonISR, FALLING);

  drawDisplay();
}

void loop() {
  static uint32_t lastBtnShown = 0;
  bool needRedraw = false;

  // --- Software motion detection (no INT pin needed) ---
  float ax, ay, az;
  readAccel(ax, ay, az);
  float mag = sqrtf(ax * ax + ay * ay + az * az);  // ~1.0 at rest
  float dev = fabsf(mag - 1.0);                    // deviation from gravity

  bool moving = (dev > MOTION_THRESH);
  uint32_t now = millis();

  // Count one event on the still -> moving transition, with a refractory gap
  if (moving && !wasMoving && (now - lastMotionMs > MOTION_REFRACTORY)) {
    mpuCount++;
    lastMotionMs = now;
    needRedraw = true;
  }
  wasMoving = moving;

  // --- Button count (updated in ISR) ---
  if (btnCount != lastBtnShown) {
    lastBtnShown = btnCount;
    needRedraw = true;
  }

  if (needRedraw) {
    Serial.printf("MPU=%lu  BTN=%lu  dev=%.2f\n",
                  (unsigned long)mpuCount, (unsigned long)btnCount, dev);
    drawDisplay();
  }

  delay(10);   // ~100 Hz sampling
}

