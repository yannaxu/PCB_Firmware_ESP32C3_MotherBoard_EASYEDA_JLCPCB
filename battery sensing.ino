#include <Wire.h>
#include <U8g2lib.h>

// SSD1306 128x64 OLED (normal orientation R0)
U8G2_SSD1306_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, /* reset=*/ U8X8_PIN_NONE);

// === Settings ===
const int   BAT_PIN  = 3;     // A3 = GPIO3 (ESP32-C3 ADC1 channel)
const float DIVIDER  = 2.0;   // 0.5x voltage divider, so multiply by 2 to restore
const int   SAMPLES  = 32;    // Number of averaging samples (noise reduction)

void setup() {
  Serial.begin(115200);

  Wire.begin(8, 9);              // SDA=GPIO8, SCL=GPIO9
  u8g2.begin();

  // ADC configuration
  analogReadResolution(12);                   // 0..4095 (12-bit)
  analogSetPinAttenuation(BAT_PIN, ADC_11db); // approx 0..3.3V input range
}

void loop() {
  long sumRaw = 0;
  long sumMv  = 0;

  // Take several readings and average
  for (int i = 0; i < SAMPLES; i++) {
    sumRaw += analogRead(BAT_PIN);            // raw ADC value (0..4095)
    sumMv  += analogReadMilliVolts(BAT_PIN);  // calibrated pin voltage (mV)
    delay(2);
  }

  int   adcRaw  = sumRaw / SAMPLES;                   // average ADC value
  float pinVolt = (sumMv / (float)SAMPLES) / 1000.0;  // pin voltage (V)
  float batVolt = pinVolt * DIVIDER;                  // actual battery voltage (V)

  // Also print to the serial monitor
  Serial.printf("ADC=%d  Pin=%.3fV  Battery=%.3fV\n", adcRaw, pinVolt, batVolt);

  // === OLED display ===
  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_ncenB10_tr);
  u8g2.drawStr(8, 14, "Battery Monitor");
  u8g2.drawHLine(0, 18, 128);

  char buf[24];
  u8g2.setFont(u8g2_font_ncenB08_tr);

  snprintf(buf, sizeof(buf), "ADC : %d", adcRaw);
  u8g2.drawStr(8, 36, buf);

  snprintf(buf, sizeof(buf), "Pin : %.2f V", pinVolt);
  u8g2.drawStr(8, 50, buf);

  // Show battery voltage in a larger font for emphasis
  u8g2.setFont(u8g2_font_ncenB12_tr);
  snprintf(buf, sizeof(buf), "BAT %.2fV", batVolt);
  u8g2.drawStr(8, 64, buf);

  u8g2.sendBuffer();

  delay(200); // refresh 5 times per second
}
