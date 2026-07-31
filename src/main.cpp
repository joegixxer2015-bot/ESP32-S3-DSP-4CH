#include <Arduino.h>
#include <WiFi.h>
#include "rom/gpio.h"        // Header สำหรับ GPIO Matrix
#include "driver/gpio.h"
#include "AudioTools.h"

// ==========================================
// WIFI SETTINGS
// ==========================================
const char* ssid     = "YOUR_WIFI_SSID";
const char* password = "YOUR_WIFI_PASSWORD";

// ==========================================
// PIN DEFINITIONS (ESP32-S3 N16R8)
// ==========================================
#define I2S_BCK_PIN   15   // Bit Clock (PCM5102A #1 & #2)
#define I2S_LRCK_PIN  16   // Word Select / LCK (PCM5102A #1 & #2)
#define I2S_DATA1_PIN 17   // DIN 1 (PCM5102A #1)
#define I2S_DATA2_PIN 18   // DIN 2 (PCM5102A #2)

AudioInfo info(44100, 2, 16);
I2SStream i2s;

void setupI2S() {
  auto config = i2s.defaultConfig(TX_MODE);
  config.copyFrom(info);
  config.pin_bck = I2S_BCK_PIN;
  config.pin_ws  = I2S_LRCK_PIN;
  config.pin_data = I2S_DATA1_PIN; // DIN 1
  
  i2s.begin(config);

  // สำเนาสัญญาณ I2S Data ออกไปยัง DIN 2 (GPIO 18) คู่ขนานผ่าน GPIO Matrix
  esp_rom_gpio_connect_out_signal(I2S_DATA2_PIN, i2s_periph_signal[I2S_NUM_0].data_out_sig, false, false);
}

void setup() {
  Serial.begin(115200);
  delay(1000);

  // 1. เริ่มระบบ I2S สำหรับ Dual PCM5102A
  setupI2S();

  // 2. เชื่อมต่อ Wi-Fi
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  Serial.print("Connecting to WiFi");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nWiFi Connected!");
  Serial.print("IP Address: ");
  Serial.println(WiFi.localIP());

  Serial.println(">> Audio System Initialized Successfully <<");
}

void loop() {
  delay(10);
}
