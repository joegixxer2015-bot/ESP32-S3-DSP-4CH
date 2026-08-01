#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include "rom/gpio.h"
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
#define I2S_BCK_PIN   15   // Bit Clock
#define I2S_LRCK_PIN  16   // Word Select / LCK
#define I2S_DATA1_PIN 17   // DIN 1 (PCM5102A #1)
#define I2S_DATA2_PIN 18   // DIN 2 (PCM5102A #2)

// โครงสร้างข้อมูลใหม่จาก ESP32-C3 (ต้องตรงกับ C3 100%)
typedef struct struct_message {
    int controlMode;  // 0: Volume, 1: Bass, 2: Mid, 3: Treble
    int value;        // ค่าความดัง หรือค่า dB
    bool isMuted;     // สถานะ Mute
} struct_message;

struct_message incomingData;

AudioInfo info(44100, 2, 16);
I2SStream i2s;
VolumeStream volumeControl(i2s); // ตัวจัดการ Volume

void setupI2S() {
  auto config = i2s.defaultConfig(TX_MODE);
  config.copyFrom(info);
  config.pin_bck = I2S_BCK_PIN;
  config.pin_ws  = I2S_LRCK_PIN;
  config.pin_data = I2S_DATA1_PIN; // DIN 1
  
  i2s.begin(config);

  // สำเนาสัญญาณ I2S Data ออกไปยัง DIN 2 (GPIO 18) คู่ขนาน
  gpio_matrix_out(I2S_DATA2_PIN, i2s_periph_signal[I2S_NUM_0].data_out_sig, false, false);

  // ตั้งค่า VolumeStream
  auto vconfig = volumeControl.defaultConfig();
  vconfig.copyFrom(info);
  volumeControl.begin(vconfig);
  volumeControl.setVolume(0.8); // เริ่มต้นที่ 80%
}

// Callback เมื่อได้รับข้อมูล ESP-NOW จาก ESP32-C3 (รองรับ Arduino ESP32 core v2.x)
void OnDataRecv(const uint8_t *mac_addr, const uint8_t *incomingDataPtr, int len) {
  memcpy(&incomingData, incomingDataPtr, sizeof(incomingData));
  
  const char* modeNames[] = {"MASTER VOLUME", "BASS", "MID", "TREBLE"};
  Serial.printf("\n[ESP-NOW] Mode: %s | Value: %d | Mute: %s\n", 
                modeNames[incomingData.controlMode],
                incomingData.value, 
                incomingData.isMuted ? "YES" : "NO");

  // 1. จัดการระบบ Mute
  if (incomingData.isMuted) {
    volumeControl.setVolume(0.0);
    return;
  }

  // 2. จัดการรับค่าแยกตามโหมด
  switch (incomingData.controlMode) {
    case 0: { // Master Volume (0 - 100)
      float volFactor = (float)incomingData.value / 100.0f;
      volumeControl.setVolume(volFactor);
      break;
    }
    case 1: // Bass Gain (-10 ถึง +10 dB)
      Serial.printf("--> Set Bass Gain: %d dB\n", incomingData.value);
      break;
    case 2: // Mid Gain (-10 ถึง +10 dB)
      Serial.printf("--> Set Mid Gain: %d dB\n", incomingData.value);
      break;
    case 3: // Treble Gain (-10 ถึง +10 dB)
      Serial.printf("--> Set Treble Gain: %d dB\n", incomingData.value);
      break;
  }
}

void setup() {
  Serial.begin(115200);
  delay(1000);

  // 1. เริ่มระบบ I2S + Volume Control
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

  // 3. เริ่มระบบ ESP-NOW Receiver
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }
  esp_now_register_recv_cb(OnDataRecv);

  Serial.println(">> Audio & DSP EQ Receiver Ready <<");
}

void loop() {
  delay(10);
}
