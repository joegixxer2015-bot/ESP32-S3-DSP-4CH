#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include "rom/gpio.h"
#include "driver/gpio.h"
#include "AudioTools.h"

// --- พินฝั่งขาออก (Output ไปยัง DAC PCM5102A) ---
#define I2S_BCK_PIN   15   
#define I2S_LRCK_PIN  16   
#define I2S_DATA1_PIN 17   
#define I2S_DATA2_PIN 18   // ขาพ่วงข้อมูลเสริม (ถ้ามี)

// --- พินฝั่งขาเข้า (Input รับจาก ESP32 DevKit V1) ---
#define I2S_IN_BCK    4
#define I2S_IN_LRCK   5
#define I2S_IN_DATA   6

// โครงสร้างข้อมูลคำสั่งจากรีโมท (ESP-NOW)
typedef struct struct_message {
    int controlMode;  
    int value;        
    bool isMuted;     
} struct_message;

struct_message incomingData;

// ตั้งค่ารูปแบบเสียงพื้นฐาน: 44.1kHz, Stereo, 16-bit
AudioInfo info(44100, 2, 16);

// --- ระบบสตรีมมิ่งเส้นทางตรง (ตัดท่อ EQ ออกแล้ว) ---
I2SStream i2sStreamIn;   
I2SStream i2sStreamOut;  

// ถังพักข้อมูลฝั่งรับ (32KB บน PSRAM)
BufferedStream bufferedIn(i2sStreamIn, 32768); 

// ตัวปรับระดับความดังปลายทางก่อนออก DAC
VolumeStream volumeControl(i2sStreamOut); 

// ตัวเชื่อมสายพาน: ดึงจาก Buffer เข้า VolumeStream แล้วออก DAC โดยตรง
StreamCopy copier(volumeControl, bufferedIn);

unsigned long lastKeepAliveCheck = 0;

// --- ฟังก์ชันรับข้อมูล ESP-NOW ---
#if defined(ESP_ARDUINO_VERSION_MAJOR) && ESP_ARDUINO_VERSION_MAJOR >= 3
void OnDataRecv(const esp_now_recv_info_t * recv_info, const uint8_t *incomingDataPtr, int len) {
#else
void OnDataRecv(const uint8_t * mac_addr, const uint8_t *incomingDataPtr, int len) {
#endif
  if (len < sizeof(incomingData)) return;
  
  memcpy(&incomingData, incomingDataPtr, sizeof(incomingData));
  
  const char* modeNames[] = {"MASTER VOLUME", "BASS", "MID", "TREBLE"};
  int safeMode = constrain(incomingData.controlMode, 0, 3);

  Serial.printf("\n[ESP-NOW] Mode: %s | Value: %d | Mute: %s\n", 
                modeNames[safeMode],
                incomingData.value, 
                incomingData.isMuted ? "YES" : "NO");

  if (incomingData.isMuted) {
    volumeControl.setVolume(0.0);
    return;
  }

  switch (safeMode) {
    case 0: { // Master Volume (0-100 แปลงเป็น 0.0 - 1.0)
      float volFactor = (float)incomingData.value / 100.0f;
      volumeControl.setVolume(volFactor);
      break;
    }
    case 1: 
      Serial.printf("--> Set Bass Gain: %d dB (EQ bypassed for test)\n", incomingData.value);
      break;
    case 2: 
      Serial.printf("--> Set Mid Gain: %d dB (EQ bypassed for test)\n", incomingData.value);
      break;
    case 3: 
      Serial.printf("--> Set Treble Gain: %d dB (EQ bypassed for test)\n", incomingData.value);
      break;
  }
}

// --- ตั้งค่าระบบ Wi-Fi และ ESP-NOW ล็อกช่อง 1 ---
void initWifiAndEspNow() {
  WiFi.persistent(false);
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  
  esp_wifi_set_promiscuous(true);
  esp_wifi_set_channel(1, WIFI_SECOND_CHAN_NONE);
  esp_wifi_set_promiscuous(false);

  if (esp_now_init() == ESP_OK) {
    esp_now_register_recv_cb(OnDataRecv);
    Serial.println(">> ESP-NOW Initialized & Ready on Channel 1 <<");
  } else {
    Serial.println("ESP-NOW Init Failed!");
  }
}

// --- ตั้งค่าพอร์ต I2S ทั้งขาเข้าและขาออก ---
void setupI2S() {
  // 1. ตั้งค่าฝั่งรับเข้า (Input - Slave RX) จาก DevKit V1
  auto configIn = i2sStreamIn.defaultConfig(RX_MODE);
  configIn.sample_rate = 44100;
  configIn.channels = 2;
  configIn.bits_per_sample = 16;
  configIn.pin_bck = I2S_IN_BCK;
  configIn.pin_ws  = I2S_IN_LRCK;
  configIn.pin_data = I2S_IN_DATA;
  configIn.is_master = false; 
  i2sStreamIn.begin(configIn);

  // เริ่มต้น Buffer บน PSRAM
  bufferedIn.begin();

  // 2. ตั้งค่าฝั่งส่งออก (Output - Master TX) ไปยัง DAC PCM5102A
  auto configOut = i2sStreamOut.defaultConfig(TX_MODE);
  configOut.sample_rate = 44100;
  configOut.channels = 2;
  configOut.bits_per_sample = 16;
  configOut.pin_bck = I2S_BCK_PIN;
  configOut.pin_ws  = I2S_LRCK_PIN;
  configOut.pin_data = I2S_DATA1_PIN; 
  i2sStreamOut.begin(configOut);
  
  // พ่วงสัญญาณขาออกชุดที่ 2
  gpio_matrix_out(I2S_DATA2_PIN, i2s_periph_signal[I2S_NUM_0].data_out_sig, false, false);

  // 3. เริ่มต้นระบบ Volume ที่ความดังเริ่มต้น 80%
  auto vconfig = volumeControl.defaultConfig();
  vconfig.copyFrom(info);
  volumeControl.begin(vconfig);
  volumeControl.setVolume(0.8);

  // 4. เริ่มต้นตัวคัดลอกข้อมูลสตรีม
  copier.begin();
}

void setup() {
  Serial.begin(115200);
  delay(1000);

  setupI2S();
  initWifiAndEspNow();

  Serial.print("ESP32-S3 STA MAC Address: ");
  Serial.println(WiFi.macAddress());
  Serial.println(">> ESP32-S3 Direct Test Ready (No EQ)! <<");
}

void loop() {
  copier.copy();

  if (millis() - lastKeepAliveCheck > 3000) {
    uint8_t primaryChan;
    wifi_second_chan_t secondChan;
    esp_wifi_get_channel(&primaryChan, &secondChan);

    if (primaryChan != 1) {
      esp_wifi_set_channel(1, WIFI_SECOND_CHAN_NONE);
    }
    lastKeepAliveCheck = millis();
  }
}
