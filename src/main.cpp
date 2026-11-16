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

// ตั้งค่ารูปแบบเสียง: 44.1kHz, Stereo, 16-bit
AudioInfo info(44100, 2, 16);

// --- ระบบสตรีมมิ่งและสายพานประมวลผลเสียง ---
I2SStream i2sStreamIn;   
I2SStream i2sStreamOut;  

// เรียงลำดับท่อส่งข้อมูล (จากปลายทางออก DAC ย้อนขึ้นไปหาต้นทาง)
VolumeStream volumeControl(i2sStreamOut); 
VolumeStream eqBassStream(volumeControl);
VolumeStream eqMidStream(eqBassStream);
VolumeStream eqTrebleStream(eqMidStream);

// **ตัวแก้ปัญหา Clock Drift:** สร้าง Ring Buffer ขนาด 4KB คั่นกลางฝั่งรับ
BufferedStream bufferedIn(i2sStreamIn, 4096); 

// **ตัวเชื่อมสายพาน:** ดึงข้อมูลจาก Buffer ผ่านท่อ EQ และ Volume วิ่งออก DAC อัตโนมัติ
StreamCopy copier(eqTrebleStream, bufferedIn);

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
      Serial.printf("--> Set Bass Gain: %d dB\n", incomingData.value);
      // ตรงนี้สามารถผูกกับฟังก์ชันปรับ Gain ของ EQ เพิ่มเติมได้ในอนาคต
      break;
    case 2: 
      Serial.printf("--> Set Mid Gain: %d dB\n", incomingData.value);
      break;
    case 3: 
      Serial.printf("--> Set Treble Gain: %d dB\n", incomingData.value);
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
  configIn.copyFrom(info);
  configIn.pin_bck = I2S_IN_BCK;
  configIn.pin_ws  = I2S_IN_LRCK;
  configIn.pin_data = I2S_IN_DATA;
  i2sStreamIn.begin(configIn);

  // 2. ตั้งค่าฝั่งส่งออก (Output - Master TX) ไปยัง DAC PCM5102A
  auto configOut = i2sStreamOut.defaultConfig(TX_MODE);
  configOut.copyFrom(info);
  configOut.pin_bck = I2S_BCK_PIN;
  configOut.pin_ws  = I2S_LRCK_PIN;
  configOut.pin_data = I2S_DATA1_PIN; 
  i2sStreamOut.begin(configOut);
  
  // พ่วงสัญญาณขาออกชุดที่ 2 (ถ้าต้องการส่งออก 2 ทาง)
  gpio_matrix_out(I2S_DATA2_PIN, i2s_periph_signal[I2S_NUM_0].data_out_sig, false, false);

  // 3. เริ่มต้นระบบ Volume และสายพานตัวกรองทั้งหมด
  auto vconfig = volumeControl.defaultConfig();
  vconfig.copyFrom(info);
  
  volumeControl.begin(vconfig);
  volumeControl.setVolume(0.8); // ตั้งค่าความดังเริ่มต้นที่ 80%

  eqBassStream.begin(vconfig);
  eqMidStream.begin(vconfig);
  eqTrebleStream.begin(vconfig);

  // เริ่มต้นทำงาน Ring Buffer คั่นกลาง
  bufferedIn.begin();
}

void setup() {
  Serial.begin(115200);
  delay(1000);

  // เริ่มต้นระบบฮาร์ดแวร์
  setupI2S();
  initWifiAndEspNow();

  Serial.print("ESP32-S3 STA MAC Address: ");
  Serial.println(WiFi.macAddress());
  Serial.println(">> ESP32-S3 Audio DSP System Ready! <<");
}

void loop() {
  // ประมวลผลเสียงอัตโนมัติ: ดึงจาก Buffer -> ผ่าน EQ/Volume -> พ่นออก DAC
  copier.copy();

  // ตรวจสอบและรักษากลุ่มช่องสัญญาณ Wi-Fi ให้คงอยู่ที่ช่อง 1 เสมอทุกๆ 3 วินาที
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
