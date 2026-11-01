#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include "rom/gpio.h"
#include "driver/gpio.h"
#include "AudioTools.h"

// --- พินฝั่งขาออก (Output ไปยัง DAC) ---
#define I2S_BCK_PIN   15   
#define I2S_LRCK_PIN  16   
#define I2S_DATA1_PIN 17   
#define I2S_DATA2_PIN 18   

// --- พินฝั่งขาเข้า (Input รับจาก ESP32 DevKit V1) ---
#define I2S_IN_BCK    4
#define I2S_IN_LRCK   5
#define I2S_IN_DATA   6

typedef struct struct_message {
    int controlMode;  
    int value;        
    bool isMuted;     
} struct_message;

struct_message incomingData;

AudioInfo info(44100, 2, 16);

// ประกาศสตรีมทั้งฝั่งรับ (RX) และฝั่งส่ง (TX)
I2SStream i2sStreamIn;   
I2SStream i2sStreamOut;  
VolumeStream volumeControl(i2sStreamOut); 

// ประกาศตัวกรอง Biquad Filter สำหรับ 3 ย่าน (Bass, Mid, Treble)
BiquadFilter bassFilter;
BiquadFilter midFilter;
BiquadFilter trebleFilter;

unsigned long lastKeepAliveCheck = 0;

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

  // อัปเดตค่าตามโหมดที่ส่งมาจากรีโมต C3
  switch (safeMode) {
    case 0: { 
      float volFactor = (float)incomingData.value / 100.0f;
      volumeControl.setVolume(volFactor);
      break;
    }
    case 1: 
      bassFilter.setGain(incomingData.value); // ปรับ Gain เบส (หน่วยเป็น dB)
      Serial.printf("--> Set Bass Gain: %d dB\n", incomingData.value);
      break;
    case 2: 
      midFilter.setGain(incomingData.value);  // ปรับ Gain เสียงกลาง (หน่วยเป็น dB)
      Serial.printf("--> Set Mid Gain: %d dB\n", incomingData.value);
      break;
    case 3: 
      trebleFilter.setGain(incomingData.value); // ปรับ Gain เสียงแหลม (หน่วยเป็น dB)
      Serial.printf("--> Set Treble Gain: %d dB\n", incomingData.value);
      break;
  }
}

void initWifiAndEspNow() {
  WiFi.persistent(false);
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  
  // ล็อก Channel 1 บอร์ด S3
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

void setupI2S() {
  // 1. ตั้งค่าฟิลเตอร์เริ่มต้น (Bass, Mid, Treble)
  bassFilter.setLowShelf(250, 0, 1.0);     // ย่านเบสตัดที่ 250Hz, เริ่มต้น 0dB
  midFilter.setPeak(1000, 0, 1.0);         // ย่านกลางที่ 1000Hz, เริ่มต้น 0dB
  trebleFilter.setHighShelf(4000, 0, 1.0); // ย่านแหลมที่ 4000Hz, เริ่มต้น 0dB

  // 2. ตั้งค่าฝั่งรับเข้า (Input - Slave RX) จาก DevKit V1
  auto configIn = i2sStreamIn.defaultConfig(RX_MODE);
  configIn.copyFrom(info);
  configIn.pin_bck = I2S_IN_BCK;
  configIn.pin_ws  = I2S_IN_LRCK;
  configIn.pin_data = I2S_IN_DATA;
  i2sStreamIn.begin(configIn);

  // 3. ตั้งค่าฝั่งส่งออก (Output - Master TX) ไปยัง DAC
  auto configOut = i2sStreamOut.defaultConfig(TX_MODE);
  configOut.copyFrom(info);
  configOut.pin_bck = I2S_BCK_PIN;
  configOut.pin_ws  = I2S_LRCK_PIN;
  configOut.pin_data = I2S_DATA1_PIN; 
  
  i2sStreamOut.begin(configOut);
  gpio_matrix_out(I2S_DATA2_PIN, i2s_periph_signal[I2S_NUM_0].data_out_sig, false, false);

  // 4. ตั้งค่าระบบ Volume และ DSP Pipeline
  auto vconfig = volumeControl.defaultConfig();
  vconfig.copyFrom(info);
  volumeControl.begin(vconfig);
  volumeControl.setVolume(0.8); 
}

void setup() {
  Serial.begin(115200);
  delay(1000);

  setupI2S();
  initWifiAndEspNow();

  Serial.print("ESP32-S3 STA MAC Address: ");
  Serial.println(WiFi.macAddress());
  Serial.println(">> ESP32-S3 Audio DSP & EQ System Ready! <<");
}

void loop() {
  // อ่านข้อมูลเสียงจากฝั่งรับ (DevKit V1) เข้ามาประมวลผลผ่าน EQ
  uint8_t buffer[512];
  size_t bytesRead = i2sStreamIn.readBytes(buffer, sizeof(buffer));
  
  if (bytesRead > 0) {
    // แปลง Buffer ให้เป็นชุดข้อมูลตัวอย่างเสียง (16-bit PCM Samples) เพื่อผ่าน Biquad Filter
    int16_t *samples = (int16_t*)buffer;
    int numSamples = bytesRead / sizeof(int16_t);
    
    for (int i = 0; i < numSamples; i++) {
      float val = (float)samples[i];
      
      // วิ่งผ่านตัวกรองทีละย่าน (Bass -> Mid -> Treble)
      val = bassFilter.filter(val);
      val = midFilter.filter(val);
      val = trebleFilter.filter(val);
      
      // ป้องกันค่าสัญญาณเกินขอบเขต 16-bit แล้วบันทึกกลับลง Buffer
      samples[i] = (int16_t)constrain(val, -32768, 32767);
    }
    
    // ส่งข้อมูลเสียงที่ผ่านการปรับ EQ แล้ว ผ่านระบบ Volume ออกไปยัง DAC
    volumeControl.write(buffer, bytesRead);
  }

  // สแตนบายด์ตรึง Channel 1 ไว้สำหรับ ESP-NOW
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
