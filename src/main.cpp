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

#define I2S_BCK_PIN   15   
#define I2S_LRCK_PIN  16   
#define I2S_DATA1_PIN 17   
#define I2S_DATA2_PIN 18   

typedef struct struct_message {
    int controlMode;  
    int value;        
    bool isMuted;     
} struct_message;

struct_message incomingData;

AudioInfo info(44100, 2, 16);
I2SStream i2s;
VolumeStream volumeControl(i2s); 

void setupI2S() {
  auto config = i2s.defaultConfig(TX_MODE);
  config.copyFrom(info);
  config.pin_bck = I2S_BCK_PIN;
  config.pin_ws  = I2S_LRCK_PIN;
  config.pin_data = I2S_DATA1_PIN; 
  
  i2s.begin(config);
  gpio_matrix_out(I2S_DATA2_PIN, i2s_periph_signal[I2S_NUM_0].data_out_sig, false, false);

  auto vconfig = volumeControl.defaultConfig();
  vconfig.copyFrom(info);
  volumeControl.begin(vconfig);
  volumeControl.setVolume(0.8); 
}

void OnDataRecv(const uint8_t *mac_addr, const uint8_t *incomingDataPtr, int len) {
  memcpy(&incomingData, incomingDataPtr, sizeof(incomingData));
  
  const char* modeNames[] = {"MASTER VOLUME", "BASS", "MID", "TREBLE"};
  Serial.printf("\n[ESP-NOW] Mode: %s | Value: %d | Mute: %s\n", 
                modeNames[incomingData.controlMode],
                incomingData.value, 
                incomingData.isMuted ? "YES" : "NO");

  if (incomingData.isMuted) {
    volumeControl.setVolume(0.0);
    return;
  }

  switch (incomingData.controlMode) {
    case 0: { 
      float volFactor = (float)incomingData.value / 100.0f;
      volumeControl.setVolume(volFactor);
      break;
    }
    case 1: 
      Serial.printf("--> Set Bass Gain: %d dB\n", incomingData.value);
      break;
    case 2: 
      Serial.printf("--> Set Mid Gain: %d dB\n", incomingData.value);
      break;
    case 3: 
      Serial.printf("--> Set Treble Gain: %d dB\n", incomingData.value);
      break;
  }
}

void setup() {
  Serial.begin(115200);
  delay(1000);

  setupI2S();

  WiFi.mode(WIFI_STA);
  
  // พิมพ์ MAC Address ของ S3 ออกทาง Serial Monitor เพื่อเอาไปใส่ใน C3
  Serial.print("ESP32-S3 STA MAC Address: ");
  Serial.println(WiFi.macAddress());

  WiFi.begin(ssid, password);
  Serial.print("Connecting to WiFi");
  int timeout = 0;
  while (WiFi.status() != WL_CONNECTED && timeout < 20) {
    delay(500);
    Serial.print(".");
    timeout++;
  }
  
  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("\nWiFi Connected!");
    Serial.printf("Connected Channel: %d\n", WiFi.channel());
  } else {
    Serial.println("\nWiFi Connect Failed! Operating in standalone ESP-NOW mode.");
  }

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
