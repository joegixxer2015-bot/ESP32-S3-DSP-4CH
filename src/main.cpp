#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <math.h>
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

// ค่าเกนเริ่มต้น = 0.0 dB (เทสสายพานก่อนค่อยเพิ่มเกน)
float bassGainDb   = 0.0f;
float midGainDb    = 0.0f;
float trebleGainDb = 0.0f;

float dbToLinear(float db) {
  return powf(10.0f, db / 20.0f);
}

// --- คลาสฟิลเตอร์ Biquad สำหรับ AudioTools ---
class SimpleBiquad : public AudioEffect {
public:
  enum Mode { LOWPASS, HIGHPASS };

  SimpleBiquad(Mode mode, float freq, float sampleRate, float q = 0.707f)
      : mode_(mode), freq_(freq), sampleRate_(sampleRate), q_(q), gainLinear_(1.0f) {
    updateCoeffs();
  }

  AudioEffect* clone() override {
    return new SimpleBiquad(*this);
  }

  void setGainDb(float db) {
    gainDb_ = db;
    gainLinear_ = dbToLinear(db);
  }

  float getGainDb() const {
    return gainDb_;
  }

  effect_t process(effect_t input) override {
    if (!active()) return input;

    double x = (double)input;
    double y = b0_ * x + b1_ * x1_ + b2_ * x2_ - a1_ * y1_ - a2_ * y2_;
    x2_ = x1_;
    x1_ = x;
    y2_ = y1_;
    y1_ = y;

    double bandSignal = y;
    double out = x + (bandSignal - x) * (gainLinear_ - 1.0);

    if (out > 32767.0) out = 32767.0;
    if (out < -32768.0) out = -32768.0;
    return (effect_t)out;
  }

private:
  void updateCoeffs() {
    float omega = 2.0f * (float)M_PI * freq_ / sampleRate_;
    float sn = sinf(omega);
    float cs = cosf(omega);
    float alpha = sn / (2.0f * q_);

    double a0 = 1.0;
    if (mode_ == LOWPASS) {
      b0_ = (1.0 - cs) / 2.0;
      b1_ = 1.0 - cs;
      b2_ = (1.0 - cs) / 2.0;
      a0 = 1.0 + alpha;
      a1_ = -2.0 * cs;
      a2_ = 1.0 - alpha;
    } else {
      b0_ = (1.0 + cs) / 2.0;
      b1_ = -(1.0 + cs);
      b2_ = (1.0 + cs) / 2.0;
      a0 = 1.0 + alpha;
      a1_ = -2.0 * cs;
      a2_ = 1.0 - alpha;
    }

    b0_ /= a0;
    b1_ /= a0;
    b2_ /= a0;
    a1_ /= a0;
    a2_ /= a0;
  }

  Mode mode_;
  float freq_;
  float sampleRate_;
  float q_;
  float gainDb_ = 0.0f;
  float gainLinear_ = 1.0f;

  double b0_ = 1.0, b1_ = 0.0, b2_ = 0.0, a1_ = 0.0, a2_ = 0.0;
  double x1_ = 0.0, x2_ = 0.0, y1_ = 0.0, y2_ = 0.0;
};

// --- ระบบสตรีมมิ่งพร้อมท่อ EQ (เริ่มต้นเกน 0 dB ทุกย่าน) ---
I2SStream i2sStreamIn;   
I2SStream i2sStreamOut;  

// ถังพักข้อมูลฝั่งรับบน PSRAM
BufferedStream bufferedIn(i2sStreamIn, 16384);

// ท่อเอฟเฟกต์ EQ 3 ย่าน
AudioEffectStream eqStream(bufferedIn);

SimpleBiquad bassFilter(SimpleBiquad::LOWPASS, 120.0f, (float)info.sample_rate);
SimpleBiquad midFilter(SimpleBiquad::LOWPASS, 1000.0f, (float)info.sample_rate);
SimpleBiquad trebleFilter(SimpleBiquad::HIGHPASS, 4500.0f, (float)info.sample_rate);

// ตัวปรับระดับความดังปลายทางก่อนออก DAC
VolumeStream volumeControl(eqStream);

// ตัวเชื่อมสายพาน: volumeControl -> i2sStreamOut
StreamCopy copier(i2sStreamOut, volumeControl);

unsigned long lastKeepAliveCheck = 0;

void applyEqGains() {
  bassFilter.setGainDb(bassGainDb);
  midFilter.setGainDb(midGainDb);
  trebleFilter.setGainDb(trebleGainDb);
}

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
      float volFactor = constrain((float)incomingData.value / 100.0f, 0.0f, 1.0f);
      volumeControl.setVolume(volFactor);
      break;
    }
    case 1: { // Bass Gain (-12 ถึง +12 dB)
      bassGainDb = constrain((float)incomingData.value, -12.0f, 12.0f);
      applyEqGains();
      Serial.printf("--> Set Bass Gain: %.1f dB\n", bassGainDb);
      break;
    }
    case 2: { // Mid Gain (-12 ถึง +12 dB)
      midGainDb = constrain((float)incomingData.value, -12.0f, 12.0f);
      applyEqGains();
      Serial.printf("--> Set Mid Gain: %.1f dB\n", midGainDb);
      break;
    }
    case 3: { // Treble Gain (-12 ถึง +12 dB)
      trebleGainDb = constrain((float)incomingData.value, -12.0f, 12.0f);
      applyEqGains();
      Serial.printf("--> Set Treble Gain: %.1f dB\n", trebleGainDb);
      break;
    }
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

void setupEQ() {
  applyEqGains();
  eqStream.addEffect(bassFilter);
  eqStream.addEffect(midFilter);
  eqStream.addEffect(trebleFilter);

  auto eqCfg = eqStream.defaultConfig();
  eqCfg.copyFrom(info);
  if (!eqStream.begin(eqCfg)) {
    Serial.println("[ERROR] EQ Stream begin failed!");
  }
}

// --- ตั้งค่าพอร์ต I2S ทั้งขาเข้า (Port 0) และขาออก (Port 1) ---
void setupI2S() {
  // 1. ตั้งค่าฝั่งรับเข้า (Input - Slave RX) จาก DevKit V1 ใช้ I2S Port 0
  auto configIn = i2sStreamIn.defaultConfig(RX_MODE);
  configIn.port_no = 0;
  configIn.sample_rate = 44100;
  configIn.channels = 2;
  configIn.bits_per_sample = 16;
  configIn.pin_bck = I2S_IN_BCK;
  configIn.pin_ws  = I2S_IN_LRCK;
  configIn.pin_data = I2S_IN_DATA;
  configIn.is_master = false; 
  if (!i2sStreamIn.begin(configIn)) {
    Serial.println("[ERROR] I2S RX (Port 0) begin failed!");
  }

  // เริ่มต้น Buffer บน PSRAM
  bufferedIn.begin();

  // เริ่มต้นท่อ EQ (เกนเริ่มต้น 0 dB)
  setupEQ();

  // 2. ตั้งค่าฝั่งส่งออก (Output - Master TX) ไปยัง DAC PCM5102A ใช้ I2S Port 1
  auto configOut = i2sStreamOut.defaultConfig(TX_MODE);
  configOut.port_no = 1;
  configOut.sample_rate = 44100;
  configOut.channels = 2;
  configOut.bits_per_sample = 16;
  configOut.pin_bck = I2S_BCK_PIN;
  configOut.pin_ws  = I2S_LRCK_PIN;
  configOut.pin_data = I2S_DATA1_PIN; 
  if (!i2sStreamOut.begin(configOut)) {
    Serial.println("[ERROR] I2S TX (Port 1) begin failed!");
  }
  
  // พ่วงสัญญาณขาออกชุดที่ 2 จาก I2S Port 1
  gpio_matrix_out(I2S_DATA2_PIN, i2s_periph_signal[I2S_NUM_1].data_out_sig, false, false);

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
  Serial.println(">> ESP32-S3 EQ Test Ready (Initial Gain = 0.0 dB) <<");
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
