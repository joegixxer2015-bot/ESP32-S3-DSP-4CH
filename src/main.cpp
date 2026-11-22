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

// ตัวแปรสถานะสำหรับรับค่าจาก ESP-NOW อย่างปลอดภัย (ไม่ให้กวนลูปเสียง)
volatile float targetBassGainDb   = 0.0f;
volatile float targetMidGainDb    = 0.0f;
volatile float targetTrebleGainDb = 0.0f;
volatile float targetVolume       = 0.8f;
volatile bool  targetMuted        = false;

volatile bool  eqDirty            = false;
volatile bool  volDirty           = false;
volatile bool  logDirty           = false;
volatile int   lastModeLog        = 0;
volatile int   lastValLog         = 0;

float dbToLinear(float db) {
  return powf(10.0f, db / 20.0f);
}

// --- คลาสฟิลเตอร์ Biquad สำหรับ AudioTools (float ล้วน + ข้ามตอนเกน 0 dB) ---
class SimpleBiquad : public AudioEffect {
public:
  enum Mode { LOWPASS, HIGHPASS };

  SimpleBiquad(Mode mode, float freq, float sampleRate, float q = 0.707f)
      : mode_(mode), freq_(freq), sampleRate_(sampleRate), q_(q) {
    updateCoeffs();
    setGainDb(0.0f);
    liveInstance_ = this;
  }

  SimpleBiquad(const SimpleBiquad& other) = default;

  AudioEffect* clone() override {
    SimpleBiquad* copy = new SimpleBiquad(*this);
    copy->liveInstance_ = copy;
    // เก็บพอยน์เตอร์ตัวที่ถูก AudioEffectStream โคลนไปใช้งานจริง
    liveInstance_ = copy;
    return copy;
  }

  SimpleBiquad* activeInstance() {
    return liveInstance_ ? liveInstance_ : this;
  }

  void setGainDb(float db) {
    gainDb_ = db;
    gainLinear_ = dbToLinear(db);
  }

  float getGainDb() const {
    return gainDb_;
  }

  effect_t process(effect_t input) override {
    if (!active() || fabsf(gainDb_) < 0.05f) return input;

    float x = (float)input;
    float y = b0_ * x + b1_ * x1_ + b2_ * x2_ - a1_ * y1_ - a2_ * y2_;
    x2_ = x1_;
    x1_ = x;
    y2_ = y1_;
    y1_ = y;

    float out = x + (y - x) * (gainLinear_ - 1.0f);
    if (out > 32767.0f) out = 32767.0f;
    if (out < -32768.0f) out = -32768.0f;
    return (effect_t)out;
  }

private:
  void updateCoeffs() {
    float omega = 2.0f * (float)M_PI * freq_ / sampleRate_;
    float sn = sinf(omega);
    float cs = cosf(omega);
    float alpha = sn / (2.0f * q_);

    float a0 = 1.0f;
    if (mode_ == LOWPASS) {
      b0_ = (1.0f - cs) * 0.5f;
      b1_ = 1.0f - cs;
      b2_ = (1.0f - cs) * 0.5f;
      a0 = 1.0f + alpha;
      a1_ = -2.0f * cs;
      a2_ = 1.0f - alpha;
    } else {
      b0_ = (1.0f + cs) * 0.5f;
      b1_ = -(1.0f + cs);
      b2_ = (1.0f + cs) * 0.5f;
      a0 = 1.0f + alpha;
      a1_ = -2.0f * cs;
      a2_ = 1.0f - alpha;
    }

    float invA0 = 1.0f / a0;
    b0_ *= invA0;
    b1_ *= invA0;
    b2_ *= invA0;
    a1_ *= invA0;
    a2_ *= invA0;
  }

  Mode mode_;
  float freq_;
  float sampleRate_;
  float q_;
  float gainDb_ = 0.0f;
  float gainLinear_ = 1.0f;

  float b0_ = 1.0f, b1_ = 0.0f, b2_ = 0.0f, a1_ = 0.0f, a2_ = 0.0f;
  float x1_ = 0.0f, x2_ = 0.0f, y1_ = 0.0f, y2_ = 0.0f;

  SimpleBiquad* liveInstance_ = nullptr;
};

// --- ระบบสตรีมมิ่งพร้อมท่อ EQ (เริ่มต้นเกน 0 dB ทุกย่าน) ---
I2SStream i2sStreamIn;
I2SStream i2sStreamOut;

// ถังพักข้อมูลฝั่งรับ (32768 ไบต์)
BufferedStream bufferedIn(i2sStreamIn, 32768);

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
  if (bassFilter.activeInstance())   bassFilter.activeInstance()->setGainDb(targetBassGainDb);
  if (midFilter.activeInstance())    midFilter.activeInstance()->setGainDb(targetMidGainDb);
  if (trebleFilter.activeInstance()) trebleFilter.activeInstance()->setGainDb(targetTrebleGainDb);
}

// --- ฟังก์ชันรับข้อมูล ESP-NOW (ทำงานให้สั้นและเร็วที่สุด) ---
#if defined(ESP_ARDUINO_VERSION_MAJOR) && ESP_ARDUINO_VERSION_MAJOR >= 3
void OnDataRecv(const esp_now_recv_info_t * recv_info, const uint8_t *incomingDataPtr, int len) {
#else
void OnDataRecv(const uint8_t * mac_addr, const uint8_t *incomingDataPtr, int len) {
#endif
  if (len < sizeof(incomingData)) return;

  memcpy(&incomingData, incomingDataPtr, sizeof(incomingData));

  int safeMode = constrain(incomingData.controlMode, 0, 3);
  lastModeLog = safeMode;
  lastValLog  = incomingData.value;
  logDirty    = true;

  if (incomingData.isMuted != targetMuted) {
    targetMuted = incomingData.isMuted;
    volDirty = true;
  }

  if (incomingData.isMuted) return;

  switch (safeMode) {
    case 0: { // Master Volume (0-100 แปลงเป็น 0.0 - 1.0)
      targetVolume = constrain((float)incomingData.value / 100.0f, 0.0f, 1.0f);
      volDirty = true;
      break;
    }
    case 1: { // Bass Gain (-12 ถึง +12 dB)
      targetBassGainDb = constrain((float)incomingData.value, -12.0f, 12.0f);
      eqDirty = true;
      break;
    }
    case 2: { // Mid Gain (-12 ถึง +12 dB)
      targetMidGainDb = constrain((float)incomingData.value, -12.0f, 12.0f);
      eqDirty = true;
      break;
    }
    case 3: { // Treble Gain (-12 ถึง +12 dB)
      targetTrebleGainDb = constrain((float)incomingData.value, -12.0f, 12.0f);
      eqDirty = true;
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
  eqStream.addEffect(bassFilter);
  eqStream.addEffect(midFilter);
  eqStream.addEffect(trebleFilter);

  auto eqCfg = eqStream.defaultConfig();
  eqCfg.copyFrom(info);
  if (!eqStream.begin(eqCfg)) {
    Serial.println("[ERROR] EQ Stream begin failed!");
  }

  applyEqGains();
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

  bufferedIn.begin();
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
  volumeControl.setVolume(targetVolume);

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
  Serial.println(">> ESP32-S3 EQ Ready <<");
}

void loop() {
  copier.copy();

  if (volDirty) {
    volDirty = false;
    if (targetMuted) {
      volumeControl.setVolume(0.0);
    } else {
      volumeControl.setVolume(targetVolume);
    }
  }

  if (eqDirty) {
    eqDirty = false;
    applyEqGains();
  }

  if (logDirty) {
    logDirty = false;
    const char* modeNames[] = {"MASTER VOLUME", "BASS", "MID", "TREBLE"};
    Serial.printf("[ESP-NOW] Mode: %s | Value: %d | Mute: %s (B:%.1f M:%.1f T:%.1f)\n",
                  modeNames[lastModeLog],
                  lastValLog,
                  targetMuted ? "YES" : "NO",
                  targetBassGainDb,
                  targetMidGainDb,
                  targetTrebleGainDb);
  }

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
