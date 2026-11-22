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

  SimpleBiquad(Mode mode, float freq, float sampleRate, int effectId, float q = 0.707f)
      : mode_(mode), freq_(freq), sampleRate_(sampleRate), q_(q) {
    setId(effectId); // กำหนด ID เพื่อใช้อ้างอิงตัวโคลนข้างใน AudioEffectStream
    updateCoeffs();
    setGainDb(0.0f);
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
    // ถ้าปิดใช้งานหรือเกนใกล้ 0 dB ให้ปล่อยผ่านทันที ลดภาระ CPU
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
      
