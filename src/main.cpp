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

// ตัวแปรสถานะสำหรับรับค่าจาก ESP-NOW อย่างปลอดภัย
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

// --- คลาสฟิลเตอร์ EQ สเตอริโอ (แยกสถานะซ้าย L / ขวา R ขาดจากกัน) ---
class StereoEqBand : public AudioEffect {
public:
  enum BandType { LOW_SHELF, PEAKING, HIGH_SHELF };

  StereoEqBand(BandType type, float freq, float sampleRate, float q = 0.8f)
      : type_(type), freq_(freq), sampleRate_(sampleRate), q_(q) {
    setGainDb(0.0f);
    liveInstance_ = this;
  }

  StereoEqBand(const StereoEqBand& other) = default;

  AudioEffect* clone() override {
    StereoEqBand* copy = new StereoEqBand(*this);
    resetState(copy);
    liveInstance_ = copy;
    return copy;
  }

  StereoEqBand* activeInstance() {
    return liveInstance_ ? liveInstance_ : this;
  }

  void setGainDb(float db) {
    gainDb_ = db;
    updateCoeffs();
  }

  float getGainDb() const {
    return gainDb_;
  }

  effect_t process(effect_t input) override {
    if (!active() || fabsf(gainDb_) < 0.05f) {
      isLeftSample_ = !isLeftSample_;
      return input;
    }

    float x = (float)input;
    float y;

    if (!isLeftSample_) {
      // แซมเปิลฝั่งซ้าย (L)
      y = b0_ * x + b1_ * x1L_ + b2_ * x2L_ - a1_ * y1L_ - a2_ * y2L_;
      x2L_ = x1L_;
      x1L_ = x;
      y2L_ = y1L_;
      y1L_ = y;
    } else {
      // แซมเปิลฝั่งขวา (R)
      y = b0_ * x + b1_ * x1R_ + b2_ * x2R_ - a1_ * y1R_ - a2_ * y2R_;
      x2R_ = x1R_;
      x1R_ = x;
      y2R_ = y1R_;
      y1R_ = y;
    }

    // สลับข้าง L <-> R ทุกแซมเปิลตามโครงสร้าง Interleaved Stereo
    isLeftSample_ = !isLeftSample_;

    if (y > 32767.0f) y = 32767.0f;
    if (y < -32768.0f) y = -32768.0f;
    return (effect_t)y;
  }

private:
  static void resetState(StereoEqBand* band) {
    band->isLeftSample_ = false;
    band->x1L_ = band->x2L_ = band->y1L_ = band->y2L_ = 0.0f;
    band->x1R_ = band->x2R_ = band->y1R_ = band->y2R_ = 0.0f;
  }

  void updateCoeffs() {
    float A = powf(10.0f, gainDb_ / 40.0f);
    float omega = 2.0f * (float)M_PI * freq_ / sampleRate_;
    float sn = sinf(omega);
    float cs = cosf(omega);
    float alpha = sn / (2.0f * q_);

    float b0 = 1.0f, b1 = 0.0f, b2 = 0.0f, a0 = 1.0f, a1 = 0.0f, a2 = 0.0f;

    if (type_ == LOW_SHELF) {
      float twoSqrtAAlpha = 2.0f * sqrtf(A) * alpha;
      b0 = A * ((A + 1.0f) - (A - 1.0f) * cs + twoSqrtAAlpha);
      b1 = 2.0f * A * ((A - 1.0f) - (A + 1.0f) * cs);
      b2 = A * ((A + 1.0f) - (A - 1.0f) * cs - twoSqrtAAlpha);
      a0 = (A + 1.0f) + (A - 1.0f) * cs + twoSqrtAAlpha;
      a1 = -2.0f * ((A - 1.0f) + (A + 1.0f) * cs);
      a2 = (A + 1.0f) + (A - 1.0f) * cs - twoSqrtAAlpha;
    } else if (type_ == HIGH_SHELF) {
      float twoSqrtAAlpha = 2.0f * sqrtf(A) * alpha;
      b0 = A * ((A + 1.0f) + (A - 1.0f) * cs + twoSqrtAAlpha);
      b1 = -2.0f * A * ((A - 1.0f) + (A + 1.0f) * cs);
      b2 = A * ((A + 1.0f) + (A - 1.0f) * cs - twoSqrtAAlpha);
      a0 = (A + 1.0f) - (A - 1.0f) * cs + twoSqrtAAlpha;
      a1 = 2.0f * ((A - 1.0f) - (A + 1.0f) * cs);
      a2 = (A + 1.0f) - (A - 1.0f) * cs - twoSqrtAAlpha;
    } else {
      // PEAKING (ย่านกลาง)
      b0 = 1.0f + alpha * A;
      b1 = -2.0f * cs;
      b2 = 1.0f - alpha * A;
      a0 = 1.0f + alpha /
