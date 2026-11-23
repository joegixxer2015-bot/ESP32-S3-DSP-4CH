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
      a0 = 1.0f + alpha / A;
      a1 = -2.0f * cs;
      a2 = 1.0f - alpha / A;
    }

    float invA0 = 1.0f / a0;
    b0_ = b0 * invA0;
    b1_ = b1 * invA0;
    b2_ = b2 * invA0;
    a1_ = a1 * invA0;
    a2_ = a2 * invA0;
  }

  BandType type_;
  float freq_;
  float sampleRate_;
  float q_;
  float gainDb_ = 0.0f;

  float b0_ = 1.0f, b1_ = 0.0f, b2_ = 0.0f, a1_ = 0.0f, a2_ = 0.0f;

  bool isLeftSample_ = false;
  float x1L_ = 0.0f, x2L_ = 0.0f, y1L_ = 0.0f, y2L_ = 0.0f;
  float x1R_ = 0.0f, x2R_ = 0.0f, y1R_ = 0.0f, y2R_ = 0.0f;

  StereoEqBand* liveInstance_ = nullptr;
};

// --- ระบบสตรีมมิ่งพร้อมท่อ EQ แบบปรีแอมป์ (เริ่มต้น 0 dB ทุกย่าน) ---
I2SStream i2sStreamIn;
I2SStream i2sStreamOut;

BufferedStream bufferedIn(i2sStreamIn, 65536);
AudioEffectStream eqStream(bufferedIn);

// จูนเบสที่ 125Hz (Q=0.7) เพื่อดึงเนื้อเบสให้ชัดขึ้น ส่วน Mid 1000Hz และ Treble 6500Hz คงฟีลเดิม
StereoEqBand bassEq(StereoEqBand::LOW_SHELF, 125.0f, (float)info.sample_rate, 0.7f);
StereoEqBand midEq(StereoEqBand::PEAKING, 1000.0f, (float)info.sample_rate, 0.9f);
StereoEqBand trebleEq(StereoEqBand::HIGH_SHELF, 6500.0f, (float)info.sample_rate, 0.75f);

VolumeStream volumeControl(eqStream);
StreamCopy copier(i2sStreamOut, volumeControl);

unsigned long lastKeepAliveCheck = 0;

void applyEqGains() {
  if (bassEq.activeInstance())   bassEq.activeInstance()->setGainDb(targetBassGainDb);
  if (midEq.activeInstance())    midEq.activeInstance()->setGainDb(targetMidGainDb);
  if (trebleEq.activeInstance()) trebleEq.activeInstance()->setGainDb(targetTrebleGainDb);
}

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
    case 0: {
      targetVolume = constrain((float)incomingData.value / 100.0f, 0.0f, 1.0f);
      volDirty = true;
      break;
    }
    case 1: {
      // ขยายช่วงเบสเป็น -15 ถึง +15 dB ให้เนื้อมาชัดขึ้น
      targetBassGainDb = constrain((float)incomingData.value, -15.0f, 15.0f);
      eqDirty = true;
      break;
    }
    case 2: {
      targetMidGainDb = constrain((float)incomingData.value, -12.0f, 12.0f);
      eqDirty = true;
      break;
    }
    case 3: {
      targetTrebleGainDb = constrain((float)incomingData.value, -12.0f, 12.0f);
      eqDirty = true;
      break;
    }
  }
}

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
  eqStream.addEffect(bassEq);
  eqStream.addEffect(midEq);
  eqStream.addEffect(trebleEq);

  auto eqCfg = eqStream.defaultConfig();
  eqCfg.copyFrom(info);
  if (!eqStream.begin(eqCfg)) {
    Serial.println("[ERROR] EQ Stream begin failed!");
  }

  applyEqGains();
}

void setupI2S() {
  auto configIn = i2sStreamIn.defaultConfig(RX_MODE);
  configIn.copyFrom(info);
  configIn.rx_tx_mode = RX_MODE;
  configIn.port_no = 0;
  configIn.pin_bck = I2S_IN_BCK;
  configIn.pin_ws  = I2S_IN_LRCK;
  configIn.pin_data = I2S_IN_DATA;
  configIn.is_master = false;
  if (!i2sStreamIn.begin(configIn)) {
    Serial.println("[ERROR] I2S RX (Port 0) begin failed!");
  }

  bufferedIn.begin();
  setupEQ();

  auto configOut = i2sStreamOut.defaultConfig(TX_MODE);
  configOut.copyFrom(info);
  configOut.rx_tx_mode = TX_MODE;
  configOut.port_no = 1;
  configOut.pin_bck = I2S_BCK_PIN;
  configOut.pin_ws  = I2S_LRCK_PIN;
  configOut.pin_data = I2S_DATA1_PIN;
  if (!i2sStreamOut.begin(configOut)) {
    Serial.println("[ERROR] I2S TX (Port 1) begin failed!");
  }

  gpio_matrix_out(I2S_DATA2_PIN, i2s_periph_signal[I2S_NUM_1].data_out_sig, false, false);

  auto vconfig = volumeControl.defaultConfig();
  vconfig.copyFrom(info);
  volumeControl.begin(vconfig);
  volumeControl.setVolume(targetVolume);

  copier.begin(i2sStreamOut, volumeControl);
}

void setup() {
  Serial.begin(115200);
  delay(1000);

  setupI2S();
  // พัก Wi-Fi และ ESP-NOW ชั่วคราวเพื่อตัดตัวแปรคลื่น 2.4 GHz กวนเสียง
  // initWifiAndEspNow();

  // Serial.print("ESP32-S3 STA MAC Address: ");
  // Serial.println(WiFi.macAddress());
  Serial.println(">> ESP32-S3 Preamp EQ Ready (Wi-Fi/ESP-NOW OFF) <<");
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

  delay(1);
}
