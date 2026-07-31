#include "AudioTools.h"
#include "AudioLibs/AudioSourceSDFAT.h"

I2SStream i2s;
VolumeStream volume(i2s);
SineWaveGenerator<int16_t> sine(1000);

void setup() {
  Serial.begin(115200);
  AudioLogger::instance().begin(Serial, AudioLogger::Info);

  auto cfg = i2s.defaultConfig(TX_MODE);
  cfg.pin_bck = 15;
  cfg.pin_ws = 16;
  cfg.pin_data = 17;
  cfg.pin_data2 = 18;
  cfg.sample_rate = 48000;
  cfg.channels = 4;
  cfg.bits_per_sample = 32;
  cfg.port_no = 0;
  i2s.begin(cfg);

  volume.begin(cfg);
  volume.setVolume(0.7);
  sine.begin(cfg, N_B4);
}

void loop() {
  // ตอนนี้จะปล่อยเสียง Sine 1kHz ออก 4CH เพื่อเทสว่า PCM 2 ตัวดังครบไหม
  // พอเทสผ่านแล้ว เราจะเปลี่ยนเป็นรับ Bluetooth / EQ ต่อ
  volume.write(sine.read(), 512);
}
