#include <Arduino.h>
#include "AudioTools.h"

// กำหนด Pin I2S
#define I2S_BCK_PIN   15   // BCK พ่วงเข้า PCM5102A ทั้ง 2 ตัว
#define I2S_LRCK_PIN  16   // LCK พ่วงเข้า PCM5102A ทั้ง 2 ตัว
#define I2S_DATA1_PIN 17   // DIN1 (DAC ตัวที่ 1)
#define I2S_DATA2_PIN 18   // DIN2 (DAC ตัวที่ 2)

AudioInfo info(44100, 2, 16);
SineWaveGenerator<int16_t> sineWave(32000); // ความดังสัญญาณ
GeneratedSoundStream<int16_t> sound(sineWave);
I2SStream i2s;
StreamCopy copier(i2s, sound);

void setup() {
  Serial.begin(115200);
  
  // Configure I2S Config
  auto config = i2s.defaultConfig(TX_MODE);
  config.copyFrom(info);
  config.pin_bck = I2S_BCK_PIN;
  config.pin_ws  = I2S_LRCK_PIN;
  config.pin_data = I2S_DATA1_PIN; // Primary DIN 1
  
  i2s.begin(config);

  // ต่อสัญญาณ DIN2 (GPIO 18) ให้ขับข้อมูลคู่ขนานกับ DIN1 (GPIO 17)
  esp_rom_gpio_connect_out_signal(I2S_DATA2_PIN, i2s_periph_signal[I2S_NUM_0].data_out_sigs[0], false, false);

  // กำหนดความถี่เสียงทดสอบ 440Hz (เสียงตัว โน้ต A)
  sineWave.begin(info, N_A4);
  Serial.println(">> Audio Test Stream Started on GPIO 15, 16, 17, 18 <<");
}

void loop() {
  // ยิงสัญญาณ Sine Wave ไปยัง DAC ต่อเนื่อง
  copier.copy();
}
