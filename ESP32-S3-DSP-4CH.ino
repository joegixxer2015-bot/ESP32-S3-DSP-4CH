// TEST DSP 4CH Pin 15,16,17,18 - แบบง่ายสุดให้ Build ผ่าน
#include <Arduino.h>
#include "driver/i2s_std.h"

i2s_chan_handle_t tx_chan;
void setup() {
  Serial.begin(115200);
  i2s_chan_config_t chan_cfg = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_0, I2S_ROLE_MASTER);
  i2s_new_channel(&chan_cfg, &tx_chan, NULL);
  i2s_std_config_t std_cfg = {
   .clk_cfg = I2S_STD_CLK_DEFAULT_CONFIG(48000),
   .slot_cfg = I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_32BIT, I2S_SLOT_MODE_STEREO),
   .gpio_cfg = {.mclk = I2S_GPIO_UNUSED,.bclk = GPIO_NUM_15,.ws = GPIO_NUM_16,.dout = GPIO_NUM_17,.din = I2S_GPIO_UNUSED,.invert_flags = {}}
  };
  i2s_channel_init_std_mode(tx_chan, &std_cfg);
  i2s_channel_enable(tx_chan);
}
void loop() {
  // ส่งเสียงเงียบๆไปก่อน
  int32_t data[4] = {0,0,0,0};
  size_t w; i2s_channel_write(tx_chan, data, sizeof(data), &w, 100);
}
