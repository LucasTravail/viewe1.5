#ifndef INMP441_H
#define INMP441_H
#include "esp_log.h"
#include "driver/i2s_std.h"
#include "driver/gpio.h"
#define INMP441_SCK_GPIO    1   // INMP441 SCK（BCLK）
#define INMP441_SD_GPIO     2   // INMP441 SD（数据输出）
#define INMP441_WS_GPIO     42  // INMP441 WS（LRCLK）
#define INMP441_SAMPLE_RATE         16000   // 采样率16kHz
#define INMP441_BITS_PER_SAMPLE     I2S_DATA_BIT_WIDTH_24BIT // INMP441输出24位数据
extern i2s_chan_handle_t rx_chan; // INMP441的I²S接收通道
void inmp441_i2s_init(void);
#endif