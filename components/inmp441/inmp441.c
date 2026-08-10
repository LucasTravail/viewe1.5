#include "inmp441.h"
i2s_chan_handle_t rx_chan; // INMP441的I²S接收通道
// 初始化INMP441的I²S接收
void inmp441_i2s_init(void) {
    // I²S通道配置
    i2s_chan_config_t rx_chan_cfg = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_0, I2S_ROLE_MASTER);
    ESP_ERROR_CHECK(i2s_new_channel(&rx_chan_cfg, NULL, &rx_chan));

    // 标准模式配置
    i2s_std_config_t std_cfg_rx = {
        .clk_cfg = I2S_STD_CLK_DEFAULT_CONFIG(INMP441_SAMPLE_RATE),
        .slot_cfg = I2S_STD_MSB_SLOT_DEFAULT_CONFIG(INMP441_BITS_PER_SAMPLE, I2S_SLOT_MODE_STEREO),
        .gpio_cfg = {
            .bclk = INMP441_SCK_GPIO,
            .ws = INMP441_WS_GPIO,
            .din = INMP441_SD_GPIO,
            .dout = I2S_GPIO_UNUSED,
            .mclk = I2S_GPIO_UNUSED,
            .invert_flags = {
                .mclk_inv = false,
                .bclk_inv = false,
                .ws_inv = false,
            },
        },
    };
    ESP_ERROR_CHECK(i2s_channel_init_std_mode(rx_chan, &std_cfg_rx));
    ESP_ERROR_CHECK(i2s_channel_enable(rx_chan));
    ESP_LOGI("inmp441", "inmp441 i2s初始化完成");
}