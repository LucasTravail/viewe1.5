#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "driver/gpio_filter.h"
#include "esp_heap_caps.h"
#include "esp_err.h"
#include "esp_check.h"
#include "esp_log.h"
#include "esp_lcd_panel_io_interface.h"
#include "esp_lcd_panel_ops.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_co5300.h"
#include "qmi8658.h"
#include "sd3078.h"
#include "ns4168.h"
#include "sdcard.h"
#include "inmp441.h"
#include "rgbled.h"
#include "driver/i2c_master.h"
#include "esp_lcd_touch_cst816s.h"
#include "esp_lcd_touch.h"
#include "driver/pulse_cnt.h"
#include "esp_timer.h"
#include "lvgl.h"
#define LV_TICK_PERIOD_MS 1

#define LCD_HOST (SPI2_HOST)
#define LCD_H_RES (471) // 宽
#define LCD_V_RES (466) // 高
#define LCD_BIT_PER_PIXEL (16)

#define PIN_NUM_LCD_CS (GPIO_NUM_12)
#define PIN_NUM_LCD_PCLK (GPIO_NUM_48)
#define PIN_NUM_LCD_DATA0 (GPIO_NUM_13)
#define PIN_NUM_LCD_DATA1 (GPIO_NUM_47)
#define PIN_NUM_LCD_DATA2 (GPIO_NUM_21)
#define PIN_NUM_LCD_DATA3 (GPIO_NUM_14)
#define PIN_NUM_LCD_RST (GPIO_NUM_11)
#define PIN_NUM_LCD_DC (GPIO_NUM_NC)
#define LCD_BK_LIGHT_ON_LEVEL 1
#define LCD_BK_LIGHT_OFF_LEVEL !LCD_BK_LIGHT_ON_LEVEL
#define PIN_NUM_BK_LIGHT (GPIO_NUM_45)
// 触摸设置
#define TOUCH_I2C_NUM (0)
#define TOUCH_I2C_CLK_HZ (400000)

// 触摸引脚
#define TOUCH_I2C_SCL (GPIO_NUM_18)
#define TOUCH_I2C_SDA (GPIO_NUM_17)
#define TOUCH_I2C_RST (GPIO_NUM_10)
#define TOUCH_GPIO_INT (GPIO_NUM_9)
// 编码器
#define PCNT_HIGH_LIMIT 100
#define PCNT_LOW_LIMIT -100
#define ENCODER_GPIO_A (GPIO_NUM_3)
#define ENCODER_GPIO_B (GPIO_NUM_8)
// 按钮
#define ENCODER_KEY (GPIO_NUM_0)
static char *TAG = "co5300测试";
int pulse_count = 0;
int event_count = 0;
SemaphoreHandle_t lvglmutex;
pcnt_unit_handle_t pcnt_unit = NULL;

static const co5300_lcd_init_cmd_t lcd_init_cmds[] = {
    {0xFE, (uint8_t[]){0x00}, 0, 0},
    {0xC4, (uint8_t[]){0x80}, 1, 0},
    {0x3A, (uint8_t[]){0x55}, 1, 0},
    {0x35, (uint8_t[]){0x00}, 1, 10},
    {0x53, (uint8_t[]){0x20}, 1, 10}, // 亮度控制开关
    {0x51, (uint8_t[]){0xFF}, 1, 10}, // 亮度0-ff
    {0x63, (uint8_t[]){0xFF}, 1, 10},
    {0x2A, (uint8_t[]){0x00, 0x06, 0x01, 0xDC}, 4, 0}, // 列设置，从0-470
    {0x2B, (uint8_t[]){0x00, 0x00, 0x01, 0xD1}, 4, 0}, // 行设置，从0-465
    {0x11, (uint8_t[]){0x00}, 0, 60},
    {0x29, (uint8_t[]){0x00}, 0, 0},
};
extern lv_obj_t *btn;
extern void lvgl_demo_ui(lv_disp_t *disp);
extern void btn_cb(lv_event_t *e);
void encoderinit(void)
{
    ESP_LOGI(TAG, "安装PCNT单元");
    pcnt_unit_config_t unit_config = {
        .high_limit = PCNT_HIGH_LIMIT,
        .low_limit = PCNT_LOW_LIMIT,
    };
    ESP_ERROR_CHECK(pcnt_new_unit(&unit_config, &pcnt_unit));

    ESP_LOGI(TAG, "设置毛刺过滤器");
    pcnt_glitch_filter_config_t filter_config = {
        .max_glitch_ns = 1000,
    };
    ESP_ERROR_CHECK(pcnt_unit_set_glitch_filter(pcnt_unit, &filter_config));

    ESP_LOGI(TAG, "安装PCNT通道");

    pcnt_chan_config_t chan_a_config = {
        .edge_gpio_num = ENCODER_GPIO_A,
        .level_gpio_num = ENCODER_GPIO_B,
    };
    pcnt_channel_handle_t pcnt_a_chan = NULL;
    ESP_ERROR_CHECK(pcnt_new_channel(pcnt_unit, &chan_a_config, &pcnt_a_chan));

    ESP_LOGI(TAG, "设置 PCNT 通道的边缘和水平动作");
    ESP_ERROR_CHECK(pcnt_channel_set_edge_action(pcnt_a_chan, PCNT_CHANNEL_EDGE_ACTION_INCREASE, PCNT_CHANNEL_EDGE_ACTION_DECREASE));
    ESP_ERROR_CHECK(pcnt_channel_set_level_action(pcnt_a_chan, PCNT_CHANNEL_LEVEL_ACTION_KEEP, PCNT_CHANNEL_LEVEL_ACTION_INVERSE));

    ESP_LOGI(TAG, "启用 PCNT 单元");
    ESP_ERROR_CHECK(pcnt_unit_enable(pcnt_unit));
    ESP_LOGI(TAG, "清除 pcnt 单元");
    ESP_ERROR_CHECK(pcnt_unit_clear_count(pcnt_unit));
    ESP_LOGI(TAG, "启动 PCNT 单元");
    ESP_ERROR_CHECK(pcnt_unit_start(pcnt_unit));
}

static bool notify_lvgl_flush_ready(esp_lcd_panel_io_handle_t panel_io, esp_lcd_panel_io_event_data_t *edata, void *user_ctx)
{
    lv_disp_drv_t *disp_driver = (lv_disp_drv_t *)user_ctx;
    lv_disp_flush_ready(disp_driver);
    return false;
}
static void lvgl_flush_cb(lv_disp_drv_t *drv, const lv_area_t *area, lv_color_t *color_map)
{
    esp_lcd_panel_handle_t panel_handle = (esp_lcd_panel_handle_t)drv->user_data;
    int offsetx1, offsetx2, offsety1, offsety2;
    offsetx1 = area->x1;
    offsetx2 = area->x2;
    offsety1 = area->y1;
    offsety2 = area->y2;
    esp_lcd_panel_draw_bitmap(panel_handle, offsetx1, offsety1, offsetx2 + 1, offsety2 + 1, color_map);
}
static void lvgl_rounder_cb(struct _lv_disp_drv_t *disp_drv, lv_area_t *area)
{
    uint16_t x1 = area->x1;
    uint16_t x2 = area->x2;
    uint16_t y1 = area->y1;
    uint16_t y2 = area->y2;

    // 将 Area 的 start 向下舍入到最接近的偶数
    area->x1 = (x1 >> 1) << 1;
    area->y1 = (y1 >> 1) << 1;

    // 将 End of Area 向上舍入到最接近的奇数
    area->x2 = ((x2 >> 1) << 1) + 1;
    area->y2 = ((y2 >> 1) << 1) + 1;
}
static void lvgl_update_cb(lv_disp_drv_t *drv)
{
    esp_lcd_panel_handle_t panel_handle = (esp_lcd_panel_handle_t)drv->user_data;
    switch (drv->rotated)
    {
    case LV_DISP_ROT_NONE:
        esp_lcd_panel_swap_xy(panel_handle, false);
        esp_lcd_panel_mirror(panel_handle, false, false);
        break;
    case LV_DISP_ROT_90:
        esp_lcd_panel_swap_xy(panel_handle, true);
        esp_lcd_panel_mirror(panel_handle, false, true);
        break;
    case LV_DISP_ROT_180:
        esp_lcd_panel_swap_xy(panel_handle, false);
        esp_lcd_panel_mirror(panel_handle, true, true);
        break;
    case LV_DISP_ROT_270:
        esp_lcd_panel_swap_xy(panel_handle, true);
        esp_lcd_panel_mirror(panel_handle, true, false);
        break;
    }
}
void touch_driver_read(lv_indev_drv_t *drv, lv_indev_data_t *data)
{
    uint16_t touchpad_x[1] = {0};
    uint16_t touchpad_y[1] = {0};
    uint8_t touchpad_cnt = 0;

    /* 将数据从触摸控制器读取到内存中 */
    esp_lcd_touch_read_data(drv->user_data);

    /* 从触摸控制器读取数据 */
    bool touchpad_pressed = esp_lcd_touch_get_coordinates(drv->user_data, touchpad_x, touchpad_y, NULL, &touchpad_cnt, 1);

    if (touchpad_pressed && touchpad_cnt > 0)
    {
        data->point.x = touchpad_x[0];
        data->point.y = touchpad_y[0];
        data->state = LV_INDEV_STATE_PRESSED;
        ESP_LOGI(TAG, "x:%d,y:%d", data->point.x, data->point.y);
    }
    else
    {
        data->state = LV_INDEV_STATE_RELEASED;
    }
}
int enc_pressed(void)
{
    if (gpio_get_level(ENCODER_KEY) == 0)
    {
        return 1;
    }
    else
    {
        return 0;
    }
}
void encoder_read_cb(lv_indev_drv_t *drv, lv_indev_data_t *data)
{
    pcnt_unit_get_count(pcnt_unit, &pulse_count);
    static int32_t last_encoded = 0;
    int32_t encoded = pulse_count;

    // 处理编码器旋转
    int32_t diff = last_encoded - encoded;
    if (diff < 0)
    {
        // 编码器逆时针旋转
        data->enc_diff = -1;
        ESP_LOGI(TAG, "脉冲计数: %d", pulse_count);
    }
    else if (diff > 0)
    {
        // 编码器顺时针旋转
        data->enc_diff = 1;
        ESP_LOGI(TAG, "脉冲计数: %d", pulse_count);
    }
    last_encoded = encoded;
    // 检测编码器按钮状态
    static bool was_pressed;
    float roll, pitch, yaw;
    esp_err_t ret = qmi8658_calculate_euler_angles(&roll, &pitch, &yaw);
    if (ret == ESP_OK)
    {
        if (pitch > 50)
        {
            ESP_LOGI(TAG, "姿态: Roll = %.2f°, Pitch = %.2f°, Yaw = %.2f°", roll, pitch, yaw);
            was_pressed = false;
            data->state = LV_INDEV_STATE_PRESSED;
            if (!was_pressed)
            {
                // 直接调用按钮的回调函数，模拟按钮点击
                lv_event_t e;
                e.user_data = lv_disp_get_default(); // 使用默认显示器作为用户数据
                e.target = btn;
                btn_cb(&e); // 手动调用按钮回调函数
            }
            was_pressed = true;
        }
    }
    else
    {
        ESP_LOGE(TAG, "计算欧拉角失败");
    }
    if (enc_pressed())
    {
        data->state = LV_INDEV_STATE_PRESSED;
        ESP_LOGI(TAG, "编码器按钮按下");

        // 如果是首次按下，则触发按钮回调
        was_pressed = false;
        if (!was_pressed)
        {
            // 直接调用按钮的回调函数，模拟按钮点击
            lv_event_t e;
            e.user_data = lv_disp_get_default(); // 使用默认显示器作为用户数据
            e.target = btn;
            btn_cb(&e); // 手动调用按钮回调函数
        }
        was_pressed = true;
    }
    else
    {
        data->state = LV_INDEV_STATE_RELEASED;
        was_pressed = false; // 重置按下状态
    }
}

static void lv_tick_task(void *arg)
{
    lv_tick_inc(LV_TICK_PERIOD_MS);
}
static void lvgl_port_task(void *arg)
{
    ESP_LOGI(TAG, "启动 LVGL 任务");
    // app_touch_init();
    gpio_set_direction(ENCODER_KEY, GPIO_MODE_INPUT_OUTPUT);
    gpio_set_level(GPIO_NUM_0, 1);
    gpio_set_direction(ENCODER_GPIO_A, GPIO_MODE_INPUT);
    gpio_set_direction(ENCODER_GPIO_B, GPIO_MODE_INPUT);
    encoderinit();
    static lv_disp_draw_buf_t disp_buf;
    static lv_disp_drv_t disp_drv;

#if PIN_NUM_BK_LIGHT >= 0
    ESP_LOGI(TAG, "打开 LCD 背光");
    gpio_config_t bk_gpio_config = {
        .mode = GPIO_MODE_OUTPUT,
        .pin_bit_mask = 1ULL << PIN_NUM_BK_LIGHT};
    ESP_ERROR_CHECK(gpio_config(&bk_gpio_config));
    gpio_set_level(PIN_NUM_BK_LIGHT, LCD_BK_LIGHT_ON_LEVEL);
#endif

    ESP_LOGI(TAG, "初始化 QSPI 总线");
    const spi_bus_config_t buscfg = CO5300_PANEL_BUS_QSPI_CONFIG(PIN_NUM_LCD_PCLK,
                                                                 PIN_NUM_LCD_DATA0,
                                                                 PIN_NUM_LCD_DATA1,
                                                                 PIN_NUM_LCD_DATA2,
                                                                 PIN_NUM_LCD_DATA3,
                                                                 LCD_H_RES * LCD_V_RES * LCD_BIT_PER_PIXEL / 8);
    ESP_ERROR_CHECK(spi_bus_initialize(LCD_HOST, &buscfg, SPI_DMA_CH_AUTO));

    ESP_LOGI(TAG, "安装面板 IO");
    esp_lcd_panel_io_handle_t io_handle = NULL;
    const esp_lcd_panel_io_spi_config_t io_config = CO5300_PANEL_IO_QSPI_CONFIG(PIN_NUM_LCD_CS, notify_lvgl_flush_ready, &disp_drv);
    // 将 LCD 连接到 SPI 总线
    ESP_ERROR_CHECK(esp_lcd_new_panel_io_spi((esp_lcd_spi_bus_handle_t)LCD_HOST, &io_config, &io_handle));

    ESP_LOGI(TAG, "安装 CO5300 的 LCD 驱动程序");
    esp_lcd_panel_handle_t panel_handle = NULL;
    const co5300_vendor_config_t vendor_config = {
        .init_cmds = lcd_init_cmds,
        .init_cmds_size = sizeof(lcd_init_cmds) / sizeof(lcd_init_cmds[0]),
        .flags = {
            .use_qspi_interface = 1,
        },
    };
    const esp_lcd_panel_dev_config_t panel_config = {
        .reset_gpio_num = PIN_NUM_LCD_RST,
        .rgb_ele_order = LCD_RGB_ELEMENT_ORDER_RGB,
        .bits_per_pixel = LCD_BIT_PER_PIXEL,
        .vendor_config = (void *)&vendor_config,
    };
    ESP_ERROR_CHECK(esp_lcd_new_panel_co5300(io_handle, &panel_config, &panel_handle));
    esp_lcd_panel_reset(panel_handle);
    esp_lcd_panel_init(panel_handle);
    esp_lcd_panel_disp_on_off(panel_handle, true);

#if PIN_NUM_BK_LIGHT >= 0
    ESP_LOGI(TAG, "打开 LCD 背光");
    gpio_set_level(PIN_NUM_BK_LIGHT, LCD_BK_LIGHT_ON_LEVEL);
#endif
    /* 初始化 I2C */
    gpio_config_t touch_rst_config = {
        .mode = GPIO_MODE_OUTPUT,
        .pin_bit_mask = 1ULL << TOUCH_I2C_RST};
    ESP_ERROR_CHECK(gpio_config(&touch_rst_config));
    gpio_config_t touch_int_config = {
        .mode = GPIO_MODE_INPUT_OUTPUT,
        .pin_bit_mask = 1ULL << TOUCH_GPIO_INT};
    ESP_ERROR_CHECK(gpio_config(&touch_int_config));
    gpio_set_level(TOUCH_GPIO_INT, 0);
    gpio_set_level(TOUCH_I2C_RST, 0);
    vTaskDelay(200);
    gpio_set_level(TOUCH_I2C_RST, 1);
    vTaskDelay(200);
    esp_lcd_panel_io_handle_t tp_io_handle = NULL;
    esp_lcd_touch_handle_t tp = NULL;

    const i2c_master_bus_config_t i2c_conf = {
        .i2c_port = 0,
        .sda_io_num = TOUCH_I2C_SDA,
        .scl_io_num = TOUCH_I2C_SCL,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };
    ESP_ERROR_CHECK(i2c_new_master_bus(&i2c_conf, &i2c_bus));

    uint8_t address;
    printf("     0  1  2  3  4  5  6  7  8  9  a  b  c  d  e  f\r\n");
    for (int i = 0; i < 128; i += 16)
    {
        printf("%02x: ", i);
        for (int j = 0; j < 16; j++)
        {
            fflush(stdout);
            address = i + j;
            esp_err_t ret = i2c_master_probe(i2c_bus, address, pdMS_TO_TICKS(500));
            if (ret == ESP_OK)
            {
                printf("%02x ", address);
            }
            else if (ret == ESP_ERR_TIMEOUT)
            {
                printf("UU ");
            }
            else
            {
                printf("-- ");
            }
        }
        printf("\r\n");
    }

    // 初始化触摸屏
    esp_lcd_panel_io_i2c_config_t tp_config = ESP_LCD_TOUCH_IO_I2C_CST816S_CONFIG();
    tp_config.scl_speed_hz = TOUCH_I2C_CLK_HZ;
    esp_lcd_touch_config_t tp_cfg = {
        .x_max = LCD_H_RES,
        .y_max = LCD_V_RES,
        .rst_gpio_num = -1,
        .int_gpio_num = -1,
        .levels = {
            .reset = 0,
            .interrupt = 0,
        },
        .flags = {
            .swap_xy = 0,
            .mirror_x = 0,
            .mirror_y = 0,
        },
    };
    ESP_ERROR_CHECK(esp_lcd_new_panel_io_i2c(i2c_bus, &tp_config, &tp_io_handle));
    ESP_ERROR_CHECK(esp_lcd_touch_new_i2c_cst816s(tp_io_handle, &tp_cfg, &tp));
    // 初始化陀螺仪
    if (qmi8658_init() != ESP_OK)
    {
        return;
    }
    qmi8658_run_cod(); // 按需校准陀螺仪。
    // qmi8658_calibrate();//校准加速度仪。
    ESP_ERROR_CHECK(qmi8658_init_yaw_kalman(0.0f));
    ESP_LOGI(TAG, "初始化 LVGL 库");
    lv_init();
    void *buf1 = NULL;
    void *buf2 = NULL;
    ESP_LOGI(TAG, "从 PSRAM 分配单独的 LVGL 绘制缓冲区");
    buf1 = heap_caps_malloc(LCD_H_RES * 48 * sizeof(lv_color_t), MALLOC_CAP_DMA);
    // buf2 = heap_caps_malloc(LCD_H_RES * 48 * sizeof(lv_color_t), MALLOC_CAP_DMA);
    assert(buf1);
    // assert(buf2);
    // 初始化LVGL缓存
    lv_disp_draw_buf_init(&disp_buf, buf1, buf2, LCD_H_RES * 48);

    ESP_LOGI(TAG, "将显示驱动程序注册到 LVGL");
    lv_disp_drv_init(&disp_drv);
    disp_drv.hor_res = LCD_H_RES;
    disp_drv.ver_res = LCD_V_RES;
    disp_drv.flush_cb = lvgl_flush_cb;
    disp_drv.rounder_cb = lvgl_rounder_cb;
    disp_drv.drv_update_cb = lvgl_update_cb;
    disp_drv.sw_rotate = 0;
    disp_drv.draw_buf = &disp_buf;
    disp_drv.user_data = panel_handle;
    lv_disp_t *disp = lv_disp_drv_register(&disp_drv);

    /*在LVGL中注册触摸屏*/
    static lv_indev_drv_t indev_drv;
    lv_indev_drv_init(&indev_drv);          /*输入设备基本初始化*/
    indev_drv.type = LV_INDEV_TYPE_POINTER; /*输入设备类型.*/
    indev_drv.disp = disp;
    indev_drv.user_data = tp;
    indev_drv.read_cb = touch_driver_read; /*回调函数.*/
    /*在LVGL中注册驱动程序并保存创建的输入设备对象*/
    lv_indev_drv_register(&indev_drv);
    /*在LVGL中注册编码器*/
    lv_indev_drv_t encoder;
    lv_indev_drv_init(&encoder);
    encoder.type = LV_INDEV_TYPE_ENCODER;
    encoder.read_cb = encoder_read_cb;
    lv_indev_t *ec_button = lv_indev_drv_register(&encoder);
    lv_point_t btn_points[1] = {
        {230, 300},
    };
    lv_indev_set_button_points(ec_button, btn_points);
    ESP_LOGI(TAG, "安装 LVGL 滴答计时器");
    const esp_timer_create_args_t lvgl_tick_timer_args = {
        .callback = &lv_tick_task,
        .name = "lvgl_tick"};
    esp_timer_handle_t lvgl_tick_timer = NULL;
    ESP_ERROR_CHECK(esp_timer_create(&lvgl_tick_timer_args, &lvgl_tick_timer));
    ESP_ERROR_CHECK(esp_timer_start_periodic(lvgl_tick_timer, LV_TICK_PERIOD_MS * 1000));
    sd3078_init();
    sd3078_set_charge_current();

    // 定义变量以存储时间信息
    uint8_t seconds = 0, minutes = 0, hours = 0;
    uint8_t weekday = 0, day = 0, month = 0, year = 0;
    // sd3078_write_time(seconds, minutes, hours, weekday, day, month, year);
    // 调用 sd3078_read_time 函数，传递变量地址
    sd3078_read_time(&seconds, &minutes, &hours, &weekday, &day, &month, &year);

    // 打印或处理读取的时间信息（可选）
    ESP_LOGI(TAG, "时间: %02d:%02d:%02d, 日期: 20%02d/%02d/%02d, 星期: %d",
             hours, minutes, seconds, year, month, day, weekday);
    float sd3078temper;
    sd3078_read_temperature(&sd3078temper);
    ESP_LOGI(TAG, "SD3078温度: %.2f℃", sd3078temper);
    float sd3078voltage;
    sd3078_read_battery_voltage(&sd3078voltage);
    ESP_LOGI(TAG, "SD3078电池电压: %.2fV", sd3078voltage);
    cardinit();
    ns4168_gpio_init();
    ns4168_i2s_init();
    rgbledinit();
    // inmp441_i2s_init();
    ESP_LOGI(TAG, "显示 LVGL 界面");
    lvgl_demo_ui(disp);
    while (1)
    {
        xSemaphoreTake(lvglmutex, portMAX_DELAY);
        lv_timer_handler();
        xSemaphoreGive(lvglmutex);
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}
void app_main(void)
{

    lvglmutex = xSemaphoreCreateMutex();
    ESP_LOGI(TAG, "创建 LVGL 任务");
    xTaskCreatePinnedToCoreWithCaps(lvgl_port_task, "LVGL", 1024 * 1024, NULL, 2, NULL, 1, MALLOC_CAP_SPIRAM);
    vTaskDelete(NULL);
}