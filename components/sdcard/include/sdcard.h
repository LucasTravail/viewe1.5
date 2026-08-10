#ifndef SDCARD_H
#define SDCARD_H

#ifdef __cplusplus
extern "C"
{
#endif
#include "esp_vfs.h"
#include "esp_vfs_fat.h"
#include "sdmmc_cmd.h"
#include "driver/sdmmc_host.h"
#include "driver/gpio.h"
#include "dirent.h"
#include "lvgl.h"
#define SDCARD_PIN_CMD GPIO_NUM_38
#define SDCARD_PIN_CLK GPIO_NUM_39
#define SDCARD_PIN_D0 GPIO_NUM_40
#define SDCARD_PIN_CD GPIO_NUM_41 // SD卡检测IO
#define MOUNT_POINT "/sdcard"     // SD卡挂载路径
    extern sdmmc_card_t *card;
    void cardinit(void);
    // void sdlist_timer_cb(lv_timer_t *timer);
#ifdef __cplusplus
} /*extern "C"*/
#endif
#endif