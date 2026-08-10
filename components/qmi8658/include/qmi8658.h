#ifndef QMI8658_H
#define QMI8658_H
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <stdint.h>
#include <stdbool.h>
#include <esp_err.h> // 添加ESP-IDF的错误码定义头文件
#include "lvgl.h"
#include "driver/i2c_master.h"
// QMI8658 I2C地址
#define QMI8658_I2C_ADDR (0x6A)

// 配置陀螺仪
#define QMI8658_I2C_DEV_CONFIG()           \
{                                          \
    .dev_addr_length = I2C_ADDR_BIT_LEN_7, \
    .device_address = QMI8658_I2C_ADDR,    \
    .scl_speed_hz = 400000                 \
}                                          

// 寄存器地址定义
#define QMI8658_REG_CHIP_ID 0x00
#define QMI8658_REG_CTRL1 0x02
#define QMI8658_REG_ACCEL_X 0x35
#define QMI8658_REG_GYRO_X 0x3B

// 配置选项
#define QMI8658_ACCEL_RANGE_2G 0x00
#define QMI8658_ACCEL_RANGE_4G 0x01
#define QMI8658_ACCEL_RANGE_8G 0x02
#define QMI8658_ACCEL_RANGE_16G 0x03

#define QMI8658_GYRO_RANGE_250DPS 0x00
#define QMI8658_GYRO_RANGE_500DPS 0x01
#define QMI8658_GYRO_RANGE_1000DPS 0x02
#define QMI8658_GYRO_RANGE_2000DPS 0x03

extern i2c_master_bus_handle_t i2c_bus;
// 校准数据结构
typedef struct
{
    float accel_offset[3];
    float gyro_offset[3];
} qmi8658_calibration_t;

// 初始化QMI8658
esp_err_t qmi8658_init(void);
esp_err_t qmi8658_init_yaw_kalman(float initial_yaw);
// 读取加速度计和陀螺仪数据
esp_err_t qmi8658_read_data(float *accel_x, float *accel_y, float *accel_z, float *gyro_x, float *gyro_y, float *gyro_z);
esp_err_t qmi8658_calculate_euler_angles(float *roll, float *pitch, float *yaw);

// 校准函数
esp_err_t qmi8658_run_cod(void);
esp_err_t qmi8658_calibrate(void);
#endif // QMI8658_H