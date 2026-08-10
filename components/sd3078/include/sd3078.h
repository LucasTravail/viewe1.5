#ifndef SD3078_H
#define SD3078_H
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "qmi8658.h"
#define SD3078_I2C_ADDR (0x32)
// 配置陀螺仪
#define SD3078_I2C_DEV_CONFIG()           \
{                                          \
    .dev_addr_length = I2C_ADDR_BIT_LEN_7, \
    .device_address = SD3078_I2C_ADDR,    \
    .scl_speed_hz = 400000                 \
}   

// 初始化SD3078
esp_err_t sd3078_init(void);

// 读取寄存器
esp_err_t sd3078_read_register(i2c_master_dev_handle_t handle, uint8_t reg, uint8_t *data, size_t len);

// 写入寄存器
esp_err_t sd3078_write_register(i2c_master_dev_handle_t handle, uint8_t reg, const uint8_t *data, size_t len);

// 读取时间
esp_err_t sd3078_read_time(uint8_t *seconds, uint8_t *minutes, uint8_t *hours, uint8_t *weekday, uint8_t *day, uint8_t *month, uint8_t *year);

// 写入时间
esp_err_t sd3078_write_time(uint8_t seconds, uint8_t minutes, uint8_t hours, uint8_t weekday, uint8_t day, uint8_t month, uint8_t year);

// 读取报警时间
esp_err_t sd3078_read_alarm(i2c_master_dev_handle_t handle, uint8_t *seconds, uint8_t *minutes, uint8_t *hours, uint8_t *weekday, uint8_t *day, uint8_t *month, uint8_t *year);

// 写入报警时间
esp_err_t sd3078_write_alarm(i2c_master_dev_handle_t handle, uint8_t seconds, uint8_t minutes, uint8_t hours, uint8_t weekday, uint8_t day, uint8_t month, uint8_t year);

// 读取电池电压
esp_err_t sd3078_read_battery_voltage(float *voltage);

// 读取温度
esp_err_t sd3078_read_temperature(float *temperature);

// 设置充电电流
esp_err_t sd3078_set_charge_current();

// 设置IIC控制
esp_err_t sd3078_set_i2c_control(i2c_master_dev_handle_t handle, uint8_t bat_iic);

// 设置扩展控制
esp_err_t sd3078_set_extended_control(i2c_master_dev_handle_t handle, uint8_t ints_e2, uint8_t ints_e1, uint8_t ints_e0, uint8_t intbhe, uint8_t intble);

#endif