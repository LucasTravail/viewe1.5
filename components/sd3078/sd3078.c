#include "sd3078.h"
#include "driver/i2c_master.h"
#include "esp_log.h"

static const char *TAG = "SD3078";
i2c_master_dev_handle_t sd3078_handle=NULL;
esp_err_t sd3078_init(void)
{
    // 配置SD3078的I2C设备
    i2c_device_config_t sd3078_config = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = SD3078_I2C_ADDR,
        .scl_speed_hz = 400000,
    };

    // 添加设备到I2C总线
    
    esp_err_t ret = i2c_master_bus_add_device(i2c_bus, &sd3078_config, &sd3078_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "无法将SD3078设备添加到I2C总线: %s", esp_err_to_name(ret));
        return ret;
    }

    // 检查设备ID
    uint8_t chip_id[8];
    uint8_t reg_chip_id_start = 0x72; // 芯片ID寄存器起始地址
    ret = i2c_master_transmit_receive(sd3078_handle, &reg_chip_id_start, 1, chip_id, 8, 1000 / portTICK_PERIOD_MS);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "无法读取SD3078芯片ID: %s", esp_err_to_name(ret));
        return ret;
    }

    // 打印芯片ID
    ESP_LOGI(TAG, "SD3078 芯片ID: %02X %02X %02X %02X %02X %02X %02X %02X", 
             chip_id[0], chip_id[1], chip_id[2], chip_id[3], 
             chip_id[4], chip_id[5], chip_id[6], chip_id[7]);

    ESP_LOGI(TAG, "SD3078 初始化成功!");
    return ESP_OK;
}

esp_err_t sd3078_read_register(i2c_master_dev_handle_t handle, uint8_t reg, uint8_t *data, size_t len) {
    esp_err_t ret = i2c_master_transmit_receive(handle, &reg, 1, data, len, 1000 / portTICK_PERIOD_MS);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "无法读取寄存器 0x%02x: %s", reg, esp_err_to_name(ret));
    }
    return ret;
}

esp_err_t sd3078_write_register(i2c_master_dev_handle_t handle, uint8_t reg, const uint8_t *data, size_t len) {
    uint8_t buffer[len + 1];
    buffer[0] = reg;
    memcpy(&buffer[1], data, len);
    
    esp_err_t ret = i2c_master_transmit(handle, buffer, len + 1, 1000 / portTICK_PERIOD_MS);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "无法写入寄存器 0x%02x: %s", reg, esp_err_to_name(ret));
    }
    return ret;
}

// 辅助函数：BCD编码转换为十进制
static uint8_t bcd_to_dec(uint8_t bcd) {
    return ((bcd >> 4) * 10) + (bcd & 0x0F);
}

// 辅助函数：十进制转换为BCD编码
static uint8_t dec_to_bcd(uint8_t dec) {
    return ((dec / 10) << 4) | (dec % 10);
}

esp_err_t sd3078_read_time(uint8_t *seconds, uint8_t *minutes, uint8_t *hours, uint8_t *weekday, uint8_t *day, uint8_t *month, uint8_t *year) {
    uint8_t time_data[7];
    esp_err_t ret = sd3078_read_register(sd3078_handle, 0x00, time_data, 7);
    if (ret != ESP_OK) {
        return ret;
    }

    *seconds = bcd_to_dec(time_data[0] & 0x7F); // 秒寄存器，最高位为0
    *minutes = bcd_to_dec(time_data[1] & 0x7F); // 分钟寄存器，最高位为0
    *hours = bcd_to_dec(time_data[2] & 0x1F);   // 小时寄存器，最高三位为0
    *weekday = bcd_to_dec(time_data[3] & 0x07); // 星期寄存器，最高五位为0
    *day = bcd_to_dec(time_data[4] & 0x3F);     // 日寄存器，最高两位为0
    *month = bcd_to_dec(time_data[5] & 0x1F);   // 月寄存器，最高三位为0
    *year = bcd_to_dec(time_data[6]);           // 年寄存器，无掩码

    return ESP_OK;
}

esp_err_t sd3078_write_time(uint8_t seconds, uint8_t minutes, uint8_t hours, uint8_t weekday, uint8_t day, uint8_t month, uint8_t year) {
    uint8_t time_data[7];
    time_data[0] = dec_to_bcd(seconds & 0x7F);  // 秒寄存器，最高位为0
    time_data[1] = dec_to_bcd(minutes & 0x7F);  // 分钟寄存器，最高位为0
    time_data[2] = dec_to_bcd(hours & 0x1F);    // 小时寄存器，最高位1为24小时，0为12小时 d5位为PM/AM控制位，AM为1是PM，0为AM
    time_data[3] = dec_to_bcd(weekday & 0x07);  // 星期寄存器，最高五位为0
    time_data[4] = dec_to_bcd(day & 0x3F);      // 日寄存器，最高两位为0
    time_data[5] = dec_to_bcd(month & 0x1F);    // 月寄存器，最高三位为0
    time_data[6] = dec_to_bcd(year);            // 年寄存器，无掩码

    // 设置写允许位
    uint8_t wrtc1 = 0x80;
    uint8_t wrtc23 = 0x84;
    sd3078_write_register(sd3078_handle, 0x10, &wrtc1, 1);
    sd3078_write_register(sd3078_handle, 0x0f, &wrtc23, 1);

    // 写入时间数据
    sd3078_write_register(sd3078_handle, 0x00, time_data, 7);

    // 清除写允许位
    uint8_t wrtcnull = 0x00;
    sd3078_write_register(sd3078_handle, 0x0f, &wrtcnull, 1);
    return sd3078_write_register(sd3078_handle, 0x10, &wrtcnull, 1);
}

esp_err_t sd3078_read_alarm(i2c_master_dev_handle_t handle, uint8_t *seconds, uint8_t *minutes, uint8_t *hours, uint8_t *weekday, uint8_t *day, uint8_t *month, uint8_t *year) {
    uint8_t alarm_data[7];
    esp_err_t ret = sd3078_read_register(handle, 0x07, alarm_data, 7);
    if (ret != ESP_OK) {
        return ret;
    }

    *seconds = alarm_data[0] & 0x7F; // 秒报警寄存器，最高位为0
    *minutes = alarm_data[1] & 0x7F; // 分钟报警寄存器，最高位为0
    *hours = alarm_data[2] & 0x3F;   // 小时报警寄存器，最高两位为0
    *weekday = alarm_data[3] & 0x07; // 星期报警寄存器，最高五位为0
    *day = alarm_data[4] & 0x3F;     // 日报警寄存器，最高两位为0
    *month = alarm_data[5] & 0x1F;   // 月报警寄存器，最高三位为0
    *year = alarm_data[6];           // 年报警寄存器，无掩码

    return ESP_OK;
}

esp_err_t sd3078_write_alarm(i2c_master_dev_handle_t handle, uint8_t seconds, uint8_t minutes, uint8_t hours, uint8_t weekday, uint8_t day, uint8_t month, uint8_t year) {
    uint8_t alarm_data[7];
    alarm_data[0] = seconds & 0x7F;  // 秒报警寄存器，最高位为0
    alarm_data[1] = minutes & 0x7F;  // 分钟报警寄存器，最高位为0
    alarm_data[2] = hours & 0x3F;    // 小时报警寄存器，最高两位为0
    alarm_data[3] = weekday & 0x07;  // 星期报警寄存器，最高五位为0
    alarm_data[4] = day & 0x3F;      // 日报警寄存器，最高两位为0
    alarm_data[5] = month & 0x1F;    // 月报警寄存器，最高三位为0
    alarm_data[6] = year;            // 年报警寄存器，无掩码

    return sd3078_write_register(handle, 0x07, alarm_data, 7);
}

esp_err_t sd3078_read_battery_voltage(float *voltage) {
    uint8_t bat8_val, bat_val;
    esp_err_t ret = sd3078_read_register(sd3078_handle, 0x1A, &bat8_val, 1);
    if (ret != ESP_OK) {
        return ret;
    }
    ret = sd3078_read_register(sd3078_handle, 0x1B, &bat_val, 1);
    if (ret != ESP_OK) {
        return ret;
    }

    uint16_t bat_voltage = ((bat8_val & 0x80) << 1) | bat_val;
    *voltage = bat_voltage / 100.0f;

    return ESP_OK;
}

esp_err_t sd3078_read_temperature(float *temperature) {
    uint8_t temp_val;
    esp_err_t ret = sd3078_read_register(sd3078_handle, 0x16, &temp_val, 1);
    if (ret != ESP_OK) {
        return ret;
    }

    // 解析温度值
    if (temp_val & 0x80) {
        // 负温度
        *temperature = (temp_val - 256);
    } else {
        // 正温度
        *temperature = temp_val;
    }

    return ESP_OK;
}

esp_err_t sd3078_set_charge_current() {
    uint8_t wrtc23 = 0x84;
    uint8_t wrtc1 = 0x80;
    uint8_t wrtcnull = 0x00;
    uint8_t charge =0x82;
    sd3078_write_register(sd3078_handle, 0x10, &wrtc1, 1);
    sd3078_write_register(sd3078_handle, 0x0f, &wrtc23, 1);

    sd3078_write_register(sd3078_handle, 0x18, &charge, 1);

    sd3078_write_register(sd3078_handle, 0x0f, &wrtcnull, 1);
    return sd3078_write_register(sd3078_handle, 0x10, &wrtcnull, 1);
    
}

esp_err_t sd3078_set_i2c_control(i2c_master_dev_handle_t handle, uint8_t bat_iic) {
    uint8_t i2c_control = (bat_iic & 0x01) << 7;
    return sd3078_write_register(handle, 0x17, &i2c_control, 1);
}

esp_err_t sd3078_set_extended_control(i2c_master_dev_handle_t handle, uint8_t ints_e2, uint8_t ints_e1, uint8_t ints_e0, uint8_t intbhe, uint8_t intble) {
    uint8_t ext_control = ((ints_e2 & 0x01) << 6) | ((ints_e1 & 0x01) << 5) | ((ints_e0 & 0x01) << 4) | ((intbhe & 0x01) << 1) | (intble & 0x01);
    return sd3078_write_register(handle, 0x19, &ext_control, 1);
}