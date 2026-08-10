#include "qmi8658.h"
#include "esp_log.h"
#include "driver/i2c_master.h"
#include <math.h>
#include "esp_timer.h"

static const char *TAG = "QMI8658";
i2c_master_bus_handle_t i2c_bus = NULL;
static i2c_master_dev_handle_t qmi8658_handle = NULL;

// 校准偏移量
static float accel_x_offset = 0.0f, accel_y_offset = 0.0f, accel_z_offset = 0.0f;
static float gyro_x_offset = 0.0f, gyro_y_offset = 0.0f, gyro_z_offset = 0.0f;

// 卡尔曼滤波器状态
typedef struct {
    float q; // 过程噪声协方差（陀螺仪的信任程度）
    float r; // 测量噪声协方差（测量值的信任程度）
    float x; // 状态估计值（Yaw）
    float p; // 状态误差协方差
    float k; // 卡尔曼增益
} kalman_state_t;

static kalman_state_t yaw_kalman_filter;

// 初始化卡尔曼滤波器
void kalman_init(kalman_state_t *state, float initial_yaw, float process_noise, float measurement_noise) {
    state->q = process_noise;
    state->r = measurement_noise;
    state->x = initial_yaw;
    state->p = 1.0f; // 初始误差协方差
}

// 更新卡尔曼滤波器
float kalman_update(kalman_state_t *state, float gyro_z, float dt, float measured_yaw) {
    // 预测阶段
    state->x += gyro_z * dt; // 根据陀螺仪预测新的状态
    state->p += state->q;    // 更新预测误差协方差

    // 更新阶段
    state->k = state->p / (state->p + state->r); // 计算卡尔曼增益
    state->x += state->k * (measured_yaw - state->x); // 结合测量值进行校正
    state->p = (1 - state->k) * state->p; // 更新状态误差协方差

    return state->x; // 返回校正后的 Yaw 角
}

// 将角度归一化到 [-180°, +180°]
float normalize_yaw(float angle) {
    while (angle > 180.0f) angle -= 360.0f;
    while (angle <= -180.0f) angle += 360.0f;
    return angle;
}

// 初始化QMI8658
esp_err_t qmi8658_init(void)
{
    i2c_device_config_t qmi8658_config = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = QMI8658_I2C_ADDR,
        .scl_speed_hz = 400000,
    };

    ESP_ERROR_CHECK(i2c_master_bus_add_device(i2c_bus, &qmi8658_config, &qmi8658_handle));

    // 设置加速度计和陀螺仪的工作模式
    uint8_t buffer[2];
    buffer[0] = 0x02;
    buffer[1] = 0x40; // CTRL1: 自动递增模式
    ESP_ERROR_CHECK(i2c_master_transmit(qmi8658_handle, buffer, 2, 1000 / portTICK_PERIOD_MS));
    buffer[0] = 0x03;
    buffer[1] = 0x93; // CTRL2: 加速度计设置
    ESP_ERROR_CHECK(i2c_master_transmit(qmi8658_handle, buffer, 2, 1000 / portTICK_PERIOD_MS));
    buffer[0] = 0x04;
    buffer[1] = 0xd3; // CTRL3: 陀螺仪设置
    ESP_ERROR_CHECK(i2c_master_transmit(qmi8658_handle, buffer, 2, 1000 / portTICK_PERIOD_MS));
    buffer[0] = 0x06;
    buffer[1] = 0x00; // CTRL4: 默认配置
    ESP_ERROR_CHECK(i2c_master_transmit(qmi8658_handle, buffer, 2, 1000 / portTICK_PERIOD_MS));
    buffer[0] = 0x08;
    buffer[1] = 0x03; // CTRL5: 启用加速度计和陀螺仪
    ESP_ERROR_CHECK(i2c_master_transmit(qmi8658_handle, buffer, 2, 1000 / portTICK_PERIOD_MS));

    // 检查设备温度
    float qmi8658_temp;
    uint8_t chip_temp[2];
    uint8_t reg_chip_temp = 0x33; // 温度寄存器地址
    ESP_ERROR_CHECK(i2c_master_transmit_receive(qmi8658_handle, &reg_chip_temp, 1, chip_temp, 2, 1000 / portTICK_PERIOD_MS));
    qmi8658_temp = chip_temp[1] + (chip_temp[0] / 256.0f);
    ESP_LOGE(TAG, "QMI8658温度: %.2f℃", qmi8658_temp);

    // 检查设备ID
    uint8_t chip_id;
    uint8_t reg_chip_id = QMI8658_REG_CHIP_ID; // 芯片ID寄存器地址
    esp_err_t ret = i2c_master_transmit_receive(qmi8658_handle, &reg_chip_id, 1, &chip_id, 1, 1000 / portTICK_PERIOD_MS);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "读取芯片ID失败: %s", esp_err_to_name(ret));
        return ret;
    }
    if (chip_id != 0x05)
    {
        ESP_LOGE(TAG, "QMI8658芯片ID不匹配: 0x%02x", chip_id);
        return ESP_ERR_NOT_FOUND;
    }
    ESP_LOGI(TAG, "QMI8658 初始化成功！");
    return ESP_OK;
}

// 读取加速度计和陀螺仪数据
esp_err_t qmi8658_read_data(float *accel_x, float *accel_y, float *accel_z, float *gyro_x, float *gyro_y, float *gyro_z)
{
    if (!qmi8658_handle)
    {
        ESP_LOGE(TAG, "I2C设备句柄为NULL");
        return ESP_ERR_INVALID_STATE;
    }

    uint8_t data[12];
    uint8_t reg_accel_x = QMI8658_REG_ACCEL_X; // 加速度计X轴寄存器地址
    esp_err_t ret = i2c_master_transmit_receive(qmi8658_handle, &reg_accel_x, 1, data, 12, 1000 / portTICK_PERIOD_MS);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "I2C事务失败: %s", esp_err_to_name(ret));
        return ret;
    }

    int16_t raw_accel_x = (int16_t)((data[1] << 8) | data[0]);
    int16_t raw_accel_y = (int16_t)((data[3] << 8) | data[2]);
    int16_t raw_accel_z = (int16_t)((data[5] << 8) | data[4]);

    int16_t raw_gyro_x = (int16_t)((data[7] << 8) | data[6]);
    int16_t raw_gyro_y = (int16_t)((data[9] << 8) | data[8]);
    int16_t raw_gyro_z = (int16_t)((data[11] << 8) | data[10]);

    // 单位转换
    float accel_scale = 4.0f / 32768.0f;   // ±4g量程
    float gyro_scale = 2000.0f / 32768.0f; // ±2000°/s量程

    if (accel_x)
        *accel_x = raw_accel_x * accel_scale - accel_x_offset;
    if (accel_y)
        *accel_y = raw_accel_y * accel_scale - accel_y_offset;
    if (accel_z)
        *accel_z = raw_accel_z * accel_scale - accel_z_offset;

    if (gyro_x)
        *gyro_x = raw_gyro_x * gyro_scale - gyro_x_offset;
    if (gyro_y)
        *gyro_y = raw_gyro_y * gyro_scale - gyro_y_offset;
    if (gyro_z)
        *gyro_z = raw_gyro_z * gyro_scale - gyro_z_offset;

    return ESP_OK;
}

// 执行按需校准（COD）
esp_err_t qmi8658_run_cod(void)
{
    ESP_LOGI(TAG, "开始按需校准（COD）...");

    uint8_t buffer[2];
    buffer[0] = 0x07; // CTRL7 寄存器地址
    buffer[1] = 0x00; // aEN = 0, gEN = 0
    ESP_ERROR_CHECK(i2c_master_transmit(qmi8658_handle, buffer, 2, 1000 / portTICK_PERIOD_MS));

    buffer[0] = 0x09; // CTRL9 寄存器地址
    buffer[1] = 0xA2; // CTRL_CMD_ON_DEMAND_CALIBRATION
    ESP_ERROR_CHECK(i2c_master_transmit(qmi8658_handle, buffer, 2, 1000 / portTICK_PERIOD_MS));
    vTaskDelay(pdMS_TO_TICKS(1500));

    uint8_t cod_status;
    uint8_t reg_cod_status = 0x46; // COD_STATUS 寄存器地址
    ESP_ERROR_CHECK(i2c_master_transmit_receive(qmi8658_handle, &reg_cod_status, 1, &cod_status, 1, 1000 / portTICK_PERIOD_MS));

    if (cod_status != 0x00)
    {
        ESP_LOGE(TAG, "按需校准失败: COD_STATUS = 0x%02x", cod_status);
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "按需校准成功！");

    uint8_t gain_data[6];
    uint8_t reg_gain_start = 0x51; // dVX_L 寄存器地址
    ESP_ERROR_CHECK(i2c_master_transmit_receive(qmi8658_handle, &reg_gain_start, 1, gain_data, 6, 1000 / portTICK_PERIOD_MS));

    ESP_LOGI(TAG, "新增益参数: Gyro-X gain = 0x%02x%02x, Gyro-Y gain = 0x%02x%02x, Gyro-Z gain = 0x%02x%02x",
             gain_data[1], gain_data[0], gain_data[3], gain_data[2], gain_data[5], gain_data[4]);

    buffer[0] = 0x07; // CTRL7 寄存器地址
    buffer[1] = 0x03; // aEN = 1, gEN = 1
    ESP_ERROR_CHECK(i2c_master_transmit(qmi8658_handle, buffer, 2, 1000 / portTICK_PERIOD_MS));

    return ESP_OK;
}

// 初始化卡尔曼滤波器
esp_err_t qmi8658_init_yaw_kalman(float initial_yaw) {
    kalman_init(&yaw_kalman_filter, initial_yaw, 0.001f, 0.1f); // 初始化参数可以根据需要调整
    return ESP_OK;
}

// 计算欧拉角（Roll, Pitch, Yaw）
esp_err_t qmi8658_calculate_euler_angles(float *roll, float *pitch, float *yaw)
{
    if (!qmi8658_handle)
    {
        ESP_LOGE(TAG, "I2C设备句柄为NULL");
        return ESP_ERR_INVALID_STATE;
    }

    // 读取加速度计和陀螺仪数据
    float accel_x, accel_y, accel_z;
    float gyro_x, gyro_y, gyro_z;
    esp_err_t ret = qmi8658_read_data(&accel_x, &accel_y, &accel_z, &gyro_x, &gyro_y, &gyro_z);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "读取传感器数据失败: %s", esp_err_to_name(ret));
        return ret;
    }

    // 计算 Roll 和 Pitch（基于加速度计数据）
    float roll_angle = atan2(accel_y, accel_z) * (180.0f / M_PI); // 单位：度
    float pitch_angle = atan2(-accel_x, sqrt(accel_y * accel_y + accel_z * accel_z)) * (180.0f / M_PI);

    // 计算 Yaw（基于陀螺仪数据，简单积分）
    static uint32_t last_time = 0; // 上一次的时间戳
    uint32_t current_time = esp_timer_get_time() / 1000; // 当前时间（毫秒）

    if (last_time == 0) {
        last_time = current_time; // 初始化时间戳
    }

    float dt = (current_time - last_time) / 1000.0f; // 时间差（秒）
    last_time = current_time;

    // 使用陀螺仪积分作为测量值
    static float previous_yaw = 0.0f;
    float measured_yaw = previous_yaw + gyro_z * dt; // 简单积分作为测量值

    // 使用卡尔曼滤波器校正 Yaw
    float corrected_yaw = kalman_update(&yaw_kalman_filter, gyro_z, dt, measured_yaw);

    // 更新上一次的 Yaw
    previous_yaw = corrected_yaw;

    // 归一化 Yaw 角
    corrected_yaw = normalize_yaw(corrected_yaw);

    // 返回结果
    if (roll) *roll = roll_angle;
    if (pitch) *pitch = pitch_angle;
    if (yaw) *yaw = corrected_yaw;

    // 打印欧拉角信息
    // ESP_LOGI(TAG, "欧拉角: Roll = %.2f°, Pitch = %.2f°, Yaw = %.2f°", roll_angle, pitch_angle, corrected_yaw);

    return ESP_OK;
}

// 校准函数
esp_err_t qmi8658_calibrate(void)
{
    float accel_x_sum = 0, accel_y_sum = 0, accel_z_sum = 0;

    const int calibration_samples = 100;

    // 采集校准数据
    for (int i = 0; i < calibration_samples; i++)
    {
        float accel_x, accel_y, accel_z;

        if (qmi8658_read_data(&accel_x, &accel_y, &accel_z, NULL, NULL, NULL) != ESP_OK)
        {
            ESP_LOGE(TAG, "校准时读取数据失败");
            return ESP_FAIL;
        }

        accel_x_sum += accel_x;
        accel_y_sum += accel_y;
        accel_z_sum += accel_z;

        vTaskDelay(pdMS_TO_TICKS(10)); // 等待10ms
    }

    // 计算平均值作为偏移量
    accel_x_offset = accel_x_sum / calibration_samples;
    accel_y_offset = accel_y_sum / calibration_samples;
    accel_z_offset = accel_z_sum / calibration_samples; // 减去重力加速度

    // 保存校准数据
    ESP_LOGI(TAG, "校准完成。偏移量: Accel X: %.2fg, Y: %.2fg, Z: %.2fg;",
             accel_x_offset, accel_y_offset, accel_z_offset);

    return ESP_OK;
}