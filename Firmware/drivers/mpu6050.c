#include "mpu6050.h"

#include <math.h>
#include <string.h>

#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "MPU6050";

/* MPU6050 register addresses */
#define REG_SMPLRT_DIV       0x19
#define REG_CONFIG            0x1A
#define REG_GYRO_CONFIG       0x1B
#define REG_ACCEL_CONFIG      0x1C
#define REG_ACCEL_XOUT_H      0x3B
#define REG_TEMP_OUT_H        0x41
#define REG_PWR_MGMT_1        0x6B
#define REG_WHO_AM_I          0x75

#define MPU6050_WHO_AM_I_VALUE 0x68

/* Full-scale configuration */
#define ACCEL_RANGE_2G        0x00
#define GYRO_RANGE_250DPS     0x00

#define COMPLEMENTARY_ALPHA   0.98f

static esp_err_t mpu6050_write_register(
    mpu6050_t *sensor,
    uint8_t reg,
    uint8_t value
)
{
    uint8_t buffer[2] = {reg, value};

    return i2c_master_transmit(
        sensor->device_handle,
        buffer,
        sizeof(buffer),
        1000
    );
}

static esp_err_t mpu6050_read_registers(
    mpu6050_t *sensor,
    uint8_t start_register,
    uint8_t *data,
    size_t length
)
{
    return i2c_master_transmit_receive(
        sensor->device_handle,
        &start_register,
        1,
        data,
        length,
        1000
    );
}

static int16_t combine_bytes(uint8_t high, uint8_t low)
{
    return (int16_t)(((uint16_t)high << 8) | low);
}

esp_err_t mpu6050_reset(mpu6050_t *sensor)
{
    if (sensor == NULL || !sensor->initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    esp_err_t err = mpu6050_write_register(
        sensor,
        REG_PWR_MGMT_1,
        0x80
    );

    if (err != ESP_OK) {
        return err;
    }

    vTaskDelay(pdMS_TO_TICKS(100));

    return mpu6050_write_register(
        sensor,
        REG_PWR_MGMT_1,
        0x01
    );
}

esp_err_t mpu6050_init(
    mpu6050_t *sensor,
    gpio_num_t sda_gpio,
    gpio_num_t scl_gpio,
    uint8_t i2c_address
)
{
    if (sensor == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    memset(sensor, 0, sizeof(mpu6050_t));

    i2c_master_bus_config_t bus_config = {
        .i2c_port = I2C_NUM_0,
        .sda_io_num = sda_gpio,
        .scl_io_num = scl_gpio,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true
    };

    esp_err_t err = i2c_new_master_bus(
        &bus_config,
        &sensor->bus_handle
    );

    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to create I2C bus: %s",
                 esp_err_to_name(err));
        return err;
    }

    i2c_device_config_t device_config = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = i2c_address,
        .scl_speed_hz = MPU6050_I2C_FREQUENCY_HZ,
        .scl_wait_us = 0,
        .flags.disable_ack_check = false
    };

    err = i2c_master_bus_add_device(
        sensor->bus_handle,
        &device_config,
        &sensor->device_handle
    );

    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to add MPU6050 device: %s",
                 esp_err_to_name(err));
        i2c_del_master_bus(sensor->bus_handle);
        return err;
    }

    sensor->initialized = true;

    uint8_t who_am_i = 0;

    err = mpu6050_read_registers(
        sensor,
        REG_WHO_AM_I,
        &who_am_i,
        1
    );

    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Could not read WHO_AM_I");
        return err;
    }

    if (who_am_i != MPU6050_WHO_AM_I_VALUE) {
        ESP_LOGE(
            TAG,
            "Unexpected WHO_AM_I value: 0x%02X",
            who_am_i
        );
        return ESP_ERR_NOT_FOUND;
    }

    err = mpu6050_reset(sensor);
    if (err != ESP_OK) {
        return err;
    }

    /*
     * Sample-rate divider:
     * Gyroscope output rate is 1 kHz when DLPF is enabled.
     * Divider 9 produces approximately 100 Hz.
     */
    err = mpu6050_write_register(
        sensor,
        REG_SMPLRT_DIV,
        9
    );

    if (err != ESP_OK) {
        return err;
    }

    /*
     * Digital low-pass filter.
     * 0x03 gives a moderate bandwidth and reduces vibration noise.
     */
    err = mpu6050_write_register(
        sensor,
        REG_CONFIG,
        0x03
    );

    if (err != ESP_OK) {
        return err;
    }

    /*
     * Gyroscope range: ±250 degrees/second.
     */
    err = mpu6050_write_register(
        sensor,
        REG_GYRO_CONFIG,
        GYRO_RANGE_250DPS
    );

    if (err != ESP_OK) {
        return err;
    }

    /*
     * Accelerometer range: ±2 g.
     */
    err = mpu6050_write_register(
        sensor,
        REG_ACCEL_CONFIG,
        ACCEL_RANGE_2G
    );

    if (err != ESP_OK) {
        return err;
    }

    sensor->filtered_pitch = 0.0f;
    sensor->filtered_roll = 0.0f;
    sensor->last_sample_time_us = esp_timer_get_time();

    ESP_LOGI(
        TAG,
        "MPU6050 initialized at address 0x%02X",
        i2c_address
    );

    return ESP_OK;
}

esp_err_t mpu6050_read(
    mpu6050_t *sensor,
    mpu6050_data_t *data
)
{
    if (sensor == NULL || data == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    if (!sensor->initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    uint8_t raw[14] = {0};

    esp_err_t err = mpu6050_read_registers(
        sensor,
        REG_ACCEL_XOUT_H,
        raw,
        sizeof(raw)
    );

    if (err != ESP_OK) {
        return err;
    }

    int16_t accel_x_raw = combine_bytes(raw[0], raw[1]);
    int16_t accel_y_raw = combine_bytes(raw[2], raw[3]);
    int16_t accel_z_raw = combine_bytes(raw[4], raw[5]);

    int16_t temp_raw = combine_bytes(raw[6], raw[7]);

    int16_t gyro_x_raw = combine_bytes(raw[8], raw[9]);
    int16_t gyro_y_raw = combine_bytes(raw[10], raw[11]);
    int16_t gyro_z_raw = combine_bytes(raw[12], raw[13]);

    /*
     * MPU6050 conversion factors:
     * ±2 g: 16384 LSB/g
     * ±250 dps: 131 LSB/(degrees/sec)
     */
    data->accel_x_g = accel_x_raw / 16384.0f;
    data->accel_y_g = accel_y_raw / 16384.0f;
    data->accel_z_g = accel_z_raw / 16384.0f;

    data->gyro_x_dps = gyro_x_raw / 131.0f;
    data->gyro_y_dps = gyro_y_raw / 131.0f;
    data->gyro_z_dps = gyro_z_raw / 131.0f;

    data->temperature_c =
        (temp_raw / 340.0f) + 36.53f;

    float accel_pitch = atan2f(
        data->accel_y_g,
        sqrtf(
            data->accel_x_g * data->accel_x_g +
            data->accel_z_g * data->accel_z_g
        )
    ) * 180.0f / (float)M_PI;

    float accel_roll = atan2f(
        -data->accel_x_g,
        data->accel_z_g
    ) * 180.0f / (float)M_PI;

    int64_t now_us = esp_timer_get_time();

    float dt =
        (float)(now_us - sensor->last_sample_time_us)
        / 1000000.0f;

    sensor->last_sample_time_us = now_us;

    if (dt <= 0.0f || dt > 0.5f) {
        dt = 0.01f;
    }

    float gyro_pitch =
        sensor->filtered_pitch +
        data->gyro_x_dps * dt;

    float gyro_roll =
        sensor->filtered_roll +
        data->gyro_y_dps * dt;

    sensor->filtered_pitch =
        COMPLEMENTARY_ALPHA * gyro_pitch +
        (1.0f - COMPLEMENTARY_ALPHA) * accel_pitch;

    sensor->filtered_roll =
        COMPLEMENTARY_ALPHA * gyro_roll +
        (1.0f - COMPLEMENTARY_ALPHA) * accel_roll;

    data->pitch_deg = sensor->filtered_pitch;
    data->roll_deg = sensor->filtered_roll;

    return ESP_OK;
}