#ifndef MPU6050_H
#define MPU6050_H

#include <stdbool.h>
#include "esp_err.h"
#include "driver/gpio.h"
#include "driver/i2c_master.h"

#ifdef __cplusplus
extern "C" {
#endif

#define MPU6050_I2C_ADDRESS        0x68
#define MPU6050_SDA_GPIO           GPIO_NUM_6
#define MPU6050_SCL_GPIO           GPIO_NUM_7
#define MPU6050_I2C_FREQUENCY_HZ   100000

typedef struct {
    float accel_x_g;
    float accel_y_g;
    float accel_z_g;

    float gyro_x_dps;
    float gyro_y_dps;
    float gyro_z_dps;

    float temperature_c;

    float pitch_deg;
    float roll_deg;
} mpu6050_data_t;

typedef struct {
    i2c_master_bus_handle_t bus_handle;
    i2c_master_dev_handle_t device_handle;

    float filtered_pitch;
    float filtered_roll;

    int64_t last_sample_time_us;
    bool initialized;
} mpu6050_t;

esp_err_t mpu6050_init(
    mpu6050_t *sensor,
    gpio_num_t sda_gpio,
    gpio_num_t scl_gpio,
    uint8_t i2c_address
);

esp_err_t mpu6050_read(
    mpu6050_t *sensor,
    mpu6050_data_t *data
);

esp_err_t mpu6050_reset(
    mpu6050_t *sensor
);

#ifdef __cplusplus
}
#endif

#endif