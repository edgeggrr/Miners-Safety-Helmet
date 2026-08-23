#ifndef MQ2_H
#define MQ2_H

#include "esp_err.h"
#include "driver/gpio.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    adc_oneshot_unit_handle_t adc_handle;
    adc_channel_t channel;
    adc_cali_handle_t calibration_handle;

    int raw_value;
    int voltage_mv;

    bool calibration_enabled;
    bool initialized;
} mq2_t;

typedef struct {
    int raw_adc;
    int voltage_mv;
    float normalized_level;
} mq2_reading_t;

/**
 * Initialize MQ-2 on an ESP32-C3 ADC1 GPIO.
 *
 * For the XIAO ESP32C3, GPIO3 corresponds to ADC1 channel 3.
 */
esp_err_t mq2_init(
    mq2_t *sensor,
    gpio_num_t gpio
);

/**
 * Read the MQ-2 analog output.
 */
esp_err_t mq2_read(
    mq2_t *sensor,
    mq2_reading_t *reading
);

/**
 * Deinitialize the MQ-2 ADC resources.
 */
esp_err_t mq2_deinit(
    mq2_t *sensor
);

#ifdef __cplusplus
}
#endif

#endif