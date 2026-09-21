
#ifndef BUZZER_H
#define BUZZER_H

#include <stdbool.h>
#include "esp_err.h"
#include "driver/gpio.h"
#include "driver/ledc.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    gpio_num_t buzzer_gpio;
    gpio_num_t led_gpio;

    ledc_channel_t buzzer_channel;
    ledc_timer_t buzzer_timer;

    bool alarm_active;
    bool led_state;
    bool initialized;

    int64_t last_toggle_time_us;
    uint32_t toggle_interval_ms;
} buzzer_t;

/**
 * Initialize the buzzer and alert LED.
 */
esp_err_t buzzer_init(
    buzzer_t *buzzer,
    gpio_num_t buzzer_gpio,
    gpio_num_t led_gpio
);

/**
 * Update the alarm output.
 *
 * When alarm_active is true:
 * - The buzzer produces the full alarm tone.
 * - The LED flashes.
 *
 * No short pre-alarm beeps are generated.
 */
esp_err_t buzzer_update(
    buzzer_t *buzzer,
    bool alarm_active
);

/**
 * Stop the buzzer and turn off the alert LED.
 */
esp_err_t buzzer_stop(
    buzzer_t *buzzer
);

#ifdef __cplusplus
}
#endif

#endif