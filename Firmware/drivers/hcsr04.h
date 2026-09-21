#ifndef HCSR04_H
#define HCSR04_H

#include "esp_err.h"
#include "driver/gpio.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    gpio_num_t trigger_gpio;
    gpio_num_t echo_gpio;

    uint32_t timeout_us;
    float last_distance_cm;

    bool initialized;
} hcsr04_t;

typedef struct {
    float distance_cm;
    bool valid;
} hcsr04_reading_t;

esp_err_t hcsr04_init(
    hcsr04_t *sensor,
    gpio_num_t trigger_gpio,
    gpio_num_t echo_gpio
);

esp_err_t hcsr04_read(
    hcsr04_t *sensor,
    hcsr04_reading_t *reading
);

#ifdef __cplusplus
}
#endif

#endif