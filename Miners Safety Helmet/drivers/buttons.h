#ifndef BUTTONS_H
#define BUTTONS_H

#include <stdbool.h>
#include "esp_err.h"
#include "driver/gpio.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    BUTTON_EVENT_NONE = 0,
    BUTTON_EVENT_TRIGGER,
    BUTTON_EVENT_RESET
} button_event_t;

typedef struct {
    gpio_num_t trigger_gpio;
    gpio_num_t reset_gpio;

    uint32_t debounce_ms;

    int trigger_last_level;
    int reset_last_level;

    int64_t trigger_last_change_us;
    int64_t reset_last_change_us;

    bool initialized;
} buttons_t;

esp_err_t buttons_init(
    buttons_t *buttons,
    gpio_num_t trigger_gpio,
    gpio_num_t reset_gpio
);

esp_err_t buttons_poll(
    buttons_t *buttons,
    button_event_t *event
);

#ifdef __cplusplus
}
#endif

#endif