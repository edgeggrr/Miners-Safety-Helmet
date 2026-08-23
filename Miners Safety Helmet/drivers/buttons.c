#include "buttons.h"

#include <string.h>
#include "esp_timer.h"

#define BUTTON_ACTIVE_LEVEL       0
#define BUTTON_INACTIVE_LEVEL     1
#define BUTTON_DEBOUNCE_MS        50

static int64_t now_us(void)
{
    return esp_timer_get_time();
}

static esp_err_t configure_button_gpio(
    gpio_num_t gpio
)
{
    gpio_config_t config = {
        .pin_bit_mask = 1ULL << gpio,
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };

    return gpio_config(&config);
}

esp_err_t buttons_init(
    buttons_t *buttons,
    gpio_num_t trigger_gpio,
    gpio_num_t reset_gpio
)
{
    if (buttons == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    memset(buttons, 0, sizeof(buttons_t));

    esp_err_t err = configure_button_gpio(
        trigger_gpio
    );

    if (err != ESP_OK) {
        return err;
    }

    err = configure_button_gpio(reset_gpio);
    if (err != ESP_OK) {
        return err;
    }

    buttons->trigger_gpio = trigger_gpio;
    buttons->reset_gpio = reset_gpio;
    buttons->debounce_ms = BUTTON_DEBOUNCE_MS;

    buttons->trigger_last_level =
        gpio_get_level(trigger_gpio);

    buttons->reset_last_level =
        gpio_get_level(reset_gpio);

    buttons->trigger_last_change_us = now_us();
    buttons->reset_last_change_us = now_us();

    buttons->initialized = true;

    return ESP_OK;
}

static bool button_pressed_once(
    gpio_num_t gpio,
    int *last_level,
    int64_t *last_change_us,
    uint32_t debounce_ms
)
{
    int current_level = gpio_get_level(gpio);
    int64_t now = now_us();

    if (current_level != *last_level) {
        *last_level = current_level;
        *last_change_us = now;
        return false;
    }

    if (current_level == BUTTON_ACTIVE_LEVEL &&
        (now - *last_change_us) >=
        ((int64_t)debounce_ms * 1000)) {

        /*
         * Change the stored state so that one physical press
         * produces only one event.
         */
        *last_level = BUTTON_INACTIVE_LEVEL;
        *last_change_us = now;

        return true;
    }

    return false;
}

esp_err_t buttons_poll(
    buttons_t *buttons,
    button_event_t *event
)
{
    if (buttons == NULL || event == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    if (!buttons->initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    *event = BUTTON_EVENT_NONE;

    /*
     * Reset has priority over trigger.
     */
    if (button_pressed_once(
            buttons->reset_gpio,
            &buttons->reset_last_level,
            &buttons->reset_last_change_us,
            buttons->debounce_ms)) {

        *event = BUTTON_EVENT_RESET;
        return ESP_OK;
    }

    if (button_pressed_once(
            buttons->trigger_gpio,
            &buttons->trigger_last_level,
            &buttons->trigger_last_change_us,
            buttons->debounce_ms)) {

        *event = BUTTON_EVENT_TRIGGER;
        return ESP_OK;
    }

    return ESP_OK;
}