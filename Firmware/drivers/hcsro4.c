#include "hcsr04.h"

#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "HC-SR04";

#define HCSR04_SOUND_SPEED_CM_PER_US 0.0343f
#define HCSR04_MAX_DISTANCE_CM       400.0f
#define HCSR04_MIN_DISTANCE_CM       2.0f
#define HCSR04_DEFAULT_TIMEOUT_US    30000UL

static int64_t now_us(void)
{
    return esp_timer_get_time();
}

static esp_err_t wait_for_level(
    gpio_num_t gpio,
    int level,
    uint32_t timeout_us
)
{
    int64_t start = now_us();

    while (gpio_get_level(gpio) != level) {
        if ((uint32_t)(now_us() - start) >= timeout_us) {
            return ESP_ERR_TIMEOUT;
        }
    }

    return ESP_OK;
}

esp_err_t hcsr04_init(
    hcsr04_t *sensor,
    gpio_num_t trigger_gpio,
    gpio_num_t echo_gpio
)
{
    if (sensor == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    sensor->trigger_gpio = trigger_gpio;
    sensor->echo_gpio = echo_gpio;
    sensor->timeout_us = HCSR04_DEFAULT_TIMEOUT_US;
    sensor->last_distance_cm = -1.0f;
    sensor->initialized = false;

    gpio_config_t trigger_config = {
        .pin_bit_mask = 1ULL << trigger_gpio,
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };

    esp_err_t err = gpio_config(&trigger_config);
    if (err != ESP_OK) {
        return err;
    }

    gpio_config_t echo_config = {
        .pin_bit_mask = 1ULL << echo_gpio,
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };

    err = gpio_config(&echo_config);
    if (err != ESP_OK) {
        return err;
    }

    gpio_set_level(trigger_gpio, 0);

    sensor->initialized = true;

    ESP_LOGI(
        TAG,
        "Initialized: TRIG GPIO%d, ECHO GPIO%d",
        trigger_gpio,
        echo_gpio
    );

    return ESP_OK;
}

esp_err_t hcsr04_read(
    hcsr04_t *sensor,
    hcsr04_reading_t *reading
)
{
    if (sensor == NULL || reading == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    if (!sensor->initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    reading->distance_cm = -1.0f;
    reading->valid = false;

    /*
     * Ensure the trigger line is low before starting.
     */
    gpio_set_level(sensor->trigger_gpio, 0);
    ets_delay_us(2);

    /*
     * HC-SR04 requires a trigger pulse of at least 10 us.
     */
    gpio_set_level(sensor->trigger_gpio, 1);
    ets_delay_us(10);
    gpio_set_level(sensor->trigger_gpio, 0);

    /*
     * Wait for the Echo signal to become high.
     */
    esp_err_t err = wait_for_level(
        sensor->echo_gpio,
        1,
        sensor->timeout_us
    );

    if (err != ESP_OK) {
        ESP_LOGW(TAG, "Echo rising-edge timeout");
        return err;
    }

    int64_t echo_start = now_us();

    /*
     * Wait for Echo to return low.
     */
    err = wait_for_level(
        sensor->echo_gpio,
        0,
        sensor->timeout_us
    );

    if (err != ESP_OK) {
        ESP_LOGW(TAG, "Echo falling-edge timeout");
        return err;
    }

    uint32_t echo_time_us =
        (uint32_t)(now_us() - echo_start);

    float distance_cm =
        (echo_time_us * HCSR04_SOUND_SPEED_CM_PER_US) / 2.0f;

    if (distance_cm < HCSR04_MIN_DISTANCE_CM ||
        distance_cm > HCSR04_MAX_DISTANCE_CM) {
        ESP_LOGW(
            TAG,
            "Distance out of range: %.2f cm",
            distance_cm
        );
        return ESP_ERR_INVALID_RESPONSE;
    }

    reading->distance_cm = distance_cm;
    reading->valid = true;
    sensor->last_distance_cm = distance_cm;

    return ESP_OK;
}