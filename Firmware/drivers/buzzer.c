#include "buzzer.h"

#include "esp_log.h"
#include "esp_timer.h"

static const char *TAG = "BUZZER";

#define BUZZER_FREQUENCY_HZ       1200
#define BUZZER_DUTY_PERCENT       50
#define BUZZER_DUTY_RESOLUTION    LEDC_TIMER_10_BIT

#define ALARM_TOGGLE_INTERVAL_MS  250

static uint32_t duty_from_percentage(
    uint32_t percentage
)
{
    uint32_t max_duty =
        (1U << BUZZER_DUTY_RESOLUTION) - 1U;

    return (max_duty * percentage) / 100U;
}

static esp_err_t buzzer_tone_on(
    buzzer_t *buzzer
)
{
    esp_err_t err = ledc_set_freq(
        LEDC_LOW_SPEED_MODE,
        buzzer->buzzer_timer,
        BUZZER_FREQUENCY_HZ
    );

    if (err != ESP_OK) {
        return err;
    }

    err = ledc_set_duty(
        LEDC_LOW_SPEED_MODE,
        buzzer->buzzer_channel,
        duty_from_percentage(BUZZER_DUTY_PERCENT)
    );

    if (err != ESP_OK) {
        return err;
    }

    return ledc_update_duty(
        LEDC_LOW_SPEED_MODE,
        buzzer->buzzer_channel
    );
}

static esp_err_t buzzer_tone_off(
    buzzer_t *buzzer
)
{
    esp_err_t err = ledc_set_duty(
        LEDC_LOW_SPEED_MODE,
        buzzer->buzzer_channel,
        0
    );

    if (err != ESP_OK) {
        return err;
    }

    return ledc_update_duty(
        LEDC_LOW_SPEED_MODE,
        buzzer->buzzer_channel
    );
}

esp_err_t buzzer_init(
    buzzer_t *buzzer,
    gpio_num_t buzzer_gpio,
    gpio_num_t led_gpio
)
{
    if (buzzer == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    buzzer->buzzer_gpio = buzzer_gpio;
    buzzer->led_gpio = led_gpio;
    buzzer->buzzer_channel = LEDC_CHANNEL_0;
    buzzer->buzzer_timer = LEDC_TIMER_0;
    buzzer->alarm_active = false;
    buzzer->led_state = false;
    buzzer->initialized = false;
    buzzer->toggle_interval_ms =
        ALARM_TOGGLE_INTERVAL_MS;
    buzzer->last_toggle_time_us =
        esp_timer_get_time();

    gpio_config_t led_config = {
        .pin_bit_mask = 1ULL << led_gpio,
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };

    esp_err_t err = gpio_config(&led_config);
    if (err != ESP_OK) {
        return err;
    }

    gpio_set_level(led_gpio, 0);

    ledc_timer_config_t timer_config = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .timer_num = buzzer->buzzer_timer,
        .duty_resolution = BUZZER_DUTY_RESOLUTION,
        .freq_hz = BUZZER_FREQUENCY_HZ,
        .clk_cfg = LEDC_AUTO_CLK
    };

    err = ledc_timer_config(&timer_config);
    if (err != ESP_OK) {
        return err;
    }

    ledc_channel_config_t channel_config = {
        .gpio_num = buzzer_gpio,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel = buzzer->buzzer_channel,
        .intr_type = LEDC_INTR_DISABLE,
        .timer_sel = buzzer->buzzer_timer,
        .duty = 0,
        .hpoint = 0
    };

    err = ledc_channel_config(&channel_config);
    if (err != ESP_OK) {
        return err;
    }

    buzzer->initialized = true;

    ESP_LOGI(
        TAG,
        "Buzzer initialized on GPIO%d; LED on GPIO%d",
        buzzer_gpio,
        led_gpio
    );

    return ESP_OK;
}

esp_err_t buzzer_update(
    buzzer_t *buzzer,
    bool alarm_active
)
{
    if (buzzer == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    if (!buzzer->initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    buzzer->alarm_active = alarm_active;

    if (!alarm_active) {
        return buzzer_stop(buzzer);
    }

    int64_t now_us = esp_timer_get_time();

    if ((now_us - buzzer->last_toggle_time_us) <
        ((int64_t)buzzer->toggle_interval_ms * 1000)) {
        return ESP_OK;
    }

    buzzer->last_toggle_time_us = now_us;
    buzzer->led_state = !buzzer->led_state;

    if (buzzer->led_state) {
        gpio_set_level(buzzer->led_gpio, 1);
        return buzzer_tone_on(buzzer);
    }

    gpio_set_level(buzzer->led_gpio, 0);
    return buzzer_tone_off(buzzer);
}

esp_err_t buzzer_stop(
    buzzer_t *buzzer
)
{
    if (buzzer == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    if (!buzzer->initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    buzzer->alarm_active = false;
    buzzer->led_state = false;

    gpio_set_level(buzzer->led_gpio, 0);

    return buzzer_tone_off(buzzer);
}