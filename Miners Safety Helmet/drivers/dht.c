#include "dht.h"

#include <string.h>
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "DHT22";

#define DHT22_START_LOW_US          1200
#define DHT22_START_RELEASE_US      40
#define DHT22_RESPONSE_TIMEOUT_US   150
#define DHT22_BIT_TIMEOUT_US        100
#define DHT22_MIN_INTERVAL_MS       2000

static int64_t time_us(void)
{
    return esp_timer_get_time();
}

static esp_err_t wait_for_level(
    gpio_num_t gpio,
    int expected_level,
    uint32_t timeout_us
)
{
    int64_t start = time_us();

    while (gpio_get_level(gpio) != expected_level) {
        if ((uint32_t)(time_us() - start) > timeout_us) {
            return ESP_ERR_TIMEOUT;
        }
    }

    return ESP_OK;
}

static esp_err_t measure_level_duration(
    gpio_num_t gpio,
    int level,
    uint32_t timeout_us,
    uint32_t *duration_us
)
{
    esp_err_t err;
    int64_t start;

    err = wait_for_level(gpio, level, timeout_us);
    if (err != ESP_OK) {
        return err;
    }

    start = time_us();

    err = wait_for_level(gpio, !level, timeout_us);
    if (err != ESP_OK) {
        return err;
    }

    *duration_us = (uint32_t)(time_us() - start);
    return ESP_OK;
}

esp_err_t dht22_init(dht22_t *sensor, gpio_num_t gpio)
{
    if (sensor == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    memset(sensor, 0, sizeof(dht22_t));

    sensor->gpio = gpio;
    sensor->timeout_us = DHT22_BIT_TIMEOUT_US;
    sensor->min_read_interval_ms = DHT22_MIN_INTERVAL_MS;
    sensor->last_read_time_us = 0;

    gpio_config_t config = {
        .pin_bit_mask = 1ULL << gpio,
        .mode = GPIO_MODE_INPUT_OUTPUT_OD,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };

    esp_err_t err = gpio_config(&config);
    if (err != ESP_OK) {
        return err;
    }

    gpio_set_level(gpio, 1);

    sensor->initialized = true;

    ESP_LOGI(TAG, "DHT22 initialized on GPIO%d", gpio);
    return ESP_OK;
}

bool dht22_is_ready(const dht22_t *sensor)
{
    if (sensor == NULL || !sensor->initialized) {
        return false;
    }

    int64_t elapsed_us = time_us() - sensor->last_read_time_us;
    return elapsed_us >=
           ((int64_t)sensor->min_read_interval_ms * 1000);
}

esp_err_t dht22_read(
    dht22_t *sensor,
    dht22_data_t *data
)
{
    if (sensor == NULL || data == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    if (!sensor->initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    if (!dht22_is_ready(sensor)) {
        return ESP_ERR_INVALID_STATE;
    }

    uint8_t bytes[5] = {0};
    uint32_t duration_us = 0;
    esp_err_t err;

    /*
     * Start signal:
     * Host pulls DATA low for at least 1 ms,
     * then releases it and waits for the sensor response.
     */
    gpio_set_direction(sensor->gpio, GPIO_MODE_OUTPUT_OD);
    gpio_set_level(sensor->gpio, 0);

    ets_delay_us(DHT22_START_LOW_US);

    gpio_set_level(sensor->gpio, 1);
    gpio_set_direction(sensor->gpio, GPIO_MODE_INPUT);

    ets_delay_us(DHT22_START_RELEASE_US);

    /*
     * Sensor response:
     * approximately 80 us low, followed by 80 us high.
     */
    err = measure_level_duration(
        sensor->gpio,
        0,
        DHT22_RESPONSE_TIMEOUT_US,
        &duration_us
    );

    if (err != ESP_OK) {
        gpio_set_level(sensor->gpio, 1);
        return err;
    }

    err = measure_level_duration(
        sensor->gpio,
        1,
        DHT22_RESPONSE_TIMEOUT_US,
        &duration_us
    );

    if (err != ESP_OK) {
        gpio_set_level(sensor->gpio, 1);
        return err;
    }

    /*
     * Each bit begins with approximately 50 us low.
     * The following high pulse determines the bit value:
     *
     *   approximately 26–28 us = 0
     *   approximately 70 us    = 1
     */
    for (int bit_index = 0; bit_index < 40; bit_index++) {
        err = measure_level_duration(
            sensor->gpio,
            0,
            sensor->timeout_us,
            &duration_us
        );

        if (err != ESP_OK) {
            gpio_set_level(sensor->gpio, 1);
            return err;
        }

        err = measure_level_duration(
            sensor->gpio,
            1,
            sensor->timeout_us,
            &duration_us
        );

        if (err != ESP_OK) {
            gpio_set_level(sensor->gpio, 1);
            return err;
        }

        bytes[bit_index / 8] <<= 1;

        if (duration_us > 50) {
            bytes[bit_index / 8] |= 1;
        }
    }

    gpio_set_level(sensor->gpio, 1);

    uint8_t checksum =
        (uint8_t)(bytes[0] + bytes[1] + bytes[2] + bytes[3]);

    if (checksum != bytes[4]) {
        ESP_LOGW(
            TAG,
            "Checksum error: calculated=0x%02X received=0x%02X",
            checksum,
            bytes[4]
        );
        return ESP_ERR_INVALID_CRC;
    }

    uint16_t humidity_raw =
        ((uint16_t)bytes[0] << 8) | bytes[1];

    uint16_t temperature_raw =
        ((uint16_t)bytes[2] << 8) | bytes[3];

    data->humidity_percent = humidity_raw / 10.0f;

    if (temperature_raw & 0x8000) {
        temperature_raw &= 0x7FFF;
        data->temperature_c = -(temperature_raw / 10.0f);
    } else {
        data->temperature_c = temperature_raw / 10.0f;
    }

    sensor->last_read_time_us = time_us();

    ESP_LOGI(
        TAG,
        "Temperature: %.1f C, Humidity: %.1f %%",
        data->temperature_c,
        data->humidity_percent
    );

    return ESP_OK;
}