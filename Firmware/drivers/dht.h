#ifndef DHT_H
#define DHT_H

#include <stdbool.h>
#include "esp_err.h"
#include "driver/gpio.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    gpio_num_t gpio;
    uint32_t timeout_us;
    uint32_t min_read_interval_ms;
    int64_t last_read_time_us;
    bool initialized;
} dht22_t;

typedef struct {
    float temperature_c;
    float humidity_percent;
} dht22_data_t;

/**
 * Initialize a DHT22 device.
 *
 * The DATA pin requires an external 4.7 kΩ–10 kΩ pull-up resistor
 * connected to 3.3 V.
 */
esp_err_t dht22_init(dht22_t *sensor, gpio_num_t gpio);

/**
 * Read temperature and humidity from the DHT22.
 *
 * A minimum interval of approximately two seconds is enforced
 * between successful sensor reads.
 */
esp_err_t dht22_read(
    dht22_t *sensor,
    dht22_data_t *data
);

/**
 * Check whether the minimum DHT22 read interval has elapsed.
 */
bool dht22_is_ready(const dht22_t *sensor);

#ifdef __cplusplus
}
#endif

#endif