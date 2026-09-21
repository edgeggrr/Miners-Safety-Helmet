#include "mq2.h"

#include <string.h>
#include "esp_log.h"
#include "esp_check.h"
#include "esp_adc/adc_cali_scheme.h"

static const char *TAG = "MQ2";

#define MQ2_ADC_UNIT              ADC_UNIT_1
#define MQ2_ADC_BIT_WIDTH         ADC_BITWIDTH_DEFAULT
#define MQ2_ADC_ATTENUATION       ADC_ATTEN_DB_12

#define MQ2_ADC_MAX_VALUE         4095.0f
#define MQ2_MAX_VOLTAGE_MV        3300.0f

static esp_err_t create_adc_calibration(
    mq2_t *sensor
)
{
    esp_err_t err = ESP_FAIL;

#if ADC_CALI_SCHEME_CURVE_FITTING_SUPPORTED

    adc_cali_curve_fitting_config_t cali_config = {
        .unit_id = MQ2_ADC_UNIT,
        .chan = sensor->channel,
        .atten = MQ2_ADC_ATTENUATION,
        .bitwidth = MQ2_ADC_BIT_WIDTH
    };

    err = adc_cali_create_scheme_curve_fitting(
        &cali_config,
        &sensor->calibration_handle
    );

#elif ADC_CALI_SCHEME_LINE_FITTING_SUPPORTED

    adc_cali_line_fitting_config_t cali_config = {
        .unit_id = MQ2_ADC_UNIT,
        .atten = MQ2_ADC_ATTENUATION,
        .bitwidth = MQ2_ADC_BIT_WIDTH
    };

    err = adc_cali_create_scheme_line_fitting(
        &cali_config,
        &sensor->calibration_handle
    );

#else

    err = ESP_ERR_NOT_SUPPORTED;

#endif

    if (err == ESP_OK) {
        sensor->calibration_enabled = true;
        ESP_LOGI(TAG, "ADC calibration enabled");
    } else {
        sensor->calibration_enabled = false;
        sensor->calibration_handle = NULL;
        ESP_LOGW(
            TAG,
            "ADC calibration unavailable; using raw conversion"
        );
    }

    return ESP_OK;
}

esp_err_t mq2_init(
    mq2_t *sensor,
    gpio_num_t gpio
)
{
    if (sensor == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    memset(sensor, 0, sizeof(mq2_t));

    esp_err_t err = adc_oneshot_io_to_channel(
        gpio,
        NULL,
        &sensor->channel
    );

    if (err != ESP_OK) {
        ESP_LOGE(
            TAG,
            "GPIO%d is not a valid ADC pin",
            gpio
        );
        return err;
    }

    adc_unit_t detected_unit;

    err = adc_oneshot_io_to_channel(
        gpio,
        &detected_unit,
        &sensor->channel
    );

    if (err != ESP_OK) {
        return err;
    }

    if (detected_unit != MQ2_ADC_UNIT) {
        ESP_LOGE(TAG, "MQ-2 pin is not connected to ADC1");
        return ESP_ERR_INVALID_ARG;
    }

    adc_oneshot_unit_init_cfg_t unit_config = {
        .unit_id = MQ2_ADC_UNIT,
        .ulp_mode = ADC_ULP_MODE_DISABLE
    };

    err = adc_oneshot_new_unit(
        &unit_config,
        &sensor->adc_handle
    );

    if (err != ESP_OK) {
        ESP_LOGE(
            TAG,
            "ADC unit initialization failed: %s",
            esp_err_to_name(err)
        );
        return err;
    }

    adc_oneshot_chan_cfg_t channel_config = {
        .bitwidth = MQ2_ADC_BIT_WIDTH,
        .atten = MQ2_ADC_ATTENUATION
    };

    err = adc_oneshot_config_channel(
        sensor->adc_handle,
        sensor->channel,
        &channel_config
    );

    if (err != ESP_OK) {
        adc_oneshot_del_unit(sensor->adc_handle);
        sensor->adc_handle = NULL;
        return err;
    }

    create_adc_calibration(sensor);

    sensor->initialized = true;

    ESP_LOGI(
        TAG,
        "MQ-2 initialized on GPIO%d, ADC1 channel %d",
        gpio,
        sensor->channel
    );

    return ESP_OK;
}

esp_err_t mq2_read(
    mq2_t *sensor,
    mq2_reading_t *reading
)
{
    if (sensor == NULL || reading == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    if (!sensor->initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    int raw_value = 0;

    esp_err_t err = adc_oneshot_read(
        sensor->adc_handle,
        sensor->channel,
        &raw_value
    );

    if (err != ESP_OK) {
        ESP_LOGE(
            TAG,
            "ADC read failed: %s",
            esp_err_to_name(err)
        );
        return err;
    }

    int voltage_mv = 0;

    if (sensor->calibration_enabled) {
        err = adc_cali_raw_to_voltage(
            sensor->calibration_handle,
            raw_value,
            &voltage_mv
        );

        if (err != ESP_OK) {
            voltage_mv =
                (int)((raw_value / MQ2_ADC_MAX_VALUE)
                      * MQ2_MAX_VOLTAGE_MV);
        }
    } else {
        voltage_mv =
            (int)((raw_value / MQ2_ADC_MAX_VALUE)
                  * MQ2_MAX_VOLTAGE_MV);
    }

    reading->raw_adc = raw_value;
    reading->voltage_mv = voltage_mv;

    /*
     * This is a normalized hazard proxy, not a true ppm value.
     * Proper ppm calculation requires sensor-specific calibration,
     * load resistance, heater conditions, and gas reference samples.
     */
    reading->normalized_level =
        ((float)voltage_mv / MQ2_MAX_VOLTAGE_MV) * 100.0f;

    sensor->raw_value = raw_value;
    sensor->voltage_mv = voltage_mv;

    return ESP_OK;
}

esp_err_t mq2_deinit(
    mq2_t *sensor
)
{
    if (sensor == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    if (sensor->calibration_enabled &&
        sensor->calibration_handle != NULL) {

#if ADC_CALI_SCHEME_CURVE_FITTING_SUPPORTED
        adc_cali_delete_scheme_curve_fitting(
            sensor->calibration_handle
        );
#elif ADC_CALI_SCHEME_LINE_FITTING_SUPPORTED
        adc_cali_delete_scheme_line_fitting(
            sensor->calibration_handle
        );
#endif

        sensor->calibration_handle = NULL;
        sensor->calibration_enabled = false;
    }

    if (sensor->adc_handle != NULL) {
        esp_err_t err = adc_oneshot_del_unit(
            sensor->adc_handle
        );

        sensor->adc_handle = NULL;
        sensor->initialized = false;

        return err;
    }

    sensor->initialized = false;
    return ESP_OK;
}