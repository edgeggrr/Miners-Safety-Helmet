#include <string.h>
#include <math.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"

#include "esp_log.h"
#include "esp_timer.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "esp_http_client.h"
#include "nvs_flash.h"

#include "dht.h"
#include "mpu6050.h"
#include "mq2.h"
#include "hcsr04.h"
#include "buzzer.h"
#include "buttons.h"

static const char *TAG = "SAFETY_MONITOR";

/* ------------------------- WiFi / ThingSpeak ------------------------- */
#define WIFI_SSID           "YOUR_WIFI_SSID"
#define WIFI_PASSWORD       "YOUR_WIFI_PASSWORD"
#define THINGSPEAK_API_KEY  "YOUR_THINGSPEAK_API_KEY"
#define THINGSPEAK_URL      "http://api.thingspeak.com/update"

/* ThingSpeak free tier rejects updates more often than every 15 s.
 * Main loop runs at ~1 Hz, so upload every 15th iteration. */
#define UPLOAD_EVERY_N_LOOPS 15

static EventGroupHandle_t s_wifi_event_group;
#define WIFI_CONNECTED_BIT BIT0

/* ------------------------------ Pinout -------------------------------
 * Regular ESP32 DevKit pin mapping (carried over from the original
 * Arduino sketch; MPU6050 uses the ESP32's default I2C pins).
 */
#define DHT_GPIO             GPIO_NUM_4
#define MQ2_GPIO              GPIO_NUM_34
#define HCSR04_TRIG_GPIO       GPIO_NUM_25
#define HCSR04_ECHO_GPIO       GPIO_NUM_26
#define LED_GPIO                GPIO_NUM_2
#define BUZZER_GPIO              GPIO_NUM_15
#define BUTTON_TRIGGER_GPIO       GPIO_NUM_13
#define BUTTON_RESET_GPIO          GPIO_NUM_12
#define MPU_SDA_GPIO                 GPIO_NUM_21
#define MPU_SCL_GPIO                  GPIO_NUM_22

/* ----------------------------- Thresholds ----------------------------- */
#define SAFE_TEMP_C            35.0f
#define SAFE_HUMIDITY_PCT      80.0f

#define GAS_ALARM_RAW_ADC       350
#define GAS_MILD_RAW_ADC        250

#define SPACE_ALERT_DISTANCE_CM 30.0f
#define SPACE_ALERT_TIMEOUT_US  (30LL * 60 * 1000000)   /* 30 minutes  */
#define SPACE_FLASH_INTERVAL_US (60LL * 1000000)        /* 60 seconds */

#define TILT_ALARM_DEG          50.0f
#define G_PER_MS2               (1.0f / 9.80665f)
#define JERK_XY_THRESHOLD_G     (10.0f * G_PER_MS2)
#define JERK_Z_THRESHOLD_G      (15.0f * G_PER_MS2)

/* ------------------------------ Devices ------------------------------- */
static dht22_t    s_dht;
static mpu6050_t   s_mpu;
static mq2_t         s_mq2;
static hcsr04_t        s_hcsr04;
static buzzer_t           s_buzzer;
static buttons_t            s_buttons;

/* ------------------------------ State ---------------------------------- */
static bool    s_manual_alarm_latched = false;

static bool    s_space_alert_active   = false;
static int64_t s_space_alert_start_us = 0;
static int64_t s_last_space_flash_us  = 0;

static float   s_last_temp_c   = 0.0f;
static float   s_last_hum_pct  = 0.0f;

/* ============================ WiFi setup =============================== */

static void wifi_event_handler(void *arg, esp_event_base_t event_base,
                                int32_t event_id, void *event_data)
{
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        ESP_LOGW(TAG, "WiFi disconnected, retrying...");
        esp_wifi_connect();
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t *event = (ip_event_got_ip_t *) event_data;
        ESP_LOGI(TAG, "Got IP: " IPSTR, IP2STR(&event->ip_info.ip));
        xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
    }
}

static void wifi_init_sta(void)
{
    s_wifi_event_group = xEventGroupCreate();

    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID,
                                                &wifi_event_handler, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP,
                                                &wifi_event_handler, NULL));

    wifi_config_t wifi_config = {
        .sta = {
            .ssid = WIFI_SSID,
            .password = WIFI_PASSWORD,
        },
    };

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_LOGI(TAG, "Connecting to WiFi SSID: %s", WIFI_SSID);

    /* Block here so the rest of app_main only runs once we have an IP.
     * Waits indefinitely - fine for a 1-hour bring-up; add a timeout
     * later if you want the sensors to run without WiFi present. */
    xEventGroupWaitBits(s_wifi_event_group, WIFI_CONNECTED_BIT,
                         pdFALSE, pdTRUE, portMAX_DELAY);
}

/* ========================== ThingSpeak upload =========================== */

static void upload_to_thingspeak(float temp_c, float hum_pct,
                                  int gas_raw_adc, bool alarm_active)
{
    char post_data[160];
    snprintf(post_data, sizeof(post_data),
             "api_key=%s&field1=%.1f&field2=%.1f&field3=%d&field4=%d",
             THINGSPEAK_API_KEY, temp_c, hum_pct, gas_raw_adc,
             alarm_active ? 1 : 0);

    esp_http_client_config_t config = {
        .url = THINGSPEAK_URL,
        .method = HTTP_METHOD_POST,
        .timeout_ms = 5000,
    };

    esp_http_client_handle_t client = esp_http_client_init(&config);
    esp_http_client_set_header(client, "Content-Type",
                                "application/x-www-form-urlencoded");
    esp_http_client_set_post_field(client, post_data, strlen(post_data));

    esp_err_t err = esp_http_client_perform(client);
    if (err == ESP_OK) {
        ESP_LOGI(TAG, "ThingSpeak upload OK, status = %d",
                 esp_http_client_get_status_code(client));
    } else {
        ESP_LOGE(TAG, "ThingSpeak upload failed: %s", esp_err_to_name(err));
    }

    esp_http_client_cleanup(client);
}

/* ============================ Space alert =============================== */
/* Mirrors the original sketch: if something stays closer than 30 cm for
 * 30 minutes straight, flash the LED once every 60 s as a reminder.
 * This is independent of the buzzer alarm on purpose (matches original
 * behaviour, which never called playChime() from this path). */

static void update_space_alert(float distance_cm)
{
    int64_t now_us = esp_timer_get_time();

    if (distance_cm < SPACE_ALERT_DISTANCE_CM) {
        if (!s_space_alert_active) {
            s_space_alert_active   = true;
            s_space_alert_start_us = now_us;
        }

        if ((now_us - s_space_alert_start_us) > SPACE_ALERT_TIMEOUT_US) {
            if ((now_us - s_last_space_flash_us) > SPACE_FLASH_INTERVAL_US) {
                gpio_set_level(LED_GPIO, 1);
                vTaskDelay(pdMS_TO_TICKS(200));
                gpio_set_level(LED_GPIO, 0);
                s_last_space_flash_us = now_us;
            }
        }
    } else {
        s_space_alert_active = false;
    }
}

/* =============================== app_main ================================ */

void app_main(void)
{
    /* --- NVS (required by WiFi) --- */
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    wifi_init_sta();

    /* --- Sensor / actuator init --- */
    if (dht22_init(&s_dht, DHT_GPIO) != ESP_OK) {
        ESP_LOGE(TAG, "DHT22 init failed");
    }
    if (mpu6050_init(&s_mpu, MPU_SDA_GPIO, MPU_SCL_GPIO, MPU6050_I2C_ADDRESS) != ESP_OK) {
        ESP_LOGE(TAG, "MPU6050 init failed - continuing without it");
    }
    if (mq2_init(&s_mq2, MQ2_GPIO) != ESP_OK) {
        ESP_LOGE(TAG, "MQ-2 init failed");
    }
    if (hcsr04_init(&s_hcsr04, HCSR04_TRIG_GPIO, HCSR04_ECHO_GPIO) != ESP_OK) {
        ESP_LOGE(TAG, "HC-SR04 init failed");
    }
    if (buzzer_init(&s_buzzer, BUZZER_GPIO, LED_GPIO) != ESP_OK) {
        ESP_LOGE(TAG, "Buzzer init failed");
    }
    if (buttons_init(&s_buttons, BUTTON_TRIGGER_GPIO, BUTTON_RESET_GPIO) != ESP_OK) {
        ESP_LOGE(TAG, "Buttons init failed");
    }

    ESP_LOGI(TAG, "System ready");

    uint32_t loop_count = 0;

    while (1) {
        bool temp_hum_alert = false;
        bool gas_alert       = false;
        bool tilt_or_jerk_alert = false;

        /* --- Buttons: trigger latches the alarm, reset clears it --- */
        button_event_t btn_event = BUTTON_EVENT_NONE;
        if (buttons_poll(&s_buttons, &btn_event) == ESP_OK) {
            if (btn_event == BUTTON_EVENT_TRIGGER) {
                s_manual_alarm_latched = true;
            } else if (btn_event == BUTTON_EVENT_RESET) {
                s_manual_alarm_latched = false;
            }
        }

        /* --- DHT22 (rate-limited internally to ~1 read / 2 s) --- */
        if (dht22_is_ready(&s_dht)) {
            dht22_data_t dht_data;
            if (dht22_read(&s_dht, &dht_data) == ESP_OK) {
                s_last_temp_c  = dht_data.temperature_c;
                s_last_hum_pct = dht_data.humidity_percent;
            }
        }
        temp_hum_alert = (s_last_temp_c > SAFE_TEMP_C) ||
                          (s_last_hum_pct > SAFE_HUMIDITY_PCT);

        /* --- MQ-2 gas sensor --- */
        mq2_reading_t gas_reading = {0};
        mq2_read(&s_mq2, &gas_reading);
        gas_alert = gas_reading.raw_adc > GAS_ALARM_RAW_ADC;
        if (!gas_alert && gas_reading.raw_adc > GAS_MILD_RAW_ADC) {
            ESP_LOGW(TAG, "Mild gas level detected: raw=%d", gas_reading.raw_adc);
        }

        /* --- HC-SR04 distance / space monitor --- */
        hcsr04_reading_t distance_reading = {0};
        if (hcsr04_read(&s_hcsr04, &distance_reading) == ESP_OK &&
            distance_reading.valid) {
            update_space_alert(distance_reading.distance_cm);
        }

        /* --- MPU6050 tilt / jerk detection --- */
        mpu6050_data_t mpu_data = {0};
        if (mpu6050_read(&s_mpu, &mpu_data) == ESP_OK) {
            float tilt_deg = sqrtf(mpu_data.pitch_deg * mpu_data.pitch_deg +
                                    mpu_data.roll_deg * mpu_data.roll_deg);

            bool jerk = (fabsf(mpu_data.accel_x_g) > JERK_XY_THRESHOLD_G) ||
                        (fabsf(mpu_data.accel_y_g) > JERK_XY_THRESHOLD_G) ||
                        (fabsf(mpu_data.accel_z_g) > JERK_Z_THRESHOLD_G);

            tilt_or_jerk_alert = (tilt_deg > TILT_ALARM_DEG) || jerk;
        }

        /* --- Combine everything into one buzzer/LED alarm state --- */
        bool alarm_active = temp_hum_alert || gas_alert ||
                             tilt_or_jerk_alert || s_manual_alarm_latched;

        buzzer_update(&s_buzzer, alarm_active);

        /* --- Periodic ThingSpeak upload (rate-limited) --- */
        loop_count++;
        if (loop_count % UPLOAD_EVERY_N_LOOPS == 0) {
            upload_to_thingspeak(s_last_temp_c, s_last_hum_pct,
                                  gas_reading.raw_adc, alarm_active);
        }

        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
