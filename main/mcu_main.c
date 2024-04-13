#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

#include "driver/pulse_cnt.h"
#include "driver/gpio.h"

#include "esp_log.h"
#include "esp_sleep.h"
#include "esp_timer.h"

#include "CANopenNode_ESP32.h"
#include "OD.h"

static const char *TAG = "MCU";
static pcnt_unit_handle_t pcnt_unit = NULL;

float pi = 3.1415926535897932385;
float diameter = 0.508f; // 20" = 0.508 m
uint8_t pulses_per_revolution = 6;
uint32_t speed_calc_tick_ms = 500;

int pulses = 0;

static void speed_timer_on_tick(void *arg)
{
    ESP_ERROR_CHECK(pcnt_unit_get_count(pcnt_unit, &pulses));
    float speed = pi * diameter * ((float)pulses / pulses_per_revolution) * (3.6f * 1000 / speed_calc_tick_ms);
    if (pulses > 0)
    {
        ESP_LOGI(TAG, "Vehicle speed: %.3f km/h (%d pulses)", speed, pulses);
        pcnt_unit_clear_count(pcnt_unit);
        // CO_TPDOsendRequest();
        // OD_requestTPDO()

        OD_RAM.x6400_vehicleSpeed = speed;
    }
}

static void configure_pulse_counter()
{
    ESP_LOGI(TAG, "install pcnt unit");
    pcnt_unit_config_t unit_config = {
        .high_limit = CONFIG_SPEED_PCNT_HIGH_LIMIT,
        .low_limit = CONFIG_SPEED_PCNT_LOW_LIMIT,
    };

    ESP_ERROR_CHECK(pcnt_new_unit(&unit_config, &pcnt_unit));

    ESP_LOGI(TAG, "set glitch filter");
    pcnt_glitch_filter_config_t filter_config = {
        .max_glitch_ns = 1000,
    };
    ESP_ERROR_CHECK(pcnt_unit_set_glitch_filter(pcnt_unit, &filter_config));

    ESP_LOGI(TAG, "install pcnt channels");
    pcnt_chan_config_t chan_a_config = {
        .edge_gpio_num = CONFIG_SPEED_SENSOR_GPIO_NUM,
        .level_gpio_num = -1,
    };
    pcnt_channel_handle_t pcnt_chan_a = NULL;
    ESP_ERROR_CHECK(pcnt_new_channel(pcnt_unit, &chan_a_config, &pcnt_chan_a));

    ESP_LOGI(TAG, "set edge and level actions for pcnt channels");
    ESP_ERROR_CHECK(pcnt_channel_set_edge_action(pcnt_chan_a, PCNT_CHANNEL_EDGE_ACTION_INCREASE, PCNT_CHANNEL_LEVEL_ACTION_KEEP));

    ESP_LOGI(TAG, "enable pcnt unit");
    ESP_ERROR_CHECK(pcnt_unit_enable(pcnt_unit));
    ESP_LOGI(TAG, "clear pcnt unit");
    ESP_ERROR_CHECK(pcnt_unit_clear_count(pcnt_unit));
    ESP_LOGI(TAG, "start pcnt unit");
    ESP_ERROR_CHECK(pcnt_unit_start(pcnt_unit));

#if CONFIG_EXAMPLE_WAKE_UP_LIGHT_SLEEP
    // EC11 channel output high level in normal state, so we set "low level" to wake up the chip
    ESP_ERROR_CHECK(gpio_wakeup_enable(EXAMPLE_EC11_GPIO_A, GPIO_INTR_LOW_LEVEL));
    ESP_ERROR_CHECK(esp_sleep_enable_gpio_wakeup());
    ESP_ERROR_CHECK(esp_light_sleep_start());
#endif
}

static void configure_speed_calc_timer()
{
    const esp_timer_create_args_t periodic_timer_args = { .callback = &speed_timer_on_tick, .name = "speed_timer" };

    esp_timer_handle_t periodic_timer;
    ESP_ERROR_CHECK(esp_timer_create(&periodic_timer_args, &periodic_timer));
    ESP_ERROR_CHECK(esp_timer_start_periodic(periodic_timer, speed_calc_tick_ms * 1000));
}

void app_main(void)
{
    CO_ESP32_init();

    configure_pulse_counter();
    configure_speed_calc_timer();

    while (1)
    {
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}