#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "bh1750.h"

static const char *TAG = "MAIN";

void app_main(void)
{
    // 1. Define the sensor configuration
    bh1750_config_t lux_config = {
        .sda_pin = 21,
        .scl_pin = 22,
        .i2c_frequency = 100000, // 100kHz Standard Mode
        .i2c_port_t = I2C_NUM_0
    };

    // 2. Initialize the sensor
    esp_err_t ret = bh1750_init(&lux_config);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "BH1750 Initialization failed!");
        return;
    }

    ESP_LOGI(TAG, "BH1750 Initialized. Starting readings...");

    // 3. Main loop
    while (1) {
        float lux = bh1750_read_lux();

        if (lux >= 0) {
            ESP_LOGI(TAG, "Light Intensity: %.2f lx", lux);
        } else {
            ESP_LOGE(TAG, "Failed to read sensor!");
        }

        // Wait 1 second before next reading
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}