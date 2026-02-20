#include <stdio.h>
#include <stdbool.h>
#include <stdint.h>
#include "bh1750.h"
#include "driver/i2c.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

// Internal variables
// Storing i2c port number globally

static uint8_t i2c_port_num;
static const char *TAG = "BH1750";

// i2c master settings

#define I2C_MASTER_TX_BUF_DISABLE 0
#define I2C_MASTER_RX_BUF_DISABLE 0
#define I2C_MASTER_TIMEOUT_MS 1000

#define BH1750_OPCODE_HIGH_RES_MODE 0x10

static i2c_port_t g_i2c_port = I2C_NUM_0;
static bool g_is_initialized = false;

// Initialization function

esp_err_t bh1750_init(const bh1750_config_t *config) {
        if( g_is_initialized) {
                ESP_LOGW(TAG, "BH1750 already initialized");
                return ESP_OK;
        }

    g_i2c_port = config -> i2c_port_t;

    // config i2c bus

    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = config -> sda_pin,
        .scl_io_num = config -> scl_pin,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = config -> i2c_frequency,
        .clk_flags = 0,
    };


    // apply config

    esp_err_t err = i2c_param_config(g_i2c_port, &conf);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure I2C parameters: %s", esp_err_to_name(err));
        return err;
    }
    
    err = i2c_driver_install(g_i2c_port, conf.mode, I2C_MASTER_RX_BUF_DISABLE, I2C_MASTER_TX_BUF_DISABLE, 0);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to install I2C driver: %s", esp_err_to_name(err));
        return err;
    }

    g_is_initialized = true;
    ESP_LOGI(TAG, "BH1750 initialized on port %d", g_i2c_port);
    return ESP_OK;

}

// reading logic

float bh1750_read_lux(void) {
    if(!g_is_initialized){
        ESP_LOGE(TAG, "Driver not initialized!");
        return -1.0;
    }

    // send measurement command

    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);

    // write address with write bit

    i2c_master_write_byte(cmd, (BH1750_ADDR_LOW << 1) | I2C_MASTER_WRITE, true);

    // write comand to start high res mode

    i2c_master_write_byte(cmd, BH1750_OPCODE_HIGH_RES_MODE, true);

    i2c_master_stop(cmd);

    // execute command

    esp_err_t ret = i2c_master_cmd_begin(g_i2c_port, cmd, I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd); // delete to free mem

    if (ret != ESP_OK){
        ESP_LOGE(TAG, "Write Failed: %s", esp_err_to_name(ret));
        return -1.0;
    }


    // wait for measurement to be taken

    vTaskDelay(120 / portTICK_PERIOD_MS);
    
    // read 2 bytes of data

    uint8_t data[2];
    cmd=i2c_cmd_link_create();
    i2c_master_start(cmd);

    // write address with read bit

    i2c_master_write_byte(cmd, (BH1750_ADDR_LOW << 1) | I2C_MASTER_READ, true);

    // read high byte

    i2c_master_read_byte(cmd, &data[0], I2C_MASTER_ACK);

    // read low byte

    i2c_master_read_byte(cmd, &data[1], I2C_MASTER_NACK);

    i2c_master_stop(cmd);

    // execute read command

    ret = i2c_master_cmd_begin(g_i2c_port, cmd, I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);

    if(ret != ESP_OK){
        ESP_LOGE(TAG, "Read Failed: %s", esp_err_to_name(ret));
        return -1.0;
    }

    // convert data to lux

    uint16_t raw_val = (data[0] << 8) | data[1];

    return (float)raw_val / 1.2; // convert to lux using formula from datasheet
 
}