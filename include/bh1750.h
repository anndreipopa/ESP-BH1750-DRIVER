#ifndef BH1750_H
#define BH1750_H

#include <stdint.h>
#include "esp_err.h"
#include "driver/i2c.h"

// defining bh1750 sensor commands from datasheet

#define BH1750_ADDR_LOW 0x23 // Address when the address pin is low
#define BH1750_CMD_POWER_ON 0x01 // Power on the sensor
#define BH1750_CMD_HIGH_RES_MODE 0x10 // 1 lx resolution, 120ms measurement time

// config struct

typedef struct {
    int sda_pin;
    int scl_pin;
    uint32_t i2c_frequency;
    uint8_t i2c_port_t;
} bh1750_config_t;

// function prototypes

esp_err_t bh1750_init(const bh1750_config_t *config);
float bh1750_read_lux(void);

#endif // BH1750_H