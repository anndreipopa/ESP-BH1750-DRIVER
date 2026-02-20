# BH1750 Driver for ESP-IDF

This is a modular ESP-IDF component for the BH1750 digital light sensor.
I developed this driver to provide a clean, non-blocking interface for
light intensity measurements within the ESP-IDF v5.x environment.

## Overview

The driver manages the I2C handshake and handles the specific timing
requirements for the sensor's high-resolution mode. To maintain system
efficiency, it uses FreeRTOS delays (`vTaskDelay`) during the 120ms
integration period, ensuring the CPU is not stalled while waiting for
the ADC conversion.

## Hardware Connections

The following pinout was used for testing on an ESP32:

| Sensor Pin | ESP32 Pin | Function |
| :--- | :--- | :--- |
| VCC | 3.3V | Power |
| GND | GND | Ground |
| SDA | GPIO 21 | I2C Data |
| SCL | GPIO 22 | I2C Clock |
| ADDR | GND | Sets address to 0x23 |

## Installation

To use this in your project, copy the `bh1750` folder into your
project's `components` directory.

### Build System Configuration

In your `main/CMakeLists.txt`, add the component to the requirements:

``` cmake
idf_component_register(SRCS "main.c" PRIV_REQUIRES bh1750)
```

## Basic Usage Example

``` c
#include "bh1750.h"

void app_main(void) {
    // Sensor configuration
    bh1750_config_t config = {
        .sda_pin = 21,
        .scl_pin = 22,
        .i2c_frequency = 100000,
        .i2c_port = I2C_NUM_0
    };

    // Initialize and start reading loop
    if (bh1750_init(&config) == ESP_OK) {
        while (1) {
            float lux = bh1750_read_lux();
            if (lux != -1.0) {
                printf("Ambient Light: %.2f lx\n", lux);
            }
            vTaskDelay(pdMS_TO_TICKS(1000));
        }
    }
}
```

## Technical Implementation Details

-   **Operation Mode:** Configured for Continuous High-Resolution Mode
    (opcode `0x10`).
-   **Timing Control:** Includes a 120ms delay in the read function to
    satisfy the maximum integration time required by the BH1750 ADC, as
    specified in the datasheet.
-   **Error Handling:** Functions return `ESP_OK` or `-1.0` (for float
    results) to allow the calling application to handle I2C bus timeouts
    or communication failures gracefully.

## License

MIT. See the LICENSE file for details.
