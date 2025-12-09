// tsl2561.h
#pragma once
#include <stdint.h>
#include "esp_err.h"
#include "driver/i2c.h"

// Change if ADDR pin tied differently
#define TSL2561_I2C_ADDR_DEFAULT 0x39

typedef struct {
    i2c_port_t port;
    gpio_num_t sda;
    gpio_num_t scl;
    uint32_t   clk_speed_hz;
    uint8_t    i2c_addr;
    // config
    uint8_t    gain_16x;        // 0=1x, 1=16x
    uint8_t    integ_time_code; // 0=13.7ms, 1=101ms, 2=402ms
} tsl2561_t;

// Init I2C bus (if not already) and power up + configure TSL2561
esp_err_t tsl2561_init(tsl2561_t *dev);

// Read raw channels (broadband = ch0, IR = ch1)
esp_err_t tsl2561_read_channels(tsl2561_t *dev, uint16_t *ch0, uint16_t *ch1);

// Approx lux conversion using standard coefficients
float tsl2561_calculate_lux(tsl2561_t *dev, uint16_t ch0, uint16_t ch1);
