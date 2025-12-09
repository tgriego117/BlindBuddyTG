// tsl2561.c
#include "tsl2561.h"
#include "esp_log.h"
#include "driver/i2c.h"
#include <math.h>


static const char *TAG = "TSL2561";

// Register map + command bit
#define TSL2561_CMD            0x80
#define TSL2561_REG_CONTROL    0x00
#define TSL2561_REG_TIMING     0x01
#define TSL2561_REG_CH0_LOW    0x0C
#define TSL2561_REG_CH0_HIGH   0x0D
#define TSL2561_REG_CH1_LOW    0x0E
#define TSL2561_REG_CH1_HIGH   0x0F

#define TSL2561_POWER_ON       0x03
#define TSL2561_POWER_OFF      0x00

static esp_err_t write8(tsl2561_t *dev, uint8_t reg, uint8_t val) {
    uint8_t buf[2] = { (uint8_t)(TSL2561_CMD | reg), val };
    return i2c_master_write_to_device(dev->port, dev->i2c_addr, buf, 2, pdMS_TO_TICKS(100));
}

static esp_err_t read8(tsl2561_t *dev, uint8_t reg, uint8_t *val) {
    uint8_t cmd = (uint8_t)(TSL2561_CMD | reg);
    esp_err_t err = i2c_master_write_read_device(dev->port, dev->i2c_addr, &cmd, 1, val, 1, pdMS_TO_TICKS(100));
    return err;
}

static esp_err_t read16(tsl2561_t *dev, uint8_t reg_low, uint16_t *out) {
    uint8_t lo=0, hi=0;
    esp_err_t e1 = read8(dev, reg_low,     &lo);
    esp_err_t e2 = read8(dev, reg_low + 1, &hi);
    if (e1 != ESP_OK || e2 != ESP_OK) return ESP_FAIL;
    *out = (uint16_t)((hi << 8) | lo);
    return ESP_OK;
}

esp_err_t tsl2561_init(tsl2561_t *dev) {
    if (!dev->clk_speed_hz) dev->clk_speed_hz = 100000; // 100 kHz default
    if (!dev->i2c_addr)     dev->i2c_addr     = TSL2561_I2C_ADDR_DEFAULT;
    if (!dev->port)         dev->port         = I2C_NUM_0;

    // Configure I2C (safe to call even if already installed)
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = dev->sda,
        .scl_io_num = dev->scl,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = dev->clk_speed_hz
    };
    ESP_ERROR_CHECK(i2c_param_config(dev->port, &conf));
    // If driver already installed, this will return ESP_FAIL; ignore if already running
    i2c_driver_install(dev->port, conf.mode, 0, 0, 0);

    // Power on
    ESP_ERROR_CHECK(write8(dev, TSL2561_REG_CONTROL, TSL2561_POWER_ON));

    // TIMING: integration + gain
    // integ_time_code: 0=13.7ms, 1=101ms, 2=402ms
    uint8_t timing = (dev->integ_time_code & 0x03);
    if (dev->gain_16x) timing |= 0x10;
    ESP_ERROR_CHECK(write8(dev, TSL2561_REG_TIMING, timing));

    ESP_LOGI(TAG, "TSL2561 powered on, gain=%s, integ=%s",
        dev->gain_16x ? "16x" : "1x",
        (dev->integ_time_code==2) ? "402ms" :
        (dev->integ_time_code==1) ? "101ms" : "13.7ms");

    return ESP_OK;
}

esp_err_t tsl2561_read_channels(tsl2561_t *dev, uint16_t *ch0, uint16_t *ch1) {
    // Datasheet recommends wait for integration time; read when steady
    esp_err_t e0 = read16(dev, TSL2561_REG_CH0_LOW, ch0);
    esp_err_t e1 = read16(dev, TSL2561_REG_CH1_LOW, ch1);
    return (e0==ESP_OK && e1==ESP_OK) ? ESP_OK : ESP_FAIL;
}

// Lux calculation (Adafruit-style coefficients for T/FN package)
float tsl2561_calculate_lux(tsl2561_t *dev, uint16_t ch0, uint16_t ch1) {
    if (ch0 == 0) return 0.0f;
    float ratio = (float)ch1 / (float)ch0;

    // Scale for integration time and gain (normalize to 402ms, 1x)
    float ch0f = (float)ch0;
    float ch1f = (float)ch1;
    // Integration scaling
    if      (dev->integ_time_code == 0) { ch0f *= 402.0f/13.7f; ch1f *= 402.0f/13.7f; }
    else if (dev->integ_time_code == 1) { ch0f *= 402.0f/101.0f; ch1f *= 402.0f/101.0f; }
    // Gain scaling
    if (dev->gain_16x) { ch0f /= 16.0f; ch1f /= 16.0f; }
    ratio = (ch0f > 0.0f) ? (ch1f / ch0f) : 0.0f;

    float lux;
    if      (ratio <= 0.50f) lux = 0.0304f * ch0f - 0.062f  * ch0f * powf(ratio, 1.4f);
    else if (ratio <= 0.61f) lux = 0.0224f * ch0f - 0.031f  * ch1f;
    else if (ratio <= 0.80f) lux = 0.0128f * ch0f - 0.0153f * ch1f;
    else if (ratio <= 1.30f) lux = 0.00146f* ch0f - 0.00112f* ch1f;
    else                     lux = 0.0f;

    if (lux < 0) lux = 0.0f;
    return lux;
}
