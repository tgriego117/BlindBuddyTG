// main/main.c
#include <math.h>
#include "nvs_flash.h"
#include "esp_log.h"
#include "esp_err.h"

#include "driver/gpio.h"
#include "driver/i2c.h"          // for i2c_scan()
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "simple_wifi.h"
#include "http_server.h"
#include "signal.h"
#include "tsl2561.h"
//#include "light_control.h"

static const char *TAG = "APP";

// Exposed to the HTTP page
volatile float g_latest_lux1 = 0.0f;  // sensor on I2C0 (SDA=16, SCL=15)
volatile float g_latest_lux2 = 0.0f;  // sensor on I2C1 (SDA=39, SCL=40)
volatile float g_lux_difference = 0.0f;

static float calculate_lux_difference(void) {
    float diff = g_latest_lux1 - g_latest_lux2;
    return diff < 0 ? -diff : diff;
}

// Simple I2C scanner (run after a bus is configured/installed)
static void i2c_scan(i2c_port_t port) {
    ESP_LOGI(TAG, "I2C scan on port %d", port);
    for (int addr = 0x03; addr <= 0x77; ++addr) {
        i2c_cmd_handle_t cmd = i2c_cmd_link_create();
        i2c_master_start(cmd);
        i2c_master_write_byte(cmd, (addr << 1) | I2C_MASTER_WRITE, true);
        i2c_master_stop(cmd);
        esp_err_t r = i2c_master_cmd_begin(port, cmd, pdMS_TO_TICKS(50));
        i2c_cmd_link_delete(cmd);
        if (r == ESP_OK) {
            ESP_LOGI(TAG, "  Found device at 0x%02X", addr);
        }
    }
}

static void sensor_task(void *arg) {
    (void)arg;

    // ===== I2C pins on your PCB =====
    // SDA -> GPIO21
    // SCL -> GPIO20
    const gpio_num_t SDA0 = (gpio_num_t)21;
    const gpio_num_t SCL0 = (gpio_num_t)20;

    // ---- Sensor A on I2C0 ----
    // ADDR floating -> 0x39
    tsl2561_t devA = {
        .port            = I2C_NUM_0,
        .sda             = SDA0,
        .scl             = SCL0,
        .clk_speed_hz    = 100000,
        .i2c_addr        = 0x39,      // sensor A address
        .gain_16x        = 0,
        .integ_time_code = 2          // 402 ms
    };

    // ---- Sensor B on SAME bus I2C0 ----
    // ADDR tied LOW (GND) -> 0x29 (make sure your wiring matches this)
    tsl2561_t devB = {
        .port            = I2C_NUM_0,
        .sda             = SDA0,
        .scl             = SCL0,
        .clk_speed_hz    = 100000,
        .i2c_addr        = 0x29,      // sensor B address
        .gain_16x        = 0,
        .integ_time_code = 2
    };

    esp_err_t errA = tsl2561_init(&devA);
    if (errA != ESP_OK) {
        ESP_LOGE(TAG, "TSL2561 A init failed (%s). SDA=%d SCL=%d addr=0x%02X",
                 esp_err_to_name(errA), (int)devA.sda, (int)devA.scl, devA.i2c_addr);
    }

    esp_err_t errB = tsl2561_init(&devB);
    if (errB != ESP_OK) {
        ESP_LOGE(TAG, "TSL2561 B init failed (%s). SDA=%d SCL=%d addr=0x%02X",
                 esp_err_to_name(errB), (int)devB.sda, (int)devB.scl, devB.i2c_addr);
    }

    // Scan the ONE bus so you can confirm both addresses show up
    i2c_scan(I2C_NUM_0);

    while (1) {
        uint16_t a0=0, a1=0, b0=0, b1=0;

        if (errA == ESP_OK && tsl2561_read_channels(&devA, &a0, &a1) == ESP_OK) {
            g_latest_lux1 = tsl2561_calculate_lux(&devA, a0, a1);
            ESP_LOGI(TAG, "[A] ch0=%u ch1=%u lux=%.1f", a0, a1, g_latest_lux1);
        } else {
            ESP_LOGW(TAG, "[A] read failed");
        }

        if (errB == ESP_OK && tsl2561_read_channels(&devB, &b0, &b1) == ESP_OK) {
            g_latest_lux2 = tsl2561_calculate_lux(&devB, b0, b1);
            ESP_LOGI(TAG, "[B] ch0=%u ch1=%u lux=%.1f", b0, b1, g_latest_lux2);
        } else {
            ESP_LOGW(TAG, "[B] read failed");
        }

        g_lux_difference = calculate_lux_difference();
        ESP_LOGI(TAG, "[DIFF] Lux difference = %.1f", g_lux_difference);

        vTaskDelay(pdMS_TO_TICKS(500));
    }
}

void app_main(void) {
    // 0) NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ESP_ERROR_CHECK(nvs_flash_init());
    }

    // 1) Motors
    motor_init();

    // 2) Wi-Fi
    ESP_LOGI(TAG, "Starting Wi-Fi…");
    wifi_init_sta();

    // 3) HTTP server
    ESP_LOGI(TAG, "Starting HTTP server…");
    (void)start_webserver();

    // 4) Sensors task
    xTaskCreate(sensor_task, "sensor_task", 4096, NULL, 5, NULL);

    // 4( Light control loop)
    start_auto_light_task();

    // 5) Optional: automatic light control (disabled for now)
    /*
    bool ok = light_control_start_tsl2561(I2C_NUM_0, 16, 15, 0x39, 1000, 0.25f);
    if (!ok) {
        ESP_LOGE(TAG, "Failed to start TSL2561 light control");
    }
    */
}
