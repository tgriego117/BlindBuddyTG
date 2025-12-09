#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "sdkconfig.h"
#include "cJSON.h"
#include "simple_wifi.h"
#include "http_server.h"

//get signal from http_server
//send signal to LED to indicate task received
#define LED_PIN_1 6 //rgb leds are gpio42 and 41 OR use gpio 6 and 7 for external LED
#define LED_PIN_2 7
static const char *TAG = "app";

// Parse JSON command and act on LED
static void handle_command_json(const char *json_str)
{
    if (!json_str) return;
    cJSON *root = cJSON_Parse(json_str);
    if (!root) {
        ESP_LOGE(TAG, "Invalid JSON");
        return;
    }

    cJSON *cmd = cJSON_GetObjectItemCaseSensitive(root, "command");
    if (!cJSON_IsString(cmd)) {
        ESP_LOGE(TAG, "Missing 'command' field");
        cJSON_Delete(root);
        return;
    }

    if (strcmp(cmd->valuestring, "led") == 0) {
        cJSON *state = cJSON_GetObjectItemCaseSensitive(root, "state");
        if (cJSON_IsString(state)) {
            if (strcmp(state->valuestring, "on") == 0) {
                gpio_set_level(LED_PIN_1, 1);
                ESP_LOGI(TAG, "LED ON");
            } else if (strcmp(state->valuestring, "off") == 0) {
                gpio_set_level(LED_PIN_1, 0);
                ESP_LOGI(TAG, "LED OFF");
            } else if (strcmp(state->valuestring, "blink") == 0) {
                cJSON *dur = cJSON_GetObjectItemCaseSensitive(root, "duration_ms");
                int duration = (cJSON_IsNumber(dur) ? dur->valueint : 1000);
                ESP_LOGI(TAG, "LED BLINK for %d ms", duration);
                // Simple blink: on for duration then off
                gpio_set_level(LED_PIN_1, 1);
                vTaskDelay(pdMS_TO_TICKS(duration));
                gpio_set_level(LED_PIN_1, 0);
            } else {
                ESP_LOGW(TAG, "Unknown led state: %s", state->valuestring);
            }
        } else {
            ESP_LOGE(TAG, "Missing 'state' for led command");
        }
    } else {
        ESP_LOGW(TAG, "Unknown command: %s", cmd->valuestring);
    }

    cJSON_Delete(root);
}

void app_main(void)
{
    //Initialize NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
      ESP_ERROR_CHECK(nvs_flash_erase());
      ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);
    
    ESP_LOGI(TAG, "ESP_WIFI_MODE_STA");
    wifi_init_sta();

    start_webserver();
    // Initialize the LED GPIO
    gpio_reset_pin(LED_PIN_1);
    gpio_set_direction(LED_PIN_1, GPIO_MODE_OUTPUT);
    gpio_set_level(LED_PIN_1, 0); // Turn off LED initially

    gpio_reset_pin(LED_PIN_2);
    gpio_set_direction(LED_PIN_2, GPIO_MODE_OUTPUT);
    gpio_set_level(LED_PIN_2, 0); // Turn off LED initially

    if (task_post_received) { // Flag set by HTTP POST handler
        task_post_received = false; // Reset the flag
        // Blink the LED to indicate task received
        gpio_set_level(LED_PIN_1, 1); // Turn on LED
        vTaskDelay(100 / portTICK_PERIOD_MS);
        gpio_set_level(LED_PIN_2, 1); // Turn on diff color LED
    }
}