//send file to JSON script
//send output of JSON script to signal.c, then send to gpio

#include <esp_http_client.h>
#include "http_client.h"
#include "tsl2561.h"

esp_err_t http_get(const char *url, char *response, size_t max_len) {
    esp_http_client_config_t config = {
        .url = url,
        .method = HTTP_METHOD_GET,
    };
    esp_http_client_handle_t client = esp_http_client_init(&config);
    if (client == NULL) {
        return ESP_FAIL;
    }

    esp_err_t err = esp_http_client_perform(client);
    if (err == ESP_OK) {
        int content_length = esp_http_client_get_content_length(client);
        if (content_length < max_len) {
            int read_len = esp_http_client_read(client, response, max_len - 1);
            if (read_len >= 0) {
                response[read_len] = '\0'; // Null-terminate the response
            } else {
                esp_http_client_cleanup(client);
                return ESP_FAIL;
            }
        } else {
            esp_http_client_cleanup(client);
            return ESP_FAIL;
        }
    } else {
        esp_http_client_cleanup(client);
        return err;
    }

    esp_http_client_cleanup(client);
    return ESP_OK;
}

esp_err_t http_post(const char *url, const char *post_data, char *response, size_t max_len) {
    esp_http_client_config_t config = {
        .url = url,
        .method = HTTP_METHOD_POST,
    };
    esp_http_client_handle_t client = esp_http_client_init(&config);
    if (client == NULL) {
        return ESP_FAIL;
    }

    esp_http_client_set_post_field(client, post_data, strlen(post_data));
    esp_http_client_set_header(client, "Content-Type", "application/json");

    esp_err_t err = esp_http_client_perform(client);
    if (err == ESP_OK) {
        int content_length = esp_http_client_get_content_length(client);
        if (content_length < max_len) {
            int read_len = esp_http_client_read(client, response, max_len - 1);
            if (read_len >= 0) {
                response[read_len] = '\0'; // Null-terminate the response
            } else {
                esp_http_client_cleanup(client);
                return ESP_FAIL;
            }
        } else {
            esp_http_client_cleanup(client);
            return ESP_FAIL;
        }
    } else {
        esp_http_client_cleanup(client);
        return err;
    }

    esp_http_client_cleanup(client);
    return ESP_OK;
}

