#ifndef HTTP_CLIENT_H
#define HTTP_CLIENT_H

#include <esp_http_client.h>

esp_err_t http_get(const char *url, char *response, size_t max_len);
esp_err_t http_post(const char *url, const char *post_data, char *response, size_t max_len);

#endif