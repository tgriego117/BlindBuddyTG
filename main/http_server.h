

#ifndef HTTP_SERVER_H
#define HTTP_SERVER_H

#include <esp_http_server.h>

httpd_handle_t start_webserver(void);
void motor_init(void);
void start_auto_light_task(void);

extern volatile float g_latest_lux1;
extern volatile float g_latest_lux2;
extern int blinds_position;

#endif // HTTP_SERVER_H
