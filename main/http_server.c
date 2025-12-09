// http_server.c — BlindBuddy (two PWMs: lift + tilt)
// - Lift on PWMA (GPIO 39)
// - Tilt on PWMB (GPIO 42)
// - s_moving / s_tilting guards to prevent concurrent motion
// - Standby pin held HIGH
// - /status handler implemented

#include "signal.h"
#include "http_server.h"



#include <stdio.h>
#include <string.h>
#include <stdbool.h>
#include <stdint.h>
#include "esp_http_server.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"


// ===== Globals =====
static const char *TAG = "HTTP";

// Provided by main.c (your lux task)
// Provided by main.c (your lux task)
extern volatile float g_latest_lux1;
extern volatile float g_latest_lux2;

// Shared motor state from signal.c
extern int blinds_position;
extern volatile bool s_moving;
extern volatile bool s_tilting;
extern int tilt_status;
extern int tilt_percent;
extern int global_is_closed;




// ===== Auto light control =====
typedef enum {
    AUTO_OFF = 0,
    AUTO_LOW,
    AUTO_MED_LOW,
    AUTO_MED,
    AUTO_MED_HIGH,
    AUTO_HIGH
} auto_light_mode_t;

static auto_light_mode_t s_auto_mode = AUTO_OFF;

// Target inside/outside light ratios for each mode
// inside_lux ≈ ratio * outside_lux
#define AUTO_RATIO_LOW        5.00f   // essentially blackout
#define AUTO_RATIO_MED_LOW    25.00f
#define AUTO_RATIO_MED        50.00f
#define AUTO_RATIO_MED_HIGH   75.00f
#define AUTO_RATIO_HIGH       100.00f

#define AUTO_HYSTERESIS       0.05f   // +/-5% band so it doesn't hunt
#define AUTO_LOOP_PERIOD_MS   2000    // check every 2s

// How hard each "nudge" of tilt moves (as % of full range)
#define TILT_NUDGE_PCT        1

static const char *auto_mode_str(auto_light_mode_t m)
{
    switch (m) {
        case AUTO_LOW:       return "Low";
        case AUTO_MED_LOW:   return "Med-Low";
        case AUTO_MED:       return "Medium";
        case AUTO_MED_HIGH:  return "Med-High";
        case AUTO_HIGH:      return "High";
        case AUTO_OFF:
        default:             return "Manual";
    }
}


static float auto_get_target_ratio(auto_light_mode_t mode)
{
    switch (mode) {
        case AUTO_LOW:       return AUTO_RATIO_LOW;
        case AUTO_MED_LOW:   return AUTO_RATIO_MED_LOW;
        case AUTO_MED:       return AUTO_RATIO_MED;
        case AUTO_MED_HIGH:  return AUTO_RATIO_MED_HIGH;
        case AUTO_HIGH:      return AUTO_RATIO_HIGH;
        case AUTO_OFF:
        default:             return AUTO_RATIO_HIGH;
    }
}

static void auto_set_mode(auto_light_mode_t mode)
{
    s_auto_mode = mode;
    ESP_LOGI(TAG, "Auto light mode -> %s", auto_mode_str(mode));
}

static void auto_light_task(void *pv)
{
    (void)pv;
    while (1) {
        auto_light_mode_t mode = s_auto_mode;

        if (mode != AUTO_OFF) {
            float out = g_latest_lux1;
            float in  = g_latest_lux2;

            // If sensors swapped in your wiring, just swap here later.

            if (mode == AUTO_HIGH) {
                // Full light: blinds up, slats open
                move_blinds_to_percent_time_async(0);
                tilt_blinds_async(0);
            }
            else if (mode == AUTO_LOW) {
                // Blackout: blinds down, slats closed
                move_blinds_to_percent_time_async(100);
                tilt_blinds_async(1);
            }
            else {
                // Intermediate: blinds down, adjust tilt to hit target ratio
                float target = auto_get_target_ratio(mode);
                move_blinds_to_percent_time_async(100); // all the way down

                if (out > 5.0f) {
                    float ratio = (in <= 0.0f) ? 0.0f : (in / out);

                    if (ratio < target - AUTO_HYSTERESIS) {
                        // Too dark inside -> open a bit
                        int new_pct = clamp_int(tilt_percent + TILT_NUDGE_PCT, 0, 100);
                        tilt_to_percent_blocking(new_pct);
                    } else if (ratio > target + AUTO_HYSTERESIS) {
                        // Too bright inside -> close a bit
                        int new_pct = clamp_int(tilt_percent - TILT_NUDGE_PCT, 0, 100);
                        tilt_to_percent_blocking(new_pct);
                    }
                } else {
                    // It's dark outside anyway: favor opening to grab what we can
                    int new_pct = clamp_int(tilt_percent + TILT_NUDGE_PCT, 0, 100);
                    tilt_to_percent_blocking(new_pct);
                }
            }
        }

        vTaskDelay(pdMS_TO_TICKS(AUTO_LOOP_PERIOD_MS));
    }
}

void start_auto_light_task(void)
{
    xTaskCreatePinnedToCore(auto_light_task, "auto_light", 4096, NULL, 5, NULL, tskNO_AFFINITY);
}

// Forward declaration for local renderer
static esp_err_t render_home(httpd_req_t *req);


static esp_err_t auto_post_handler(httpd_req_t *req)
{
    char buf[64];
    int ret = httpd_req_recv(req, buf, sizeof(buf) - 1);
    if (ret <= 0) { httpd_resp_send_500(req); return ESP_FAIL; }
    buf[ret] = '\0';

    if (strstr(buf, "mode=off"))         auto_set_mode(AUTO_OFF);
    else if (strstr(buf, "mode=low"))    auto_set_mode(AUTO_LOW);
    else if (strstr(buf, "mode=medlow")) auto_set_mode(AUTO_MED_LOW);
    else if (strstr(buf, "mode=med"))    auto_set_mode(AUTO_MED);
    else if (strstr(buf, "mode=medhi"))  auto_set_mode(AUTO_MED_HIGH);
    else if (strstr(buf, "mode=high"))   auto_set_mode(AUTO_HIGH);
    else {
        httpd_resp_set_status(req, "400 Bad Request");
        return httpd_resp_sendstr(req, "Unknown auto mode");
    }

    // Re-render the homepage with updated mode/status
    return render_home(req);
}


// ===== Page rendering (chunked, low stack) =====
static inline esp_err_t _send_chunk(httpd_req_t *req, const char *s) {
    return httpd_resp_sendstr_chunk(req, s);
}

static esp_err_t render_home(httpd_req_t *req) {
    const int pos = blinds_position;

    httpd_resp_set_type(req, "text/html");
    httpd_resp_set_hdr(req, "Cache-Control", "no-store");

    ESP_ERROR_CHECK(_send_chunk(req,
        "<!DOCTYPE html><html><head><title>Blinds Control</title>"
        "<meta name='viewport' content='width=device-width,initial-scale=1'/>"
        "<style>"
        "h1,h2{text-align:center;margin:8px 0}"
        ".row{display:flex;justify-content:center;gap:12px;margin-top:12px;flex-wrap:wrap}"
        "button{font-size:20px;padding:10px 18px;border-radius:10px;"
            "background:#4CAF50;color:#fff;border:none;cursor:pointer}"
        "button:hover{background:#45a049}"
        "button:disabled{background:#8f8f8f;cursor:not-allowed}"
        "body{font-family:system-ui,-apple-system,Segoe UI,Roboto,Arial}"
        "</style></head><body><h1>Set Light Level</h1>"
    ));

      // Automatic light level controls
    ESP_ERROR_CHECK(_send_chunk(req,
        "<h2>Automatic Light Level</h2>"
        "<form method='POST' action='/auto'><div class='row'>"
        "<button name='mode' value='off'>Manual</button>"
        "<button name='mode' value='low'>Low</button>"
        "<button name='mode' value='medlow'>Med-Low</button>"
        "<button name='mode' value='med'>Medium</button>"
        "<button name='mode' value='medhi'>Med-High</button>"
        "<button name='mode' value='high'>High</button>"
        "</div></form>"
    ));


    // Tilt controls
    ESP_ERROR_CHECK(_send_chunk(req,
        "<form method='POST' action='/tilt'><div class='row'>"
        "<button name='tilt' value='open'>Open Tilt</button>"
        "<button name='tilt' value='close'>Close Tilt</button>"
        "</div></form>"
        "<form method='POST' action='/stop'><div class='row'>"
        "<button>EMERGENCY STOP</button>"
        "</div></form>"
        "<form method='POST' action='/recalibrate'><div class='row'>"
        "<button>Recalibrate</button>"
        "</div></form>"
    ));

    char line[192];
   extern volatile float g_lux_difference;  // in http_server.c

snprintf(line, sizeof(line),
         "<h2>Lux A (outside): %.1f lx</h2>"
         "<h2>Lux B (inside): %.1f lx</h2>"
         "<h2>Lux Difference: %.1f lx</h2>",
         g_latest_lux1, g_latest_lux2, g_lux_difference);

    ESP_ERROR_CHECK(_send_chunk(req, line));

snprintf(line, sizeof(line),
         "<h2>Current Blinds Position: %d%%</h2>"
         "<h2>Tilt: %s (%d%% open)</h2>"
         "<h2>Auto Mode: %s</h2>"
         "<h2>Fully Closed?: %s</h2>"
         "<h2>Busy: Lift=%s, Tilt=%s</h2>",
         pos,
         (tilt_status==0) ? "OPEN" : "CLOSED",
         tilt_percent,
         auto_mode_str(s_auto_mode),
         (global_is_closed==1) ? "YES" : "NO",
         __atomic_load_n(&s_moving, __ATOMIC_SEQ_CST) ? "YES" : "NO",
         __atomic_load_n(&s_tilting, __ATOMIC_SEQ_CST) ? "YES" : "NO");

    ESP_ERROR_CHECK(_send_chunk(req, line));

    snprintf(line, sizeof(line),
    "</body></html>");

    ESP_ERROR_CHECK(_send_chunk(req, line));

    return httpd_resp_sendstr_chunk(req, NULL);
}

// ===== HTTP handlers =====
static esp_err_t homepage_handler(httpd_req_t *req) { return render_home(req); }

static esp_err_t get_handler(httpd_req_t *req) {
    httpd_resp_set_type(req, "text/plain");
    return httpd_resp_sendstr(req, "Hello from /get!");
}

static esp_err_t task_post_handler(httpd_req_t *req) {
    char content[96];
    int received = httpd_req_recv(req, content, sizeof(content) - 1);
    if (received <= 0) { httpd_resp_send_500(req); return ESP_FAIL; }
    content[received] = '\0';
    ESP_LOGI(TAG, "Task: %s", content);
    httpd_resp_set_type(req, "text/plain");
    return httpd_resp_sendstr(req, "Task received");
}

static esp_err_t stop_post_handler(httpd_req_t *req) {
    // Hard stop both subsystems
    motor_stop();
    tilt_motor_stop();
    // Optionally drop standby to guarantee off:
    // gpio_set_level(STBY_GPIO, 0);

    __atomic_store_n(&s_moving, false, __ATOMIC_SEQ_CST);
    __atomic_store_n(&s_tilting, false, __ATOMIC_SEQ_CST);
    httpd_resp_set_type(req, "text/plain");
    return httpd_resp_sendstr(req, "All motors stopped");
}

static esp_err_t tilt_post_handler(httpd_req_t *req) {
    char buf[64];
    int ret = httpd_req_recv(req, buf, sizeof(buf) - 1);
    if (ret <= 0) {
        httpd_resp_send_500(req);
        return ESP_FAIL;
    }
    buf[ret] = '\0';

    // Manual interaction -> exit auto mode
    auto_set_mode(AUTO_OFF);

    const int STEP = 10;  // 10% per click; tweak this if you want bigger/smaller nudges

    if (strstr(buf, "tilt=open")) {
        int target = clamp_int(tilt_percent + STEP, 0, 100);
        tilt_to_percent_blocking(target);
    } else if (strstr(buf, "tilt=close")) {
        int target = clamp_int(tilt_percent - STEP, 0, 100);
        tilt_to_percent_blocking(target);
    } else {
        httpd_resp_set_status(req, "400 Bad Request");
        return httpd_resp_sendstr(req, "Unknown tilt command");
    }

    return render_home(req);
}

static esp_err_t blinds_post_handler(httpd_req_t *req) {
    char buf[96];
    int ret = httpd_req_recv(req, buf, sizeof(buf) - 1);
    if (ret <= 0) { httpd_resp_send_500(req); return ESP_FAIL; }
    buf[ret] = '\0';

    // Manual interaction -> exit auto mode
    auto_set_mode(AUTO_OFF);

    int requested = blinds_position;

    // Interpret request
    if (strstr(buf, "level=light"))       requested = 0;
    else if (strstr(buf, "level=medium")) requested = 50;
    else if (strstr(buf, "level=dark"))   requested = 100;
    else if (strstr(buf, "delta=up10"))   requested = clamp_int(blinds_position - 10, 0, 100);
    else if (strstr(buf, "delta=down10")) requested = clamp_int(blinds_position + 10, 0, 100);
    else {
        httpd_resp_set_status(req, "400 Bad Request");
        return httpd_resp_sendstr(req, "Unknown command");
    }

    if (requested == blinds_position) {
        httpd_resp_set_status(req, "409 Conflict");
        return httpd_resp_sendstr(req, "No change");
    }

    // Pre-tilt close if needed so slats don't catch (only if no tilt active)
    //if (!__atomic_load_n(&s_tilting, __ATOMIC_SEQ_CST) && tilt_status != 1) {
    //    tilt_blinds_async(1); // CLOSE
    //}

    // Move lift (async)
    ESP_LOGI(TAG, "Move %d%% -> %d%%", blinds_position, requested);
    move_blinds_to_percent_time_async(requested);
    blinds_position = requested;      // optimistic

    global_is_closed = (requested >= 100) ? 1 : 0;

    // Post-move tilt preference (only if not tilting)
    //if (!__atomic_load_n(&s_tilting, __ATOMIC_SEQ_CST)) {
    //    tilt_blinds_async(requested == 50 ? 0 : 1);
    //}     

    return render_home(req);
}

static esp_err_t recalibrate_handler(httpd_req_t *req) 
{
    xTaskCreatePinnedToCore(recal_task, "recal_task", 3072, NULL, 7, NULL, tskNO_AFFINITY);
    httpd_resp_set_type(req, "text/plain");
    return httpd_resp_sendstr(req, "Recalibration started");
}

// /status handler
static esp_err_t status_handler(httpd_req_t *req) {
    char response[160];
    snprintf(response, sizeof(response),
             "Blinds Position: %d%%, Tilt:%s, FullyClosed:%s, Busy(Lift)=%s, Busy(Tilt)=%s",
             blinds_position,
             (tilt_status==0) ? "OPEN" : "CLOSED",
             (global_is_closed==1) ? "YES" : "NO",
             __atomic_load_n(&s_moving, __ATOMIC_SEQ_CST) ? "YES" : "NO",
             __atomic_load_n(&s_tilting, __ATOMIC_SEQ_CST) ? "YES" : "NO");
    httpd_resp_set_type(req, "text/plain");
    return httpd_resp_send(req, response, HTTPD_RESP_USE_STRLEN);
}

// ===== HTTP server bootstrap & routes =====
static void register_uris(httpd_handle_t server) {
    const httpd_uri_t homepage_uri   = {.uri="/",           .method=HTTP_GET,  .handler=homepage_handler};
    const httpd_uri_t get_uri        = {.uri="/get",        .method=HTTP_GET,  .handler=get_handler};
    const httpd_uri_t task_uri       = {.uri="/task",       .method=HTTP_POST, .handler=task_post_handler};
    const httpd_uri_t blinds_uri     = {.uri="/blinds",     .method=HTTP_POST, .handler=blinds_post_handler};
    const httpd_uri_t tilt_uri       = {.uri="/tilt",       .method=HTTP_POST, .handler=tilt_post_handler};
    const httpd_uri_t stop_uri       = {.uri="/stop",       .method=HTTP_POST, .handler=stop_post_handler};
    const httpd_uri_t status_uri     = {.uri="/status",     .method=HTTP_GET,  .handler=status_handler};
    const httpd_uri_t recal_uri      = {.uri="/recalibrate",.method=HTTP_POST, .handler=recalibrate_handler};

    httpd_register_uri_handler(server, &homepage_uri);
    httpd_register_uri_handler(server, &get_uri);
    httpd_register_uri_handler(server, &task_uri);
    httpd_register_uri_handler(server, &blinds_uri);
    httpd_register_uri_handler(server, &tilt_uri);
    httpd_register_uri_handler(server, &stop_uri);
    httpd_register_uri_handler(server, &status_uri);
    httpd_register_uri_handler(server, &recal_uri);
    const httpd_uri_t auto_uri = {
    .uri = "/auto",
    .method = HTTP_POST,
    .handler = auto_post_handler
    };
    httpd_register_uri_handler(server, &auto_uri);
}

httpd_handle_t start_webserver(void) {
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.stack_size = 10240;        // bigger stack for handlers/logging safety
    config.max_uri_handlers = 12;     // allow more than the default 8

    httpd_handle_t server = NULL;
    esp_err_t err = httpd_start(&server, &config);
    if (err == ESP_OK && server) {
        ESP_LOGI(TAG, "httpd started on port %d", config.server_port);
        register_uris(server);
    } else {
        ESP_LOGE(TAG, "httpd_start failed: %s", esp_err_to_name(err));
    }
    return server;
}
