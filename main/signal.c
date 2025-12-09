// signal.c
#include "signal.h"

static const char *TAG = "SIGNAL";

// ===== Shared state =====
int blinds_position      = 0;
volatile bool s_moving   = false;
volatile bool s_tilting  = false;
// tilt_status: 0 = slats open, 1 = slats closed
int tilt_status          = 1;   // assume start CLOSED
int tilt_percent         = 0;   // 0% open at boot
int global_is_closed     = 0;   // assume blinds not fully down yet

int clamp_int(int v, int lo, int hi)
{
    return v < lo ? lo : (v > hi ? hi : v);
}

// ===== Lift motor (DC H-bridge) helpers =====
static inline void motor_apply(bool ain1, bool ain2)
{
    gpio_set_level(AIN1_GPIO, ain1 ? 1 : 0);
    gpio_set_level(AIN2_GPIO, ain2 ? 1 : 0);
}

static inline void motor_set_cw(void)    { motor_apply(false, true); }  // CW
static inline void motor_set_ccw(void)   { motor_apply(true,  false); } // CCW
static inline void motor_brake(void)     { motor_apply(true,  true ); }
static inline void motor_coast(void)     { motor_apply(false, false); }

static inline void motor_pwm_set_percent(int pct)
{
    if (pct < 0)   pct = 0;
    if (pct > 100) pct = 100;
    uint32_t duty = (LEDC_DUTY_MAX * (uint32_t)pct) / 100U;
    ledc_set_duty(LEDC_MODE, LEDC_CH_LIFT, duty);
    ledc_update_duty(LEDC_MODE, LEDC_CH_LIFT);
}

// dir: 1 = forward, 0 = backward
static inline void motor_set_dir(int dir)
{
    const bool forward = (dir != 0);
    const bool want_cw = FORWARD_IS_CW ? forward : !forward;
    if (want_cw) motor_set_cw(); else motor_set_ccw();
}

void motor_stop(void)
{
    motor_pwm_set_percent(0);
#if STOP_MODE_BRAKE
    motor_brake(); // holds position better
#else
    motor_coast(); // free spin
#endif
}

static void motor_run_for_ms_blocking(int dir, int ms)
{
    if (ms <= 0) { motor_stop(); return; }
    motor_set_dir(dir);
    motor_pwm_set_percent(RUN_SPEED_PCT);
    vTaskDelay(pdMS_TO_TICKS(ms));
    motor_stop();
}

// ===== Async move task for lift motor =====
typedef struct {
    int dir;            // 1=fwd, 0=back
    int ms;             // duration to run
    int target_percent;
} move_args_t;

static void move_task(void *pv) 
{
    move_args_t *a = (move_args_t *)pv;
    motor_run_for_ms_blocking(a->dir, a->ms);

    blinds_position  = a->target_percent;
    global_is_closed = (blinds_position >= 100) ? 1 : 0;

    __atomic_store_n(&s_moving, false, __ATOMIC_SEQ_CST);
    vPortFree(a);
    vTaskDelete(NULL);
}


void move_blinds_to_percent_time_async(int target_percent)
{
    target_percent = clamp_int(target_percent, 0, 100);
    int cur = blinds_position;
    if (target_percent == cur) return;

    // prevent overlap with other lift motion or any ongoing tilt
    if (__atomic_load_n(&s_tilting, __ATOMIC_SEQ_CST)) {
        ESP_LOGW(TAG, "tilt active; ignoring lift move to %d%%", target_percent);
        return;
    }
    if (__atomic_exchange_n(&s_moving, true, __ATOMIC_SEQ_CST)) {
        ESP_LOGW(TAG, "lift moving; ignoring request to %d%%", target_percent);
        return;
    }

    int dir       = (target_percent > cur) ? 1 : 0;
    int delta_pct = dir ? (target_percent - cur) : (cur - target_percent);
    int ms        = (delta_pct * FULL_TRAVEL_MS) / 100;

    move_args_t *a = pvPortMalloc(sizeof(*a));
    if (!a) {
        ESP_LOGE(TAG, "malloc failed");
        __atomic_store_n(&s_moving, false, __ATOMIC_SEQ_CST);
        return;
    }
    a->dir            = dir;
    a->ms             = ms;
    a->target_percent = target_percent;

    if (xTaskCreatePinnedToCore(move_task, "move_task", 3072, a, 7, NULL, tskNO_AFFINITY) != pdPASS) {
        ESP_LOGE(TAG, "xTaskCreate(move_task) failed");
        vPortFree(a);
        __atomic_store_n(&s_moving, false, __ATOMIC_SEQ_CST);
    }
}

// ===== Tilt motor control (PWM on ENB_PWMB_GPIO) =====
static inline void tilt_pwm_set_percent(int pct)
{
    if (pct < 0)   pct = 0;
    if (pct > 100) pct = 100;
    uint32_t duty = (LEDC_DUTY_MAX * (uint32_t)pct) / 100U;
    ledc_set_duty(LEDC_MODE, LEDC_CH_TILT, duty);
    ledc_update_duty(LEDC_MODE, LEDC_CH_TILT);
}

static inline void tilt_motor_apply(bool in1, bool in2)
{
    gpio_set_level(TILT_IN1_GPIO, in1 ? 1 : 0);
    gpio_set_level(TILT_IN2_GPIO, in2 ? 1 : 0);
}

void tilt_motor_stop(void)
{
    // your board stops on low/low, and we also zero PWM
    tilt_pwm_set_percent(0);
    tilt_motor_apply(false, false);
}

static void tilt_run_for_ms_blocking(bool open_direction, int ms)
{
    // Safety: never run if lift is moving
    if (__atomic_load_n(&s_moving, __ATOMIC_SEQ_CST)) {
        ESP_LOGW(TAG, "lift moving; ignoring tilt jog");
        return;
    }
    if (ms <= 0) { tilt_motor_stop(); return; }

    // Drive
    if (open_direction) {
        tilt_motor_apply(false, true);   // OPEN polarity
    } else {
        tilt_motor_apply(true, false);   // CLOSE polarity
    }
    tilt_pwm_set_percent(TILT_RUN_PCT);

    // Timed jog
    vTaskDelay(pdMS_TO_TICKS(ms));

    // Always stop
    tilt_motor_stop();
}

// Core tilt move (no s_tilting lock; caller must ensure exclusivity)
static void tilt_to_percent_blocking_core(int target_pct)
{
    target_pct = clamp_int(target_pct, 0, 100);
    int cur    = tilt_percent;
    if (target_pct == cur) {
        return;
    }

    bool open_direction = (target_pct > cur);
    int  delta          = open_direction ? (target_pct - cur) : (cur - target_pct);

    int base_ms = open_direction ? TILT_OPEN_MS : TILT_CLOSE_MS;
    int ms      = (delta * base_ms) / 100;
    if (ms < 10) ms = 10; // minimum jog

    tilt_run_for_ms_blocking(open_direction, ms);

    tilt_percent = target_pct;
    // 1 = CLOSED when 0% open (your convention)
    tilt_status  = (tilt_percent == 0) ? 1 : 0;
}


// Move tilt to a specific "percent open" using timing model
void tilt_to_percent_blocking(int target_pct)
{
    if (__atomic_load_n(&s_moving, __ATOMIC_SEQ_CST)) {
        ESP_LOGW(TAG, "lift moving; skip tilt_to_percent");
        return;
    }
    if (__atomic_exchange_n(&s_tilting, true, __ATOMIC_SEQ_CST)) {
        ESP_LOGW(TAG, "tilt busy; skip tilt_to_percent");
        return;
    }

    tilt_to_percent_blocking_core(target_pct);

    __atomic_store_n(&s_tilting, false, __ATOMIC_SEQ_CST);
}

// desired_tilt_status: 0=open, 1=closed  (BLOCKING helper)
static void tilt_blinds(int desired_tilt_status)
{
    desired_tilt_status = desired_tilt_status ? 1 : 0;
    if (desired_tilt_status == tilt_status) return;

    if (desired_tilt_status == 0) {
        // fully open
        tilt_to_percent_blocking_core(100);
    } else {
        // fully closed
        tilt_to_percent_blocking_core(0);
    }
}


// === Async tilt wrapper (never overlap with lift or other tilt) ===
typedef struct { int desired_tilt_status; } tilt_args_t;

static void tilt_task(void *pv)
{
    tilt_args_t *a = (tilt_args_t*)pv;

    // Guard enter
    if (__atomic_load_n(&s_moving, __ATOMIC_SEQ_CST)) {
        ESP_LOGW(TAG, "lift moving; skip tilt request");
        goto out;
    }
    if (__atomic_exchange_n(&s_tilting, true, __ATOMIC_SEQ_CST)) {
        ESP_LOGW(TAG, "tilt already active; skip");
        goto out;
    }

    // Do the work
    tilt_blinds(a->desired_tilt_status);

    // Guard exit
    __atomic_store_n(&s_tilting, false, __ATOMIC_SEQ_CST);

out:
    vPortFree(a);
    vTaskDelete(NULL);
}

void tilt_blinds_async(int desired_tilt_status)
{
    // refuse if lift is moving
    if (__atomic_load_n(&s_moving, __ATOMIC_SEQ_CST)) {
        ESP_LOGW(TAG, "lift moving; not starting tilt");
        return;
    }
    tilt_args_t *a = pvPortMalloc(sizeof(*a));
    if (!a) return;
    a->desired_tilt_status = desired_tilt_status ? 1 : 0;
    if (xTaskCreatePinnedToCore(tilt_task, "tilt_task", 2048, a, 6, NULL, tskNO_AFFINITY) != pdPASS) {
        vPortFree(a);
        ESP_LOGE(TAG, "xTaskCreate(tilt_task) failed");
    }
}

// ===== Recalibration sequence (used by HTTP handler) =====
void recal_task(void *pv)
{
    (void)pv;
    motor_stop();
    tilt_motor_stop();

    // Ensure slats CLOSED (blocking)
    if (!__atomic_load_n(&s_moving, __ATOMIC_SEQ_CST)) {
        tilt_blinds(1);
    }

    // Run lift fully down
    move_blinds_to_percent_time_async(100);
    blinds_position  = 100;
    global_is_closed = 1;

    // Final guarantee tilt is closed
    tilt_blinds(1);
    vTaskDelete(NULL);
}

// ===== HW init (call once in app_main) =====
void motor_init(void)
{
    // Lift motor IN pins
    gpio_config_t io = {
        .pin_bit_mask = (1ULL<<AIN1_GPIO) | (1ULL<<AIN2_GPIO),
        .mode         = GPIO_MODE_OUTPUT,
        .pull_up_en   = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_DISABLE,
    };
    ESP_ERROR_CHECK(gpio_config(&io));
    gpio_set_level(AIN1_GPIO, 0);
    gpio_set_level(AIN2_GPIO, 0);

    // Standby pin
    gpio_config_t stby_io = {
        .pin_bit_mask = (1ULL << STBY_GPIO),
        .mode         = GPIO_MODE_OUTPUT,
        .pull_up_en   = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_DISABLE,
    };
    ESP_ERROR_CHECK(gpio_config(&stby_io));
    ESP_ERROR_CHECK(gpio_set_level(STBY_GPIO, 1));
    vTaskDelay(pdMS_TO_TICKS(5));

    // LEDC timer
    ledc_timer_config_t tcfg = {
        .speed_mode      = LEDC_MODE,
        .timer_num       = LEDC_TIMER,
        .duty_resolution = LEDC_DUTY_RES,
        .freq_hz         = LEDC_FREQ_HZ,
        .clk_cfg         = LEDC_AUTO_CLK
    };
    ESP_ERROR_CHECK(ledc_timer_config(&tcfg));

    // LEDC channel for lift (PWMA)
    ledc_channel_config_t ccfg_a = {
        .gpio_num   = ENA_PWMA_GPIO,
        .speed_mode = LEDC_MODE,
        .channel    = LEDC_CH_LIFT,
        .timer_sel  = LEDC_TIMER,
        .duty       = 0,
        .hpoint     = 0,
        .intr_type  = LEDC_INTR_DISABLE
    };
    ESP_ERROR_CHECK(ledc_channel_config(&ccfg_a));

    // LEDC channel for tilt (PWMB)
    ledc_channel_config_t ccfg_b = {
        .gpio_num   = ENB_PWMB_GPIO,
        .speed_mode = LEDC_MODE,
        .channel    = LEDC_CH_TILT,
        .timer_sel  = LEDC_TIMER,
        .duty       = 0,
        .hpoint     = 0,
        .intr_type  = LEDC_INTR_DISABLE
    };
    ESP_ERROR_CHECK(ledc_channel_config(&ccfg_b));

    // Tilt motor pins
    gpio_config_t tilt_io = {
        .pin_bit_mask = (1ULL<<TILT_IN1_GPIO) | (1ULL<<TILT_IN2_GPIO),
        .mode         = GPIO_MODE_OUTPUT,
        .pull_up_en   = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_DISABLE,
    };
    ESP_ERROR_CHECK(gpio_config(&tilt_io));
    tilt_motor_stop();

    // Startup assumed state
    motor_stop();
    tilt_status      = 1;
    tilt_percent     = 0;
    blinds_position  = 0;
    global_is_closed = 0;

    ESP_LOGI(TAG,
             "Motor init: lift AIN1=%d AIN2=%d ENA(PWM)=%d | tilt IN1=%d IN2=%d ENB(PWM)=%d | STBY=%d",
             AIN1_GPIO, AIN2_GPIO, ENA_PWMA_GPIO,
             TILT_IN1_GPIO, TILT_IN2_GPIO, ENB_PWMB_GPIO,
             STBY_GPIO);
}
