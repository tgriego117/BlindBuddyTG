// signal.h
// signal.h
#ifndef SIGNAL_H
#define SIGNAL_H

#include <stdbool.h>
#include <stdint.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "esp_log.h"

// ===== Pins & motor timing (match http_server.c mapping) =====
// Lift (motor A)
#define AIN1_GPIO              38   // Lift motor AIN1
#define AIN2_GPIO              37   // Lift motor AIN2
#define ENA_PWMA_GPIO          39   // Lift motor ENA / PWMA (LEDC PWM output)

// Tilt (motor B)
#define TILT_IN1_GPIO          40   // Tilt motor IN1
#define TILT_IN2_GPIO          41   // Tilt motor IN2
#define ENB_PWMB_GPIO          42   // Tilt motor ENB / PWMB (LEDC PWM output)

#define STBY_GPIO              36   // H-bridge standby (active HIGH)

// Map "forward" to CW or CCW (depends on your mechanical setup)
#define FORWARD_IS_CW          1    // 1: forward=CW ; 0: forward=CCW

// Choose how to stop the LIFT motor at the end of a move
#define STOP_MODE_BRAKE        1    // 1: brake (AIN1=AIN2=1), 0: coast (AIN1=AIN2=0)

// PWM settings (2 channels on the same timer)
#define LEDC_MODE              LEDC_LOW_SPEED_MODE
#define LEDC_TIMER             LEDC_TIMER_0
#define LEDC_CH_LIFT           LEDC_CHANNEL_0   // channel for PWMA
#define LEDC_CH_TILT           LEDC_CHANNEL_1   // channel for PWMB
#define LEDC_FREQ_HZ           20000            // 20 kHz
#define LEDC_DUTY_RES          LEDC_TIMER_10_BIT
#define LEDC_DUTY_MAX          ((1U<<10)-1)

// Motion model (lift travel timing model)
#define FULL_TRAVEL_MS         48000            // 0% <-> 100% travel = 30 s
#define RUN_SPEED_PCT          90               // duty % during lift motion

// Tilt timing (slat open/close jog)
#define TILT_OPEN_MS           400  // ms spin to go CLOSED -> OPEN
#define TILT_CLOSE_MS          400  // ms spin to go OPEN   -> CLOSED
#define TILT_RUN_PCT           60   // PWM % for tilt jog

// ===== Shared state exported to HTTP/UI =====
extern int blinds_position;
extern volatile bool s_moving;
extern volatile bool s_tilting;
// tilt_status: 0 = slats open, 1 = slats closed
extern int tilt_status;
extern int tilt_percent;
extern int global_is_closed;

extern volatile bool g_emergency_stop;


// ===== API used by http_server.c =====
int  clamp_int(int v, int lo, int hi);

void motor_init(void);
void motor_stop(void);
void tilt_motor_stop(void);

void move_blinds_to_percent_time_async(int target_percent);
void tilt_blinds_async(int desired_tilt_status);
// used by auto-light controller for fine nudges
void tilt_to_percent_blocking(int target_pct);

// same task you were previously creating from the HTTP handler
void recal_task(void *pv);

#endif // SIGNAL_H