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

//LIDT MOTOR
#define AIN1_GPIO              38  
#define AIN2_GPIO              37  
#define ENA_PWMA_GPIO          39   //pwm gpio

//TILT MOTOR
#define TILT_IN1_GPIO          40   
#define TILT_IN2_GPIO          41  
#define ENB_PWMB_GPIO          42   //pwm tilt gpio

#define STBY_GPIO              36   //ALWAYS HIGH
#define FORWARD_IS_CW          1    // 1=forward is clockwise 0=forward is CCW
#define STOP_MODE_BRAKE        1    // 1=brake 0=coast

//pwm
#define LEDC_MODE              LEDC_LOW_SPEED_MODE
#define LEDC_TIMER             LEDC_TIMER_0
#define LEDC_CH_LIFT           LEDC_CHANNEL_0   //pwma channel
#define LEDC_CH_TILT           LEDC_CHANNEL_1   //pwmb channel
#define LEDC_FREQ_HZ           20000            //20 khz
#define LEDC_DUTY_RES          LEDC_TIMER_10_BIT
#define LEDC_DUTY_MAX          ((1U<<10)-1)

//lift timing
#define FULL_TRAVEL_MS         40000            //40s travel time
#define RUN_SPEED_PCT          90               // duty % during lift motion

//tilt timing
#define TILT_OPEN_MS           400  //close to open
#define TILT_CLOSE_MS          400  //open to close
#define TILT_RUN_PCT           60   //pwm% for tilt

extern int blinds_position;
extern volatile bool s_moving;
extern volatile bool s_tilting;
extern int tilt_status; //0=open 1=closed
extern int tilt_percent;
extern int global_is_closed;

extern volatile bool g_emergency_stop;

int  clamp_int(int v, int lo, int hi);

void motor_init(void);
void motor_stop(void);
void tilt_motor_stop(void);
void move_blinds_to_percent_time_async(int target_percent);
void tilt_blinds_async(int desired_tilt_status);
void tilt_to_percent_blocking(int target_pct);
void recal_task(void *pv);
void blinds_motion_abort_update_position(void);

#endif 