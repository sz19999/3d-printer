#ifndef THERMAL_TASK_H
#define THERMAL_TASK_H

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "pid_autotune.h"
#include "pid_controller.h"

/* Queue Item Payload */
typedef struct {
    int cmd_num;
    union {
        float target_temp;
        struct {
            float target_temp;
            tuning_method_t method;
        } autotune;
        struct {
            float kp;
            float ki;
            float kd;
        } pid_gains;
    } payload;
} thermal_cmd_t;

/* Exported Queue Handle and Task Entry Point */
extern QueueHandle_t xThermalCommandQueue;
void vThermalTask(void *pvParameters);

/* Thread-safe Command Enqueue Helpers */
bool thermal_set_target_temp(float target_temp);
bool thermal_start_autotune(float target_temp, tuning_method_t method);
bool thermal_turn_off(void);
bool thermal_update_gains(float kp, float ki, float kd);

#endif // THERMAL_TASK_H