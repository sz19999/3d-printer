#ifndef THERMAL_TASK_H
#define THERMAL_TASK_H

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "pid_autotune.h"
#include "pid_controller.h"

void thermal_task(void *pvParameters);

bool thermal_set_target_temp(float target_temp);
bool thermal_start_autotune(float target_temp, tuning_method_t method);
bool thermal_turn_off(void);
bool thermal_update_gains(float kp, float ki, float kd);

#endif // THERMAL_TASK_H