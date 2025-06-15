#ifndef pid_H
#define pid_H

#include "main.h"
#include "stdio.h"


typedef struct {
    float kp, ki, kd;
    float error, last_error, integral;
    float setpoint;
    float output_min, output_max;
	float output;
	float integral_max;
} PID_TypeDef;

#define temp_target 22
#define pid_max 100
#define pid_min -100
#define temp_kp 1.0
#define temp_ki 0.1
#define temp_kd 0

void pid_Init(PID_TypeDef *pid);
void PID_Update(PID_TypeDef *pid, float current_temp);
void Temperature_Control_Task(void);  // 主控制任务

#endif
