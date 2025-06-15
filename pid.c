#include "pid.h"
#include "ds18b20.h"
#include "motor.h"

// 温度PID控制器实例
PID_TypeDef pid_temp;
static uint8_t pid_initialized = 0;  // 初始化标志

// PID控制器初始化
void pid_Init(PID_TypeDef *pid)
{
	// 设置PID参数
	pid->kp = temp_kp;
	pid->ki = temp_ki;
	pid->kd = temp_kd;
	pid->setpoint = temp_target;
	pid->output_min = pid_min;
	pid->output_max = pid_max;
	pid->integral_max = 50.0f;  // 设置积分限制
	
	// 初始化所有状态变量
	pid->integral = 0.0f;       // 初始化积分项
	pid->error = 0.0f;          // 初始化误差
	pid->last_error = 0.0f;     // 初始化上次误差
	pid->output = 0.0f;         // 初始化输出
}

// PID控制器更新计算
void PID_Update(PID_TypeDef *pid, float current_temp)
{	
	// 计算当前误差
	pid->last_error = pid->error;
	pid->error = pid->setpoint - current_temp;  // 目标温度 - 实际温度
	pid->integral += pid->error;                // 积分累加
	
	// 积分饱和保护（双向限制）
	if(pid->integral > pid->integral_max)	
		pid->integral = pid->integral_max;
	else if(pid->integral < -pid->integral_max)	
		pid->integral = -pid->integral_max;
	
	// PID公式计算输出
	pid->output = pid->kp * pid->error + pid->ki * pid->integral + pid->kd * (pid->error - pid->last_error);
	
	// PID输出限制
	if(pid->output > pid->output_max)
		pid->output = pid->output_max;
	else if(pid->output < pid->output_min)
		pid->output = pid->output_min;
}

// 温度控制主任务
void Temperature_Control_Task(void)
{
	// 确保PID已初始化
	if(!pid_initialized)
	{
		pid_Init(&pid_temp);
		pid_initialized = 1;
	}
	
	// 读取DS18B20温度传感器
	float curr_temp = ds18b20_get_temperature();
	
	// 温度数据有效性检查
	if(curr_temp == DS18B20_ERROR_TEMP || curr_temp < -50.0f || curr_temp > 100.0f)
	{
		// 温度读取失败，停止风扇，保持安全状态
		Motor_SetSpeed(0);
		return;
	}
	
	// 执行PID控制算法
	PID_Update(&pid_temp, curr_temp);
	
	// 将PID输出转换为风扇转速
	int32_t speed;
	if(pid_temp.output < 0)
	{
		// 温度高于目标，需要制冷，启动风扇
		speed = (int32_t)(-pid_temp.output);
		if(speed > 99)	speed = 99;     // 限制最大转速
		Motor_SetSpeed(speed);
	}
	else
	{
		// 温度低于目标，不需要制冷，停止风扇
		speed = 0;
		Motor_SetSpeed(speed);
	}
}
