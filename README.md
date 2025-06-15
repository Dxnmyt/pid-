# PID算法笔记：基于温度控制风扇速度

## 这是什么？

PID算法是一种经典的控制方法，全称是**比例（Proportional）**、**积分（Integral）**、**微分（Derivative）**。它通过这三部分协作，自动调整系统达到我们想要的状态。

在这个例子中，我们用PID算法控制风扇速度，让房间温度尽量保持在22°C。简单来说：

- **温度高了**，风扇转快点。
- **温度低了**，风扇慢下来甚至停转。

---

## PID算法怎么工作？

PID算法的核心是根据温度的“误差”（目标温度 - 实际温度）来调整风扇速度。它分三步走：

### 1. 比例（P）：快速反应

- **作用**：看当前温度偏离目标多少，立刻调整风扇速度。
- **公式**：`kp * 误差`（代码里`kp`是1.0）。
- **特点**：反应快，但单独用可能稳不住目标值。
- **举例**：温度是25°C，目标22°C，误差3°C，风扇就稍微加速。

### 2. 积分（I）：消除长期误差

- **作用**：把过去的误差累积起来，确保温度最终稳定在目标值。
- **公式**：`ki * integral`（代码里`ki`是0.1，`integral`是误差的累计）。
- **特点**：能解决小误差长期存在的问题，但调不好会让系统晃来晃去。
- **举例**：如果温度一直比22°C高一点点，积分会慢慢加大风扇速度。

### 3. 微分（D）：预测趋势

- **作用**：看误差变化的速度，防止温度调整过头。
- **公式**：`kd * (当前误差 - 上次误差)`（代码里`kd`是0，没用）。
- **特点**：能让系统更平稳，但温度变化慢时可以忽略。
- **举例**：如果温度快速下降，微分会减慢风扇，避免降过头。

---

## 你的PID设置

在你的代码里，PID参数是这样的：

- **kp = 1.0**（比例系数）
- **ki = 0.1**（积分系数）
- **kd = 0**（微分系数，没启用）
- **目标温度**：22°C
- **风扇速度范围**：代码里是-100到100，但实际用0到99。

---

## 代码怎么实现的？

### 1. 初始化PID (`pid_Init`)

- **做什么**：设置PID参数、目标值和输出范围，清零状态变量。
- **关键点**：
    - 积分上限设为50，防止累积太多。
    - 初始时误差、积分、输出都设为0。

```c
void pid_Init(PID_TypeDef *pid) {
    pid->kp = temp_kp;          // 1.0
    pid->ki = temp_ki;          // 0.1
    pid->kd = temp_kd;          // 0
    pid->setpoint = temp_target;// 22°C
    pid->output_min = pid_min;  // -100
    pid->output_max = pid_max;  // 100
    pid->integral_max = 50.0f;  // 积分限制

    pid->integral = 0.0f;
    pid->error = 0.0f;
    pid->last_error = 0.0f;
    pid->output = 0.0f;
}

```

### 2. 更新PID (`PID_Update`)

- **做什么**：根据当前温度算误差，更新输出。
- **步骤**：
    1. 计算误差：`目标温度 - 当前温度`。
    2. 累加积分，但限制在±50内。
    3. 用PID公式算输出：`P + I + D`。
    4. 输出限制在-100到100。

```c
void PID_Update(PID_TypeDef *pid, float current_temp) {
    pid->last_error = pid->error;
    pid->error = pid->setpoint - current_temp;
    pid->integral += pid->error;

    // 积分限制
    if (pid->integral > pid->integral_max)
        pid->integral = pid->integral_max;
    else if (pid->integral < -pid->integral_max)
        pid->integral = -pid->integral_max;

    // PID公式
    pid->output = pid->kp * pid->error + pid->ki * pid->integral + pid->kd * (pid->error - pid->last_error);

    // 输出限制
    if (pid->output > pid->output_max)
        pid->output = pid->output_max;
    else if (pid->output < pid->output_min)
        pid->output = pid->output_min;
}

```

### 3. 控制任务 (`Temperature_Control_Task`)

- **做什么**：读取温度，运行PID，调整风扇。
- **逻辑**：
    - 没初始化就先初始化。
    - 温度异常（比如传感器坏了），停风扇。
    - 温度高时，风扇转速根据PID输出调整（0到99）。
    - 温度低时，风扇停转。

```c
void Temperature_Control_Task(void) {
    if (!pid_initialized) {
        pid_Init(&pid_temp);
        pid_initialized = 1;
    }

    float curr_temp = ds18b20_get_temperature();

    if (curr_temp == DS18B20_ERROR_TEMP || curr_temp < -50.0f || curr_temp > 100.0f) {
        Motor_SetSpeed(0);
        return;
    }

    PID_Update(&pid_temp, curr_temp);

    int32_t speed;
    if (pid_temp.output < 0) {
        speed = (int32_t)(-pid_temp.output);
        if (speed > 99) speed = 99;
        Motor_SetSpeed(speed);
    } else {
        speed = 0;
        Motor_SetSpeed(speed);
    }
}

```

---

## 在实际中怎么用？

这个PID控制风扇的逻辑是：

- **温度 > 22°C**：PID输出负值，风扇转起来，转速和误差成正比。
- **温度 ≤ 22°C**：PID输出正值，风扇停转。

这样可以让温度稳定在22°C附近，避免忽快忽慢的烦人情况。

---
