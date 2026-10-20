/**
 * @file    pid.c
 * @brief   通用位置式 PID 控制器实现
 */
#include "pid.h"

/* ========================================================================== */
/*                                  接口实现                                   */
/* ========================================================================== */

void Pid_Init(pid_t *pid,
              float kp, float ki, float kd,
              float output_min, float output_max,
              float integral_limit)
{
    if (pid == 0)
    {
        return;
    }

    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;

    pid->integral       = 0.0f;
    pid->last_error     = 0.0f;
    pid->integral_limit = integral_limit;
    pid->output_min     = output_min;
    pid->output_max     = output_max;
}

void Pid_SetGains(pid_t *pid, float kp, float ki, float kd)
{
    if (pid == 0)
    {
        return;
    }

    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
}

void Pid_Reset(pid_t *pid, float initial_error)
{
    if (pid == 0)
    {
        return;
    }

    pid->integral   = 0.0f;
    pid->last_error = initial_error;
}

float Pid_Update(pid_t *pid, float error, float dt_s)
{
    float derivative;
    float output;

    if ((pid == 0) || (dt_s <= 0.0f))
    {
        return 0.0f;
    }

    /* 1. 积分项累加，并按 integral_limit 限幅（抗积分饱和） */
    pid->integral += error * dt_s;
    if (pid->integral_limit > 0.0f)
    {
        if (pid->integral > pid->integral_limit)
        {
            pid->integral = pid->integral_limit;
        }
        else if (pid->integral < -pid->integral_limit)
        {
            pid->integral = -pid->integral_limit;
        }
    }

    /* 2. 微分项：用误差差分，避免设定值突变引起微分冲击 */
    derivative = (error - pid->last_error) / dt_s;
    pid->last_error = error;

    /* 3. 合成输出 */
    output = (pid->kp * error)
           + (pid->ki * pid->integral)
           + (pid->kd * derivative);

    /* 4. 输出限幅 */
    if (output > pid->output_max)
    {
        output = pid->output_max;
    }
    else if (output < pid->output_min)
    {
        output = pid->output_min;
    }

    return output;
}
