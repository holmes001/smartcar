/**
 * @file    motor.c
 * @brief   直流电机 PWM 驱动实现
 * @details 用 TIM12 / TIM15 的两个通道组成 H 桥的方向控制：
 *            - CH1 输出占空比、CH2 输出 0  -> 正转
 *            - CH1 输出 0、CH2 输出占空比  -> 反转
 *            - 两个通道都输出 0            -> 自由停车
 *
 * @note    本文件不再使用 "Motor_L / Motor_R" 宏去别名 HAL 句柄。
 *          句柄只存在于文件内部的 s_motor_tim[] 表里，对外完全隐藏。
 */
#include "motor.h"

#include "main.h"
#include "tim.h"

/* ========================================================================== */
/*                                内部常量与状态                                */
/* ========================================================================== */

/**
 * @brief 电机编号 -> 定时器句柄映射表
 * @note  索引必须与 motor_id_t 的取值一致：MOTOR_L 用 TIM15、MOTOR_R 用 TIM12。
 */
static TIM_HandleTypeDef *const s_motor_tim[MOTOR_COUNT] =
{
    &htim15,    /* MOTOR_L */
    &htim12     /* MOTOR_R */
};

/** @brief 最近一次设置的占空比（百分比，带符号），供 Motor_GetDuty() 查询 */
static float s_motor_duty[MOTOR_COUNT] = { 0.0f, 0.0f };

/* ========================================================================== */
/*                                  内部函数                                   */
/* ========================================================================== */

/**
 * @brief  判断电机编号是否合法
 * @param  id 待检查的编号
 * @retval 1 合法；0 非法
 */
static uint8_t Motor_IsValid(motor_id_t id)
{
    return ((uint32_t)id < (uint32_t)MOTOR_COUNT) ? 1U : 0U;
}

/* ========================================================================== */
/*                                  接口实现                                   */
/* ========================================================================== */

void Motor_Init(void)
{
    for (uint32_t i = 0U; i < (uint32_t)MOTOR_COUNT; i++)
    {
        TIM_HandleTypeDef *tim = s_motor_tim[i];

        (void)HAL_TIM_PWM_Start(tim, TIM_CHANNEL_1);
        (void)HAL_TIM_PWM_Start(tim, TIM_CHANNEL_2);

        __HAL_TIM_SET_COMPARE(tim, TIM_CHANNEL_1, 0U);
        __HAL_TIM_SET_COMPARE(tim, TIM_CHANNEL_2, 0U);

        s_motor_duty[i] = 0.0f;
    }
}

void Motor_SetDuty(motor_id_t id, float duty_pct)
{
    TIM_HandleTypeDef *tim;
    float              duty;
    uint32_t           autoreload;
    uint32_t           compare;

    if (Motor_IsValid(id) == 0U)
    {
        return;
    }

    tim = s_motor_tim[id];

    /* 1. 限幅到 [-100, +100] */
    duty = duty_pct;
    if (duty > MOTOR_DUTY_MAX)
    {
        duty = MOTOR_DUTY_MAX;
    }
    else if (duty < MOTOR_DUTY_MIN)
    {
        duty = MOTOR_DUTY_MIN;
    }
    s_motor_duty[id] = duty;

    /* 2. 百分比 -> 定时器比较值，并夹紧到 ARR 以内 */
    autoreload = __HAL_TIM_GET_AUTORELOAD(tim);
    compare    = (uint32_t)((duty < 0.0f ? -duty : duty) * (float)autoreload / 100.0f);
    if (compare > autoreload)
    {
        compare = autoreload;
    }

    /* 3. 按符号决定哪个通道输出 */
    if (duty > 0.0f)
    {
        __HAL_TIM_SET_COMPARE(tim, TIM_CHANNEL_1, compare);
        __HAL_TIM_SET_COMPARE(tim, TIM_CHANNEL_2, 0U);
    }
    else if (duty < 0.0f)
    {
        __HAL_TIM_SET_COMPARE(tim, TIM_CHANNEL_1, 0U);
        __HAL_TIM_SET_COMPARE(tim, TIM_CHANNEL_2, compare);
    }
    else
    {
        __HAL_TIM_SET_COMPARE(tim, TIM_CHANNEL_1, 0U);
        __HAL_TIM_SET_COMPARE(tim, TIM_CHANNEL_2, 0U);
    }
}

void Motor_Stop(motor_id_t id)
{
    Motor_SetDuty(id, 0.0f);
}

void Motor_StopAll(void)
{
    for (uint32_t i = 0U; i < (uint32_t)MOTOR_COUNT; i++)
    {
        Motor_Stop((motor_id_t)i);
    }
}

float Motor_GetDuty(motor_id_t id)
{
    if (Motor_IsValid(id) == 0U)
    {
        return 0.0f;
    }
    return s_motor_duty[id];
}
