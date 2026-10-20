/**
 * @file    encoder.c
 * @brief   编码器测速实现
 * @details 测速原理：
 *            1. 编码器 A/B 相中的一相接到定时器的输入捕获通道（CH2），另一相
 *               接普通 GPIO 用来判断方向；
 *            2. 定时器配置为"从模式复位"（TIM_SLAVEMODE_RESET），即每次捕获
 *               事件都会把计数器清零；
 *            3. 于是"下一次捕获读到的计数值"就是相邻两脉冲之间的时间间隔；
 *            4. 速度 = 轮周长 / (每圈脉冲数 x 间隔) x 定时器频率。
 *
 * @note    本文件实现了 HAL 的弱回调 HAL_TIM_IC_CaptureCallback() 与
 *          HAL_TIM_PeriodElapsedCallback()。**整个工程只能有一处实现**，
 *          如果以后还有别的模块需要定时器周期回调，请在这里分发，
 *          不要再另外定义一份，否则会链接冲突。
 */
#include "encoder.h"

#include "main.h"
#include "tim.h"

/* ========================================================================== */
/*                                内部常量与状态                                */
/* ========================================================================== */

/** @brief 编码器接口定时器的计数频率（Hz），等于定时器时钟 / 预分频 */
#define ENCODER_TIMER_TICK_HZ       (1.0E7f)

/** @brief 判定堵转的捕获值阈值（tick），超过则认为轮子不转 */
#define ENCODER_STALL_TIMEOUT_TICKS (7000U)

/** @brief 编码器每圈脉冲数（4 倍频后的计数） */
static const uint32_t s_encoder_resolution = 10000U;

/** @brief 车轮周长，单位 mm */
static const float s_wheel_circumference = 75.0f;

/**
 * @brief 车轮编号 -> 编码器定时器句柄映射表
 * @note  索引与 motor_id_t 一致：MOTOR_L 用 TIM4、MOTOR_R 用 TIM5。
 */
static TIM_HandleTypeDef *const s_encoder_tim[MOTOR_COUNT] =
{
    &htim4,     /* MOTOR_L */
    &htim5      /* MOTOR_R */
};

/**
 * @brief 车轮编号 -> 方向引脚
 * @note  左轮（TIM4）方向极性为低电平前进，右轮（TIM5）为高电平前进，
 *        这与两侧编码器的安装方向有关，不要随意对调。
 */
static GPIO_TypeDef *const s_encoder_dir_port[MOTOR_COUNT] =
{
    Encoder2_DIR_GPIO_Port,     /* MOTOR_L */
    Encoder1_DIR_GPIO_Port      /* MOTOR_R */
};

static const uint16_t s_encoder_dir_pin[MOTOR_COUNT] =
{
    Encoder2_DIR_Pin,           /* MOTOR_L */
    Encoder1_DIR_Pin            /* MOTOR_R */
};

/** @brief 各轮线速度，单位 mm/s，由捕获中断更新 */
static volatile float s_encoder_speed[MOTOR_COUNT] = { 0.0f, 0.0f };

/** @brief 各轮最近一次捕获值（定时器 tick） */
static volatile uint32_t s_encoder_pulse[MOTOR_COUNT] = { 0U, 0U };

/** @brief 各轮是否已拿到过第一次捕获（第一次用于初始化，不参与测速） */
static uint8_t s_encoder_captured[MOTOR_COUNT] = { 0U, 0U };

/** @brief 测速采样次数 */
static volatile uint32_t s_encoder_sample_count = 0U;

/* ---- 分频节拍钩子（给上层的高频控制环用） ---- */

/** @brief TIM6 的中断频率（Hz）：240MHz/(239+1)/(99+1) = 10kHz */
#define ENCODER_TICK_BASE_HZ    (10000U)

/** @brief 上层注册的节拍回调；0 表示未注册 */
static void (*s_tick_handler)(void) = 0;

/** @brief 分频系数：TIM6 中断几次才调一次回调 */
static uint32_t s_tick_divider = 0U;

/** @brief 分频计数器 */
static uint32_t s_tick_count = 0U;

/* ========================================================================== */
/*                                  内部函数                                   */
/* ========================================================================== */

/**
 * @brief  判断车轮编号是否合法
 * @param  id 待检查的编号
 * @retval 1 合法；0 非法
 */
static uint8_t Encoder_IsValid(motor_id_t id)
{
    return ((uint32_t)id < (uint32_t)MOTOR_COUNT) ? 1U : 0U;
}

/**
 * @brief  根据编码器编号反查车轮编号
 * @param  htim 触发捕获的定时器句柄
 * @return 匹配的车轮编号；无匹配返回 MOTOR_COUNT
 */
static motor_id_t Encoder_FindByHandle(const TIM_HandleTypeDef *htim)
{
    for (uint32_t i = 0U; i < (uint32_t)MOTOR_COUNT; i++)
    {
        if (htim->Instance == s_encoder_tim[i]->Instance)
        {
            return (motor_id_t)i;
        }
    }
    return MOTOR_COUNT;
}

/* ========================================================================== */
/*                                  接口实现                                   */
/* ========================================================================== */

void Encoder_Init(void)
{
    TIM_SlaveConfigTypeDef slave_config = { 0 };

    slave_config.SlaveMode       = TIM_SLAVEMODE_RESET;
    slave_config.InputTrigger    = TIM_TS_TI2FP2;
    slave_config.TriggerPolarity = TIM_TRIGGERPOLARITY_NONINVERTED;
    slave_config.TriggerPrescaler = TIM_TRIGGERPRESCALER_DIV1;
    slave_config.TriggerFilter   = 0U;

    for (uint32_t i = 0U; i < (uint32_t)MOTOR_COUNT; i++)
    {
        if (HAL_TIM_SlaveConfigSynchro(s_encoder_tim[i], &slave_config) != HAL_OK)
        {
            Error_Handler();
        }
        if (HAL_TIM_IC_Start_IT(s_encoder_tim[i], TIM_CHANNEL_2) != HAL_OK)
        {
            Error_Handler();
        }
    }

    if (HAL_TIM_Base_Start_IT(&htim6) != HAL_OK)
    {
        Error_Handler();
    }
}

void Encoder_Update(void)
{
    s_encoder_sample_count++;

    for (uint32_t i = 0U; i < (uint32_t)MOTOR_COUNT; i++)
    {
        /* 捕获值超过阈值说明长时间没有新脉冲 —— 轮子被堵住或已停转 */
        if (__HAL_TIM_GET_COUNTER(s_encoder_tim[i]) >= ENCODER_STALL_TIMEOUT_TICKS)
        {
            s_encoder_speed[i] = 0.0f;
        }
    }
}

float Encoder_GetSpeed(motor_id_t id)
{
    if (Encoder_IsValid(id) == 0U)
    {
        return 0.0f;
    }
    return s_encoder_speed[id];
}

uint32_t Encoder_GetPulse(motor_id_t id)
{
    if (Encoder_IsValid(id) == 0U)
    {
        return 0U;
    }
    return s_encoder_pulse[id];
}

uint32_t Encoder_GetSampleCount(void)
{
    return s_encoder_sample_count;
}

void Encoder_SetTickHandler(void (*handler)(void), uint32_t divider)
{
    if (divider == 0U)
    {
        divider = 1U;
    }

    s_tick_handler = handler;
    s_tick_divider = divider;
    s_tick_count   = 0U;
}

float Encoder_GetTickPeriod(void)
{
    if (s_tick_divider == 0U)
    {
        return 0.0f;
    }

    return (float)s_tick_divider / (float)ENCODER_TICK_BASE_HZ;
}

/* ========================================================================== */
/*                                HAL 回调实现                                 */
/* ========================================================================== */

/**
 * @brief  输入捕获中断回调：更新速度与捕获值
 * @param  htim 触发中断的定时器句柄
 * @note   由 HAL 在 TIM 捕获中断中调用。本工程只使用 CH2 做捕获。
 */
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
    motor_id_t id;
    uint32_t   pulse;
    float      speed;

    if (htim->Channel != HAL_TIM_ACTIVE_CHANNEL_2)
    {
        return;
    }

    id = Encoder_FindByHandle(htim);
    if (id == MOTOR_COUNT)
    {
        return;
    }

    pulse = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_2);
    s_encoder_pulse[id] = pulse;

    /* 第一次捕获只用来初始化，因为此时间隔是从计数器任意值开始的，不可信 */
    if ((pulse > 0U) && (s_encoder_captured[id] != 0U))
    {
        speed = s_wheel_circumference * ENCODER_TIMER_TICK_HZ /
                ((float)s_encoder_resolution * (float)pulse);

        if (HAL_GPIO_ReadPin(s_encoder_dir_port[id], s_encoder_dir_pin[id]) == GPIO_PIN_SET)
        {
            /* 右轮：方向引脚为高 -> 前进；左轮：方向引脚为高 -> 后退 */
            s_encoder_speed[id] = (id == MOTOR_R) ? speed : -speed;
        }
        else
        {
            s_encoder_speed[id] = (id == MOTOR_R) ? -speed : speed;
        }
    }

    s_encoder_captured[id] = 1U;
}

/**
 * @brief  定时器周期中断回调：TIM6 触发测速采样
 * @param  htim 触发中断的定时器句柄
 * @note   如果以后有别的定时器也需要周期回调，请在本函数内按 htim->Instance 分发。
 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM6)
    {
        Encoder_Update();

        /* 分频出一个慢速节拍给上层（级联 PID 的内环用）。
         * TIM6 = 10kHz，divider = 20 -> 500Hz。
         * 回调在中断上下文执行，必须短小。 */
        if (s_tick_handler != 0)
        {
            s_tick_count++;
            if (s_tick_count >= s_tick_divider)
            {
                s_tick_count = 0U;
                s_tick_handler();
            }
        }
    }
}
