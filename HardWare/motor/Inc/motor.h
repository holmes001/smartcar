/**
 * @file    motor.h
 * @brief   直流电机 PWM 驱动接口
 * @details 通过 TIM12 / TIM15 的互补双通道输出控制两路直流电机：
 *            - 左轮 MOTOR_L  -> TIM15 (CH1 正转 / CH2 反转)
 *            - 右轮 MOTOR_R  -> TIM12 (CH1 正转 / CH2 反转)
 *
 * @note    分层约束（重要）
 *          本文件是**驱动层**接口，属于 L1。它：
 *            - 只通过 tim.h 访问 HAL 句柄，句柄不对外暴露；
 *            - 不 include app 层或控制层的任何头文件；
 *            - 对外参数一律使用 motor_id_t 枚举，调用者不需要、也不允许
 *              知道底层用的是哪个 TIM_HandleTypeDef。
 *
 *          这样做的直接好处：上层不再需要写
 *              extern TIM_HandleTypeDef Motor_L;
 *              if (motor_id->Instance == Motor_R.Instance) { ... }
 *          这类"用硬件身份区分逻辑对象"的代码，函数才可以被单独调用和测试。
 *
 * @note    原来定义在 main.c 里的 duty_1 / duty_2 已收进本模块内部（static），
 *          应用层不再持有电机的中间状态。
 */
#ifndef MOTOR_H
#define MOTOR_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ========================================================================== */
/*                                  类型定义                                   */
/* ========================================================================== */

/**
 * @brief 电机编号
 * @note  取值顺序与内部句柄表 s_motor_tim[] 一一对应，不要随意调整。
 */
typedef enum
{
    MOTOR_L = 0,    /**< 左轮（TIM15） */
    MOTOR_R = 1,    /**< 右轮（TIM12） */
    MOTOR_COUNT     /**< 电机数量，供数组定长使用 */
} motor_id_t;

/** @brief 占空比上限（百分比） */
#define MOTOR_DUTY_MAX      (100.0f)
/** @brief 占空比下限（百分比），负值表示反转 */
#define MOTOR_DUTY_MIN      (-100.0f)

/* ========================================================================== */
/*                                  接口函数                                   */
/* ========================================================================== */

/**
 * @brief  初始化电机 PWM 通道
 * @details 启动两路定时器的 CH1/CH2 PWM 输出，并把比较值清零（上电不转）。
 * @note   必须在 MX_TIM12_Init() / MX_TIM15_Init() 之后调用。
 */
void Motor_Init(void);

/**
 * @brief  设置指定电机的转速与方向
 * @param  id       电机编号，见 motor_id_t
 * @param  duty_pct 占空比百分比，正数前进、负数后退，内部会限幅到
 *                  [MOTOR_DUTY_MIN, MOTOR_DUTY_MAX]
 * @note   占空比的物理上限取决于定时器的 ARR，函数内部会再次夹紧，
 *         避免传入值超过自动重装值时输出异常。
 */
void Motor_SetDuty(motor_id_t id, float duty_pct);

/**
 * @brief  让指定电机自由停车（两个通道都输出 0）
 * @param  id 电机编号
 */
void Motor_Stop(motor_id_t id);

/**
 * @brief  让两个电机同时停车
 */
void Motor_StopAll(void);

/**
 * @brief  读取指定电机最近一次被设置的占空比
 * @param  id 电机编号
 * @return 占空比百分比；id 非法时返回 0
 */
float Motor_GetDuty(motor_id_t id);

#ifdef __cplusplus
}
#endif

#endif /* MOTOR_H */
