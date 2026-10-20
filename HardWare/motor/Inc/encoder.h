/**
 * @file    encoder.h
 * @brief   编码器测速接口
 * @details 两路增量式编码器接在 TIM4（左轮）与 TIM5（右轮）的输入捕获通道上，
 *          通过"从模式复位"测量相邻两个脉冲之间的定时器计数，换算成线速度。
 *          速度由 TIM6 周期中断（Encoder_Update）定期刷新。
 *
 * @note    分层约束
 *          本文件属于驱动层（L1），只依赖 HAL 与同子系统的 motor_id_t，
 *          不依赖控制层或应用层。
 *
 * @note    原来的 encoder.h 与 motor.h 存在"一个函数两个头文件都声明"的分裂问题，
 *          现已归位：编码器相关的一切都由本文件对外提供。
 */
#ifndef ENCODER_H
#define ENCODER_H

#include <stdint.h>

#include "motor.h"   /* 复用 motor_id_t，保证 L/R 编号一致 */

#ifdef __cplusplus
extern "C" {
#endif

/* ========================================================================== */
/*                                  接口函数                                   */
/* ========================================================================== */

/**
 * @brief  初始化两路编码器
 * @details 配置 TIM4/TIM5 的从模式复位、启动输入捕获中断，并启动 TIM6 作为
 *          测速采样时基。
 * @note   必须在 MX_TIM4_Init() / MX_TIM5_Init() / MX_TIM6_Init() 之后调用。
 *         初始化失败会进入 Error_Handler()。
 */
void Encoder_Init(void);

/**
 * @brief  测速采样与堵转判断（由 TIM6 周期中断调用）
 * @details 每调用一次：
 *            - 采样计数加一；
 *            - 若捕获值超过堵转阈值，认为轮子不转，速度清零。
 * @note   本函数运行在中断上下文，内部不含阻塞操作。
 */
void Encoder_Update(void);

/**
 * @brief  读取指定车轮的线速度
 * @param  id 车轮编号，见 motor_id_t
 * @return 线速度，单位 mm/s，带符号（正为前进）
 */
float Encoder_GetSpeed(motor_id_t id);

/**
 * @brief  读取指定车轮最近一次捕获到的定时器计数值
 * @param  id 车轮编号
 * @return 捕获值（定时器 tick 数）；id 非法时返回 0
 */
uint32_t Encoder_GetPulse(motor_id_t id);

/**
 * @brief  读取测速采样次数，可用于判断编码器是否在工作
 * @return 自 Encoder_Init() 以来的采样次数
 */
uint32_t Encoder_GetSampleCount(void);

/* ========================================================================== */
/*                        分频节拍钩子（给上层高频控制环用）                    */
/* ========================================================================== */

/**
 * @brief  注册一个"分频节拍"回调
 * @param  handler 回调函数；传 0 取消注册
 * @param  divider 分频系数。TIM6 的中断频率是 10kHz，所以：
 *                 divider = 20  -> 500 Hz
 *                 divider = 10  ->   1 kHz
 *                 divider = 1   ->  10 kHz
 * @details 用途：级联 PID 的**内环**需要比外环（摄像头帧率，约 30Hz）
 *          快一个量级，但主循环会被图像处理和刷屏阻塞，跑不到那么快。
 *          所以这里从 TIM6 硬件节拍里分出一个固定频率给内环用。
 *
 * @note    这是驱动层给上层的**钩子**：驱动只负责"按固定节拍喊一声"，
 *          不关心上层拿它做什么，所以不破坏分层（驱动不依赖控制层）。
 * @warning 回调运行在**中断上下文**：必须短小、不能阻塞、
 *          不能调用 HAL_Delay / 串口阻塞发送 / 动态内存分配。
 */
void Encoder_SetTickHandler(void (*handler)(void), uint32_t divider);

/**
 * @brief  读取分频节拍的实际周期
 * @return 周期，单位秒（500Hz 返回 0.002f）；未注册时返回 0
 * @note   给内环 PID 直接当 dt 用 —— 硬件定时比软件测量更稳，
 *         抖动只有约 100µs，比 HAL_GetTick() 的 1ms 量化好得多。
 */
float Encoder_GetTickPeriod(void);

#ifdef __cplusplus
}
#endif

#endif /* ENCODER_H */
