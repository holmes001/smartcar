/**
 * @file    app.h
 * @brief   应用层入口
 * @details 这是整个工程**唯一**允许被 CubeMX 生成的 main.c 引用的应用头文件。
 *          main.c 的用户代码区只需要两行：
 *
 *              App_Init();
 *              App_Loop();
 *
 * @note    分层约束（重构的核心约定）
 *          应用层在最上面，它负责"编排"：
 *
 *              读传感器 -> 调算法 -> 决定动作 -> 交给控制层 -> 显示结果
 *
 *          各层的调用方向严格自上而下，不允许反向：
 *
 *              app.c  ──►  Control/(chassis, pid)
 *                     ──►  camera_process/（视觉算法）
 *                     ──►  UI/(panel, telemetry)
 *                     ──►  HardWare/（仅在初始化时）
 *
 *          **除 main.c 的入口以外，任何模块都不允许 include 本文件。**
 *          原来 app.h include 了 main.h / gpio.h / tim.h / Serial.h /
 *          lcd_spi_200.h / motor.h 六个头文件，导致任何 include app.h 的
 *          文件都会被动地把整个硬件层拖进来（Element_recognition.c 就是
 *          这样拿到电机接口的）。现在这个头文件只声明两个函数。
 */
#ifndef APP_H
#define APP_H

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief  应用层初始化
 * @details 依次完成：
 *            1. 底盘初始化（含电机 PWM 与编码器，见 Chassis_Init）；
 *            2. 串口接收启动；
 *            3. OV2640 初始化并启动 DCMI + DMA 连续采集；
 *            4. LCD 初始化与字库、数字显示模式设置；
 *            5. 蜂鸣器与面板按键初始化；
 *            6. 打印就绪提示并做一次上电等待。
 * @note   必须在所有 MX_xxx_Init() 之后调用（即 main.c 的 USER CODE BEGIN 2）。
 */
void App_Init(void);

/**
 * @brief  应用层主循环体
 * @details 每调用一次完成：
 *            - 扫描按键并处理（非阻塞）；
 *            - 若有新图像帧：自动阈值 -> 二值化 -> 扫线 -> 算偏差 ->
 *              交给底盘控制 / 或停车 -> 绘制结果 -> 刷屏；
 *            - 按 100 ms 节流发送遥测数据并刷新速度显示。
 * @note   本函数**不会阻塞**，直接放在 while(1) 里即可。
 */
void App_Loop(void);

/**
 * @brief  致命错误上报
 * @details 供 CubeMX 生成的 Error_Handler() 调用，负责把故障信息通过串口发出。
 *          这样 main.c 不需要直接依赖串口驱动，只需要认识 app.h。
 * @note   本函数返回后由调用者停机（关中断 + 死循环）。
 */
void App_OnFatalError(void);

#ifdef __cplusplus
}
#endif

#endif /* APP_H */
