/**
 * @file    app.c
 * @brief   应用层实现：把各层组装成控制回路，并用状态机管理"调节 / 运行"
 *
 * @details 两个工作模式（转移矩阵见 app_fsm.h）：
 *
 *            【调节态 ADJUST】上电默认
 *              视觉流水线照常跑，屏幕上实时显示二值化图与扫线结果；
 *              PD8 / PD9 调整阈值偏移量，**电机始终停转**。
 *              用途：把车放上赛道，先看着画面把阈值调准。
 *
 *            【运行态 RUN】在调节态按一下 PD10 进入
 *              视觉结果 -> 横向偏差 -> 底盘串级 PID -> 两轮 PWM。
 *              再按一下 PD10 结束运行：停车并回到调节态。
 *
 * @details 每帧图像的数据流：
 *
 *      DCMI 中断置位 DCMI_FrameState
 *                |
 *                v
 *      Vision_OtsuThreshold()   自动阈值（并限幅到 80~120）
 *                |
 *                v
 *      Vision_Binarize(delta)   二值化（delta 由 PD8/PD9 现场调整）
 *                |
 *                v
 *      Vision_ScanLine() -> Vision_ComputePathCenter()
 *                |
 *                v
 *      ┌── 当前是运行态？ ──否──► 只显示，不动电机
 *      │是
 *      v
 *      Vision_IsPathValid() ? ──否──► Chassis_Stop()
 *                |是
 *                v
 *      e_norm = Vision_GetLateralErrorNorm()      归一化到 -1.0 ~ +1.0
 *                |
 *                v
 *      Chassis_Update(e_norm, dt)   —— 内部三级串级：
 *                ├─ ① 偏航角速度环 PID :  e_norm      -> ω_ref [rad/s]
 *                ├─ ② 物理换算（纯几何）:  ω_ref × L/2 -> 差速 Δv [mm/s]
 *                │                         L/2 = 90mm（半径，不是轮间距 180mm）
 *                └─ ③ 轮速环 PID       :  base ± Δv   -> 占空比 -> 两轮
 *                |
 *                v
 *      Vision_DrawEdges() + Panel_ShowVisionImage()
 *
 * @note    分层约束
 *          本文件是唯一的"编排者"：所有模块在这里被组装，但本文件不含
 *          算法细节、不碰寄存器。状态机（app_fsm.c）只输出"要做什么"，
 *          副作用（改阈值、停车、发提示）全部在这里执行。
 *
 * @note    本次重构
 *          原来的 app.c 是"杂物间"，同时装了按键、控制算法、LCD 显示、
 *          通信协议四类职责，并用 8 个 extern 伸进别的模块内部。现在：
 *              - 控制算法 -> Control/(pid.c, chassis.c)
 *              - 显示与按键 -> UI/panel.c
 *              - 状态机 -> Base/app_fsm.c
 *              - 通信协议 -> UI/telemetry.c
 *          本文件只剩编排，且 0 个 extern。
 */
#include "app.h"

#include "main.h"

#include "app_fsm.h"

#include "buzzer.h"
#include "dcmi_ov2640.h"
#include "encoder.h"
#include "lcd_spi_200.h"
#include "motor.h"
#include "Serial.h"
#include "usart.h"      /* huart2：用于运行时重配波特率（不改 usart.c） */

#include "chassis.h"

#include "Binarization.h"
#include "Element_recognition.h"
#include "scan_line.h"
#include "vision_types.h"

#include "panel.h"
#include "telemetry.h"

/* ========================================================================== */
/*                                  可调参数                                   */
/* ========================================================================== */

/** @brief 自动阈值取样的起始行 */
#define APP_OTSU_ROW            (30U)
/** @brief 自动阈值取样的行数 */
#define APP_OTSU_ROWS           (60U)
/** @brief 自动阈值取样步长（越大越快越粗糙） */
#define APP_OTSU_STEP           (10U)

/** @brief 自动阈值上限（太亮会把白色背景当赛道） */
#define APP_THRESHOLD_MAX       (120)
/** @brief 自动阈值下限（太暗会丢线） */
#define APP_THRESHOLD_MIN       (80)

/** @brief PD8 单次调整的阈值偏移量 */
#define APP_DELTA_STEP_UP       (10)
/** @brief PD9 单次调整的阈值偏移量 */
#define APP_DELTA_STEP_DOWN     (5)
/** @brief 阈值偏移量的绝对值上限，防止调到无意义的值 */
#define APP_DELTA_LIMIT         (120)

/**
 * @brief PD8 / PD9 在**运行态**下每按一次增减的右轮速度补偿量
 * @note  单位 mm/s。补偿默认 1000，按一下变 1050 / 950，20 档就是 ±1000。
 *        补偿的**绝对值上限**和夹紧逻辑都在底盘里
 *        （chassis.h 的 right_comp_limit_mm_s + Chassis_SetRightComp()），
 *        这里不用重复限制。
 */
#define APP_COMP_STEP_MM_S      (5.0f)

/** @brief 遥测发送周期（毫秒） */
#define APP_TELEMETRY_PERIOD_MS (100U)

/** @brief 上电等待时间（毫秒），给操作者时间把车放好 */
#define APP_STARTUP_DELAY_MS    (1000U)

/** @brief 控制周期下限（毫秒）：dt 小于该值就跳过本次控制，避免除零 */
#define APP_MIN_DT_MS           (1U)

/**
 * @brief 串口波特率（运行时重配，**不改** CubeMX 生成的 usart.c）
 * @note  CubeMX 里配的是 9600。一帧 4 通道 JustFloat 是 20 字节：
 *            9600 bps   -> 20.8 ms 阻塞（每 100ms 一次 = 21% 的循环时间被卡住）
 *            460800 bps -> 0.43 ms
 *        20.8ms 已经超过一个帧周期，会严重破坏控制周期的均匀性，
 *        所以这里在应用层重新调用 HAL_UART_Init 把速度提上去。
 * @warning 改了之后 **VOFA+ 的波特率也必须改成 460800**，否则收到乱码。
 *          如果你的 USB-TTL 不支持，改成 115200 并同步改 VOFA。
 */
#define APP_UART_BAUDRATE       (115200U)

/**
 * @brief 图像刷屏节流：每 N 帧刷一次 LCD
 * @note  一屏 = 188x120 个 16bit 像素 = 45120 字节。
 *        SPI4 在 60MHz 下光传输约 6ms，加逐像素格式转换接近 8ms。
 *        这 8ms 直接加在控制周期上，跑车时每 3 帧刷一次即可；
 *        调视觉、想看实时画面时改成 1。
 */
#define APP_IMAGE_DISPLAY_DIV   (3U)

/* ========================================================================== */
/*                                  内部状态                                   */
/* ========================================================================== */

/** @brief 人工叠加的二值化阈值偏移量，仅由 PD8 / PD9 在调节态调整 */
static int8_t   s_threshold_delta = 0;

/** @brief 上一次控制周期的时间戳，用于计算 dt */
static uint32_t s_last_control_tick = 0U;

/** @brief 图像刷屏节流计数器 */
static uint8_t s_image_frame_counter = 0U;

/**
 * @brief 前向声明：循环性能统计
 * @note  它的定义放在文件下方（和那段诊断代码在一起），
 *        但 App_HandleFrame() 要用它，所以这里先声明。
 */
static void App_UpdatePerfStats(float dt_s);

/* ========================================================================== */
/*                                  内部函数                                   */
/* ========================================================================== */

/**
 * @brief  把阈值偏移量夹紧到允许范围
 * @param  value 待夹紧的整数
 * @return 夹紧后的偏移量
 * @note   原实现的 delta 是无符号类型且无下限保护，减到 0 以下会回绕成 251。
 */
static int8_t App_ClampDelta(int value)
{
    if (value > APP_DELTA_LIMIT)
    {
        return (int8_t)APP_DELTA_LIMIT;
    }
    if (value < -APP_DELTA_LIMIT)
    {
        return (int8_t)(-APP_DELTA_LIMIT);
    }
    return (int8_t)value;
}

/**
 * @brief  计算本次控制周期的时间步长
 * @return dt，单位秒；未达到 APP_MIN_DT_MS 时返回 0
 */
static float App_ComputeDt(void)
{
    uint32_t now     = HAL_GetTick();
    uint32_t elapsed = now - s_last_control_tick;
    float    dt;

    if (elapsed < APP_MIN_DT_MS)
    {
        return 0.0f;
    }

    s_last_control_tick = now;
    dt = (float)elapsed / 1000.0f;
    return dt;
}

/**
 * @brief  执行状态机给出的动作
 * @param  action 动作编号，来自 AppFsm_Handle()
 * @note   状态机只做决策，副作用（改阈值、停车、发提示）都在这里发生。
 */
static void App_ApplyAction(app_action_t action)
{
    switch (action)
    {
        case APP_ACTION_THRESHOLD_UP:
            s_threshold_delta = App_ClampDelta((int)s_threshold_delta + APP_DELTA_STEP_UP);
            break;

        case APP_ACTION_THRESHOLD_DOWN:
            s_threshold_delta = App_ClampDelta((int)s_threshold_delta - APP_DELTA_STEP_DOWN);
            break;

        case APP_ACTION_START_RUN:
            /* 从静止起步：清掉历史积分，并把控制时钟对齐到当前时刻，
               否则第一个控制周期的 dt 会是"上一次调用到现在"的累计值 */
            Chassis_Stop();
            s_last_control_tick = HAL_GetTick();
            Serial_SendString("run_start\r\n");
            break;

        case APP_ACTION_STOP_RUN:
            /* 结束运行：立刻停车并清积分，避免下次起步时积分一次性打出去 */
            Chassis_Stop();
            Serial_SendString("run_stop\r\n");
            break;

        case APP_ACTION_RIGHT_COMP_UP:
            /* 运行态下 PD8：右轮速度补偿 +一档 */
            Chassis_AdjustRightComp(+APP_COMP_STEP_MM_S);
            break;

        case APP_ACTION_RIGHT_COMP_DOWN:
            /* 运行态下 PD9：右轮速度补偿 -一档 */
            Chassis_AdjustRightComp(-APP_COMP_STEP_MM_S);
            break;

        case APP_ACTION_NONE:
        default:
            break;
    }
}

/**
 * @brief  扫描按键，翻译成事件交给状态机，并执行返回的动作
 * @note   非阻塞：每次只采样一次，去抖与边沿判断在 Panel_PollKey() 内部完成。
 *         三个按键每一轮都会被轮询，去抖状态才能正常推进。
 */
static void App_PollKeys(void)
{
    if (Panel_PollKey(PANEL_KEY_UP) == PANEL_KEY_EVENT_PRESSED)
    {
        App_ApplyAction(AppFsm_Handle(APP_EVENT_THRESHOLD_UP));
    }

    if (Panel_PollKey(PANEL_KEY_DOWN) == PANEL_KEY_EVENT_PRESSED)
    {
        App_ApplyAction(AppFsm_Handle(APP_EVENT_THRESHOLD_DOWN));
    }

    if (Panel_PollKey(PANEL_KEY_MODE) == PANEL_KEY_EVENT_PRESSED)
    {
        App_ApplyAction(AppFsm_Handle(APP_EVENT_TOGGLE_RUN));
    }
}

/**
 * @brief  消费一帧图像：从阈值计算一路走到显示（是否驱动电机取决于当前状态）
 * @note   只有 DCMI_FrameState 被置位时才真正工作。
 */
static void App_HandleFrame(void)
{
    int   threshold;
    float dt;

    if (DCMI_FrameState == 0U)
    {
        return;
    }
    DCMI_FrameState = 0U;   /* 立刻清标志，允许下一帧继续写入 */

    /* ---------- 1. 自动阈值（大津法）并限幅 ---------- */
    threshold = Vision_OtsuThreshold(mt9v03x_image[APP_OTSU_ROW],
                                     (uint8_t)APP_OTSU_ROWS,
                                     (uint8_t)VISION_IMG_WIDTH,
                                     (uint8_t)APP_OTSU_STEP);
    if (threshold > APP_THRESHOLD_MAX)
    {
        threshold = APP_THRESHOLD_MAX;
    }
    else if (threshold < APP_THRESHOLD_MIN)
    {
        threshold = APP_THRESHOLD_MIN;
    }
    watch.threshold = (uint8_t)threshold;

    /* ---------- 2. 二值化 -> 扫线 -> 求路径中心 ---------- */
    Vision_Binarize(s_threshold_delta);
    Vision_ScanLine();
    Vision_ComputePathCenter();

    /* ---------- 3. 控制：只有运行态才驱动底盘 ---------- */
    dt = App_ComputeDt();

    /* 顺便统计帧率与控制周期抖动，显示在 LCD 最后一行 */
    App_UpdatePerfStats(dt);

    if (AppFsm_IsRunning() != 0U)
    {
        if (dt > 0.0f)
        {
            if (Vision_IsPathValid() != 0U)
            {
                /* 传的是**归一化**横向偏差（-1 ~ +1），不是像素值。
                 * 串级结构：本函数只跑**外环**（角速度环 -> 差速 -> 左右轮目标速度），
                 * 真正的电机输出去**内环** Chassis_UpdateInner() 里做，
                 * 内环由 TIM6 分频出的 500Hz 节拍驱动。 */
                Chassis_UpdateOuter(Vision_GetLateralErrorNorm(), dt);
            }
            else
            {
                /* 丢线时停车并清掉积分，避免重新找到线时积分一次性打出去 */
                Chassis_Stop();
            }
        }
    }
    /* 调节态不做任何电机操作：进入该状态时已由 APP_ACTION_STOP_RUN 停过车 */

    /* ---------- 4. 显示（两种状态下都要显示，方便调参） ---------- */
    Vision_DrawEdges();

    /* 刷屏节流：一屏约 8ms 阻塞，直接加在控制周期上，跑车时不必每帧都刷 */
    s_image_frame_counter++;
    if (s_image_frame_counter >= APP_IMAGE_DISPLAY_DIV)
    {
        s_image_frame_counter = 0U;
        Panel_ShowVisionImage(imo[0]);
    }

    Panel_ShowThreshold(watch.threshold, s_threshold_delta);
}

/**
 * @brief  按固定周期发送遥测数据
 * @note   通道顺序沿用原实现：目标速度R、实际速度R、目标速度L、实际速度L，
 *         这样上位机（VOFA+）里已经配好的通道不用改。
 */
static void App_UpdateTelemetry(void)
{
    static uint32_t last_tick = 0U;
    static float    frame[4];

    uint32_t now = HAL_GetTick();

    if ((uint32_t)(now - last_tick) < APP_TELEMETRY_PERIOD_MS)
    {
        return;
    }
    last_tick = now;

    frame[0] = Chassis_GetWheelTarget(MOTOR_R);
    frame[1] = Encoder_GetSpeed(MOTOR_R);
    frame[2] = Chassis_GetWheelTarget(MOTOR_L);
    frame[3] = Encoder_GetSpeed(MOTOR_L);

    Telemetry_SendJustFloat(frame, 4U);
}

/* ========================================================================== */
/*                                  接口实现                                   */
/* ========================================================================== */

/**
 * @brief  运行时重配串口波特率
 * @param  baud 目标波特率
 * @note   CubeMX 生成的 usart.c 里配的是 9600，对遥测来说太慢。
 *         这里在应用层重新调用 HAL_UART_Init，**不改任何 CubeMX 生成文件**。
 */
static void App_SetBaudrate(uint32_t baud)
{
    if (huart2.Init.BaudRate == baud)
    {
        return;
    }

    huart2.Init.BaudRate = baud;
    if (HAL_UART_Init(&huart2) != HAL_OK)
    {
        Error_Handler();
    }
}

/**
 * @brief  串级 PID 内环的节拍回调（运行在 TIM6 中断里，500Hz）
 * @note   只做"目标速度 -> 占空比"的速度闭环，耗时约 1~2 µs，不阻塞。
 *         dt 直接用 Encoder_GetTickPeriod()（硬件分频算出来的 0.002s），
 *         比软件测量稳定得多。
 */
static void App_InnerLoopTick(void)
{
    Chassis_UpdateInner(Encoder_GetTickPeriod());
}

void App_Init(void)
{
    /* ---------- 0. 状态机：上电进入调节态，电机不会转 ---------- */
    AppFsm_Init();

    /* ---------- 1. 底盘（内部会初始化电机 PWM 与编码器） ----------
     * 传 **1 = 串级档**：CHASSIS_CASCADE_CONFIG，量纲是真实 mm/s，
     *                  轮速环闭环参与。
     * 想回占空比档就改成 0，并把 App_HandleFrame 里的
     * Chassis_UpdateOuter() 换回 Chassis_Update()。 */
    Chassis_Init(1);
    Chassis_Stop();     /* 上电先确保两个轮子不转 */

    /* ---------- 1b. 注册串级内环节拍 ----------
     * TIM6 是 10kHz，分频到 CHASSIS_INNER_LOOP_HZ(500Hz) 给轮速环用。
     * 这样内环比外环（视觉帧率 ~30Hz）快一个量级，级联才真正成立。 */
    Encoder_SetTickHandler(App_InnerLoopTick,
                           (uint32_t)(10000U / CHASSIS_INNER_LOOP_HZ));

    /* ---------- 2. 串口：先把波特率提上去，再去初始化其它外设 ---------- */
    App_SetBaudrate(APP_UART_BAUDRATE);
    Serial_StartReceive();
    Serial_SendString("stm32_start\r\n");

    /* ---------- 3. 摄像头：初始化 + 启动连续 DMA 采集 ---------- */
    if (OV2640_Init() != OV2640_Success)
    {
        uint16_t sensor_id  = OV2640_ReadID();
        uint8_t  sccb_error = SCCB_LastError;

        Serial_Printf("OV2640 init failed, ID=0x%04X, SCCB error=%u\r\n",
                      (unsigned int)sensor_id, (unsigned int)sccb_error);
        Error_Handler();
    }
    OV2640_DMA_Transmit_Continuous(Camera_Buffer, OV2640_BufferSize);

    /* ---------- 4. LCD ---------- */
    LCD_Init();
    LCD_SetAsciiFont(&ASCII_Font16);
    LCD_ShowNumMode(Fill_Space);

    /* ---------- 5. 蜂鸣器与面板 ---------- */
    buzzer_init();      /* 内部启动 TIM14 CH1 的 PWM 输出 */
    Panel_Init();

    /* ---------- 6. 就绪提示与上电等待 ---------- */
    Serial_SendString("stm32_ready\r\n");
    Panel_ShowMode(AppFsm_GetStateName());
    Panel_ShowMotorSpeeds();
    buzzer_beep(50U, 100U);
    HAL_Delay(APP_STARTUP_DELAY_MS);
    Panel_ShowMotorSpeeds();

    s_last_control_tick = HAL_GetTick();
}

/* ========================================================================== */
/*                    循环性能统计（诊断用，可直接删掉不影响功能）              */
/* ========================================================================== */

/** @brief 统计窗口长度（毫秒）：每这么长时间刷新一次显示 */
#define APP_STAT_WINDOW_MS      (1000U)

/** @brief 窗口内处理的帧数 */
static uint32_t s_stat_frame_count = 0U;
/** @brief 窗口起点（HAL_GetTick） */
static uint32_t s_stat_tick = 0U;
/** @brief 上一窗口统计出的帧率（Hz） */
static uint16_t s_stat_fps = 0U;
/** @brief 窗口内观测到的**最大**控制周期，单位 ms */
static uint32_t s_stat_dt_max_ms = 0U;
/** @brief 上一窗口的最大控制周期 */
static uint32_t s_stat_dt_peak_ms = 0U;

/**
 * @brief  更新循环性能统计
 * @param  dt_s 本次控制周期的时间步长，单位秒
 * @note   这两个数用来回答"我的外环到底跑多快、抖不抖"：
 *           1. **fps** —— 视觉流水线实际帧率，也就是角度环的执行频率。
 *              级联控制要求内环比外环快 5~10 倍，所以这个数是分母。
 *           2. **dtmax** —— 窗口内的最大控制周期。
 *              如果 dtmax 远大于 1000/fps，说明主循环里有**阻塞操作**在插队。
 *              最常见的就是串口遥测：9600bps 下发一帧 4 通道 JustFloat
 *              （20 字节）要 **20.8 ms**，相当于每 100ms 卡一次控制循环。
 */
static void App_UpdatePerfStats(float dt_s)
{
    uint32_t now   = HAL_GetTick();
    uint32_t dt_ms = (uint32_t)(dt_s * 1000.0f);

    s_stat_frame_count++;

    if (dt_ms > s_stat_dt_max_ms)
    {
        s_stat_dt_max_ms = dt_ms;
    }

    if ((uint32_t)(now - s_stat_tick) >= APP_STAT_WINDOW_MS)
    {
        s_stat_fps         = (uint16_t)s_stat_frame_count;
        s_stat_frame_count = 0U;
        s_stat_dt_peak_ms  = s_stat_dt_max_ms;
        s_stat_dt_max_ms   = 0U;
        s_stat_tick        = now;
    }
}

/**
 * @brief  在 LCD 上显示帧率与最大控制周期
 * @note   放在 y = 214 一行，显示 "fps:xxx  dtmax:xxx(ms)"。
 */
static void App_ShowPerfStats(void)
{
    Panel_ShowString(0U, 214U, "fps:");
    Panel_ShowNumber(32U, 214U, (int32_t)s_stat_fps, 3U);
    Panel_ShowString(88U, 214U, "dtmax:");
    Panel_ShowNumber(144U, 214U, (int32_t)s_stat_dt_peak_ms, 3U);
}

/**
 * @brief  在 LCD 最下面一行显示右轮速度补偿值
 * @note   运行态下要靠 PD8/PD9 把补偿调到合适的值，**必须能看见当前值**，
 *         否则按了多少档、有没有到上限完全无从判断。
 *         屏幕布局：图像 0~119，模式 134，阈值 150，速度 166/182，补偿 198。
 */
static void App_ShowRightComp(void)
{
    Panel_ShowString(0U, 198U, "comp:");
    Panel_ShowNumber(48U, 198U, (int32_t)Chassis_GetRightComp(), 5U);
}

void App_Loop(void)
{
    App_PollKeys();                          /* 1. 输入：按键 -> 事件 -> 状态机 -> 动作 */
    App_HandleFrame();                       /* 2. 处理：一帧图像走完整条流水线 */
    App_UpdateTelemetry();                   /* 3. 输出：遥测 */
    Panel_ShowMode(AppFsm_GetStateName());   /* 4. 输出：当前模式 */
    Panel_ShowMotorSpeeds();                 /* 5. 输出：LCD 速度（内部 100 ms 节流） */
    App_ShowRightComp();                     /* 6. 输出：右轮速度补偿当前值 */
    App_ShowPerfStats();                     /* 7. 输出：帧率 / 最大控制周期（诊断用） */
}

void App_OnFatalError(void)
{
    /* 尽量把电机停掉，避免故障时小车还在跑 */
    Chassis_Stop();
    Motor_StopAll();

    Serial_SendString("failed\r\n");
}
