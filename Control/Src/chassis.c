/**
 * @file    chassis.c
 * @brief   底盘运动学与控制实现（两套并行的控制档位）
 *
 * @details 本文件提供**两条互斥的控制路径**，参数和量纲都不同，由调用方手动选：
 *
 *          ┌─【占空比档】Chassis_Init(0) + Chassis_Update()
 *          │    ① e_norm --[角速度环 PID]--> ω_ref [rad/s]
 *          │    ② ω_ref × (L/2) × 0.05 --> Δv [占空比 %]   ← 小系数把量纲压到占空比
 *          │    ③ base(占空比) ± Δv --> Motor_SetDuty()      ← 开环，无速度反馈
 *          │    参数表：CHASSIS_DEFAULT_CONFIG
 *          │    用途：占空比与速度高度非线性，先这么"伪开环"跑起来
 *          │
 *          └─【串级档】Chassis_Init(1) + Chassis_UpdateCascade()
 *               ① e_norm --[角速度环 PID]--> ω_ref [rad/s]
 *               ② ω_ref × (L/2) --> Δv [mm/s]                ← 真实物理量纲
 *               ③ base(mm/s) ± Δv --> 左右轮目标速度
 *               ④ 目标 − 编码器实测 --[轮速环 PID]--> 占空比 --> Motor_SetDuty()
 *               参数表：CHASSIS_CASCADE_CONFIG
 *               用途：速度闭环，抗负载/电压变化
 *
 * @warning 两套参数的**量纲不能混用**：
 *          把 mm/s 的 base 喂给占空比档 -> 电机立刻满占空比；
 *          把占空比的 base 喂给串级档 -> 轮速环永远追不上（永远饱和）。
 *          档位和更新函数必须成对使用。
 */
#include "chassis.h"

#include "encoder.h"
#include "motor.h"
#include "pid.h"

/* ========================================================================== */
/*                                内部常量与状态                                */
/* ========================================================================== */

/**
 * @brief 默认控制参数
 * @note  数值与重构前**行为等价**，换算依据：
 *            yaw_kp = 原 line_kp × VISION_IMG_CX / CHASSIS_HALF_TRACK_MM
 *                   = 5.0 × 94 / 90 ≈ 5.2
 *            yaw_kd = 原 line_kd × 94 / 90 ≈ 0.21
 *            yaw_integral_limit = 原 100 (px·s) / 94 ≈ 1.0
 *            yaw_output_ratio 不变
 *        所以烧进去之后手感应该和之前一致，只是参数变得可读了。
 */
const chassis_config_t CHASSIS_DEFAULT_CONFIG =
{
    .base_speed_mm_s        = 40.0f,

    /* 右轮速度补偿：原先硬编码在 Chassis_Update 里的 +1000，现改为可调参数。
     * 上限取 ±1000。超过这个范围就不叫"补偿不对称"了，而是在重新定义整车速度。 */
    .right_comp_mm_s        = 0.0f,  //400
    .right_comp_limit_mm_s  = 100.0f,

    /* ① 角速度环：满偏(1.0) 时要求 5.2 rad/s ≈ 298°/s 的转向角速度 */
    .yaw_kp                 = 0.8f,
    .yaw_ki                 = 0.2f,
    .yaw_kd                 = 0.0f,
    .yaw_integral_limit     = 1.0f,
    .yaw_output_ratio       = 0.5f,

    /* ③ 轮速环：速度误差 20 mm/s 就打满占空比 */
    .wheel_kp               = 5.0f,
    .wheel_ki               = 2.0f,
    .wheel_kd               = 0.0f,
    .wheel_integral_limit   = 20.0f,
    .wheel_output_limit_pct = 100.0f
};

/**
 * @brief 串级 PID 用的默认参数（**量纲是真实物理量：mm/s**）
 * @note  和上面的 CHASSIS_DEFAULT_CONFIG 的区别：
 *
 *        | 项目          | CHASSIS_DEFAULT_CONFIG（占空比档） | CHASSIS_CASCADE_CONFIG（串级档） |
 *        |---------------|------------------------------------|----------------------------------|
 *        | base_speed    | **占空比 %**（70 = 70% 占空比）     | **mm/s**（500 = 0.5 m/s）        |
 *        | 差速 delta_v  | 乘 0.05 小系数后的**占空比偏移**    | 真实**速度差 mm/s**（× L/2）     |
 *        | 轮速环        | 不参与（注释掉），直接开环给占空比  | **参与**，执行"目标速度→占空比"  |
 *        | right_comp    | 占空比 %                            | mm/s                             |
 *        | 反馈          | 无                                  | 编码器测速 mm/s                  |
 *
 *        所以这两组参数**绝对不能混用**：把 mm/s 的数值喂给占空比那条路径，
 *        电机立刻满占空比；反过来把占空比当 mm/s，轮速环永远追不上。
 *
 *        数值依据：
 *          - 实测占空比 70% ≈ 1500 mm/s  ->  被控对象增益 ≈ 1500/70
 *            ≈ **21.4 mm/s per 1% 占空比**。
 *          - 环路增益 = wheel_kp × 21.4：
 *              取 0.2  -> 约 4.3，配 300Hz(3.3ms) 采样稳定；
 *              取 5.0  -> 高达 107，一个采样周期就过冲 100 倍，
 *                         30Hz 采样下必然高频振荡（要稳住得 3kHz 采样，不现实）。
 *          - 纯 P 的稳态误差 = 稳态占空比/kp ≈ 70/0.2 = 350 mm/s，太大，
 *            所以轮速环**必须加积分**：T_i = kp/ki = 0.2/2.0 = 0.1s。
 *          - base 先取 500 mm/s（对应约 23% 占空比），留足调速余量。
 *          - yaw_output_ratio = 0.5 → 差速上限 ±250 mm/s，
 *            两轮在 250~750 mm/s 之间，内侧轮不会反转。
 *          - yaw_kp = 5.0 (rad/s per 归一化误差)：饱和点 e_norm = 2.78/5 = 0.56
 *            （约 52 像素），正常循迹不会打满。
 */
const chassis_config_t CHASSIS_CASCADE_CONFIG =
{
    .base_speed_mm_s        = 300.0f,

    .right_comp_mm_s        = 0.0f,
    .right_comp_limit_mm_s  = 400.0f,   /* 单位 mm/s */

    .yaw_kp                 = 2.5f,
    .yaw_ki                 = 2.0f,
    .yaw_kd                 = 0.0f,
    .yaw_integral_limit     = 1.0f,
    .yaw_output_ratio       = 0.5f,

    .wheel_kp               = 1.2f,
    .wheel_ki               = 0.0f,
    .wheel_kd               = 0.0f,
    .wheel_integral_limit   = 25.0f,   /* 积分最多贡献 ki×25 = 50% 占空比 */
    .wheel_output_limit_pct = 100.0f
};

/** @brief 当前生效的控制参数 */
static chassis_config_t s_cfg;

/** @brief ①角速度环（输入：归一化横向偏差，输出：期望偏航角速度 rad/s） */
static pid_t s_yaw_pid;

/** @brief ③轮速环（每轮一个） */
static pid_t s_wheel_pid[MOTOR_COUNT];

/** @brief 各轮当前目标速度，单位 mm/s */
static float s_wheel_target[MOTOR_COUNT] = { 0.0f, 0.0f };

/** @brief ①角速度环最近一次的输出，单位 rad/s（供调参观察） */
static float s_yaw_command = 0.0f;

/**
 * @brief 内环是否使能
 * @note  由 Chassis_UpdateOuter() 置 1（说明车在正常循迹），
 *       由 Chassis_Stop() 清 0。
 *        这样内环虽然在中断里 500Hz 跑着，但只要没进运行态 / 没在循迹，
 *        就完全不输出，电机保持停止。
 */
static uint8_t s_inner_enabled = 0U;

/* ========================================================================== */
/*                                  内部函数                                   */
/* ========================================================================== */

/**
 * @brief  判断车轮编号是否合法
 * @param  id 待检查的编号
 * @retval 1 合法；0 非法
 */
static uint8_t Chassis_IsValid(motor_id_t id)
{
    return ((uint32_t)id < (uint32_t)MOTOR_COUNT) ? 1U : 0U;
}

/**
 * @brief  按当前基础速度刷新①角速度环的输出限幅
 * @details 限幅的推导过程（体现物理约束，而不是拍脑袋定的数）：
 *            1. 允许的最大差速：Δv_max = base_speed × yaw_output_ratio
 *               —— 取基础速度的 50%，这样两轮速度都在 base×0.5 ~ base×1.5 之间，
 *                  内侧轮不会反转（Δv > base 时内侧轮倒转，车会原地打转）
 *            2. 换算成角速度：ω_max = Δv_max / (L/2)
 *               以 base=700、ratio=0.5、L/2=90 为例：
 *                  Δv_max = 350 mm/s，ω_max = 3.89 rad/s ≈ 223°/s
 *               对应转弯半径 R = v/ω = 180 mm = 18 cm，
 *               而差速车的几何最小转弯半径是 L/2 = 9 cm，
 *               所以留了 2 倍余量，是合适的。
 * @note   每个控制周期都会重算，所以运行中改基础速度时限幅会跟着变。
 */
static void Chassis_RefreshYawLimit(void)
{
    float delta_v_max = s_cfg.base_speed_mm_s * s_cfg.yaw_output_ratio;
    float omega_max;

    if (delta_v_max < 0.0f)
    {
        delta_v_max = -delta_v_max;
    }

    omega_max = delta_v_max / CHASSIS_HALF_TRACK_MM;
    s_yaw_pid.output_min = -omega_max;
    s_yaw_pid.output_max = omega_max;
}

/* ========================================================================== */
/*                                  接口实现                                   */
/* ========================================================================== */

void Chassis_Init(chassis_profile_t profile)
{
    /* 按档位选默认参数表。两套参数量纲不同，见 CHASSIS_CASCADE_CONFIG 的注释。 */
    s_cfg = (profile == CHASSIS_PROFILE_CASCADE)
            ? CHASSIS_CASCADE_CONFIG
            : CHASSIS_DEFAULT_CONFIG;

    /* 驱动层初始化统一由底盘负责，应用层不必再关心顺序 */
    Motor_Init();
    Encoder_Init();

    /* ① 角速度环：输出限幅先占位，随后由 Chassis_RefreshYawLimit() 按基础速度算出 */
    Pid_Init(&s_yaw_pid,
             s_cfg.yaw_kp, s_cfg.yaw_ki, s_cfg.yaw_kd,
             -1.0f, 1.0f,
             s_cfg.yaw_integral_limit);
    Chassis_RefreshYawLimit();

    /* ③ 轮速环 */
    for (uint32_t i = 0U; i < (uint32_t)MOTOR_COUNT; i++)
    {
        Pid_Init(&s_wheel_pid[i],
                 s_cfg.wheel_kp, s_cfg.wheel_ki, s_cfg.wheel_kd,
                 -s_cfg.wheel_output_limit_pct, s_cfg.wheel_output_limit_pct,
                 s_cfg.wheel_integral_limit);
        s_wheel_target[i] = 0.0f;
    }

    s_yaw_command = 0.0f;
    Chassis_Stop();
}

const chassis_config_t *Chassis_GetConfig(void)
{
    return &s_cfg;
}

void Chassis_SetBaseSpeed(float mm_s)
{
    s_cfg.base_speed_mm_s = mm_s;
    Chassis_RefreshYawLimit();
}

float Chassis_GetBaseSpeed(void)
{
    return s_cfg.base_speed_mm_s;
}

float Chassis_GetRightComp(void)
{
    return s_cfg.right_comp_mm_s;
}

void Chassis_SetRightComp(float mm_s)
{
    float limit = s_cfg.right_comp_limit_mm_s;

    if (limit < 0.0f)
    {
        limit = -limit;
    }

    if (mm_s > limit)
    {
        mm_s = limit;
    }
    else if (mm_s < -limit)
    {
        mm_s = -limit;
    }

    s_cfg.right_comp_mm_s = mm_s;
}

void Chassis_AdjustRightComp(float delta_mm_s)
{
    Chassis_SetRightComp(s_cfg.right_comp_mm_s + delta_mm_s);
}

void Chassis_SetYawGains(float kp, float ki, float kd)
{
    s_cfg.yaw_kp = kp;
    s_cfg.yaw_ki = ki;
    s_cfg.yaw_kd = kd;
    Pid_SetGains(&s_yaw_pid, kp, ki, kd);
}

void Chassis_SetWheelGains(float kp, float ki, float kd)
{
    s_cfg.wheel_kp = kp;
    s_cfg.wheel_ki = ki;
    s_cfg.wheel_kd = kd;

    for (uint32_t i = 0U; i < (uint32_t)MOTOR_COUNT; i++)
    {
        Pid_SetGains(&s_wheel_pid[i], kp, ki, kd);
    }
}

void Chassis_Update(float lateral_error_norm, float dt_s)
{
    float omega_ref;    /* ① 期望偏航角速度 [rad/s] */
    float delta_v;      /* ② 左右轮差速 [mm/s]     */

    /* ---------- ① 角速度环：归一化偏差 -> 期望偏航角速度 ---------- */
    omega_ref = Pid_Update(&s_yaw_pid, lateral_error_norm, dt_s);
    s_yaw_command = omega_ref;

    /* ---------- ② 物理换算：角速度 -> 差速（差速运动学，无可调参数） ----------
     *      v_L = v + ω·(L/2)
     *      v_R = v − ω·(L/2)
     * 注意用的是半径 L/2 = 90mm，不是轮间距 180mm。 */
    //delta_v = omega_ref * CHASSIS_HALF_TRACK_MM;
    delta_v = omega_ref * CHASSIS_HALF_TRACK_MM * 0.05;

    /* ---------- ③ 差速分配 + 轮速闭环 ----------
     * ω_ref > 0（车需要右转）时：左轮加速、右轮减速。
     * 右轮额外叠加 right_comp_mm_s —— 用来补偿左右两侧的不对称。 */
    s_wheel_target[MOTOR_L] = s_cfg.base_speed_mm_s - delta_v;
    s_wheel_target[MOTOR_R] = s_cfg.base_speed_mm_s + s_cfg.right_comp_mm_s + delta_v;

    /*for (uint32_t i = 0U; i < (uint32_t)MOTOR_COUNT; i++)
    {
        float measured = Encoder_GetSpeed((motor_id_t)i);
        float duty = Pid_Update(&s_wheel_pid[i],
                                s_wheel_target[i] - measured,
                                dt_s);
        Motor_SetDuty((motor_id_t)i, duty);
    }*/
    Motor_SetDuty(MOTOR_L, s_wheel_target[MOTOR_L]);
    Motor_SetDuty(MOTOR_R, s_wheel_target[MOTOR_R]);
}

void Chassis_UpdateCascade(float lateral_error_norm, float dt_s)
{
    /* 单速率版本：外环 + 内环在同一个调用里跑完。
     * 如果应用已经用 Encoder_SetTickHandler() 让内环在定时器里按 500Hz 跑，
     * 就**不要**用这个函数，改用 Chassis_UpdateOuter() ——
     * 否则内环会被执行两次（帧处理里一次、中断里一次）。 */
    Chassis_UpdateOuter(lateral_error_norm, dt_s);
    Chassis_UpdateInner(dt_s);
}

void Chassis_UpdateOuter(float lateral_error_norm, float dt_s)
{
    float omega_ref;    /* ① 期望偏航角速度 [rad/s] */
    float delta_v;      /* ② 左右轮差速 [mm/s]     */

    /* ---------- ① 角速度环：归一化偏差 -> 期望偏航角速度 ---------- */
    omega_ref = Pid_Update(&s_yaw_pid, lateral_error_norm, dt_s);
    s_yaw_command = omega_ref;

    /* ---------- ② 物理换算：角速度 -> 差速 ----------
     *      v_L = v + ω·(L/2)
     *      v_R = v − ω·(L/2)
     * 这里用**真实物理量纲**：ω[rad/s] × (L/2)[mm] = Δv[mm/s]，
     * 不乘任何"换算占空比"的小系数 —— 那个系数只属于占空比档。
     * 注意用的是半径 L/2 = 90mm，不是轮间距 180mm。 */
    delta_v = omega_ref * CHASSIS_HALF_TRACK_MM;

    /* ---------- ③ 差速分配成左右轮【目标速度 mm/s】 ----------
     * ω_ref > 0（车需要右转）时：左轮加速、右轮减速。
     * 右轮额外叠加 right_comp_mm_s —— 用来补偿左右两侧的不对称。 */
    s_wheel_target[MOTOR_L] = s_cfg.base_speed_mm_s - delta_v;
    s_wheel_target[MOTOR_R] = s_cfg.base_speed_mm_s + s_cfg.right_comp_mm_s + delta_v;

    /* 外环跑过一轮，说明车在正常循迹，允许内环开始输出 */
    s_inner_enabled = 1U;
}

void Chassis_UpdateInner(float dt_s)
{
    /* 未使能（还没进运行态 / 已经停车）就什么都不做，电机保持停止 */
    if (s_inner_enabled == 0U)
    {
        return;
    }
    if (dt_s <= 0.0f)
    {
        return;
    }

    /* ---------- ④ 轮速闭环：目标速度 -> 占空比 ----------
     * 这一级是串级 PID 的**内环**，也是和占空比档最本质的区别：
     * 占空比档直接把目标当占空比下发，这里是靠编码器测速做闭环。 */
    for (uint32_t i = 0U; i < (uint32_t)MOTOR_COUNT; i++)
    {
        float measured = Encoder_GetSpeed((motor_id_t)i);
        float duty = Pid_Update(&s_wheel_pid[i],
                                s_wheel_target[i] - measured,
                                dt_s);
        Motor_SetDuty((motor_id_t)i, duty);
    }
}

float Chassis_GetYawCommandRadS(void)
{
    return s_yaw_command;
}

void Chassis_SetWheelTarget(motor_id_t id, float mm_s)
{
    if (Chassis_IsValid(id) == 0U)
    {
        return;
    }
    s_wheel_target[id] = mm_s;
}

float Chassis_GetWheelTarget(motor_id_t id)
{
    if (Chassis_IsValid(id) == 0U)
    {
        return 0.0f;
    }
    return s_wheel_target[id];
}

void Chassis_UpdateWheel(motor_id_t id, float dt_s)
{
    float measured;
    float duty;

    if (Chassis_IsValid(id) == 0U)
    {
        return;
    }

    measured = Encoder_GetSpeed(id);
    duty = Pid_Update(&s_wheel_pid[id], s_wheel_target[id] - measured, dt_s);
    Motor_SetDuty(id, duty);
}

void Chassis_Stop(void)
{
    /* 先关内环：否则内环在中断里跑着，会把电机又拉起来 */
    s_inner_enabled = 0U;

    Motor_StopAll();

    for (uint32_t i = 0U; i < (uint32_t)MOTOR_COUNT; i++)
    {
        s_wheel_target[i] = 0.0f;
        Pid_Reset(&s_wheel_pid[i], 0.0f);
    }
    Pid_Reset(&s_yaw_pid, 0.0f);
    s_yaw_command = 0.0f;
}
