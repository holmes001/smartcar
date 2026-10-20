/**
 * @file    chassis.h
 * @brief   底盘运动学与三级串级控制
 *
 * @details 控制结构（三级，从外到内）：
 *
 *           ① 偏航角速度环  —— 决定"转多快"
 *              输入：归一化横向偏差 e ∈ [-1, +1]（1.0 = 路径偏到图像边缘）
 *              输出：期望偏航角速度 ω_ref [rad/s]（弧度/秒）
 *
 *           ② 物理换算  —— 纯几何，不含任何可调参数
 *              差速运动学（车绕自身中心转弯）：
 *                  v_L = v + ω_ref · (L/2)
 *                  v_R = v − ω_ref · (L/2)
 *              其中 L = CHASSIS_TRACK_WIDTH_MM（轮间距 180mm）
 *                  L/2 = 旋转半径 90mm   ← **不是 180mm**
 *
 *           ③ 轮速环  —— 决定"用多大占空比去实现"
 *              输入：速度误差 [mm/s]
 *              输出：占空比 [%]
 *
 *           图示：
 *
 *             e(-1..1) ─[①角速度环]─► ω_ref(rad/s) ─[②×L/2]─┬─► v_L ─[③轮速环]─► duty_L ─► 左电机
 *                                                             │            ▲
 *                                                             │        编码器_L
 *                                                             └─► v_R ─[③轮速环]─► duty_R ─► 右电机
 *                                                                          ▲
 *                                                                     编码器_R
 *
 * @note    符号约定（重要，容易踩坑）
 *          本工程取 **ω_ref > 0 表示车向右转（顺时针）**，即"左轮加速、右轮减速"。
 *          这样做是为了与图像偏差的符号一致：
 *              路径中心偏在图像右侧 (e > 0)  =>  车需要右转  =>  ω_ref > 0
 *          请注意这**不是**标准的"逆时针为正"偏航角速度约定，
 *          好处是从视觉偏差到控制量全程不用变号，不容易接错。
 *
 * @note    为什么要拆成三级（而不是把换算折进增益）
 *          重构前，角速度环的输出**直接就是差速**（mm/s），也就是说
 *          "乘以 L/2"这个物理换算被折进了 line_kp：
 *
 *              line_kp [mm/s per px]  =  k_yaw [rad/s per px] × (L/2) [mm]
 *
 *          后果是 line_kp 这个数字没有物理含义 —— 看不出"1 个像素的偏差
 *          对应多快的转向"，换车改了轮间距还得重调增益。
 *
 *          现在把两者分开：**物理常量归物理常量，增益归增益**。
 *          换车时只改 CHASSIS_TRACK_WIDTH_MM，增益不用动。
 *
 * @note    新旧参数对照（想复用之前调出来的手感时按这个换算）
 *          | 新参数 | 换算 | 旧参数 line_kp=5.0 时 |
 *          |--------|------|----------------------|
 *          | `yaw_kp` | `= line_kp × 94 / 90` | `5.0 × 1.044 ≈ 5.2 (rad/s)` |
 *          | `yaw_kd` | `= line_kd × 94 / 90` | `0.2 × 1.044 ≈ 0.21 (rad/s)·s` |
 *          | `yaw_integral_limit` | `= line_integral_limit / 94` | `100 / 94 ≈ 1.0` |
 *          | `yaw_output_ratio` | 不变 | `0.5` |
 *
 *          其中 94 = VISION_IMG_CX（归一化用的除数），90 = CHASSIS_HALF_TRACK_MM。
 *
 * @note    分层约束
 *          本模块属于**控制层**，向下只依赖驱动层（motor.h / encoder.h），
 *          不 include app 层、不 include 视觉层的头。
 *          **它自己不含硬件访问，也不取时间** —— dt 由应用层传入。
 */
#ifndef CHASSIS_H
#define CHASSIS_H

#include <stdint.h>

#include "motor.h"      /* 复用 motor_id_t */

#ifdef __cplusplus
extern "C" {
#endif

/* ========================================================================== */
/*                              底盘物理常量                                   */
/*                                                                            */
/*  这些是**几何常量**，不是可以随便调的增益。换车/改结构时只改这里。          */
/* ========================================================================== */

/**
 * @brief 轮间距（两轮中心距），单位 mm
 * @note  实测值：18 cm。这个值直接决定"同样转向力度下转得多快"，
 *        填错会让转向整体偏软或偏硬一倍。
 */
#define CHASSIS_TRACK_WIDTH_MM      (180.0f)

/**
 * @brief 旋转半径（车轮到车辆旋转中心的距离），单位 mm
 * @note  = 轮间距 / 2 = 90 mm。
 *        差速运动学里车绕**自身中心**转弯，所以用的是半径而不是轮间距。
 *        **把 180 代进公式会让修正量整整大一倍**，这是最容易犯的错。
 */
#define CHASSIS_HALF_TRACK_MM       (CHASSIS_TRACK_WIDTH_MM * 0.5f)

/**
 * @brief 串级 PID 的**内环**建议执行频率，单位 Hz
 * @note  外环（视觉）约 30 Hz，级联要求内环快 5~10 倍，所以取 500 Hz。
 *        实际执行由 Encoder_SetTickHandler() 从 TIM6（10kHz）分频得到：
 *            divider = 10000 / 500 = 20
 */
#define CHASSIS_INNER_LOOP_HZ       (500U)

/* ========================================================================== */
/*                                  类型定义                                   */
/* ========================================================================== */

/**
 * @brief 底盘控制参数
 * @note  分三组：基础速度、①角速度环、③轮速环。
 *        ②物理换算没有参数（它只用上面的几何常量）。
 */
typedef struct
{
    /* ---- 基础速度 ---- */
    float base_speed_mm_s;          /**< 巡线基础速度（左右轮平均速度） */

    /* ---- 右轮速度补偿 ----
     * 用来补偿左右两侧的机械/电气不对称（电机个体差异、摩擦、齿轮箱阻力）。
     * 它直接叠加在右轮的目标速度上：
     *      v_R = base_speed + right_comp − Δv
     * 符号含义：
     *      > 0  右轮目标速度更高（右轮偏弱时用）
     *      < 0  右轮目标速度更低（右轮偏强时用）
     * 运行时可以在**运行态**下用 PD8 / PD9 增减，
     * 见 app_fsm.c 的状态转移矩阵。 */
    float right_comp_mm_s;          /**< 右轮速度补偿，单位 mm/s（带符号） */
    float right_comp_limit_mm_s;    /**< 补偿的绝对值上限，单位 mm/s */

    /* ---- ① 偏航角速度环 ----
     * 输入是**归一化**横向偏差（-1 ~ +1），所以增益的物理意义很直观：
     *   yaw_kp = 7.0 表示"满偏（1.0）时要求 7 rad/s 的转向角速度" */
    float yaw_kp;                   /**< 比例增益，单位 (rad/s) per 归一化误差 */
    float yaw_ki;                   /**< 积分增益，单位 (rad/s) per (归一化误差·s) */
    float yaw_kd;                   /**< 微分增益，单位 (rad/s)·s per 归一化误差 */
    float yaw_integral_limit;       /**< 积分累加量限幅（归一化误差·s；0 = 不限幅） */
    float yaw_output_ratio;         /**< 角速度限幅 = (base_speed × 本比例) / (L/2) */

    /* ---- ③ 轮速环 ---- */
    float wheel_kp;                 /**< 比例增益，单位 %(占空比) per (mm/s) */
    float wheel_ki;                 /**< 积分增益，单位 % per (mm/s·s) */
    float wheel_kd;                 /**< 微分增益，单位 %·s per (mm/s) */
    float wheel_integral_limit;     /**< 积分累加量限幅（mm/s·s；0 = 不限幅） */
    float wheel_output_limit_pct;   /**< 轮速环输出限幅，单位：占空比 % */
} chassis_config_t;

/**
 * @brief 默认控制参数
 * @note  数值与重构前**行为等价**（原 line_kp = 5.0 的手感），
 *        换算关系见本文件头部的新旧对照表。
 */
extern const chassis_config_t CHASSIS_DEFAULT_CONFIG;

/**
 * @brief 串级 PID 档的默认参数（量纲是真实物理量 mm/s）
 * @note  两套参数的**量纲完全不同**，不能混用：
 *
 *        | 项目         | CHASSIS_DEFAULT_CONFIG  | CHASSIS_CASCADE_CONFIG    |
 *        |--------------|-------------------------|---------------------------|
 *        | base_speed   | 占空比 %（70 = 70%）     | mm/s（500 = 0.5 m/s）      |
 *        | 差速 delta_v | 占空比偏移               | 速度差 mm/s                |
 *        | 轮速环       | 不参与（开环给占空比）   | **参与**，速度→占空比闭环  |
 *        | right_comp   | 占空比 %                 | mm/s                       |
 *
 *        选哪一套由 Chassis_Init() 的档位参数决定。
 */
extern const chassis_config_t CHASSIS_CASCADE_CONFIG;

/* ========================================================================== */
/*                                  接口函数                                   */
/* ========================================================================== */

/**
 * @brief 控制档位 —— 决定 Chassis_Init() 载入哪一组默认参数
 * @note  两套参数量纲不同，见 CHASSIS_CASCADE_CONFIG 的对照表。
 */
typedef enum
{
    CHASSIS_PROFILE_DUTY    = 0,   /**< 占空比档：base/差速都当占空比 %，轮速环不参与 */
    CHASSIS_PROFILE_CASCADE = 1    /**< 串级档：base/差速都是 mm/s，轮速环闭环 */
} chassis_profile_t;

/**
 * @brief  初始化底盘
 * @param  profile 控制档位：传 0 = 占空比档，传 **1 = 串级档**
 * @details 内部会依次调用 Motor_Init() 与 Encoder_Init()，
 *          因此不需要在应用层再单独初始化电机和编码器。
 * @note   调用哪个"更新函数"要和这里选的档位**配套**：
 *             profile = 0  ->  用 Chassis_Update()
 *             profile = 1  ->  用 Chassis_UpdateCascade()
 *         两者由调用方手动切换（都放在 app.c 的 App_HandleFrame 里）。
 */
void Chassis_Init(chassis_profile_t profile);

/**
 * @brief  读取当前生效的控制参数
 * @return 指向内部参数结构体的常量指针（只读）
 */
const chassis_config_t *Chassis_GetConfig(void);

/**
 * @brief  在运行时修改巡线基础速度
 * @param  mm_s 目标基础速度，单位 mm/s
 * @note   角速度限幅会**跟着一起变**（限幅按基础速度的比例导出），
 *         所以提速之后转向能力会自动同步，不需要重调增益。
 */
void Chassis_SetBaseSpeed(float mm_s);

/**
 * @brief  读取当前巡线基础速度
 * @return 基础速度，单位 mm/s
 */
float Chassis_GetBaseSpeed(void);

/**
 * @brief  读取当前的右轮速度补偿
 * @return 补偿值，单位 mm/s（带符号，正 = 右轮目标速度更高）
 */
float Chassis_GetRightComp(void);

/**
 * @brief  在当前值上增减右轮速度补偿
 * @param  delta_mm_s 增量，单位 mm/s，可正可负
 * @note   结果会被夹到 ±right_comp_limit_mm_s。
 *         这是给"运行态下用 PD8/PD9 现场微调"用的接口。
 */
void Chassis_AdjustRightComp(float delta_mm_s);

/**
 * @brief  直接设置右轮速度补偿
 * @param  mm_s 补偿值，单位 mm/s（同样会被夹到 ±right_comp_limit_mm_s）
 */
void Chassis_SetRightComp(float mm_s);

/**
 * @brief  在线修改①角速度环增益
 * @param  kp 比例增益，(rad/s) per 归一化误差
 * @param  ki 积分增益，(rad/s) per (归一化误差·s)
 * @param  kd 微分增益，(rad/s)·s per 归一化误差
 */
void Chassis_SetYawGains(float kp, float ki, float kd);

/**
 * @brief  在线修改③轮速环增益
 * @param  kp 比例增益，% per (mm/s)
 * @param  ki 积分增益，% per (mm/s·s)
 * @param  kd 微分增益，%·s per (mm/s)
 */
void Chassis_SetWheelGains(float kp, float ki, float kd);

/**
 * @brief  执行一次巡线控制周期（**占空比档**）
 * @param  lateral_error_norm 归一化横向偏差，范围 -1.0 ~ +1.0
 * @param  dt_s               距上次调用的时间，单位秒
 * @details 这是当前在用的那条路径，**量纲是占空比 %**：
 *
 *              ① 角速度环：e_norm              -> ω_ref [rad/s]
 *              ② 换算：    ω_ref × L/2 × 0.05  -> Δv    [占空比 %]
 *              ③ 分配：    base ± Δv           -> 左右轮占空比
 *              ④ **直接** Motor_SetDuty(占空比)  —— 不走轮速环
 *
 *          所以 CHASSIS_DEFAULT_CONFIG 里的 base_speed_mm_s / right_comp_mm_s
 *          实际是"占空比 %"，名字里带 mm_s 是历史遗留，改动时注意别按 mm/s 理解。
 *
 * @note   要配套 Chassis_Init(CHASSIS_PROFILE_DUTY) 使用。
 *         如果想用带轮速闭环的串级 PID，请改用 Chassis_UpdateCascade()，
 *         并把初始化改成 Chassis_Init(CHASSIS_PROFILE_CASCADE)。
 *         线丢失时**不要调用本函数**，应当调用 Chassis_Stop()。
 */
void Chassis_Update(float lateral_error_norm, float dt_s);

/**
 * @brief  执行一次巡线控制周期（**串级 PID 版**）
 * @param  lateral_error_norm 归一化横向偏差，范围 -1.0 ~ +1.0
 * @param  dt_s               距上次调用的时间，单位秒
 * @details 和 Chassis_Update() 的区别只有一处：**它保留轮速闭环**。
 *
 *              ① 角速度环：e_norm       -> ω_ref [rad/s]
 *              ② 物理换算：ω_ref × L/2  -> Δv    [mm/s]
 *              ③ 差速分配：base ± Δv    -> 左右轮目标速度 [mm/s]
 *              ④ 轮速闭环：目标 − 实测  -> 占空比 [%] -> Motor_SetDuty()
 *
 *          Chassis_Update() 则把 ③ 的结果直接当占空比下发（开环驱动）。
 *
 * @note   要配套 Chassis_Init(CHASSIS_PROFILE_CASCADE) 使用 —— 因为
 *         CHASSIS_CASCADE_CONFIG 里的 base / 差速 / 补偿都是 mm/s 量纲。
 *         线丢失时同样**不要调用本函数**，应当调用 Chassis_Stop()。
 */
void Chassis_UpdateCascade(float lateral_error_norm, float dt_s);

/**
 * @brief  串级 PID 的**外环**：每来一帧视觉结果调用一次
 * @param  lateral_error_norm 归一化横向偏差，范围 -1.0 ~ +1.0
 * @param  dt_s               帧周期，单位秒
 * @details 只做 ① 角速度环 + ② 物理换算 + ③ 差速分配，
 *          **只更新左右轮目标速度，不下发电机**。
 *          真正的电机输出由内环 Chassis_UpdateInner() 完成。
 * @note   调用本函数会**自动使能内环**；Chassis_Stop() 会禁用它。
 */
void Chassis_UpdateOuter(float lateral_error_norm, float dt_s);

/**
 * @brief  串级 PID 的**内环**：高频调用，只做速度闭环
 * @param  dt_s 内环周期，单位秒；建议直接用 Encoder_GetTickPeriod()，
 *              它是硬件定时算出来的，比软件测量稳（抖动约 100µs）
 * @details 目标速度 − 编码器实测 --[轮速环 PID]--> 占空比 --> Motor_SetDuty()
 * @note   设计上应当**从中断里按固定频率调**（例如 TIM6 分频出的 500Hz），
 *         这样内环才比外环快一个量级，级联才真正成立。
 *         如果像以前那样从帧处理里调，内环也被绑在 30Hz，串级等于没做。
 *
 *         未使能时（还没进运行态 / 已停车）本函数立即返回，电机保持停止。
 * @warning 本函数可能在中断上下文执行，内部不含阻塞操作。
 */
void Chassis_UpdateInner(float dt_s);

/**
 * @brief  读取①角速度环最近一次输出的期望偏航角速度
 * @return ω_ref，单位 rad/s。**符号约定：正 = 右转**（左轮加速、右轮减速），
 *         与图像偏差符号一致，详见文件头部的符号约定说明。
 * @note   调参时很有用：把它发到上位机，就能看出角速度环有没有一直在饱和。
 *         如果它长期贴着 ±(base_speed × yaw_output_ratio) / (L/2)，
 *         说明 yaw_kp 太大或 yaw_output_ratio 太小。
 */
float Chassis_GetYawCommandRadS(void);

/**
 * @brief  指定某个轮子的目标速度（单轮调试用）
 * @param  id  车轮编号
 * @param  mm_s 目标线速度，单位 mm/s
 * @note   调用后可以用 Chassis_UpdateWheel() 单独跑轮速环，
 *         这样就能在断开外环的情况下调内环。
 */
void Chassis_SetWheelTarget(motor_id_t id, float mm_s);

/**
 * @brief  读取某个轮子当前的目标速度
 * @param  id 车轮编号
 * @return 目标速度，单位 mm/s；id 非法时返回 0
 * @note   供遥测显示使用 —— 这样应用层不必自己去记目标速度。
 */
float Chassis_GetWheelTarget(motor_id_t id);

/**
 * @brief  对指定轮子执行一次轮速闭环（单轮调试用）
 * @param  id   车轮编号
 * @param  dt_s 时间步长，单位秒
 */
void Chassis_UpdateWheel(motor_id_t id, float dt_s);

/**
 * @brief  停车并清空两个控制器的积分与历史误差
 * @note   积分必须一起清掉，否则重新起步时会把之前积累的积分一次性打出去。
 */
void Chassis_Stop(void);

#ifdef __cplusplus
}
#endif

#endif /* CHASSIS_H */
