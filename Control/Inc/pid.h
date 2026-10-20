/**
 * @file    pid.h
 * @brief   通用位置式 PID 控制器（纯算法）
 * @details 特性：
 *            - 积分限幅（抗积分饱和）
 *            - 输出限幅
 *            - 微分项使用"误差差分"，避免设定值突变引起的微分冲击
 *
 * @note    分层约束（重要）
 *          本模块属于**控制层**，是纯计算代码：
 *            - 不 include 任何 HAL 头文件；
 *            - 不读全局状态、不碰硬件；
 *            - 时间步长 dt 由调用者传入，模块自己不取时间。
 *
 *          这样 PID 可以脱离单片机单独做数值验证（在 PC 上喂一串误差序列即可），
 *          这也是"函数难以解耦"被解决的关键一步。
 */
#ifndef PID_H
#define PID_H

#ifdef __cplusplus
extern "C" {
#endif

/* ========================================================================== */
/*                                  类型定义                                   */
/* ========================================================================== */

/**
 * @brief PID 控制器实例
 * @note  这个结构体原来定义在 app.h 里，属于"算法类型放在应用头文件"的错位，
 *        现已归位到控制层。
 */
typedef struct
{
    float kp;               /**< 比例增益 */
    float ki;               /**< 积分增益 */
    float kd;               /**< 微分增益 */

    float integral;         /**< 积分累计量 */
    float last_error;       /**< 上一次误差，用于微分 */

    float integral_limit;   /**< 积分限幅（绝对值），0 表示不限幅 */
    float output_min;       /**< 输出下限 */
    float output_max;       /**< 输出上限 */
} pid_t;

/* ========================================================================== */
/*                                  接口函数                                   */
/* ========================================================================== */

/**
 * @brief  初始化（或重新配置）一个 PID 控制器
 * @param  pid            控制器实例
 * @param  kp             比例增益
 * @param  ki             积分增益
 * @param  kd             微分增益
 * @param  output_min     输出下限
 * @param  output_max     输出上限
 * @param  integral_limit 积分限幅（绝对值），传 0 表示不限幅
 */
void Pid_Init(pid_t *pid,
              float kp, float ki, float kd,
              float output_min, float output_max,
              float integral_limit);

/**
 * @brief  只修改增益，保留当前积分与误差状态（用于在线调参）
 * @param  pid 控制器实例
 * @param  kp  比例增益
 * @param  ki  积分增益
 * @param  kd  微分增益
 */
void Pid_SetGains(pid_t *pid, float kp, float ki, float kd);

/**
 * @brief  清零积分项与历史误差
 * @param  pid          控制器实例
 * @param  initial_error 复位时认为的当前误差，一般传当前测量误差，
 *                       这样复位后的第一个控制周期不会产生微分冲击
 */
void Pid_Reset(pid_t *pid, float initial_error);

/**
 * @brief  执行一次 PID 计算
 * @param  pid   控制器实例
 * @param  error 当前误差（设定值 - 测量值）
 * @param  dt_s  距离上次调用的时间，单位秒；小于等于 0 时直接返回 0
 * @return 限幅后的控制量
 * @note   与原来 app.c 里的实现不同，本函数**每次调用都会输出**，
 *         不再有"首次调用返回 0 且不输出"的隐含行为 —— 那属于用隐藏状态
 *         表达初始化，调用者很容易踩坑。初始化请显式调用 Pid_Reset()。
 */
float Pid_Update(pid_t *pid, float error, float dt_s);

#ifdef __cplusplus
}
#endif

#endif /* PID_H */
