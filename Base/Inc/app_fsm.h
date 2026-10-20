/**
 * @file    app_fsm.h
 * @brief   应用状态机：调节 / 运行 两态切换
 *
 * @details 状态转移矩阵：
 *
 *          | 按键 \ 状态 | 调节 (ADJUST)        | 运行 (RUN)                |
 *          |-------------|----------------------|---------------------------|
 *          | PD8  (UP)   | 阈值增加             | **右轮速度补偿增加**      |
 *          | PD9  (DOWN) | 阈值减少             | **右轮速度补偿减少**      |
 *          | PD10 (MODE) | **开始运行**         | **结束运行**              |
 *
 *          注意 **PD8/PD9 的含义随状态改变**：
 *              调节态 -> 调二值化阈值（看着屏幕上的图像调）
 *              运行态 -> 调右轮速度补偿（看着车往哪边偏来调）
 *
 *          矩阵里有转移的格子共 6 个，全部写在下方的 s_transition[] 表里。
 *          要改行为只改那张表，不要改逻辑代码。
 *
 * @note    分层约束
 *          本模块属于**应用层**，是纯决策逻辑：
 *            - 不 include 任何 HAL / 驱动 / 控制层头文件；
 *            - 不持有阈值、不碰电机；
 *            - 输入"事件"，输出"要执行的动作 + 新状态"。
 *          动作的实际执行由 app.c 负责（它才拥有阈值偏移量和底盘句柄）。
 *
 *          这样拆的好处：状态机可以脱离单片机单独验证 ——
 *          在 PC 上依次喂事件序列，检查状态与动作是否符合矩阵即可。
 */
#ifndef APP_FSM_H
#define APP_FSM_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ========================================================================== */
/*                                  类型定义                                   */
/* ========================================================================== */

/**
 * @brief 应用状态
 * @note  上电后默认进入 APP_STATE_ADJUST（调节），此时电机不会转。
 */
typedef enum
{
    APP_STATE_ADJUST = 0,   /**< 调节态：只跑视觉与显示，可以调阈值，电机停转 */
    APP_STATE_RUN,          /**< 运行态：视觉结果驱动底盘巡线 */
    APP_STATE_COUNT         /**< 状态数量 */
} app_state_t;

/**
 * @brief 状态机输入事件
 * @note  由 app.c 把按键按下沿翻译成事件，本模块不认识"按键"这个概念。
 */
typedef enum
{
    APP_EVENT_NONE = 0,         /**< 无事件（保留） */
    APP_EVENT_THRESHOLD_UP,     /**< 阈值增加请求（来自 PD8） */
    APP_EVENT_THRESHOLD_DOWN,   /**< 阈值减少请求（来自 PD9） */
    APP_EVENT_TOGGLE_RUN,       /**< 运行/停止切换请求（来自 PD10） */
    APP_EVENT_COUNT             /**< 事件数量 */
} app_event_t;

/**
 * @brief 状态机输出动作
 * @note  本模块只**声明要做什么**，具体怎么做由 app.c 执行。
 */
typedef enum
{
    APP_ACTION_NONE = 0,        /**< 什么都不做 */
    APP_ACTION_THRESHOLD_UP,    /**< 把阈值偏移量增加一档 */
    APP_ACTION_THRESHOLD_DOWN,  /**< 把阈值偏移量减少一档 */
    APP_ACTION_START_RUN,       /**< 刚进入运行态：清积分、复位控制时钟 */
    APP_ACTION_STOP_RUN,        /**< 刚退出运行态：停车 */
    APP_ACTION_RIGHT_COMP_UP,   /**< 右轮速度补偿增加一档（运行态下的 PD8） */
    APP_ACTION_RIGHT_COMP_DOWN, /**< 右轮速度补偿减少一档（运行态下的 PD9） */
    APP_ACTION_COUNT            /**< 动作数量 */
} app_action_t;

/* ========================================================================== */
/*                                  接口函数                                   */
/* ========================================================================== */

/**
 * @brief  初始化状态机
 * @details 把状态复位为 APP_STATE_ADJUST（调节）。
 * @note   应在 App_Init() 里尽早调用，早于任何按键轮询。
 */
void AppFsm_Init(void);

/**
 * @brief  向状态机投递一个事件
 * @param  event 事件
 * @return 本次转移需要执行的动作；无匹配转移时返回 APP_ACTION_NONE
 * @note   无匹配转移时状态保持不变（矩阵里的空格子即"无动作、不转移"）。
 *         本函数不会阻塞，可以放在实时循环里调用。
 */
app_action_t AppFsm_Handle(app_event_t event);

/**
 * @brief  读取当前状态
 * @return 当前状态
 */
app_state_t AppFsm_GetState(void);

/**
 * @brief  查询当前是否处于运行态
 * @retval 1 运行态；0 非运行态
 */
uint8_t AppFsm_IsRunning(void);

/**
 * @brief  读取状态名称，供 LCD 显示
 * @return 静态字符串："ADJUST" 或 "RUN"
 */
const char *AppFsm_GetStateName(void);

#ifdef __cplusplus
}
#endif

#endif /* APP_FSM_H */
