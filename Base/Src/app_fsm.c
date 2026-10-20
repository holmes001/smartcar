/**
 * @file    app_fsm.c
 * @brief   应用状态机实现（表驱动）
 *
 * @details 行为全部由 s_transition[] 描述，逻辑代码只负责查表和更新状态。
 *          状态转移矩阵：
 *
 *              | 按键 \ 状态 | 调节 (ADJUST)   | 运行 (RUN)        |
 *              |-------------|-----------------|-------------------|
 *              | PD8  (UP)   | 阈值增加        | 右轮速度补偿增加  |
 *              | PD9  (DOWN) | 阈值减少        | 右轮速度补偿减少  |
 *              | PD10 (MODE) | 开始运行        | 结束运行          |
 *
 *          注意 **PD8/PD9 的含义随状态改变**：
 *              调节态 -> 调二值化阈值（看着屏幕上的图像调）
 *              运行态 -> 调右轮速度补偿（看着车跑偏方向调）
 */
#include "app_fsm.h"

/* ========================================================================== */
/*                                  类型定义                                   */
/* ========================================================================== */

/**
 * @brief 一条状态转移规则
 * @note  对应转移矩阵里的一个格子。
 */
typedef struct
{
    app_state_t  state;     /**< 转移前的状态（对应矩阵的列） */
    app_event_t  event;     /**< 触发事件（对应矩阵的行：按键） */
    app_state_t  next;      /**< 转移后的状态 */
    app_action_t action;    /**< 转移时要执行的动作 */
} app_transition_t;

/* ========================================================================== */
/*                                转移规则表                                   */
/* ========================================================================== */

/**
 * @brief 状态转移表 —— **这就是附件里的那张矩阵**
 * @note  修改行为请只改这张表。
 *        矩阵中的空格子（未列出的组合）表示"不转移、无动作"。
 */
static const app_transition_t s_transition[] =
{
    /* 当前状态         事件                        次态              动作 */
    /* ---- 调节态 ---- */
    { APP_STATE_ADJUST, APP_EVENT_THRESHOLD_UP,   APP_STATE_ADJUST, APP_ACTION_THRESHOLD_UP   },
    { APP_STATE_ADJUST, APP_EVENT_THRESHOLD_DOWN, APP_STATE_ADJUST, APP_ACTION_THRESHOLD_DOWN },
    { APP_STATE_ADJUST, APP_EVENT_TOGGLE_RUN,     APP_STATE_RUN,    APP_ACTION_START_RUN      },
    /* ---- 运行态：PD8/PD9 改为调右轮速度补偿 ---- */
    { APP_STATE_RUN,    APP_EVENT_THRESHOLD_UP,   APP_STATE_RUN,    APP_ACTION_RIGHT_COMP_UP   },
    { APP_STATE_RUN,    APP_EVENT_THRESHOLD_DOWN, APP_STATE_RUN,    APP_ACTION_RIGHT_COMP_DOWN },
    { APP_STATE_RUN,    APP_EVENT_TOGGLE_RUN,     APP_STATE_ADJUST, APP_ACTION_STOP_RUN        },
};

/** @brief 转移表条目数 */
#define APP_TRANSITION_COUNT  (sizeof(s_transition) / sizeof(s_transition[0]))

/** @brief 状态名称表，索引与 app_state_t 一致 */
static const char *const s_state_name[APP_STATE_COUNT] =
{
    "ADJUST",   /* APP_STATE_ADJUST */
    "RUN"       /* APP_STATE_RUN    */
};

/** @brief 状态机当前状态 */
static app_state_t s_state = APP_STATE_ADJUST;

/* ========================================================================== */
/*                                  接口实现                                   */
/* ========================================================================== */

void AppFsm_Init(void)
{
    /* 上电默认进入调节态：此时电机停转，可以安心调阈值 */
    s_state = APP_STATE_ADJUST;
}

app_action_t AppFsm_Handle(app_event_t event)
{
    app_action_t action = APP_ACTION_NONE;

    /* 无事件直接返回，避免无谓的查表 */
    if ((event == APP_EVENT_NONE) || ((uint32_t)event >= (uint32_t)APP_EVENT_COUNT))
    {
        return APP_ACTION_NONE;
    }

    /* 在转移表里查 (当前状态, 事件) */
    for (uint32_t i = 0U; i < APP_TRANSITION_COUNT; i++)
    {
        if ((s_transition[i].state == s_state) && (s_transition[i].event == event))
        {
            s_state = s_transition[i].next;
            action  = s_transition[i].action;
            break;
        }
    }

    /* 查不到匹配项 = 矩阵里的空格子：状态不变、无动作 */
    return action;
}

app_state_t AppFsm_GetState(void)
{
    return s_state;
}

uint8_t AppFsm_IsRunning(void)
{
    return (s_state == APP_STATE_RUN) ? 1U : 0U;
}

const char *AppFsm_GetStateName(void)
{
    if ((uint32_t)s_state >= (uint32_t)APP_STATE_COUNT)
    {
        return "?";
    }
    return s_state_name[s_state];
}
