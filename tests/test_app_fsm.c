/**
 * @file    test_app_fsm.c
 * @brief   应用状态机的主机端验证程序（**不烧板**，只在 PC 上跑）
 *
 * @details 目的：确认 Base/Src/app_fsm.c 的实现与需求里的转移矩阵一致。
 *
 *          | 按键 \ 状态 | 调节 (ADJUST) | 运行 (RUN) |
 *          |-------------|---------------|------------|
 *          | PD8  (UP)   | 阈值增加      | —（无动作）|
 *          | PD9  (DOWN) | 阈值减少      | —（无动作）|
 *          | PD10 (MODE) | 开始运行      | 结束运行   |
 *
 * @note    为什么能这么测：app_fsm.c 是**纯逻辑**，不 include 任何 HAL 头，
 *          所以可以直接用主机端 gcc 编译运行。这正是把它从 app.c 里
 *          拆出来单独成模块的收益之一。
 *
 * @par 编译与运行（在工程根目录执行）
 * @code
 *   "D:\software\msys64\ucrt64\bin\x86_64-w64-mingw32-gcc.exe" ^
 *       -std=gnu11 -Wall -Wextra -I Base/Inc ^
 *       -o build/test_app_fsm.exe tests/test_app_fsm.c Base/Src/app_fsm.c
 *   build\test_app_fsm.exe
 * @endcode
 *          退出码 0 表示全部通过；任何一条不符合矩阵都会打印 MISMATCH
 *          并返回非 0。
 *
 * @warning 本文件**不要**加入 CMakeLists.txt —— 它是主机端测试，
 *          不属于固件，加进去会导致链接冲突（多一个 main）。
 */
#include <stdio.h>

#include "app_fsm.h"

/** @brief 测试失败计数 */
static int g_fail = 0;

/**
 * @brief 动作编号转名称
 * @param a 动作
 * @return 静态字符串
 */
static const char *ActionName(app_action_t a)
{
    switch (a)
    {
        case APP_ACTION_NONE:           return "NONE";
        case APP_ACTION_THRESHOLD_UP:   return "THRESHOLD_UP";
        case APP_ACTION_THRESHOLD_DOWN: return "THRESHOLD_DOWN";
        case APP_ACTION_START_RUN:      return "START_RUN";
        case APP_ACTION_STOP_RUN:       return "STOP_RUN";
        default:                        return "?";
    }
}

/**
 * @brief 比较两个 C 字符串是否相等
 * @param a 字符串一
 * @param b 字符串二
 * @retval 1 相等；0 不等
 */
static int StrEq(const char *a, const char *b)
{
    while ((*a != '\0') && (*a == *b))
    {
        a++;
        b++;
    }
    return ((*a == '\0') && (*b == '\0')) ? 1 : 0;
}

/**
 * @brief 投递一个事件并校验(状态, 动作)是否符合矩阵
 * @param key         测试用例名称（哪个按键、什么状态）
 * @param ev          投递的事件
 * @param want_state  期望的次态名称
 * @param want_action 期望的动作
 */
static void Step(const char *key, app_event_t ev,
                 const char *want_state, app_action_t want_action)
{
    app_action_t got_action = AppFsm_Handle(ev);
    const char  *got_state  = AppFsm_GetStateName();
    int ok = StrEq(got_state, want_state) && (got_action == want_action);

    if (ok == 0)
    {
        g_fail++;
    }

    printf("  %-26s -> state=%-7s action=%-15s %s\n",
           key, got_state, ActionName(got_action), (ok != 0) ? "OK" : "*** MISMATCH ***");
}

int main(void)
{
    printf("=== app_fsm transition test ===\n");
    AppFsm_Init();
    printf("  initial state = %s  (expect ADJUST)\n\n", AppFsm_GetStateName());

    printf("--- in ADJUST ---\n");
    Step("PD8  (UP)",   APP_EVENT_THRESHOLD_UP,   "ADJUST", APP_ACTION_THRESHOLD_UP);
    Step("PD9  (DOWN)", APP_EVENT_THRESHOLD_DOWN, "ADJUST", APP_ACTION_THRESHOLD_DOWN);
    Step("PD10 (MODE)", APP_EVENT_TOGGLE_RUN,     "RUN",    APP_ACTION_START_RUN);

    printf("--- in RUN ---\n");
    Step("PD8  (UP)",   APP_EVENT_THRESHOLD_UP,   "RUN",    APP_ACTION_NONE);
    Step("PD9  (DOWN)", APP_EVENT_THRESHOLD_DOWN, "RUN",    APP_ACTION_NONE);
    Step("PD10 (MODE)", APP_EVENT_TOGGLE_RUN,     "ADJUST", APP_ACTION_STOP_RUN);

    printf("--- back in ADJUST ---\n");
    Step("PD8  (UP)",   APP_EVENT_THRESHOLD_UP,   "ADJUST", APP_ACTION_THRESHOLD_UP);
    Step("PD10 (MODE)", APP_EVENT_TOGGLE_RUN,     "RUN",    APP_ACTION_START_RUN);
    Step("PD10 (MODE)", APP_EVENT_TOGGLE_RUN,     "ADJUST", APP_ACTION_STOP_RUN);

    printf("--- robustness (should not crash / should not change state) ---\n");
    Step("EVENT_NONE",  APP_EVENT_NONE,           "ADJUST", APP_ACTION_NONE);
    Step("out-of-range event", (app_event_t)99,   "ADJUST", APP_ACTION_NONE);

    printf("\n%s  (failures=%d)\n", (g_fail == 0) ? "ALL PASS" : "TEST FAILED", g_fail);
    return (g_fail == 0) ? 0 : 1;
}
