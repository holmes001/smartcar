/**
 * @file    Element_recognition.c
 * @brief   路径中心计算与横向偏差输出
 * @details 本文件是视觉流水线的最后一级。它只做两件事：
 *            1. 把 lineinfo[] 里的左右边线换算成每行的路径中心（供绘图）；
 *            2. 把 scan_line.c 拟合出的中心线模型换算成横向偏差（供控制）。
 *
 * @note    与重构前的对比（这是本次解耦的核心改动之一）
 *          重构前，本文件里还有 Line_Following_PID()，它：
 *            - #include "app.h"，形成视觉层 -> 应用层的反向依赖；
 *            - 调用 Set_Motor_Speed() 直接驱动电机；
 *            - 内部写死轮速环增益 5.0 / 0.0 / 0.0。
 *          现在控制器全部搬到 Control/chassis.c，本文件不再认识电机、
 *          定时器或 HAL，变成一个可以单独验证的纯算法文件。
 *
 * @note    ★ 本次改动（直角弯丢线修复）
 *          以前横向偏差取的是 path_center[VISION_CONTROL_ROW]，
 *          **只依赖第 60 行一个采样点**，于是：
 *            - 直角弯处第 60 行失效 -> 整帧判为丢线 -> 车在弯里被硬停；
 *            - 而且这个判定会闪烁（时有时无），表现为反复刹停/起步的窜车。
 *          现在改用 scan_line.c 对**整条中心线**做的加权最小二乘二次拟合
 *          `x(t) = a + b·t + c·t²`，在控制行处求值：
 *            - 第 60 行失效时，上下两侧的有效行仍能把该处的中心**外推**出来；
 *            - 只要有效行数 >= VISION_FIT_MIN_ROWS(8) 就认为有路，判定不再抖动；
 *            - 二次项能贴合弯道，另有前视行求值可做提前转向。
 *          拟合无效（有效行太少 / 矩阵奇异）时输出 0 偏差并报告无效，
 *          此时上层的行为与旧版完全一致（停车）。
 */
#include "Element_recognition.h"

/* ========================================================================== */
/*                                  内部状态                                   */
/* ========================================================================== */

/* path_center[] / path_valid[] 已在 vision_types.h 声明为全局，
 * 因为绘图层（Binarization.c）要把道路中心线画出来，需要读这两个数组。
 * 这里是它们的**定义**（extern 声明在 vision_types.h）。 */
int16_t path_center[VISION_IMG_HEIGHT];
uint8_t path_valid[VISION_IMG_HEIGHT];

/** @brief 整车视觉状态实例（定义在本文件，声明在 vision_types.h） */
struct watch_o watch;

/* ========================================================================== */
/*                                  接口实现                                   */
/* ========================================================================== */

void Vision_ComputePathCenter(void)
{
    for (int row = 0; row < VISION_IMG_HEIGHT; row++)
    {
        /* 任意一侧丢线，本行就不能用来算偏离 */
        if ((lineinfo[row].whole_lost != 0) ||
            (lineinfo[row].left_lost != 0) ||
            (lineinfo[row].right_lost != 0))
        {
            path_valid[row]  = 0U;
            path_center[row] = VISION_IMG_CX;   /* 占位值，避免后续除零或越界 */
            continue;
        }

        path_center[row] = (int16_t)((lineinfo[row].left + lineinfo[row].right) / 2);
        path_valid[row]  = 1U;
    }
}

int16_t Vision_GetLateralError(void)
{
    if (path_fit.valid == 0U)
    {
        return 0;
    }

    return (int16_t)(path_fit.x_ctrl - (float)VISION_IMG_CX);
}

float Vision_GetLateralErrorNorm(void)
{
    if (path_fit.valid == 0U)
    {
        return 0.0f;
    }

    /* 除以图像半宽 VISION_IMG_CX(=94)，把像素偏差映射到 -1.0 ~ +1.0 */
    return (path_fit.x_ctrl - (float)VISION_IMG_CX) / (float)VISION_IMG_CX;
}

float Vision_GetLateralErrorLookaheadNorm(void)
{
    if (path_fit.valid == 0U)
    {
        return 0.0f;
    }

    /* 前视行（更远）处的偏差，量程同样是 ±1。
     * 因为它看的是更远的位置，对弯道的响应更早、幅度更大 ——
     * 如果直接替换 Vision_GetLateralErrorNorm() 使用，等效增益会上升，
     * 记得同步下调 yaw_kp。 */
    return (path_fit.x_look - (float)VISION_IMG_CX) / (float)VISION_IMG_CX;
}

uint8_t Vision_IsPathValid(void)
{
    return path_fit.valid;
}
