/**
 * @file    test_path_fit.c
 * @brief   scan_line.c 中心线二次拟合的 PC 端验证程序（不参与固件构建）
 *
 * @details 用**已知**的二次曲线造出 lineinfo[]，再让 Vision_FitPath() 反推，
 *          检查它能不能把系数还原出来。验证四件事：
 *            1. 无噪声时能否精确还原 a / b / c；
 *            2. 有像素噪声时误差是否可接受；
 *            3. 直角弯那种"大量离群点"下，两遍法能不能剔除它们；
 *            4. 有效行不足时是否正确报告 valid = 0。
 *
 * @note    编译（PC 端，需要自己提供 Grayscale / watch 两个外部符号）：
 *            x86_64-w64-mingw32-gcc -std=gnu11 -Wall -Wextra \
 *              -I camera_process/Inc \
 *              -o test_path_fit.exe tests/test_path_fit.c camera_process/Src/scan_line.c
 */
#include <stdio.h>
#include <string.h>

#include "scan_line.h"
#include "vision_types.h"

/* ========================================================================== */
/* scan_line.c 依赖但由别的模块定义的外部符号，测试里自己补上                  */
/* ========================================================================== */

uint8_t Grayscale[VISION_IMG_HEIGHT][VISION_IMG_WIDTH];
struct watch_o watch;

/* ========================================================================== */
/*                                  测试框架                                   */
/* ========================================================================== */

static int g_fail = 0;

static float AbsF(float x)
{
    return (x < 0.0f) ? -x : x;
}

static void Check(const char *what, float got, float want, float tol)
{
    float err = AbsF(got - want);
    int   ok  = (err <= tol);

    if (ok == 0)
    {
        g_fail++;
    }

    printf("    %-22s got=%9.3f  want=%9.3f  err=%7.3f  %s\n",
           what, got, want, err, (ok != 0) ? "OK" : "*** FAIL ***");
}

/* ---------------- 真实（合成）曲线：x(t) = A + B·t + C·t² ---------------- */
#define TRUE_A   (94.0f)
#define TRUE_B   (-18.0f)
#define TRUE_C   (34.0f)

static float TrueX(int row)
{
    float t = ((float)row - (float)VISION_CONTROL_ROW) / ((float)VISION_IMG_HEIGHT * 0.5f);
    return TRUE_A + ((TRUE_B + (TRUE_C * t)) * t);
}

/** @brief 清空 lineinfo[]，并把 [from, to) 行填成真值曲线 + 半宽 20 */
static void FillRows(int from, int to, float noise, int outlier_start, int outlier_step)
{
    int row;

    memset(lineinfo, 0, sizeof(lineinfo));

    for (row = 0; row < VISION_IMG_HEIGHT; row++)
    {
        float x = TrueX(row);
        int   c = (int)(x + 0.5f);

        if ((row < from) || (row >= to))
        {
            /* 范围外：标记为丢线 */
            lineinfo[row].whole_lost = 1;
            lineinfo[row].left_lost  = 1;
            lineinfo[row].right_lost = 1;
            continue;
        }

        /* 制造离群点：模拟直角弯处把垂直赛道误当成边线 */
        if ((outlier_step > 0) && (row >= outlier_start) && (((row - outlier_start) % outlier_step) == 0))
        {
            c += 45;
        }

        /* 简单确定性"噪声"（不用 rand，保证可复现） */
        if (noise > 0.0f)
        {
            c += ((row * 37) % 3) - 1;
        }

        lineinfo[row].y          = (int16_t)row;
        lineinfo[row].left       = (int16_t)(c - 20);
        lineinfo[row].right      = (int16_t)(c + 20);
        lineinfo[row].left_lost  = 0;
        lineinfo[row].right_lost = 0;
        lineinfo[row].whole_lost = 0;
    }
}

/* ========================================================================== */
/*                                  测试用例                                   */
/* ========================================================================== */

static void Case_Perfect(void)
{
    printf("\n--- 用例1：无噪声、无离群点（应精确还原系数）---\n");

    FillRows(VISION_BASE_LINE, VISION_FORWARD_FAR, 0.0f, 0, 0);
    Vision_FitPath();

    printf("    valid=%u rows_used=%u\n", path_fit.valid, path_fit.rows_used);
    Check("a",          path_fit.a,      TRUE_A, 0.5f);
    Check("b",          path_fit.b,      TRUE_B, 0.5f);
    Check("c",          path_fit.c,      TRUE_C, 0.5f);
    Check("x_ctrl",     path_fit.x_ctrl, TrueX(VISION_CONTROL_ROW), 0.3f);
    Check("x_look",     path_fit.x_look, TrueX(VISION_LOOKAHEAD_ROW), 0.3f);
}

static void Case_Noisy(void)
{
    printf("\n--- 用例2：±1 像素噪声（系数应小幅偏离真值）---\n");

    FillRows(VISION_BASE_LINE, VISION_FORWARD_FAR, 1.0f, 0, 0);
    Vision_FitPath();

    printf("    valid=%u rows_used=%u\n", path_fit.valid, path_fit.rows_used);
    Check("a",      path_fit.a, TRUE_A, 1.5f);
    Check("b",      path_fit.b, TRUE_B, 2.0f);
    Check("c",      path_fit.c, TRUE_C, 4.0f);
    Check("x_ctrl", path_fit.x_ctrl, TrueX(VISION_CONTROL_ROW), 1.5f);
}

static void Case_WithOutliers(void)
{
    printf("\n--- 用例3：每 7 行一个 +45px 离群点（模拟直角弯误检）---\n");

    FillRows(VISION_BASE_LINE, VISION_FORWARD_FAR, 0.0f, 30, 7);
    Vision_FitPath();

    printf("    valid=%u rows_used=%u  (填入 110 行，其中约 12 行是离群点)\n",
           path_fit.valid, path_fit.rows_used);
    Check("a",      path_fit.a, TRUE_A, 2.0f);
    Check("b",      path_fit.b, TRUE_B, 3.0f);
    Check("c",      path_fit.c, TRUE_C, 6.0f);
    Check("x_ctrl", path_fit.x_ctrl, TrueX(VISION_CONTROL_ROW), 2.0f);
}

static void Case_ControlRowMissing(void)
{
    printf("\n--- 用例4：控制行(60)附近 50~70 行全部丢线（外推能力）---\n");

    FillRows(VISION_BASE_LINE, VISION_FORWARD_FAR, 0.0f, 0, 0);

    for (int row = 50; row <= 70; row++)
    {
        lineinfo[row].whole_lost = 1;
        lineinfo[row].left_lost  = 1;
        lineinfo[row].right_lost = 1;
    }

    Vision_FitPath();

    printf("    valid=%u rows_used=%u  (控制行本身是丢线的)\n",
           path_fit.valid, path_fit.rows_used);
    Check("a (= x_ctrl)", path_fit.a,      TRUE_A, 0.5f);
    Check("x_ctrl",       path_fit.x_ctrl, TrueX(VISION_CONTROL_ROW), 1.0f);
}

static void Case_TooFewRows(void)
{
    printf("\n--- 用例5：只有 5 行有效（应报告 valid=0）---\n");

    FillRows(20, 25, 0.0f, 0, 0);
    Vision_FitPath();

    printf("    valid=%u rows_used=%u  (期望 valid=0)\n",
           path_fit.valid, path_fit.rows_used);

    if ((path_fit.valid != 0U) || (path_fit.rows_used >= VISION_FIT_MIN_ROWS))
    {
        g_fail++;
        printf("    *** FAIL *** 有效行不足却报告了 valid=%u\n", path_fit.valid);
    }
    else
    {
        printf("    OK\n");
    }
    Check("x_ctrl fallback", path_fit.x_ctrl, (float)VISION_IMG_CX, 0.01f);
}

static void Case_EvalHelper(void)
{
    printf("\n--- 用例6：Vision_GetFitX() 与 Vision_EvalPathFit() 一致 ---\n");

    FillRows(VISION_BASE_LINE, VISION_FORWARD_FAR, 0.0f, 0, 0);
    Vision_FitPath();

    for (int row = 0; row < VISION_IMG_HEIGHT; row += 30)
    {
        Check("GetFitX == Eval", Vision_GetFitX((float)row),
              Vision_EvalPathFit((float)row), 0.001f);
    }
}

/**
 * @brief 用例7：**连续坏块**（第二层防御的核心场景）
 * @details 直道 (x=94)，行 40~60 共 21 行整体偏移 +40px ——
 *          模拟直角弯处整段垂直赛道被误检、或一段阴影/缺口。
 *
 *          旧实现（用二次曲线自己筛残差）会掩蔽：曲线被这 21 行拉弯、
 *          恰好穿过它们、残差变小、筛不掉，最后留下一条曲率很大的抛物线。
 *          新实现用**直线**筛，直线无法弯折，21 行的残差一定很大 -> 全部剔除。
 *
 * @note    注意这个用例是"**直道** + 坏块"。如果换成"强弯 + 坏块"
 *          （曲率 c=34 的弯 + 40px 坏块），强弯本身相对直线的偏差就到 19px，
 *          坏块残差只有 21~25px，两者重叠 —— 任何全局阈值都分不开。
 *          这是本方法的**内在极限**，只能靠把"偏移"和"预瞄"拆成两路来解决。
 */
static void Case_ContiguousBlock(void)
{
    printf("\n--- 用例7：直道 + 连续坏块（行 40~60 整体 +40px）---\n");

    /* 直道：所有行中心都是 94 */
    for (int row = 0; row < VISION_IMG_HEIGHT; row++)
    {
        lineinfo[row].y          = (int16_t)row;
        lineinfo[row].left       = 94 - 20;
        lineinfo[row].right      = 94 + 20;
        lineinfo[row].left_lost  = 0;
        lineinfo[row].right_lost = 0;
        lineinfo[row].whole_lost = 0;
    }
    /* 坏块 */
    for (int row = 40; row <= 60; row++)
    {
        lineinfo[row].left  += 40;
        lineinfo[row].right += 40;
    }

    Vision_FitPath();

    printf("    valid=%u rows_used=%u  (满额 110，坏块 21 行，期望剔除到约 89)\n",
           path_fit.valid, path_fit.rows_used);
    printf("    a=%.2f b=%.3f c=%.3f\n", path_fit.a, path_fit.b, path_fit.c);

    Check("坏块被剔除",         (float)path_fit.rows_used, 89.0f, 3.0f);
    Check("x_ctrl 未被带偏",     path_fit.x_ctrl, 94.0f, 2.0f);
    Check("|c| 保持很小",        path_fit.c,      0.0f, 3.0f);
    Check("斜率未被带偏",        path_fit.slope_ctrl, 0.0f, 0.02f);
}

/**
 * @brief 用例8：曲率钳位（第一层防御）
 * @details 造一条真实曲率极大的抛物线（c = 200），
 *          拟合应该能解出它，但输出必须被夹到 VISION_FIT_MAX_SAGITTA_PX。
 */
static void Case_CurvatureClamp(void)
{
    printf("\n--- 用例8：曲率钳位（真实 c=200，应被夹到 %.0f）---\n",
           (double)VISION_FIT_MAX_SAGITTA_PX);

    for (int row = 0; row < VISION_IMG_HEIGHT; row++)
    {
        float t = ((float)row - (float)VISION_CONTROL_ROW) / ((float)VISION_IMG_HEIGHT * 0.5f);
        float x = 94.0f + (200.0f * t * t);
        int   c = (int)(x + 0.5f);

        lineinfo[row].y          = (int16_t)row;
        lineinfo[row].left       = (int16_t)(c - 20);
        lineinfo[row].right      = (int16_t)(c + 20);
        lineinfo[row].left_lost  = 0;
        lineinfo[row].right_lost = 0;
        lineinfo[row].whole_lost = 0;
    }

    Vision_FitPath();

    printf("    c=%.3f  (期望 %.1f)\n", path_fit.c, (double)VISION_FIT_MAX_SAGITTA_PX);
    Check("c 被钳位", path_fit.c, VISION_FIT_MAX_SAGITTA_PX, 0.001f);
    /* c 被夹了，但偏移 a 不受影响（t=0 处曲率贡献本来就是 0） */
    Check("a 不受钳位影响", path_fit.a, 94.0f, 1.0f);
}

int main(void)
{
    printf("=== scan_line.c 中心线二次拟合测试 ===\n");
    printf("真值: x(t) = %.1f + (%.1f + %.1f*t)*t,  t=(row-60)/60\n",
           TRUE_A, TRUE_B, TRUE_C);
    printf("控制行 %d 处真值 = %.2f\n", VISION_CONTROL_ROW, TrueX(VISION_CONTROL_ROW));
    printf("前视行 %d 处真值 = %.2f\n", VISION_LOOKAHEAD_ROW, TrueX(VISION_LOOKAHEAD_ROW));

    Case_Perfect();
    Case_Noisy();
    Case_WithOutliers();
    Case_ControlRowMissing();
    Case_TooFewRows();
    Case_EvalHelper();
    Case_ContiguousBlock();
    Case_CurvatureClamp();

    printf("\n%s (failures=%d)\n", (g_fail == 0) ? "ALL PASS" : "TEST FAILED", g_fail);
    return (g_fail == 0) ? 0 : 1;
}
