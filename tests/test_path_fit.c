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
 *            gcc -std=gnu11 -Wall -Wextra \
 *              -I camera_process/Inc -I Base/Inc \
 *              -o test_path_fit.exe tests/test_path_fit.c camera_process/Src/scan_line.c -lm
 *
 * @note    用例 1~8 走完整流水线（lineinfo -> 权重 -> 筛行 -> 拟合）；
 *          用例 9~11 直接调用纯数学的 Vision_FitPoly()，用于验证正交基内核
 *          本身（等价性 / 窄跨度抖动 / 退化降阶）。
 */
#include <stdio.h>
#include <string.h>
#include <math.h>

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

/**
 * @brief 用例9~11：**正交基拟合引擎**（Vision_FitPoly）的直接数值验证
 * @details 用例1~8 走完整流水线（lineinfo -> 权重 -> 筛行 -> 拟合），
 *          验证"结果对不对"；下面三个用例直接把 (t, x, w) 喂给引擎，
 *          验证"算法本身对不对、稳不稳"：
 *            9.  满幅数据下与正规方程一致  —— 证明是同一个最小二乘解；
 *            10. 只剩近端 1/4 行时的抖动    —— 秩判据降阶带来的稳定性；
 *            11. 只有 2 个不同行号时        —— 绝对 det 门限失效的场景。
 */

/* ---------------- 参照实现：直接解 3×3 正规方程（改动前的做法） ---------------- */

/**
 * @brief  float 累加 + Cramer 展开 + det 绝对阈值 1e-6，与改动前的
 *         Fit_SolveOnce() 逐行等价，用来做对照。
 * @retval 1 求解成功；0 判定为奇异
 */
static int Ref_MonoQuad(const float *t, const float *x, const float *w, int n, float abc[3])
{
    float s0 = 0, s1 = 0, s2 = 0, s3 = 0, s4 = 0;
    float r0 = 0, r1 = 0, r2 = 0, det;

    for (int i = 0; i < n; i++)
    {
        float wi = w[i], ti = t[i], xi = x[i], t2v = ti * ti;

        s0 += wi; s1 += wi * ti; s2 += wi * t2v;
        s3 += wi * t2v * ti; s4 += wi * t2v * t2v;
        r0 += wi * xi; r1 += wi * ti * xi; r2 += wi * t2v * xi;
    }

    det = (s0 * ((s2 * s4) - (s3 * s3)))
        - (s1 * ((s1 * s4) - (s3 * s2)))
        + (s2 * ((s1 * s3) - (s2 * s2)));

    if ((det < 1.0e-6f) && (det > -1.0e-6f))
    {
        return 0;
    }

    abc[0] = ((r0 * ((s2 * s4) - (s3 * s3)))
            - (s1 * ((r1 * s4) - (s3 * r2)))
            + (s2 * ((r1 * s3) - (s2 * r2)))) / det;
    abc[1] = ((s0 * ((r1 * s4) - (s3 * r2)))
            - (r0 * ((s1 * s4) - (s3 * s2)))
            + (s2 * ((s1 * r2) - (r1 * s2)))) / det;
    abc[2] = ((s0 * ((s2 * r2) - (r1 * s3)))
            - (s1 * ((s1 * r2) - (r1 * s2)))
            + (r0 * ((s1 * s3) - (s2 * s2)))) / det;

    return 1;
}

/** @brief 确定性伪随机：[−1, +1)，保证每次运行结果一致 */
static uint32_t s_rng = 20240607U;

static float RandPm1(void)
{
    s_rng = (s_rng * 1664525U) + 1013904223U;
    return (((float)((s_rng >> 16) & 0xFFFFU) / 32767.5f) - 1.0f);
}

static void Case_OrthoMatchesMonomial(void)
{
    float t[VISION_IMG_HEIGHT], x[VISION_IMG_HEIGHT], w[VISION_IMG_HEIGHT];
    float mono[3] = { 0.0f, 0.0f, 0.0f };
    struct path_poly_s poly;
    int n = 0;

    printf("\n--- 用例9：满幅数据上，正交投影 == 正规方程（同一个最小二乘解）---\n");

    for (int row = VISION_BASE_LINE; row < VISION_FORWARD_FAR; row++)
    {
        float tt = ((float)row - (float)VISION_CONTROL_ROW) / ((float)VISION_IMG_HEIGHT * 0.5f);

        t[n] = tt;
        x[n] = TrueX(row) + (0.5f * RandPm1());
        w[n] = 1.0f / (1.0f + (0.6f * AbsF(tt)));
        n++;
    }

    if ((Vision_FitPoly(t, x, w, (uint8_t)n, 2U, &poly) == 0U) ||
        (Ref_MonoQuad(t, x, w, n, mono) == 0))
    {
        g_fail++;
        printf("    *** FAIL *** 两种解法有一次没解出来\n");
        return;
    }

    printf("    正交    : a=%9.4f b=%9.4f c=%9.4f order=%u\n",
           poly.a, poly.b, poly.c, poly.order);
    printf("    正规方程: a=%9.4f b=%9.4f c=%9.4f\n", mono[0], mono[1], mono[2]);

    Check("a 一致", poly.a, mono[0], 0.02f);
    Check("b 一致", poly.b, mono[1], 0.02f);
    Check("c 一致", poly.c, mono[2], 0.05f);
    Check("二次项被保留(order=2)", (float)poly.order, 2.0f, 0.0f);
}

/**
 * @brief 用例10：只剩近端一小段有效行 + ±2px 边线噪声，看 x_ctrl 的抖动
 * @details 真值是一条**直道**（曲率 0），只有行 0~30 有效 —— 这是直角弯
 *          近端丢线、只剩半幅以下数据的典型帧。
 *
 *          - 正规方程：数据段太短，二次项只能靠噪声凑，而外推又把噪声放大，
 *            x_ctrl 逐帧乱跳，车表现为"弯里抖、出弯甩"；
 *          - 正交基：<p2,p2> 落到相对门限下，自动降为直线，x_ctrl 只由
 *            可靠的一阶信息决定。
 *
 *          两者对这一帧的**偏差**都可以接近 0（真值是直线），差别在**抖动**。
 */
static void Case_ThinSpanJitter(void)
{
    float t[VISION_IMG_HEIGHT], x[VISION_IMG_HEIGHT], w[VISION_IMG_HEIGHT];
    struct path_poly_s poly;
    float mono[3];
    double sum_o = 0.0, sum_m = 0.0, sq_o = 0.0, sq_m = 0.0;
    double max_o = 0.0, max_m = 0.0;
    int n = 0, used_mono = 0, dropped = 0;

    for (int row = 0; row <= 30; row++)
    {
        float tt = ((float)row - (float)VISION_CONTROL_ROW) / ((float)VISION_IMG_HEIGHT * 0.5f);

        t[n] = tt;
        w[n] = 1.0f / (1.0f + (0.6f * AbsF(tt)));
        n++;
    }

    printf("\n--- 用例10：直道 + 只剩行0~30(%d行) + ±2px 噪声，200 次蒙特卡洛 ---\n", n);

    for (int k = 0; k < 200; k++)
    {
        double eo, em;

        for (int i = 0; i < n; i++)
        {
            /* 真值：直道 x = 94 − 10·t，控制行(t=0)处恰为 94 */
            x[i] = 94.0f - (10.0f * t[i]) + (2.0f * RandPm1());
        }

        if (Vision_FitPoly(t, x, w, (uint8_t)n, 2U, &poly) != 0U)
        {
            eo = ((double)poly.a) - 94.0;
            sum_o += eo;
            sq_o  += eo * eo;
            if (AbsF((float)eo) > max_o)
            {
                max_o = (double)AbsF((float)eo);
            }
            if (poly.order == 1U)
            {
                dropped++;
            }
        }

        if (Ref_MonoQuad(t, x, w, n, mono) != 0)
        {
            used_mono++;
            em = ((double)mono[0]) - 94.0;
            sum_m += em;
            sq_m  += em * em;
            if (AbsF((float)em) > max_m)
            {
                max_m = (double)AbsF((float)em);
            }
        }
    }

    {
        double mean_o = sum_o / 200.0;
        double mean_m = sum_m / (double)used_mono;
        double std_o  = sqrt((sq_o / 200.0) - (mean_o * mean_o));
        double std_m  = sqrt((sq_m / (double)used_mono) - (mean_m * mean_m));

        printf("    正交基  : x_ctrl 标准差 %6.2f px   最大偏差 %6.2f px   "
               "(降阶 %d/200, 平均偏差 %+6.2f)\n", std_o, max_o, dropped, mean_o);
        printf("    正规方程: x_ctrl 标准差 %6.2f px   最大偏差 %6.2f px   "
               "(平均偏差 %+6.2f)\n", std_m, max_m, mean_m);

        Check("正交基抖动更小", (float)std_o, 0.0f, (float)std_m);
        Check("正交基最大偏差更小", (float)max_o, 0.0f, (float)max_m);
        Check("窄跨度时自动降阶", (float)dropped, 200.0f, 0.0f);
    }
}

/**
 * @brief 用例11：只有 **2 个不同行号**、各重复采样（二次项在数学上无支撑）
 * @details 这是"绝对 det 门限"失效的典型：浮点累加让 det 变成一个
 *          **非零的小数**，大于旧代码的 1e-6 阈值，于是照样除下去，
 *          系数被放大到 1e4 以上；正交基的 <p2,p2> 是相对判据，
 *          直接塌到门限下，干净地降为直线。
 */
static void Case_TwoDistinctRows(void)
{
    float t[8], x[8], w[8];
    float mono[3] = { 0.0f, 0.0f, 0.0f };
    struct path_poly_s poly;
    int n = 0, ok_m;

    printf("\n--- 用例11：只有 2 个不同行号(t=-0.8/-0.2)各重复 4 次 ---\n");

    for (int i = 0; i < 8; i++)
    {
        t[i] = (i < 4) ? -0.8f : -0.2f;
        x[i] = 94.0f - (10.0f * t[i]);
        w[i] = 1.0f;
        n++;
    }

    (void)Vision_FitPoly(t, x, w, (uint8_t)n, 2U, &poly);
    ok_m = Ref_MonoQuad(t, x, w, n, mono);

    printf("    正交基  : order=%u  c=%12.4f  a=%10.4f\n", poly.order, poly.c, poly.a);
    printf("    正规方程: ok=%d     c=%12.4f  a=%10.4f\n",
           ok_m, ok_m ? mono[2] : 0.0f, ok_m ? mono[0] : 0.0f);

    Check("正交基放弃二次项", poly.c, 0.0f, 0.001f);
    Check("正交基降到一阶", (float)poly.order, 1.0f, 0.0f);
    Check("正交基的 a 仍在直线上", poly.a, 94.0f, 0.5f);

    /* 前提校验：正规方程在这个场景里应当给出荒谬的 c */
    if ((ok_m != 0) && (AbsF(mono[2]) < 1.0f))
    {
        g_fail++;
        printf("    *** FAIL *** 正规方程居然也解出了合理的 c，用例前提不成立\n");
    }
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
    Case_OrthoMatchesMonomial();
    Case_ThinSpanJitter();
    Case_TwoDistinctRows();

    printf("\n%s (failures=%d)\n", (g_fail == 0) ? "ALL PASS" : "TEST FAILED", g_fail);
    return (g_fail == 0) ? 0 : 1;
}
