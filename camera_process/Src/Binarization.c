/**
 * @file    Binarization.c
 * @brief   图像二值化与显示缓冲绘制实现
 * @author  原始算法：Admin (2023-06-21)，重构注释：2026
 *
 * @note    分层约束
 *          本文件不 include 任何 HardWare/ 头文件。它需要的图像帧缓冲
 *          （mt9v03x_image）、整车视觉状态（watch）以及输出缓冲
 *          （Grayscale / imo）全部由 vision_types.h 声明。
 *
 *          重构前本文件 include 了 dcmi_ov2640.h（摄像头驱动）、
 *          scan_line.h 与 Element_recognition.h，形成视觉层内部的
 *          循环依赖，现在只依赖 Binarization.h -> vision_types.h。
 */
#include "Binarization.h"

/** @brief 二值化结果：0 表示赛道，255 表示背景 */
uint8_t Grayscale[VISION_IMG_HEIGHT][VISION_IMG_WIDTH];

/** @brief 显示缓冲：二值图叠加边线与中心标记 */
uint8_t imo[VISION_IMG_HEIGHT][VISION_IMG_WIDTH];

/* 大津法直方图的量化级数：把 10 位灰度（0~1023）压缩到 6 位（0~63） */
#define _GRAY_SCALE 64

/** @brief 灰度右移位数，1024 >> 10 = 64 */
#define _SHIFT_FOR_GRAYSCALE 10

/** @brief 阈值还原系数，1/2*(256/_GRAY_SCALE) */
#define _Thresh_Mult 2

/**
 * @brief  用大津法（最大类间方差）计算二值化阈值
 * @param  img   图像数据起始指针
 * @param  img_v 参与统计的行数
 * @param  img_h 参与统计的列数
 * @param  step  采样步长，越大越快但越粗糙
 * @return 阈值（0 ~ 255）
 * @note   为避免浮点除法占用过多时间，直方图只统计 64 级；
 *         返回时再乘 _Thresh_Mult 还原到 8 位量程。
 */
int Vision_OtsuThreshold(uint16_t *img, uint8_t img_v, uint8_t img_h, uint8_t step)
{
    uint8_t grayhist[_GRAY_SCALE] = {0}; //灰度直方图
    uint16_t px_sum_all = 0;             //像素点总数
    uint32_t gray_sum_all = 0;           //总灰度积分
    uint16_t px_sum = 0;                 //像素点数量
    uint32_t gray_sum = 0;               //灰度积分

    float fTemp_maxvar = 0;
    uint8_t temp_best_th = 0;
    uint8_t temp_best_th2 = 0;
    uint8_t temp_this_pixel = 0;
    float fCal_var;
    float u0, u1, w0, w1;

    //生成：1. 灰度直方图 2. 像素点总数 3. 总灰度积分
    for (int i = 0; i < img_v; i += step)
    {
        for (int j = 0; j < img_h; j += step)
        {
            temp_this_pixel = (uint8)(img[i * img_h + j] >> _SHIFT_FOR_GRAYSCALE);
            if (temp_this_pixel < _GRAY_SCALE)
                grayhist[temp_this_pixel]++;
            gray_sum_all += temp_this_pixel;
            px_sum_all++;
        }
    }
    //  //总平均灰度
    //  float u = 1.0*gray_sum_all/px_sum_all;
    //迭代求得最大类间方差的阈值
    for (uint8_t k = 0; k < _GRAY_SCALE; k++)
    {
        px_sum += grayhist[k];       //该灰度及以下的像素点数量
        gray_sum += k * grayhist[k]; //该灰度及以下的像素点的灰度和
        w0 = 1.0 * px_sum / px_sum_all;
        w1 = 1.0 - w0;
        if (px_sum > 0)
		{
            u0 = 1.0 * gray_sum / px_sum;
		}
        else
		{
            u0 = 0.0;
		}
        if (px_sum_all - px_sum > 0)
		{
            u1 = 1.0 * (gray_sum_all - gray_sum) / (px_sum_all - px_sum);
		}
        else
		{
            u1 = 0.0;
		}
        //fCal_var = w0*(u0-u)*(u0-u)+w1*(u1-u)*(u1-u);
        fCal_var = w0 * w1 * (u0 - u1) * (u0 - u1);
        if (fCal_var > fTemp_maxvar)
        {
            fTemp_maxvar = fCal_var;
            temp_best_th = k;
            temp_best_th2 = k;
        }
        else if (fCal_var == fTemp_maxvar)
        {
            temp_best_th2 = k;
        }
    }
    return (temp_best_th + temp_best_th2) * _Thresh_Mult;
}

/**
 * @brief  对整帧图像做二值化，结果写入 Grayscale[][]
 * @param  delta 阈值偏移量（int8_t，可正可负），叠加在 watch.threshold 之上
 * @note   有效阈值 = (watch.threshold + delta) << 8。
 *         原实现直接对可能为负的 int 做左移（左移负数属于未定义行为），
 *         这里先把结果夹到 0 以上，避免极端调参时行为异常。
 */
void Vision_Binarize(int8_t delta)
{
    int row=0,colum;
    int threshold_shifted = ((int)watch.threshold + (int)delta) << 8;

    if (threshold_shifted < 0)
    {
        threshold_shifted = 0;
    }
    for(row=0;row<VISION_IMG_HEIGHT;row++)
    {
        for(colum=0;colum<VISION_IMG_WIDTH;colum++)
        {
            if(mt9v03x_image[row][colum] < (uint16_t)threshold_shifted)
			{
                Grayscale[row][colum]=0;
			}
            else
			{
                Grayscale[row][colum]=255;
			}
        }
    }
}

/* ========================================================================== */
/*                        显示缓冲像素值 → 屏幕颜色                            */
/* ========================================================================== */
/* imo[][] 的值在 lcd_spi_200.c 的 show_ov2640_image_int8() 里映射成颜色：
 *     0     -> DARK_GREY（赛道底色）
 *     1     -> LCD_GREEN
 *     2     -> LCD_RED
 *     3     -> LCD_BLUE
 *     其它  -> LCD_WHITE（背景）
 * 所以这里直接用 1/2/3 上色，不需要改 LCD 驱动。 */

/** @brief 道路左右边线颜色（绿） */
#define DRAW_COLOR_ROAD         (1U)
/** @brief 道路中心线与主点十字颜色（红） */
#define DRAW_COLOR_CENTER       (2U)
/** @brief 二次拟合曲线颜色（蓝） */
#define DRAW_COLOR_FIT          (3U)

/** @brief 道路边线加粗半宽（1 = 3 像素宽） */
#define DRAW_THICK_ROAD         (1)
/** @brief 道路中心线加粗半宽 */
#define DRAW_THICK_CENTER       (1)
/** @brief 拟合曲线加粗半宽 */
#define DRAW_THICK_FIT          (1)
/** @brief 主点十字的臂长（像素） */
#define DRAW_CROSS_ARM          (7)
/** @brief 主点十字的加粗半宽 */
#define DRAW_CROSS_THICK        (1)

/**
 * @brief  在显示缓冲里画一个加粗点（(2r+1)×(2r+1) 的方块），自动裁剪越界像素
 * @param  draw_row 显示缓冲行号（调用方已完成上下翻转）
 * @param  col      列号（算法列，未翻转）
 * @param  color    imo 的像素值，见 DRAW_COLOR_*
 * @param  radius   加粗半宽，0 = 单像素，1 = 3×3
 */
static void Draw_ThickPoint(int draw_row, int col, uint8_t color, int radius)
{
    int dy, dx;

    if ((col + radius < 0) || (col - radius >= VISION_IMG_WIDTH))
    {
        return;
    }
    if ((draw_row + radius < 0) || (draw_row - radius >= VISION_IMG_HEIGHT))
    {
        return;
    }

    for (dy = -radius; dy <= radius; dy++)
    {
        int rr = draw_row + dy;

        if ((rr < 0) || (rr >= VISION_IMG_HEIGHT))
        {
            continue;
        }

        for (dx = -radius; dx <= radius; dx++)
        {
            int cc = col + dx;

            if ((cc >= 0) && (cc < VISION_IMG_WIDTH))
            {
                imo[rr][cc] = color;
            }
        }
    }
}

/**
 * @brief  把二值图、道路边线、道路中心线、二次拟合曲线、主点十字叠加到 imo[][]
 * @details 绘制层次（后画的覆盖先画的）：
 *            1. 二值图底色（赛道 0 / 背景 255）；
 *            2. 左右道路边线       —— **绿色**，加粗；
 *            3. 道路中心线         —— **红色**，加粗；
 *            4. 二次拟合曲线       —— **蓝色**，加粗；
 *            5. 目标参考十字       —— 红色，细（图像主点 × 控制行）；
 *            6. 主点十字           —— **红色，粗**（拟合曲线在控制行的位置）。
 *
 * @note   行号做了上下翻转（VISION_IMG_HEIGHT-1-row），因为摄像头图像的
 *         第 0 行在画面顶部，而算法的第 0 行在画面底部（离车最近）。
 *         翻转后边线才能和二值图底色对齐。
 * @note   调用前必须已经执行过：
 *           - Vision_ScanLine()        -> 填 lineinfo[] 与 path_fit
 *           - Vision_ComputePathCenter() -> 填 path_center[] / path_valid[]
 *         本函数只操作内存，不刷屏；刷屏由 UI 层的 Panel_ShowVisionImage() 负责。
 */
void Vision_DrawEdges(void)
{
    int row, col;

    /* ---------- 1. 二值图底色 ---------- */
    for (row = 0; row < VISION_IMG_HEIGHT; row++)
    {
        for (col = 0; col < VISION_IMG_WIDTH; col++)
        {
            imo[row][col] = (Grayscale[row][col] == 0U) ? 0U : 255U;
        }
    }

    /* ---------- 2. 左右道路边线（绿，加粗） ---------- */
    for (row = 0; row < VISION_IMG_HEIGHT; row++)
    {
        int draw_row = VISION_IMG_HEIGHT - 1 - row;

        if ((lineinfo[row].left >= 0) && (lineinfo[row].left < VISION_IMG_WIDTH))
        {
            Draw_ThickPoint(draw_row, (int)lineinfo[row].left,
                            DRAW_COLOR_ROAD, DRAW_THICK_ROAD);
        }
        if ((lineinfo[row].right >= 0) && (lineinfo[row].right < VISION_IMG_WIDTH))
        {
            Draw_ThickPoint(draw_row, (int)lineinfo[row].right,
                            DRAW_COLOR_ROAD, DRAW_THICK_ROAD);
        }
    }

    /* ---------- 3. 道路中心线（红，加粗） ---------- */
    for (row = 0; row < VISION_IMG_HEIGHT; row++)
    {
        if (path_valid[row] == 0U)
        {
            continue;
        }

        Draw_ThickPoint(VISION_IMG_HEIGHT - 1 - row,
                        (int)path_center[row],
                        DRAW_COLOR_CENTER,
                        DRAW_THICK_CENTER);
    }

    /* ---------- 4. 二次拟合曲线（蓝，加粗） ----------
     * 这条曲线是控制真正使用的中心线模型，直线段会覆盖在红色中心点上，
     * 弯道处能看到它偏离红点、把弯"抹平"的效果。 */
    if (path_fit.valid != 0U)
    {
        for (row = VISION_BASE_LINE; row < VISION_FORWARD_FAR; row++)
        {
            int fit_col = (int)(Vision_EvalPathFit((float)row) + 0.5f);

            Draw_ThickPoint(VISION_IMG_HEIGHT - 1 - row, fit_col,
                            DRAW_COLOR_FIT, DRAW_THICK_FIT);
        }
    }

    /* ---------- 5. 目标参考十字（细，红）：图像主点 × 控制行 ----------
     * 这是"车应该在的位置"的参考，主点十字离它多远就是当前偏差。 */
    for (col = -3; col <= 3; col++)
    {
        int rr = VISION_IMG_HEIGHT - 1 - VISION_CONTROL_ROW;
        int cc = VISION_IMG_CX + col;

        if ((cc >= 0) && (cc < VISION_IMG_WIDTH))
        {
            imo[rr][cc] = DRAW_COLOR_CENTER;
        }
        if ((rr + col >= 0) && (rr + col < VISION_IMG_HEIGHT))
        {
            imo[rr + col][VISION_IMG_CX] = DRAW_COLOR_CENTER;
        }
    }

    /* ---------- 6. 主点十字（粗，红）：拟合曲线在控制行的位置 ---------- */
    if (path_fit.valid != 0U)
    {
        int main_col = (int)(path_fit.x_ctrl + 0.5f);
        int main_row = VISION_IMG_HEIGHT - 1 - VISION_CONTROL_ROW;
        int arm;

        for (arm = -DRAW_CROSS_ARM; arm <= DRAW_CROSS_ARM; arm++)
        {
            Draw_ThickPoint(main_row, main_col + arm,
                            DRAW_COLOR_CENTER, DRAW_CROSS_THICK);
            Draw_ThickPoint(main_row + arm, main_col,
                            DRAW_COLOR_CENTER, DRAW_CROSS_THICK);
        }
    }
}
