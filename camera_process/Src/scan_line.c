/*
 * scan_line.c
 *
 *  Created on: 2023年6月21日
 *      Author: Admin
 */
 #include <stdlib.h>
#include <math.h>
#include <string.h>
#include "scan_line.h"

/**
 * @file    scan_line.c
 * @brief   赛道边线扫描实现
 * @author  原始算法：Admin (2023-06-21)，重构注释：2026
 *
 * @note    分层约束
 *          本文件不 include 任何 HardWare/ 头文件，也不 include 应用层。
 *          它只读写 vision_types.h 里声明的 Grayscale[][]、lineinfo[]、watch，
 *          因此可以脱离硬件单独用一段图像数据做验证。
 *
 *          重构前本文件与 Binarization.c、Element_recognition.c 互相 include，
 *          现已统一为：三者都只依赖 vision_types.h。
 */

/** @brief 全图逐行边线信息，行 0 在图像底部 */
struct lineinfo_s lineinfo[VISION_IMG_HEIGHT];

/**
 * @brief  扫描全图赛道边线
 * @details 流程：
 *            1. 从 VISION_BASE_LINE 行开始找基准行的边线；
 *            2. 向下逐行搜到 VISION_FORWARD_NEAR；
 *            3. 向上逐行搜到 VISION_FORWARD_FAR。
 *          结果写入全局 lineinfo[]，同时刷新 watch.watch_line。
 * @note   调用前必须先完成二值化（Vision_Binarize），因为本函数读 Grayscale[][]。
 *         本函数不驱动任何执行机构，只填数据。
 */
void Vision_ScanLine(void)
{
    /* 基准行由 VISION_BASE_LINE 常量指定 */
    int y=0;
    memset(lineinfo, 0, VISION_IMG_HEIGHT * sizeof(struct lineinfo_s));
    line_single(&lineinfo[VISION_BASE_LINE], Grayscale[119-VISION_BASE_LINE]);   // 寻找基准行

    for (y = VISION_BASE_LINE-1; y >= VISION_FORWARD_NEAR; y--) // 向下搜线到near行
    {
        line_findnext(&lineinfo[y], Grayscale[119-y], &lineinfo[y + 1]);
    }

    for (y = VISION_BASE_LINE+1; y < VISION_FORWARD_FAR; y++) // 向上搜线到far行
    {
        lineinfo[y].y = y;
        line_findnext(&lineinfo[y], Grayscale[119-y], &lineinfo[y - 1]);
        watch.watch_line = y;
    }
    //line_findnext(&lineinfo[VISION_BASE_LINE], Grayscale[119-VISION_BASE_LINE], &lineinfo[VISION_BASE_LINE - 1]);

    /* 扫完立刻做中心线二次拟合：这样控制层在 ScanLine 之后就能拿到 path_fit，
     * 不必等 Element_recognition 算完逐行中心，避免两个模块的执行顺序耦合。 */
    Vision_FitPath();
}


/*寻找基准行
 * ****************************************************************************
 */
int line_single(struct lineinfo_s *lineinfo, unsigned char *inputimg)
{
    //得到所有凸边沿
    uint8_t edge_store[VISION_EDGE_STORE_SIZE] = {0};
	//扫描inputimg数组（即行数组），并将跳变沿所对应的列数依次放到edge_store数组中
    lineinfo->edge_count = get_orign_edges(inputimg, edge_store);

    //得到最大边沿
    get_max_edge(lineinfo, edge_store);
    return 0;
}


/*
 * 从基准行向下搜线
 */
int line_findnext(struct lineinfo_s *lineinfo, uint8_t *inputimg, struct lineinfo_s *lineinfo_ref)
{
    //得到所有凸边沿
    //uint8_t edge_store[VISION_EDGE_STORE_SIZE] = {0};
    uint8_t *edge_store = lineinfo->edge_store;
    lineinfo->edge_count = get_orign_edges(inputimg, edge_store);
     //watch.addline_y = lineinfo->y;
     if (lineinfo->y > 20 && lineinfo->y < 100)
	{
         zebra_detect(lineinfo, edge_store, inputimg); //斑马线检测
         //apriltag_detect(lineinfo, edge_store, inputimg); //apriltag检测
     }
    //得到最佳边沿
    get_best_edge(lineinfo, edge_store, lineinfo_ref);
    //Junc_detect(lineinfo, edge_store, inputimg); //apriltag检测

    return 0;
}


/*
 *存储所有黑白跳变沿位置到数组edge_store中
 */
uint8_t get_orign_edges(uint8_t *inputimg, uint8_t *edge_store)
{
	#if VISION_EDGE_STORE_SIZE % 2 != 0
	#error "VISION_EDGE_STORE_SIZE must be even!"
	#endif
    //查找并存储所有跳变沿对
    uint8_t edge_store_idx = 0;
    if (inputimg[0] >watch.threshold)//数组第0位大于阈值（图像最左端为白）则认为此处为跳变沿
    {
        edge_store[edge_store_idx] = 0;
        edge_store_idx++;
    }
    for (uint8_t px = 1; px < VISION_IMG_WIDTH; px++)
    {
        //if ((inputimg[px - 1]-watch.threshold)*(inputimg[px]-watch.threshold) <= 0) //分布在watch.threshold两侧认为是跳边沿。。不稳定
        if(!((inputimg[px-1]>watch.threshold&&inputimg[px]<watch.threshold)||(inputimg[px-1]<watch.threshold&&inputimg[px]>watch.threshold)))    //可变灰度区分值
            continue;
        if (edge_store_idx >= VISION_EDGE_STORE_SIZE)
            break;
        edge_store[edge_store_idx] = px;   //将跳变沿存入数组
        edge_store_idx++;
    }
    if (inputimg[VISION_IMG_WIDTH - 1] >watch.threshold) //数组最后一位大于阈值（图像最右端为白）则认为此处为跳变沿
    {
        if (edge_store_idx < VISION_EDGE_STORE_SIZE)
        {
            edge_store[edge_store_idx] = VISION_IMG_WIDTH - 1;
            edge_store_idx++;
        }
    }
    return edge_store_idx;
}


/*得到最大边沿
 * ***************************************************************************
 */
int get_max_edge(struct lineinfo_s *lineinfo, uint8_t *edge_store)
{
    //选取最大的块    --可能的BUG:此处无法忽略噪点
    int max_width = 0;
    int max_idx = 0;

    if (lineinfo->edge_count < 10)
    {
        for (int k = 0; k < lineinfo->edge_count; k += 2)
        {
            if (edge_store[k + 1] - edge_store[k] > max_width)     //获取相差最大的跳变沿
            {
                max_width = edge_store[k + 1] - edge_store[k];
                max_idx = k;
            }
        }
    }
    else
    {
        max_idx = 0;
    }

    lineinfo->left = edge_store[max_idx];
    lineinfo->right = edge_store[lineinfo->edge_count - 1]; //这个地方保持疑问
  // 可能是这个lineinfo->right = edge_store[max_idx + 1];
    //lineinfo->right = edge_store[max_idx + 1];
    //Send_two_DataToVofa(lineinfo->left,lineinfo->right);
    return 0;
}


/*
 * 寻找最佳边沿
 * 根据相邻的一行找到的左边沿和右边沿位置,迭代找出本行与之距离最小的边沿位置
 */

int get_best_edge(struct lineinfo_s *lineinfo, uint8_t *edge_store, struct lineinfo_s *lineinfo_ref)
{
    int min_abs_err;//最小偏差,在迭代过程中这个值越来越小
    uint8_t temp_left_idx = 0, temp_right_idx = 1;
    int temp_err;//本轮迭代下的偏差量
    //获得最近邻的边沿
    min_abs_err = VISION_IMG_WIDTH * 2;
    for (uint8_t k = 0; k < lineinfo->edge_count; k += 2)
    {
        temp_err = abs((int)lineinfo_ref->left - (int)edge_store[k]);
        if (temp_err < min_abs_err)
        {
            min_abs_err = temp_err;
            temp_left_idx = k;
        }
        if (edge_store[k] > lineinfo_ref->left + min_abs_err)
            break;
    }
    temp_right_idx = temp_left_idx + 1;
    //Send_four_DataToVofa(min_abs_err,lineinfo_ref->left,lineinfo_ref->y,lineinfo_ref->left_lost*50);
    min_abs_err = VISION_IMG_WIDTH * 2;
    for (uint8_t k = temp_right_idx; k < lineinfo->edge_count; k += 2)
    {
        temp_err = abs((int)lineinfo_ref->right - (int)edge_store[k]);
        if (temp_err < min_abs_err)
        {
            min_abs_err = temp_err;
            temp_right_idx = k;
        }
        if (edge_store[k] > lineinfo_ref->right + min_abs_err)
            break;
    }
    //Send_four_DataToVofa(min_abs_err,0,lineinfo_ref->y,0);
   // if (abs((int)edge_store[temp_right_idx] - edge_store[temp_left_idx]) > 4)
    if (lineinfo->y>20&&lineinfo->edge_count>1&&(edge_store[temp_right_idx] - edge_store[temp_left_idx]) > 4  )
    {
        watch.watch_lost = watch.watch_line;
        lineinfo->whole_lost=0;
        // camcfg.watch_lost=  camcfg.watch_line;
    }
    else
    {
        lineinfo->whole_lost=1;
    }

    //判断中间是否有杂点，障碍物，环岛等
    //累加中间的黑色部分求取比例
    int black_pix_count = 0;
    for (int k = temp_left_idx + 1; k < temp_right_idx; k += 2)
	{
        black_pix_count += edge_store[k + 1] - edge_store[k];
	}
    if (8 * black_pix_count > edge_store[temp_right_idx] - edge_store[temp_left_idx]) //如果有不可忽略的杂物
    {
        //选取最大的块    --可能的BUG:此处无法继续忽略噪点
        int min_width = 188;
        int max_idx = temp_left_idx;

        int dst = temp_left_idx + 1;
        uint8_t dst_flag;
        for (int i = temp_left_idx + 1; i < temp_right_idx; i += 2)
        {
            if (edge_store[i + 1] - edge_store[i] < VISION_MIN_EDGE_WIDTH)
            {
                edge_store[dst] = edge_store[i + 2];
                lineinfo->edge_count -= 2;
                dst_flag = 1;
            }
            //  else中内容注释掉现象依然正确  但可能有坑
            else
            {
                if (dst_flag == 1) //如果之前已经出现过被忽略的黑块
                {
                    edge_store[dst + 2] = edge_store[i];
                    edge_store[dst + 3] = edge_store[i + 1];
                }
                else // 当前不可忽略的黑块为第一块
                {
                    edge_store[dst] = edge_store[i];
                    edge_store[dst + 1] = edge_store[i + 1];
                }
                dst += 2;
            }
        }

        for (int k = temp_left_idx; k < lineinfo->edge_count - 1; k += 2)
        {
            if (abs(edge_store[k + 1] + edge_store[k]-188) < min_width)
            {
                min_width = abs(edge_store[k + 1] + edge_store[k]-188);
                max_idx = k;
            }
        }
        //if(watch.scan_line_advantage==0)
        {
            temp_left_idx = max_idx;
            temp_right_idx = max_idx + 1;
        }
//        else if(watch.scan_line_advantage==1)
//        {
//            temp_right_idx=temp_left_idx+1;
//        }
//        else if(watch.scan_line_advantage==2)
//        {
//            temp_left_idx=temp_right_idx-1;
//        }
    }

    lineinfo->left = edge_store[temp_left_idx];
    lineinfo->right = edge_store[temp_right_idx];
    if (lineinfo->right >= 187)
	{
        lineinfo->right_lost = 1;
	}
    else
	{
        lineinfo->right_lost = 0;
	}
    if (lineinfo->left <= 0)
	{
        lineinfo->left_lost = 1;
	}
    else
	{
        lineinfo->left_lost = 0;
	}
    return 0;
}

//斑马线识别函数
int zebra_detect(struct lineinfo_s *lineinfo, uint8_t *edge_store, uint8_t *inputimg)
{
    uint8_t white_width, zobra_white_count, edge_now;
    zobra_white_count = 0;
    if (lineinfo->edge_count > 12)
    {
		for (uint8_t k = 0; k < lineinfo->edge_count; k += 2)
		{
			edge_now = edge_store[k];
			if (edge_now < VISION_IMG_WIDTH)
			{
				if ((inputimg[edge_now] & 0x80) == 0x80)
				{
					white_width = edge_store[k + 1] - edge_store[k];
					if (white_width > 1 && white_width < 14)
					{
						zobra_white_count++;
						watch.ZebraInLine=lineinfo->y;//取满足条件时y最大值作为ZebraLine
					}                 //watch.ZebraInLine为看到的斑马线最远端
				}
			}
		}
		if (zobra_white_count > 6)
		{
			lineinfo->zebra_flag = 1;
			//beep(20);
			// if(!mycar.status)
			//ips200_draw_horizon(0, 119 - lineinfo->y, 188, 119 - lineinfo->y, BLUE);
		}
	}
    else
    {
        lineinfo->zebra_flag = 0;
    }
    return 0;
}


/* ========================================================================== */
/*                    道路中心线加权最小二乘二次拟合（新增）                     */
/* ========================================================================== */

/** @brief 权重衰减系数：w = 1/(1 + k·|t|)，k = 0.6 时端点权重 0.625 */
#define FIT_WEIGHT_K        (0.6f)

/** @brief 行坐标归一化基准 = 控制行（t = 0 落在控制行上） */
#define FIT_T_CENTER        ((float)VISION_CONTROL_ROW)

/** @brief 行坐标归一化尺度 = 半幅高度（t 覆盖 -1 ~ +1） */
#define FIT_T_SCALE         ((float)VISION_IMG_HEIGHT * 0.5f)

/** @brief 中心线拟合结果的**定义**（extern 声明在 vision_types.h） */
struct path_fit_s path_fit;

/** @brief 每行参与拟合的权重（第二遍会把离群行清 0） */
static float s_fit_weight[VISION_IMG_HEIGHT];

/** @brief 每行参与拟合的中心列 */
static float s_fit_x[VISION_IMG_HEIGHT];

/** @brief 每行是否可参与拟合 */
static uint8_t s_fit_ok[VISION_IMG_HEIGHT];

/** @brief 每行相对直线拟合的绝对残差（缓存下来避免重复计算） */
static float s_fit_resid[VISION_IMG_HEIGHT];

/**
 * @brief  用当前权重解一次 3x3 正规方程
 * @param  abc   输出二次多项式系数 a, b, c
 * @param  n_out 输出实际参与拟合的行数（可为 0）
 * @retval 1 成功；0 有效行不足或矩阵奇异
 * @note   解的是加权最小二乘的正规方程：
 *             [s0 s1 s2][a]   [t0]
 *             [s1 s2 s3][b] = [t1]     sk = Σ w·t^k , tk = Σ w·t^k·x
 *             [s2 s3 s4][c]   [t2]
 *         因为 t 归一化到 [-1,1]，s0~s4 都在同一量级，float32 足够。
 */
static uint8_t Fit_SolveOnce(float abc[3], uint8_t *n_out)
{
    float s0 = 0.0f, s1 = 0.0f, s2 = 0.0f, s3 = 0.0f, s4 = 0.0f;
    float t0 = 0.0f, t1 = 0.0f, t2 = 0.0f;
    float det;
    uint8_t n = 0U;
    int row;

    for (row = VISION_BASE_LINE; row < VISION_FORWARD_FAR; row++)
    {
        float w = s_fit_weight[row];
        float t, x, t2v;

        if ((w <= 0.0f) || (s_fit_ok[row] == 0U))
        {
            continue;
        }

        t   = ((float)row - FIT_T_CENTER) / FIT_T_SCALE;
        x   = s_fit_x[row];
        t2v = t * t;

        s0 += w;
        s1 += w * t;
        s2 += w * t2v;
        s3 += w * t2v * t;
        s4 += w * t2v * t2v;
        t0 += w * x;
        t1 += w * t * x;
        t2 += w * t2v * x;
        n++;
    }

    if (n_out != 0)
    {
        *n_out = n;
    }
    if (n < VISION_FIT_MIN_ROWS)
    {
        return 0U;
    }

    /* Cramer 法则，3x3 直接展开 */
    det = (s0 * ((s2 * s4) - (s3 * s3)))
        - (s1 * ((s1 * s4) - (s3 * s2)))
        + (s2 * ((s1 * s3) - (s2 * s2)));

    if ((det < 1.0e-6f) && (det > -1.0e-6f))
    {
        return 0U;      /* 奇异：所有行几乎落在同一个 t 上 */
    }

    abc[0] = ((t0 * ((s2 * s4) - (s3 * s3)))
            - (s1 * ((t1 * s4) - (s3 * t2)))
            + (s2 * ((t1 * s3) - (s2 * t2)))) / det;

    abc[1] = ((s0 * ((t1 * s4) - (s3 * t2)))
            - (t0 * ((s1 * s4) - (s3 * s2)))
            + (s2 * ((s1 * t2) - (t1 * s2)))) / det;

    abc[2] = ((s0 * ((s2 * t2) - (t1 * s3)))
            - (s1 * ((s1 * t2) - (t1 * s2)))
            + (t0 * ((s1 * s3) - (s2 * s2)))) / det;

    return 1U;
}

/**
 * @brief  用当前权重解一次**加权直线**最小二乘  x(t) = a + b·t
 * @param  ab    输出 a, b
 * @param  n_out 输出实际参与的行数
 * @retval 1 成功；0 有效行不足或矩阵奇异
 * @note   直线只有 2 个自由度，是本模块**第二层防御**的基石：
 *         一段连续的坏行只能把直线整体倾斜，不能把它"弯折"，
 *         所以直线的残差能可靠暴露坏行，不会出现二次曲线那种掩蔽效应。
 */
static uint8_t Fit_SolveLineOnce(float ab[2], uint8_t *n_out)
{
    float s0 = 0.0f, s1 = 0.0f, s2 = 0.0f, t0 = 0.0f, t1 = 0.0f;
    float det;
    uint8_t n = 0U;
    int row;

    for (row = VISION_BASE_LINE; row < VISION_FORWARD_FAR; row++)
    {
        float w = s_fit_weight[row];
        float t, x;

        if ((w <= 0.0f) || (s_fit_ok[row] == 0U))
        {
            continue;
        }

        t = ((float)row - FIT_T_CENTER) / FIT_T_SCALE;
        x = s_fit_x[row];

        s0 += w;
        s1 += w * t;
        s2 += w * t * t;
        t0 += w * x;
        t1 += w * t * x;
        n++;
    }

    if (n_out != 0)
    {
        *n_out = n;
    }
    if (n < VISION_FIT_MIN_ROWS)
    {
        return 0U;
    }

    det = (s0 * s2) - (s1 * s1);
    if ((det < 1.0e-6f) && (det > -1.0e-6f))
    {
        return 0U;
    }

    ab[0] = ((t0 * s2) - (t1 * s1)) / det;
    ab[1] = ((t1 * s0) - (t0 * s1)) / det;

    return 1U;
}

void Vision_FitPath(void)
{
    float   abc[3];
    float   ab[2];
    uint8_t n_line = 0U;
    uint8_t n = 0U;
    int     row;

    /* ---------- 1. 收集有效行 + 给基础权重 ---------- */
    for (row = 0; row < VISION_IMG_HEIGHT; row++)
    {
        float t, at;

        s_fit_ok[row]     = 0U;
        s_fit_weight[row] = 0.0f;

        if ((row < VISION_BASE_LINE) || (row >= VISION_FORWARD_FAR))
        {
            continue;
        }
        if ((lineinfo[row].whole_lost != 0) ||
            (lineinfo[row].left_lost  != 0) ||
            (lineinfo[row].right_lost != 0))
        {
            continue;
        }

        s_fit_ok[row] = 1U;
        s_fit_x[row]  = ((float)lineinfo[row].left + (float)lineinfo[row].right) * 0.5f;

        /* 权重随离控制行的距离衰减（近处更可信），
         * 但端点仍保留 62.5%，这样远处弯道的信息不会被压没。 */
        t  = ((float)row - FIT_T_CENTER) / FIT_T_SCALE;
        at = (t < 0.0f) ? -t : t;
        s_fit_weight[row] = 1.0f / (1.0f + (FIT_WEIGHT_K * at));
    }

    /* ======================================================================
     * 第二层防御：先用**直线**拟合一次，再用它的残差筛坏行
     * ----------------------------------------------------------------------
     * 为什么不能直接用二次曲线筛（旧实现的问题）：
     *   一段连续的坏行（直角弯的垂直赛道、阴影、缺口，通常 10~20 行）
     *   会把第一遍的二次曲线"拉弯"，曲线恰好穿过这些坏行，
     *   于是它们的残差反而很小 —— 判据认为它们"正常"，第二遍原样保留。
     *   这就是鲁棒回归里的 masking（掩蔽），表现为你看到的"曲率非常大的抛物线"。
     *
     *   直线只有 2 个自由度，连续的坏行只能让直线整体倾斜、不能让直线弯折，
     *   所以坏行在直线上的残差一定很大 —— 掩蔽效应被彻底绕开。
     * ====================================================================== */
    if (Fit_SolveLineOnce(ab, &n_line) == 0U)
    {
        /* 有效行不足或退化：清空结果，让上层退回"只看控制行"的旧逻辑 */
        path_fit.a          = (float)VISION_IMG_CX;
        path_fit.b          = 0.0f;
        path_fit.c          = 0.0f;
        path_fit.x_ctrl     = (float)VISION_IMG_CX;
        path_fit.x_look     = (float)VISION_IMG_CX;
        path_fit.slope_ctrl = 0.0f;
        path_fit.rows_used  = n_line;
        path_fit.valid      = 0U;
        return;
    }

    /* ---------- 2. 缓存每行对直线的绝对残差 ---------- */
    for (row = VISION_BASE_LINE; row < VISION_FORWARD_FAR; row++)
    {
        float t_row;

        if (s_fit_ok[row] == 0U)
        {
            s_fit_resid[row] = 0.0f;
            continue;
        }

        t_row = ((float)row - FIT_T_CENTER) / FIT_T_SCALE;
        s_fit_resid[row] = s_fit_x[row] - (ab[0] + (ab[1] * t_row));

        if (s_fit_resid[row] < 0.0f)
        {
            s_fit_resid[row] = -s_fit_resid[row];
        }
    }

    /* ---------- 3. 按**固定**门限把坏行权重清零 ----------
     * 为什么是固定阈值而不是自适应倍数，见 vision_types.h 里的实测对照表：
     * 强弯场景要求门限 > 19px，坏块场景要求门限 < 32px，两者能同时满足；
     * 而任何 `K × 中位数` 都做不到（两个场景对 K 的要求方向相反）。 */
    for (row = VISION_BASE_LINE; row < VISION_FORWARD_FAR; row++)
    {
        if (s_fit_ok[row] == 0U)
        {
            continue;
        }

        if (s_fit_resid[row] > VISION_FIT_OUTLIER_PX)
        {
            s_fit_weight[row] = 0.0f;
        }
    }

    /* ---------- 5. 用剩下的内点拟合二次曲线 ---------- */
    if (Fit_SolveOnce(abc, &n) == 0U)
    {
        /* 筛完内点不够：退回直线结果（c = 0）。宁可没有曲率，
         * 也不能让一个解不出来的二次曲线污染偏移。 */
        abc[0] = ab[0];
        abc[1] = ab[1];
        abc[2] = 0.0f;
        n      = n_line;
    }

    /* ======================================================================
     * 第一层防御：曲率物理上限
     * ----------------------------------------------------------------------
     * 即使数据筛干净了，二次项仍可能被残差噪声顶到不合理的大小。
     * 这里把 |c| 夹到 VISION_FIT_MAX_SAGITTA_PX —— 曲线在图像上下端
     * 最多比直线弯 20 像素，从源头禁掉"曲率非常大的抛物线"。
     * 注意只夹 c，不动 a（= x_ctrl），所以**偏移不受影响**。
     * ====================================================================== */
    if (abc[2] > VISION_FIT_MAX_SAGITTA_PX)
    {
        abc[2] = VISION_FIT_MAX_SAGITTA_PX;
    }
    else if (abc[2] < -VISION_FIT_MAX_SAGITTA_PX)
    {
        abc[2] = -VISION_FIT_MAX_SAGITTA_PX;
    }

    /* ---------- 6. 填写结果 ----------
     * 控制行处 t = 0，所以 x_ctrl = a；
     * 斜率 dx/drow = (b + 2c·t) / scale，在 t = 0 处即 b / scale。 */
    path_fit.a          = abc[0];
    path_fit.b          = abc[1];
    path_fit.c          = abc[2];
    path_fit.x_ctrl     = abc[0];
    path_fit.slope_ctrl = abc[1] / FIT_T_SCALE;

    {
        float t_look = ((float)VISION_LOOKAHEAD_ROW - FIT_T_CENTER) / FIT_T_SCALE;
        path_fit.x_look = abc[0] + ((abc[1] + (abc[2] * t_look)) * t_look);
    }

    path_fit.rows_used = n;
    path_fit.valid     = 1U;
}

float Vision_GetFitX(float row)
{
    return Vision_EvalPathFit(row);
}





