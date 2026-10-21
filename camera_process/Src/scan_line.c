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
/*          道路中心线拟合：离散正交多项式（三项递推）+ 加权内积投影            */
/* ========================================================================== */
/*
 * 为什么改成"正交基 + 内积投影"，而不是直接解正规方程
 * ---------------------------------------------------------------------------
 * 目标函数完全一样：
 *      min Σ w_i·(x_i − p(t_i))² ,  p(t) = a + b·t + c·t²
 * 最小二乘解唯一，所以**两种做法的答案在数学上是同一个点**。
 * 差别在"怎么求"，以及"求不出来的时候怎么办"。
 *
 * 【直接法】未知量是幂基系数 (a,b,c)，正规方程为
 *      G·[a b c]ᵀ = [s0 s1 s2]ᵀ ,   G_jk = Σ w·t^(j+k),  s_k = Σ w·x·t^k
 * G 是 Hankel 矩阵，条件数只由数据跨度决定：
 *      t ∈ [-1,1]（满幅）        κ ≈ 5.8      —— 很好
 *      t ∈ [0, 0.6]（只剩远端）  κ ≈ 1e4      —— float32 只剩 3 位有效数字
 * 更麻烦的是"退化判断"：det 会随跨度变窄**连续趋近** 0 但永远不为 0，
 * 多小才算退化只能拍一个**有量纲**的绝对阈值（旧代码是 1e-6）。
 * 直角弯近端丢线、只剩远端十几行时，det 正好在这个量级附近来回跳，
 * 于是同一场景一帧解得出来、一帧解出个曲率上百的怪抛物线 —— 车就在那抖。
 *
 * 【正交法】先正交化再投影。对加权离散内积
 *      <f,g> = Σ w_i·f(t_i)·g(t_i)
 * 用三项递推生成（Stieltjes 正交化，不需要知道样本分布）：
 *      p0 = 1
 *      p1 = (t − α1)·p0                 α1 = <t·p0,p0>/<p0,p0>
 *      p2 = (t − α2)·p1 − β2·p0         α2 = <t·p1,p1>/<p1,p1>
 *                                       β2 = <p1,p1>/<p0,p0>
 * 满足 <p_j,p_k> = 0 (j ≠ k)。于是正规方程**对角化**，系数就是一个投影：
 *      c_k = <x,p_k> / <p_k,p_k>
 * 没有矩阵、没有 Cramer 展开、没有 det，只有内积和一次除法。
 *
 * 三个实际收益：
 *   1. **降阶有客观依据**：<p_k,p_k> 就是"第 k 阶有没有被数据撑住"。
 *      跨度不足时它连续趋于 0，用**相对**门限 VISION_FIT_ORTHO_MIN_REL
 *      判"这阶解不出来"，然后退回低一阶 —— 而不是输出天文数字。
 *      （实测：满幅 0.0922、半幅 0.0058、1/3 幅 0.00075，见 vision_types.h）
 *   2. **数值稳定**：正交基下 G 是对角阵，条件数归一到 1，float32 的有效
 *      位数不再被 κ 吃掉；跨度变窄只影响范数大小，不影响解的精度。
 *   3. **可扩展**：要加三次/四次（贴透视畸变）只是多跑一趟递推，不用重写
 *      解方程；每一阶还天然给出"消掉了多少残差平方和"。
 *
 * 代价：按阶数多扫几遍样本（2 阶 = 3 遍）。样本最多 115 个、每遍约 5 次
 * 浮点运算，合计 ~1700 次，与旧的 Cramer 展开同量级；480 MHz + FPU 下
 * 是微秒级，对 30 Hz 帧率无影响。
 *
 * 换回幂基（path_fit / Vision_EvalPathFit 对外仍是 a + b·t + c·t²）：
 *      p1 = t − α1
 *      p2 = t² − (α1+α2)·t + (α1·α2 − β2)
 *   => c = c2
 *      b = c1 − c2·(α1 + α2)
 *      a = c0 − c1·α1 + c2·(α1·α2 − β2)
 * 纯代数整理，误差在 float32 舍入量级（< 1e-4 px）。
 */

/** @brief 权重衰减系数：w = 1/(1 + k·|t|)，k = 0.6 时端点权重 0.625 */
#define FIT_WEIGHT_K        (0.6f)

/** @brief 行坐标归一化基准 = 控制行（t = 0 落在控制行上） */
#define FIT_T_CENTER        ((float)VISION_CONTROL_ROW)

/** @brief 行坐标归一化尺度 = 半幅高度（t 覆盖 -1 ~ +1） */
#define FIT_T_SCALE         ((float)VISION_IMG_HEIGHT * 0.5f)

/** @brief 单帧参与拟合的最大样本数（= 扫线区间长度，一个 buf 装得下） */
#define FIT_MAX_SAMPLES     (VISION_IMG_HEIGHT)

/** @brief 中心线拟合结果的**定义**（extern 声明在 vision_types.h） */
struct path_fit_s path_fit;

/* 采样缓存：只放"有效行"，压缩成 t/x/w 三组样本。
 * 3×120×4 = 1440 B，比改动前的 weight/x/ok/resid(≈1.56 KB) 还小一点。
 * 之所以先压缩再拟合：稀疏有效行（直角弯常见只剩 20~40 行）不必再为整幅图
 * 维护"这行有没有效"的标记数组，拟合函数也能脱离图像单独测试。 */
static float   s_fit_t[FIT_MAX_SAMPLES];    /**< 归一化行坐标 t */
static float   s_fit_x[FIT_MAX_SAMPLES];    /**< 该行中心列 */
static float   s_fit_w[FIT_MAX_SAMPLES];    /**< 该行权重（离群行被清 0） */
static uint8_t s_fit_n;                     /**< 样本个数 */

/**
 * @brief  把拟合结果复位成"无效"状态
 * @param  rows 报告的有效行数
 * @note   无效时 x_ctrl / x_look 都退回图像中心：即使丢线，控制层拿到的也是
 *         0 偏差，而不是一个离谱的跳变。
 */
static void Fit_ResetInvalid(uint8_t rows)
{
    path_fit.a          = (float)VISION_IMG_CX;
    path_fit.b          = 0.0f;
    path_fit.c          = 0.0f;
    path_fit.x_ctrl     = (float)VISION_IMG_CX;
    path_fit.x_look     = (float)VISION_IMG_CX;
    path_fit.slope_ctrl = 0.0f;
    path_fit.resid_rms  = 0.0f;
    path_fit.t_span     = 0.0f;
    path_fit.order      = 0U;
    path_fit.rows_used  = rows;
    path_fit.valid      = 0U;
}

uint8_t Vision_FitPoly(const float *t, const float *x, const float *w,
                       uint8_t n, uint8_t order, struct path_poly_s *out)
{
    float n0 = 0.0f, n1 = 0.0f, n2 = 0.0f;   /* <p_k,p_k> 三个范数               */
    float p0 = 0.0f, p1 = 0.0f, p2 = 0.0f;   /* <x,p_k>   三个投影              */
    float sum_t = 0.0f;                      /* <t·p0,p0>，算 α1 用             */
    float sum_tp1 = 0.0f;                    /* <t·p1,p1>，算 α2 用             */
    float alpha1, alpha2 = 0.0f, beta2 = 0.0f;
    float c0, c1 = 0.0f, c2 = 0.0f;
    float rms = 0.0f, wsum = 0.0f;
    float t_min = 0.0f, t_max = 0.0f;
    uint8_t used = 0U;
    uint8_t reached;
    uint8_t i;

    if ((t == 0) || (x == 0) || (w == 0) || (out == 0) || (n == 0U))
    {
        return 0U;
    }

    /* ---------- 第 1 遍：p0 = 1 的范数与投影，顺带攒 α1 的分子 ---------- */
    for (i = 0U; i < n; i++)
    {
        float wi = w[i];

        if (wi <= 0.0f)
        {
            continue;
        }

        n0    += wi;
        p0    += wi * x[i];
        sum_t += wi * t[i];
        used++;
    }

    if ((used == 0U) || (n0 <= 0.0f))
    {
        return 0U;
    }

    alpha1 = sum_t / n0;

    /* ---------- 第 2 遍：p1 = t − α1 ---------- */
    for (i = 0U; i < n; i++)
    {
        float wi = w[i];
        float ti, v;

        if (wi <= 0.0f)
        {
            continue;
        }

        ti = t[i];
        v  = ti - alpha1;

        n1      += wi * v * v;
        p1      += wi * x[i] * v;
        sum_tp1 += wi * ti * v * v;
    }

    /* ---------- 第 3 遍：p2 = (t − α2)·p1 − β2（只在上一阶还撑得住时做） ---------- */
    if ((order >= 2U) && (n1 > (n0 * VISION_FIT_ORTHO_MIN_REL)))
    {
        alpha2 = sum_tp1 / n1;
        beta2  = n1 / n0;

        for (i = 0U; i < n; i++)
        {
            float wi = w[i];
            float ti, v;

            if (wi <= 0.0f)
            {
                continue;
            }

            ti = t[i];
            v  = ((ti - alpha2) * (ti - alpha1)) - beta2;

            n2 += wi * v * v;
            p2 += wi * x[i] * v;
        }
    }

    /* ---------- 秩判定：范数塌了 = 这一阶没被数据撑住 ---------- */
    reached = 0U;
    if (n1 > (n0 * VISION_FIT_ORTHO_MIN_REL))
    {
        reached = 1U;
    }
    if ((order >= 2U) && (n2 > (n0 * VISION_FIT_ORTHO_MIN_REL)))
    {
        reached = 2U;
    }

    if (reached == 0U)
    {
        /* 所有有效样本落在同一个 t 上：连斜率都定不出来 */
        out->order = 0U;
        return 0U;
    }

    /* ---------- 投影系数 → 幂基系数 ---------- */
    c0 = p0 / n0;
    if (reached >= 1U)
    {
        c1 = p1 / n1;
    }
    if (reached >= 2U)
    {
        c2 = p2 / n2;
    }

    out->c = c2;
    out->b = c1 - (c2 * (alpha1 + alpha2));
    out->a = c0 - (c1 * alpha1) + (c2 * ((alpha1 * alpha2) - beta2));

    /* ---------- 质量指标：加权内点残差 + 数据跨度 ---------- */
    {
        uint8_t first = 1U;

        for (i = 0U; i < n; i++)
        {
            float wi = w[i];
            float ti, r;

            if (wi <= 0.0f)
            {
                continue;
            }

            ti = t[i];
            r  = x[i] - (out->a + ((out->b + (out->c * ti)) * ti));

            rms  += wi * r * r;
            wsum += wi;

            if (first != 0U)
            {
                t_min = ti;
                t_max = ti;
                first = 0U;
            }
            else
            {
                if (ti < t_min)
                {
                    t_min = ti;
                }
                if (ti > t_max)
                {
                    t_max = ti;
                }
            }
        }
    }

    out->rms    = (wsum > 0.0f) ? sqrtf(rms / wsum) : 0.0f;
    out->t_span = t_max - t_min;
    out->order  = reached;
    out->n_used = used;

    return 1U;
}

void Vision_FitPath(void)
{
    struct path_poly_s line;
    struct path_poly_s quad;
    struct path_poly_s poly;
    float   t_look;
    uint8_t i;
    int     row;

    s_fit_n = 0U;

    /* ---------- 1. 收集有效行，压缩成 (t, x, w) 样本 ---------- */
    for (row = VISION_BASE_LINE; row < VISION_FORWARD_FAR; row++)
    {
        float t, at;

        if ((lineinfo[row].whole_lost != 0) ||
            (lineinfo[row].left_lost  != 0) ||
            (lineinfo[row].right_lost != 0))
        {
            continue;
        }
        if (s_fit_n >= (uint8_t)FIT_MAX_SAMPLES)
        {
            break;
        }

        t  = (((float)row) - FIT_T_CENTER) / FIT_T_SCALE;
        at = (t < 0.0f) ? -t : t;

        s_fit_t[s_fit_n] = t;
        s_fit_x[s_fit_n] = ((float)lineinfo[row].left + (float)lineinfo[row].right) * 0.5f;

        /* 权重随离控制行的距离衰减（近处更可信），
         * 但端点仍保留 62.5%，这样远处弯道的信息不会被压没。 */
        s_fit_w[s_fit_n] = 1.0f / (1.0f + (FIT_WEIGHT_K * at));
        s_fit_n++;
    }

    if (s_fit_n < VISION_FIT_MIN_ROWS)
    {
        Fit_ResetInvalid(s_fit_n);
        return;
    }

    /* ======================================================================
     * 第二层防御：先用**直线**拟合一次，再用它的残差筛坏行
     * ----------------------------------------------------------------------
     * 为什么不能直接用二次曲线筛（旧实现的问题）：
     *   一段连续的坏行（直角弯的垂直赛道、阴影、缺口，通常 10~20 行）
     *   会把第一遍的二次曲线"拉弯"，曲线恰好穿过这些坏行，
     *   于是它们的残差反而很小 —— 判据认为它们"正常"，第二遍原样保留。
     *   这就是鲁棒回归里的 masking（掩蔽），表现为"曲率非常大的抛物线"。
     *
     *   直线只有 2 个自由度，连续的坏行只能让直线整体倾斜、不能让直线弯折，
     *   所以坏行在直线上的残差一定很大 —— 掩蔽效应被彻底绕开。
     * ====================================================================== */
    if ((Vision_FitPoly(s_fit_t, s_fit_x, s_fit_w, s_fit_n, 1U, &line) == 0U) ||
        (line.n_used < VISION_FIT_MIN_ROWS))
    {
        Fit_ResetInvalid(s_fit_n);
        return;
    }

    /* ---------- 2. 缓存不了残差也没关系：就地按**固定**门限清权重 ----------
     * 为什么是固定阈值而不是自适应倍数，见 vision_types.h 里的实测对照表：
     * 强弯场景要求门限 > 19px，坏块场景要求门限 < 32px，两者能同时满足；
     * 而任何 `K × 中位数` 都做不到（两个场景对 K 的要求方向相反）。 */
    for (i = 0U; i < s_fit_n; i++)
    {
        float r;

        if (s_fit_w[i] <= 0.0f)
        {
            continue;
        }

        r = s_fit_x[i] - (line.a + (line.b * s_fit_t[i]));
        if ((r > VISION_FIT_OUTLIER_PX) || (r < -VISION_FIT_OUTLIER_PX))
        {
            s_fit_w[i] = 0.0f;
        }
    }

    /* ---------- 3. 用内点拟合二次曲线 ----------
     * 内点不足、或 t 跨度不足以支撑二次项时，Vision_FitPoly 会自己把
     * order 降到 1（<p2,p2> 塌到门限下），此时 c 一定是 0，等价于直线结果。 */
    if ((Vision_FitPoly(s_fit_t, s_fit_x, s_fit_w, s_fit_n, 2U, &quad) != 0U) &&
        (quad.n_used >= VISION_FIT_MIN_ROWS))
    {
        poly = quad;
    }
    else
    {
        /* 筛完内点不够：退回直线结果（c = 0）。宁可没有曲率，
         * 也不能让一个解不出来的二次曲线污染偏移。 */
        poly      = line;
        poly.c    = 0.0f;
        poly.order = 1U;
    }

    /* ======================================================================
     * 第一层防御：曲率物理上限
     * ----------------------------------------------------------------------
     * 即使数据筛干净了，二次项仍可能被残差噪声顶到不合理的大小。
     * 这里把 |c| 夹到 VISION_FIT_MAX_SAGITTA_PX —— 曲线在图像上下端
     * 最多比直线弯 60 像素，从源头禁掉"曲率非常大的抛物线"。
     * 注意只夹 c，不动 a（= x_ctrl），所以**偏移不受影响**；
     * 但 x_look 用的是被夹过的 c，钳位生效时前视值只作参考、不应重信。
     * ====================================================================== */
    if (poly.c > VISION_FIT_MAX_SAGITTA_PX)
    {
        poly.c = VISION_FIT_MAX_SAGITTA_PX;
    }
    else if (poly.c < -VISION_FIT_MAX_SAGITTA_PX)
    {
        poly.c = -VISION_FIT_MAX_SAGITTA_PX;
    }

    /* ---------- 4. 填写结果 ----------
     * 控制行处 t = 0，所以 x_ctrl = a；
     * 斜率 dx/drow = (b + 2c·t) / scale，在 t = 0 处即 b / scale。 */
    path_fit.a          = poly.a;
    path_fit.b          = poly.b;
    path_fit.c          = poly.c;
    path_fit.x_ctrl     = poly.a;
    path_fit.slope_ctrl = poly.b / FIT_T_SCALE;

    t_look = (((float)VISION_LOOKAHEAD_ROW) - FIT_T_CENTER) / FIT_T_SCALE;
    path_fit.x_look = poly.a + ((poly.b + (poly.c * t_look)) * t_look);

    path_fit.resid_rms = poly.rms;
    path_fit.t_span    = poly.t_span;
    path_fit.order     = poly.order;
    path_fit.rows_used = poly.n_used;
    path_fit.valid     = 1U;
}

float Vision_GetFitX(float row)
{
    return Vision_EvalPathFit(row);
}





