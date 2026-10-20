/**
 * @file    vision_types.h
 * @brief   视觉层共享数据与类型定义
 * @details 本文件是 Binarization / scan_line / Element_recognition 三个模块
 *          **唯一**的公共依赖。三条流水线的数据流向是：
 *
 *             DCMI+DMA 中断
 *                  |  填充
 *                  v
 *             mt9v03x_image[]  (原始帧，逐行指针)
 *                  |  Vision_Binarize()
 *                  v
 *             Grayscale[][]    (二值图，0/255)
 *                  |  Vision_ScanLine()
 *                  v
 *             lineinfo[]       (每行的左右边线)
 *                  |  Vision_ComputePathCenter()
 *                  v
 *             path_center[] / path_valid[]  (仅 Element_recognition.c 内部可见)
 *                  |  Vision_GetLateralError()
 *                  v
 *                 横向偏差  ->  交给控制层
 *
 * @note    分层约束（重要）
 *          - 本文件只声明**数据**，不声明任何硬件操作；
 *          - 视觉层的三个 .c **不允许** include 任何 HardWare/ 的头文件，
 *            也不允许调用电机 / 串口 / LCD。需要输出结果时，通过返回值交给
 *            应用层去处理。
 *          - 原来三个头文件互相 include（Binarization <-> scan_line <->
 *            Element_recognition）以及 Binarization.c include dcmi_ov2640.h
 *            的情况，已由本文件统一替代。
 *
 * @note    命名说明
 *          `mt9v03x_image` 这个名字来自移植过来的参考代码，本工程实际用的是
 *          OV2640。为保证与既有代码兼容暂时保留该名字，含义以注释为准。
 *
 * @warning 新写的代码请使用 stdint.h 的定长类型（uint8_t / int16_t），
 *          不要再用 type_def.h 里的 uint8 / int16 别名 —— 那套别名是为了
 *          兼容移植代码而保留的。
 */
#ifndef VISION_TYPES_H
#define VISION_TYPES_H

#include <stdint.h>

#include "type_def.h"   /* 移植代码使用的 uint8 / int16 等别名，见文件末尾说明 */

#ifdef __cplusplus
extern "C" {
#endif

/* ========================================================================== */
/*                                  图像几何                                   */
/* ========================================================================== */

/** @brief 参与视觉处理的图像宽度（列数），必须与实际显示宽度一致 */
#define VISION_IMG_WIDTH        (188)

/** @brief 参与视觉处理的图像高度（行数） */
#define VISION_IMG_HEIGHT       (120)

/** @brief 图像主点 X 坐标，横向偏差以此为基准 */
#define VISION_IMG_CX           (94)

/** @brief 用于计算横向偏差的控制行（图像中偏下的位置，看得远又不会太远） */
#define VISION_CONTROL_ROW      (60)

/* ========================================================================== */
/*                                  扫线参数                                   */
/* ========================================================================== */

/** @brief 单行内允许存储的跳变沿数量上限（必须是偶数） */
#define VISION_EDGE_STORE_SIZE  (22)

/** @brief 赛道绝对最小宽度（像素），窄于该宽度的边线对视为噪声 */
#define VISION_MIN_EDGE_WIDTH   (20)

/** @brief 寻找基准行的起始行号（从图像最下方往上数） */
#define VISION_BASE_LINE        (5)

/** @brief 向下搜线的终止行号 */
#define VISION_FORWARD_NEAR     (0)

/** @brief 向上搜线的终止行号 */
#define VISION_FORWARD_FAR      (115)

/* ========================================================================== */
/*                            中心线二次拟合参数                               */
/* ========================================================================== */

/**
 * @brief 前视行：拟合曲线在这个行号处求值，供"提前转向"使用
 * @note  行号越大越远（算法行 0 = 画面底部 = 离车最近）。
 *        控制行 60 是常规求值位置，前视取更远的 80。
 */
#define VISION_LOOKAHEAD_ROW    (80)

/** @brief 参与二次拟合的最少有效行数；少于它认为拟合不可信 */
#define VISION_FIT_MIN_ROWS     (8U)

/**
 * @brief 两遍法的离群阈值（像素）
 * @details 第一遍拟合后，残差超过本值的行判为离群点（典型情况：直角弯处
 *          垂直赛道被误当成左右边线），第二遍把这些行的权重置 0 再拟合一次。
 */
/**
 * @brief 直线门控阈值（第二层防御）：相对**直线**残差超过本值的行判为坏行
 * @details 用**固定阈值**而不是自适应倍数，是实测得出的结论。
 *
 *          | 场景                     | 残差中位数 | 合法弯最大残差 | 坏块残差 |
 *          |--------------------------|-----------|---------------|---------|
 *          | 强弯(c=34) + 稀疏离群点   |   ≈ 6     |     19        |   45    |
 *          | 直道 + 40px 连续坏块      |   ≈ 11    |     11        |   32    |
 *
 *          如果门限取 `K × 中位数`：
 *            - 要保住强弯的行，需要 K > 19/6  ≈ 3.2
 *            - 要抓住坏块，需要   K < 32/11 ≈ 2.9
 *          两个要求**方向相反**，不存在能同时满足的 K —— 自适应倍数这条路走不通
 *          （实测 K=3 时坏块漏过、K=4 时门限涨到 44 全部漏过）。
 *
 *          固定 25 px 则两个场景都有余量：19 < 25 < 32 ✔
 *
 * @note    副效果：这实际上把**可表达的曲率**限制在 c ≈ 37 左右
 *          （抛物线与直线的最大偏差 ≈ 2c/3）。超过这个曲率的行会被判坏行，
 *          曲率因此偏保守 —— 这是**有意的安全取舍**：宁可预瞄不足，
 *          也不要让荒谬的抛物线污染偏移。
 */
#define VISION_FIT_OUTLIER_PX   (25.0f)

/**
 * @brief 曲率物理上限（第一层防御）：|c| 的绝对值上限
 * @details c 在 t = ±1 处造成的横移就是 sagitta（弓高）。
 *          不加限制时，一段坏行或远端噪声就能让二次项爆到几十上百，
 *          屏幕上表现为"曲率非常大的抛物线"，而且会通过正规方程的耦合
 *          把 a（= x_ctrl）一起拽偏。
 *
 *          取值 60 px 的依据：一个"较强但真实"的弯（实测 c ≈ 34）对应
 *          sagitta ≈ 34 px，必须放行；而真正荒谬的解通常 > 100。
 *          取 60 能放行真实弯道，只拦下明显失控的解。
 *          注意：这是**最后一道保险**，不是主要手段 —— 主要手段是下面的门限筛行。
 */
#define VISION_FIT_MAX_SAGITTA_PX   (60.0f)

/* ========================================================================== */
/*                                  共享数据                                   */
/* ========================================================================== */

/**
 * @brief 原始图像帧缓冲，逐行指针
 * @note  由摄像头驱动（dcmi_ov2640.c）在启动 DMA 时把每一行指向
 *        Camera_Buffer 中的对应位置；视觉算法只读不改。
 */
extern uint16_t *mt9v03x_image[VISION_IMG_HEIGHT];

/**
 * @brief 新帧就绪标志
 * @note  由 DCMI 帧完成中断置 1，应用层在消费完一帧后清 0。
 *        这是驱动层与视觉层之间**唯一**的同步信号。
 */
extern volatile uint8_t DCMI_FrameState;

/* ========================================================================== */
/*                          道路中心线二次拟合（新增）                          */
/* ========================================================================== */

/**
 * @brief 道路中心线的二次拟合结果
 * @details 拟合模型在**归一化行坐标** t 上：
 *
 *              t   = (row − VISION_CONTROL_ROW) / (VISION_IMG_HEIGHT / 2)
 *                  ∈ [-1, +1]（控制行为 0，最近端 -1，最远端 +1）
 *              x(t) = a + b·t + c·t²
 *
 *          为什么用 t 而不是原始行号：直接拿 0~119 的 row 去解正规方程，
 *          Σ(r⁴) 会到 1e9 量级，和 Σ(w) 差 7 个数量级，float32 精度不够，
 *          解出来的 c 完全是噪声。归一化到 [-1,1] 后条件数良好。
 *
 *          为什么用二次而不是直线：直线无法表达弯道，在直角弯处会把
 *          两侧弯曲的中心线拉成一条斜线；二次项能贴合抛物线段，
 *          配合前视行求值就能提前看到弯。
 */
struct path_fit_s
{
    float   a, b, c;      /**< 二次多项式系数（t 域） */
    float   x_ctrl;       /**< 控制行(VISION_CONTROL_ROW)处的拟合中心列 */
    float   x_look;       /**< 前视行(VISION_LOOKAHEAD_ROW)处的拟合中心列 */
    float   slope_ctrl;   /**< 控制行处的斜率 dx/drow，单位 列/行 */
    uint8_t valid;        /**< 1 = 拟合有效；0 = 有效行不足，调用方应退回旧逻辑 */
    uint8_t rows_used;    /**< 最终实际参与拟合的有效行数 */
};
typedef struct path_fit_s path_fit_s;

/** @brief 中心线拟合结果实例（由 scan_line.c 的 Vision_FitPath() 维护） */
extern struct path_fit_s path_fit;

/** @brief 每行的路径中心列；无效行为 VISION_IMG_CX。由 Element_recognition.c 维护 */
extern int16_t path_center[VISION_IMG_HEIGHT];

/** @brief 每行路径是否有效（1 = 左右边线都找到）。由 Element_recognition.c 维护 */
extern uint8_t path_valid[VISION_IMG_HEIGHT];

/**
 * @brief  在任意行处求拟合曲线的列坐标
 * @param  row 算法行号（0 = 最近，119 = 最远）
 * @return 该行的拟合中心列（**未做 0~187 裁剪**，调用方自己夹紧）；
 *         拟合无效时返回 VISION_IMG_CX（图像中心），这样即使丢线也不会
 *         突然给一个离谱的偏差。
 * @note   内联放在头文件里，是为了让绘图层（Binarization.c）能直接画拟合
 *         曲线而**不必 include scan_line.h**，避免 L2 层内部互相依赖。
 */
static inline float Vision_EvalPathFit(float row)
{
    float t;

    if (path_fit.valid == 0U)
    {
        return (float)VISION_IMG_CX;
    }

    t = (row - (float)VISION_CONTROL_ROW) / ((float)VISION_IMG_HEIGHT * 0.5f);

    return path_fit.a + ((path_fit.b + (path_fit.c * t)) * t);
}

/** @brief 摄像头实际帧率（由驱动统计） */
extern volatile uint8_t OV2640_FPS;

/** @brief 二值化结果：0 表示黑（赛道），255 表示白（背景） */
extern uint8_t Grayscale[VISION_IMG_HEIGHT][VISION_IMG_WIDTH];

/** @brief 用于显示的图像：叠加了边线与中心标记 */
extern uint8_t imo[VISION_IMG_HEIGHT][VISION_IMG_WIDTH];

/* ========================================================================== */
/*                                  数据类型                                   */
/* ========================================================================== */

/**
 * @brief 单行赛道边线信息
 * @note  数组下标即行号 y（0 在图像底部），由 scan_line 模块填充。
 */
struct lineinfo_s
{
    int16_t  y;                                     /**< 行号，0 ~ VISION_IMG_HEIGHT-1 */
    int16_t  left;                                  /**< 本行左边界所在列 */
    int16_t  right;                                 /**< 本行右边界所在列 */
    int16_t  edge_count;                            /**< 本行跳变沿总数 */
    uint8_t  edge_store[VISION_EDGE_STORE_SIZE];    /**< 本行所有跳变沿的列号 */
    int16_t  left_adjust;                           /**< 补线后的左线 */
    int16_t  right_adjust;                          /**< 补线后的右线 */
    int      left_lost;                             /**< 左线丢失标志 */
    int      right_lost;                            /**< 右线丢失标志 */
    uint8_t  zebra_flag;                            /**< 斑马线标志 */
    int16_t  persp_lx;                              /**< 左线逆透视后的 X 坐标 */
    int16_t  persp_ly;                              /**< 左线逆透视后的 Y 坐标 */
    int16_t  persp_rx;                              /**< 右线逆透视后的 X 坐标 */
    int16_t  persp_ry;                              /**< 右线逆透视后的 Y 坐标 */
    int16_t  angel_left;                            /**< 左侧倾角 */
    int16_t  angel_right;                           /**< 右侧倾角 */
    int16_t  whole_lost;                            /**< 1 = 本行两侧线全丢 */
};
typedef struct lineinfo_s lineinfo_s;

/** @brief 全图逐行边线信息，下标为行号 */
extern struct lineinfo_s lineinfo[VISION_IMG_HEIGHT];

/**
 * @brief 整车视觉状态
 * @note  由视觉层维护，应用层可以读取（例如在屏幕上显示阈值），
 *        但**不要**由应用层直接写入算法内部字段。
 */
struct watch_o
{
    uint8_t threshold;      /**< 当前二值化阈值 */
    int     watch_line;     /**< 摄像头能看到的最远行 */
    int     watch_lost;     /**< 看丢的行数 */
    int     cross;          /**< 左右线同时丢失的行数 */
    int     left_lost;      /**< 左线丢失行数 */
    int     right_lost;     /**< 右线丢失行数 */
    int     left_near_lost; /**< 左线开始丢失的行号 */
    int     right_near_lost;/**< 右线开始丢失的行号 */
    int     ZebraInLine;    /**< 斑马线计数 */
};
typedef struct watch_o watch_o;

/** @brief 整车视觉状态实例 */
extern struct watch_o watch;

#ifdef __cplusplus
}
#endif

#endif /* VISION_TYPES_H */
