/**
 * @file    scan_line.h
 * @brief   赛道边线扫描接口
 * @details 视觉流水线的第二级：在二值图（Grayscale[][]）上从基准行出发，
 *          分别向上、向下逐行搜索左右边线，结果写入 lineinfo[]。
 *
 * @note    分层约束
 *          本模块属于算法层（L3），只依赖 vision_types.h。
 *          原来本文件 include 了 main.h（HAL），并且与 Binarization.h、
 *          Element_recognition.h 互相包含，现已全部切断。
 */
#ifndef CODE_CAMERA_PROCESS_SCAN_LINE_H_
#define CODE_CAMERA_PROCESS_SCAN_LINE_H_

#include "vision_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ========================================================================== */
/*                                内部辅助函数                                 */
/*                                                                            */
/*  下面这一组函数是扫描算法的内部步骤，仅供 scan_line.c 自己调用。            */
/*  之所以没有加 static，是因为它们需要跨函数传递中间数组；                    */
/*  但**外部模块不应调用**，后续如果要重构成文件内私有函数，只需在本文件里      */
/*  去掉声明、并在 scan_line.c 里给定义加 static 即可。                        */
/* ========================================================================== */

/**
 * @brief  从一行二值图中找出所有跳变沿
 * @param  inputimg   该行的像素数组
 * @param  edge_store 输出：跳变沿所在列号
 * @return 跳变沿数量
 */
uint8_t get_orign_edges(uint8_t *inputimg, uint8_t *edge_store);

/**
 * @brief  在候选跳变沿中挑出最合适的一对作为左右边线
 * @param  lineinfo      本行信息（输出）
 * @param  edge_store    候选跳变沿
 * @param  lineinfo_ref  参考行（上一行的结果），用于保持连续性
 * @return 0 成功；非 0 失败
 */
int get_best_edge(struct lineinfo_s *lineinfo, uint8_t *edge_store, struct lineinfo_s *lineinfo_ref);

/**
 * @brief  取相距最远的一对跳变沿作为左右边线
 * @param  lineinfo   本行信息（输出）
 * @param  edge_store 候选跳变沿
 * @return 0 成功；非 0 失败
 */
int get_max_edge(struct lineinfo_s *lineinfo, uint8_t *edge_store);

/**
 * @brief  基准行的边线搜索
 * @param  lineinfo 本行信息（输出）
 * @param  inputimg 该行像素数组
 * @return 0 成功；非 0 失败
 */
int line_single(struct lineinfo_s *lineinfo, unsigned char *inputimg);

/**
 * @brief  由参考行推导下一行的边线
 * @param  lineinfo     本行信息（输出）
 * @param  inputimg     该行像素数组
 * @param  lineinfo_ref 参考行
 * @return 0 成功；非 0 失败
 */
int line_findnext(struct lineinfo_s *lineinfo, uint8_t *inputimg, struct lineinfo_s *lineinfo_ref);

/**
 * @brief  斑马线检测
 * @param  lineinfo   本行信息（输出 zebra_flag）
 * @param  edge_store 候选跳变沿
 * @param  inputimg   该行像素数组
 * @return 1 检测到斑马线；0 没有
 */
int zebra_detect(struct lineinfo_s *lineinfo, uint8_t *edge_store, uint8_t *inputimg);

/* ========================================================================== */
/*                                  对外接口                                   */
/* ========================================================================== */

/**
 * @brief  扫描全图赛道边线
 * @details 从 VISION_BASE_LINE 行出发：
 *            - 向下逐行搜到 VISION_FORWARD_NEAR；
 *            - 向上逐行搜到 VISION_FORWARD_FAR。
 *          结果写入全局 lineinfo[]，同时刷新 watch.watch_line。
 * @note   调用前必须先完成二值化（Vision_Binarize），因为它读的是 Grayscale[][]。
 */
void Vision_ScanLine(void);

/**
 * @brief  对道路中心线做**加权最小二乘二次拟合** `x(t) = a + b·t + c·t²`
 * @details 数据来源：lineinfo[] 中左右边线都有效的行，取 `(left+right)/2`。
 *
 *          流程（两遍法，抗直角弯离群点）：
 *            1. 收集有效行，按到控制行的距离给权重 `w = 1/(1+0.6|t|)`
 *               —— 近处更可信，但远处仍保留 62.5% 权重，保证有弯道预判能力；
 *            2. 第一遍拟合**直线**（一阶），算每行残差，超过
 *               VISION_FIT_OUTLIER_PX(25px) 的行判为离群点（直角弯处垂直赛道、
 *               误检边线都在这一类），把它的权重清零；
 *            3. 用内点拟合**二次**曲线；内点不足或跨度太窄时自动退回直线；
 *            4. 填 path_fit：控制行/前视行的中心列、控制行斜率、拟合质量。
 *
 *          **求解内核**：两遍都不解正规方程，而是把 {1, t, t²} 先对加权离散
 *          内积正交化成 p0/p1/p2（三项递推），系数直接取内积投影
 *          `c_k = <x,p_k>/<p_k,p_k>`，正规方程因此对角化 —— 没有 3×3 求逆、
 *          没有 det 判奇异。`<p_k,p_k>` 同时就是"这一阶有没有数据支撑"的度量，
 *          跨度不足时会精确趋于 0，据此降阶而不是输出荒谬系数。
 *          数学推导与实测见 `Vision_FitPoly()`。
 *
 *          有效行数少于 VISION_FIT_MIN_ROWS(8) 时 `path_fit.valid = 0`。
 * @note   本函数由 Vision_ScanLine() 在末尾自动调用，一般不需要手工调。
 *         结果通过全局 `path_fit` 暴露，`Vision_EvalPathFit(row)` 求任意行。
 */
void Vision_FitPath(void);

/**
 * @brief  用离散正交多项式对一组加权样本做最小二乘拟合（**纯数学，无全局状态**）
 * @details 求 `min Σ w_i·(x_i − p(t_i))²`，`p` 为 1 或 2 次多项式。
 *
 *          与"直接解正规方程"是**同一个估计量**（最小二乘解唯一），区别只在
 *          数值实现：
 *
 *          | 项目     | 正规方程（旧实现）                   | 正交基投影（本函数）   |
 *          |----------|--------------------------------------|------------------------|
 *          | 未知量   | 幂基系数 a,b,c                       | 正交基系数 c0,c1,c2    |
 *          | 求解方式 | 3×3 Hankel 方程组，Cramer 展开       | 无方程组，逐个内积除法 |
 *          | 退化判断 | 依赖 det 的**绝对**阈值 1e-6（有量纲）| `<p_k,p_k>` **相对**门限 |
 *          | 跨度变窄 | det→0，系数爆掉                      | 范数→0，**自动降阶**   |
 *          | 扩展阶数 | 每加一阶要解更大的方程组             | 多跑一趟递推           |
 *
 *          内积定义为 `<f,g> = Σ w_i·f(t_i)·g(t_i)`，正交基由三项递推生成：
 *              p0 = 1
 *              p1 = (t − α1)·p0             α1 = <t·p0,p0>/<p0,p0>
 *              p2 = (t − α2)·p1 − β2·p0     α2 = <t·p1,p1>/<p1,p1>
 *                                           β2 = <p1,p1>/<p0,p0>
 *          系数 `c_k = <x,p_k>/<p_k,p_k>`，最后换回幂基：
 *              c = c2
 *              b = c1 − c2·(α1 + α2)
 *              a = c0 − c1·α1 + c2·(α1·α2 − β2)
 *
 * @param  t     归一化行坐标样本（长度 n）
 * @param  x     该行的中心列样本
 * @param  w     权重样本；`w <= 0` 的样本被视为剔除（不参与，也不计入 n_used）
 * @param  n     样本数（<= 255）
 * @param  order 期望阶数：1 = 直线，2 = 二次（给 0 也按 1 处理）
 * @param  out   输出结果；`out->order` 是**实际**支撑住的阶数，可能低于期望
 * @retval 1 成功（至少能定出常数项和斜率）；0 样本全被剔除或全部落在同一 t 上
 * @note   本函数不读 lineinfo / Grayscale，也不写全局，因此可以在 PC 上用
 *         已知系数造数据直接验证（见 tests/test_path_fit.c）。
 */
uint8_t Vision_FitPoly(const float *t, const float *x, const float *w,
                       uint8_t n, uint8_t order, struct path_poly_s *out);

/**
 * @brief  在任意行处求拟合曲线的列坐标
 * @param  row 算法行号（0 = 最近，119 = 最远）
 * @return 拟合中心列（未裁剪）；拟合无效时返回 VISION_IMG_CX
 */
float Vision_GetFitX(float row);

#ifdef __cplusplus
}
#endif

#endif /* CODE_CAMERA_PROCESS_SCAN_LINE_H_ */
