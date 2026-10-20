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
 *            2. 第一遍解 3×3 正规方程；
 *            3. 用解算出的曲线算每行残差，超过 VISION_FIT_OUTLIER_PX(12px)
 *               的行判为离群点（直角弯处垂直赛道、误检边线都在这一类），
 *               把它的权重清零；
 *            4. 第二遍重新解，得到最终系数；
 *            5. 填 path_fit：控制行/前视行的中心列、控制行斜率。
 *
 *          有效行数少于 VISION_FIT_MIN_ROWS(8) 时 `path_fit.valid = 0`。
 * @note   本函数由 Vision_ScanLine() 在末尾自动调用，一般不需要手工调。
 *         结果通过全局 `path_fit` 暴露，`Vision_EvalPathFit(row)` 求任意行。
 */
void Vision_FitPath(void);

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
