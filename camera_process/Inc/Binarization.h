/**
 * @file    Binarization.h
 * @brief   图像二值化与显示缓冲绘制接口
 * @details 视觉流水线的第一级：把摄像头送来的灰度帧转成二值图，
 *          并在处理结束后把二值图、边线、中心标记叠加到显示缓冲区。
 *
 * @note    分层约束
 *          本模块属于算法层（L3），只依赖 vision_types.h 中的数据定义，
 *          **不 include 任何 HardWare/ 头文件，也不调用电机 / 串口 / LCD**。
 */
#ifndef CODE_CAMERA_PROCESS_BINARIZATION_H_
#define CODE_CAMERA_PROCESS_BINARIZATION_H_

#include "vision_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief  用大津法（Otsu，最大类间方差）自动计算二值化阈值
 * @param  img   图像数据起始指针
 * @param  img_v 参与统计的行数
 * @param  img_h 参与统计的列数
 * @param  step  采样步长，越大越快但越粗糙（例如 10 表示每 10 行/列取一个点）
 * @return 计算出的阈值（0 ~ 255）
 * @note   为降低运算量，函数内部按 step 抽样统计灰度直方图。
 */
int Vision_OtsuThreshold(uint16_t *img, uint8_t img_v, uint8_t img_h, uint8_t step);

/**
 * @brief  对整帧图像做二值化，结果写入 Grayscale[][]
 * @param  delta 阈值偏移量，叠加在 watch.threshold 之上，用于现场微调；
 *               建议范围 -20 ~ +20
 * @note   参数类型是 **int8_t**（原实现是 uint8_t）。原来用无符号类型，
 *         现场按"减小阈值"键时会发生下溢（0 - 5 变成 251），阈值直接失控。
 */
void Vision_Binarize(int8_t delta);

/**
 * @brief  把二值图、扫线结果、中心线拟合曲线叠加到显示缓冲区 imo[][]
 * @details 绘制内容与配色（imo 值 → LCD 颜色由 show_ov2640_image_int8 映射）：
 *
 *            | 内容 | imo 值 | 颜色 | 粗细 |
 *            |------|--------|------|------|
 *            | 二值图底色 | 0 / 255 | 深灰 / 白 | — |
 *            | 左右道路边线 | 1 | **绿色** | 加粗 3px |
 *            | 道路中心线 | 2 | **红色** | 加粗 3px |
 *            | 二次拟合曲线 | 3 | **蓝色** | 加粗 3px |
 *            | 目标参考十字 | 2 | 红色 | 细 1px |
 *            | 主点十字 | 2 | 红色 | 加粗 + 臂长 7px |
 *
 *          主点 = 拟合曲线在控制行(VISION_CONTROL_ROW)的位置，
 *          也就是控制环当前认为"路在哪"。它和图像主点十字的距离就是横向偏差。
 *
 * @note   调用前必须已经执行过：
 *           - Vision_ScanLine()          （填 lineinfo[] 和 path_fit）
 *           - Vision_ComputePathCenter() （填 path_center[] / path_valid[]）
 * @note   本函数只操作内存，不直接刷屏 —— 刷屏由 UI 层负责。
 */
void Vision_DrawEdges(void);

#ifdef __cplusplus
}
#endif

#endif /* CODE_CAMERA_PROCESS_BINARIZATION_H_ */
