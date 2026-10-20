/**
 * @file    Element_recognition.h
 * @brief   路径中心计算与横向偏差输出
 * @details 视觉流水线的第三级，也是视觉层**唯一**对外输出结果的地方：
 *          把逐行的左右边线换算成"路径中心"，再取控制行上的偏差交给控制层。
 *
 * @note    分层约束（本文件是这次重构改动最大的地方之一）
 *          原来的版本里，本模块：
 *            - include 了 app.h（应用层），形成**反向依赖**；
 *            - 直接调用 Set_Motor_Speed() / Speed_Control_PID() 去开电机；
 *            - 自己持有 PID 实例和转向控制律。
 *          也就是说"看图"和"开车"混在了一个文件里，任何一边改动都会牵动另一边。
 *
 *          现在这些全部搬走：控制律移到 Control/chassis.c，
 *          本模块只做纯计算并返回结果，返回值由应用层交给控制层。
 */
#ifndef CODE_CAMERA_PROCESS_ELEMENT_RECOGNITION_H_
#define CODE_CAMERA_PROCESS_ELEMENT_RECOGNITION_H_

#include "vision_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief  计算每一行的路径中心
 * @details 对每一行：
 *            - 若该行任意一侧丢线，则标记该行无效，中心取图像中心作为占位；
 *            - 否则取左右边线的中点。
 *          结果写入全局 `path_center[]` / `path_valid[]`
 *          （原来它们是本模块的静态数组，现在改为全局，因为绘图层
 *          `Binarization.c` 要把道路中心线画出来，需要读这两个数组）。
 * @note   调用前必须先执行 Vision_ScanLine()。
 *         本函数**只服务于绘图**；控制用的偏差走的是 `path_fit`
 *         （由 Vision_ScanLine() 内部调用 Vision_FitPath() 生成）。
 */
void Vision_ComputePathCenter(void);

/**
 * @brief  取控制行处的横向偏差（像素）
 * @return 偏差 = `path_fit.x_ctrl` − 图像主点列。
 *         正值表示路径在图像右侧（车偏左），负值反之；
 *         拟合无效时返回 0。量程约 -94 ~ +94 像素。
 * @note   现在的值来自**整条中心线的二次拟合**在控制行处的外推，
 *         不是单纯读第 60 行 —— 该行失效时依然有输出。
 */
int16_t Vision_GetLateralError(void);

/**
 * @brief  取控制行处的横向偏差（**归一化**）
 * @return 偏差 / VISION_IMG_CX，量程 -1.0 ~ +1.0；拟合无效时返回 0。
 * @details 归一化的意义：把"像素"换算成"相对图像半宽的比值"，于是
 *            1.0  ==  路径偏到图像边缘（极限）
 *            0.5  ==  偏了半幅
 *          这样控制环的增益就有了**可直接读出的物理含义**：
 *            角速度环 yaw_kp = 5.2  =>  满偏时要求 5.2 rad/s 的转向
 *          而不是"5.0 (mm/s) per 像素"那种看不出力度的数字。
 *
 *          另一个好处：与图像分辨率解耦。以后换摄像头、改 Display_Width，
 *          控制增益都不用重调。
 * @note   归一化放在视觉层而不是控制层，是因为"图像半宽"是视觉层自己的
 *         几何参数；这样控制层不必 include 视觉层的头文件。
 */
float Vision_GetLateralErrorNorm(void);

/**
 * @brief  取**前视行**（VISION_LOOKAHEAD_ROW，比控制行更远）的归一化偏差
 * @return 偏差 / VISION_IMG_CX，量程 -1.0 ~ +1.0；拟合无效时返回 0。
 * @details 用途：提前转向。控制行看的是"现在偏多少"，前视行看的是
 *          "再往前一点会偏多少"，在进弯前就能给出转向指令。
 *
 * @warning 它的量程虽然同样是 ±1，但因为看的位置更远，
 *          对同一个弯给出的值**比控制行更大**。直接把
 *          Vision_GetLateralErrorNorm() 换成它，等效增益会上升，
 *          必须同步下调 yaw_kp，否则会振荡。
 * @note    当前 app.c 用的是控制行版本，本接口留作实验用。
 */
float Vision_GetLateralErrorLookaheadNorm(void);

/**
 * @brief  查询道路是否有效
 * @retval 1 有效（拟合成功，即至少 VISION_FIT_MIN_ROWS 行可用）；0 无路
 * @note   ★ 语义已变：以前判的是"第 60 行是否有效"，
 *         现在判的是"整条中心线的二次拟合是否成立"。
 *         好处是第 60 行单独失效不再导致丢线，判定也不会在直角弯处闪烁。
 *         应用层仍然按 0 -> 停车 处理，行为接口不变。
 */
uint8_t Vision_IsPathValid(void);

#ifdef __cplusplus
}
#endif

#endif /* CODE_CAMERA_PROCESS_ELEMENT_RECOGNITION_H_ */
