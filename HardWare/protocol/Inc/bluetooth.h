/**
 * @file    bluetooth.h
 * @brief   JDY-33 蓝牙模块状态查询接口
 * @details JDY-33 通过串口与主控相连，本模块负责解析其 AT 指令回包并
 *          判断当前连接状态（未连接 / SPP 已连 / BLE 已连 / 已断开）。
 *
 * @note    分层约束
 *          本文件属于驱动层（L1），只依赖 stdint，不依赖应用层。
 *
 * @note    本次修改
 *          原文件里有一行
 *              static uint8_t ATCommandFeedBack_JDY33(uint8_t recv);
 *          在头文件中声明 static 函数是错的：每个包含该头文件的源文件都会
 *          得到一份"声明了但没定义"的私有函数，一旦被调用就会报
 *          "used but never defined"，不调用又会报 unused-function。
 *          该函数是 bluetooth.c 的内部实现细节，已从头文件移除。
 */
#ifndef BLUETOOTH_H
#define BLUETOOTH_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/** @brief JDY-33 连接状态 */
typedef enum
{
    JDY33_NO_STATE = 0,     /**< 未知 / 尚未检测 */
    JDY33_SPP_CONNECT,      /**< 经典蓝牙串口（SPP）已连接 */
    JDY33_BLE_CONNECT,      /**< BLE 已连接 */
    JDY33_DISCONNECT        /**< 已断开 */
} jdy33_state_t;

/**
 * @brief  检测并刷新蓝牙连接状态
 * @note   内部带超时等待，属于**阻塞**操作；不要在图像处理的实时路径里调用。
 */
void show_bt_state(void);

#ifdef __cplusplus
}
#endif

#endif /* BLUETOOTH_H */
