/**
 * @file    Serial.h
 * @brief   串口收发接口（含行缓冲）
 * @details 基于 USART2 的中断收发：
 *            - 接收：中断逐字节填入环形缓冲，ReadLine() 按行取出；
 *            - 发送：阻塞式逐字节发送（发送量小时足够用）。
 *
 * @note    分层约束
 *          本文件属于驱动层（L1），只依赖 HAL，不依赖应用层。
 *          需要格式化输出时用 Serial_Printf()，不要在应用层直接操作寄存器。
 *
 * @note    本次修改
 *            - 补上 `#include <stdint.h>`：原文件用了 uint8_t / uint16_t
 *              却没有包含该头文件，只是"碰巧"依赖调用方先包含过。
 *            - 入参加上 const 修饰：发送类函数不修改数据，加 const 后
 *              可以直接传字符串字面量，编译器也能帮忙发现误写。
 *            - 头文件保护宏由保留标识符 `__SERIAL_H` 改为 `SERIAL_H`
 *              （以下划线开头的标识符是实现保留的）。
 */
#ifndef SERIAL_H
#define SERIAL_H

#include <stdint.h>
#include <stdio.h>

#ifdef __cplusplus
extern "C" {
#endif

/** @brief 接收缓冲区大小（字节） */
#define SERIAL_RX_BUFFER_SIZE 256U

/** @brief 清空接收缓冲 */
void Serial_ClearRxBuffer(void);

/** @brief 启动接收中断 */
void Serial_StartReceive(void);

/**
 * @brief  发送一个字节
 * @param  Byte 待发送数据
 */
void Serial_SendByte(uint8_t Byte);

/**
 * @brief  发送一段数据
 * @param  Array  数据首地址（只读）
 * @param  Length 字节数
 */
void Serial_SendArray(const uint8_t *Array, uint16_t Length);

/**
 * @brief  发送一个以 '\0' 结尾的字符串
 * @param  String 字符串（只读）
 */
void Serial_SendString(const char *String);

/**
 * @brief  发送一个整数，按固定宽度补位
 * @param  Number 待发送的整数
 * @param  Length 显示宽度（不足补空格）
 */
void Serial_SendNumber(uint32_t Number, uint8_t Length);

/**
 * @brief  格式化发送（内部使用 vsnprintf）
 * @param  format 格式串（只读）
 */
void Serial_Printf(const char *format, ...);

/** @brief 查询是否收到一整行（以换行结尾） */
uint8_t Serial_GetRxFlag(void);

/** @brief 取出一行数据 */
uint8_t Serial_GetRxData(void);

/** @brief 查询当前接收缓冲中的字节数 */
uint16_t Serial_GetRxCount(void);

/** @brief 从接收缓冲取走一个字节 */
uint8_t Serial_ReadByte(void);

/**
 * @brief  按行读取，带超时
 * @param  out        输出缓冲
 * @param  max_len    缓冲长度
 * @param  timeout_ms 超时时间
 * @retval 1 成功读到一行；0 超时
 */
uint8_t ReadLine(char *out, uint8_t max_len, uint32_t timeout_ms);

#ifdef __cplusplus
}
#endif

#endif /* SERIAL_H */
