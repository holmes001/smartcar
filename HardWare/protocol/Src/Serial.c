/**
 * @file    Serial.c
 * @brief   串口收发实现（USART2）
 * @details 发送：阻塞式逐字节发送，数据量小时足够。
 *          接收：中断逐字节填入 256 字节环形缓冲，可由 ReadLine() 按行取用。
 *
 * @note    本次修改
 *            - 发送类函数的指针参数加 const，允许直接传字符串字面量；
 *            - Serial_Printf() 的 vsprintf 换成 vsnprintf（见该函数注释）；
 *            - Serial_Pow / Serial_RxData / Serial_RxFlag 收为 static，
 *              减少对外暴露的全局符号。
 */
#include "main.h"
#include "Serial.h"
#include <stdio.h>
#include <stdarg.h>
#include <stdint.h>

extern UART_HandleTypeDef huart2;

/** @brief 接收环形缓冲 */
static volatile uint8_t Serial_RxBuffer[SERIAL_RX_BUFFER_SIZE];
/** @brief 环形缓冲写指针（中断中更新） */
static volatile uint16_t Serial_RxHead = 0U;
/** @brief 环形缓冲读指针 */
static volatile uint16_t Serial_RxTail = 0U;

/** @brief 接收中断当前的暂存字节 */
static uint8_t Serial_RxData;
/** @brief 收到新字节的标志 */
static uint8_t Serial_RxFlag;

void Serial_ClearRxBuffer(void)
{
    Serial_RxHead = 0U;
    Serial_RxTail = 0U;
}

//发送部分

void Serial_SendByte(uint8_t Byte)
{
    HAL_UART_Transmit(&huart2, &Byte, 1, HAL_MAX_DELAY);
}

/**
 * @brief  发送一段数据
 * @param  Array  数据首地址（只读）
 * @param  Length 字节数
 */
void Serial_SendArray(const uint8_t *Array, uint16_t Length)
{
    uint16_t i;
    for (i = 0; i < Length; i ++)
    {
        Serial_SendByte(Array[i]);
    }
}

/**
 * @brief  发送一个以 '\0' 结尾的字符串
 * @param  String 字符串（只读）
 */
void Serial_SendString(const char *String)
{
    uint8_t i;
    for (i = 0; String[i] != '\0'; i ++)
    {
        Serial_SendByte(String[i]);
    }
}

/**
 * @brief  计算 X 的 Y 次幂
 * @param  X 底数
 * @param  Y 指数
 * @return X^Y
 * @note   仅供 Serial_SendNumber() 做十进制拆位使用，故不对外暴露。
 */
static uint32_t Serial_Pow(uint32_t X, uint32_t Y)
{
    uint32_t Result = 1;
    while (Y --)
    {
        Result *= X;
    }
    return Result;
}

void Serial_SendNumber(uint32_t Number, uint8_t Length)
{
    uint8_t i;
    for (i = 0; i < Length; i ++)
    {
        Serial_SendByte(Number / Serial_Pow(10, Length - i - 1) % 10 + '0');
    }
}

/**
 * @brief  重定向 C 库的字符输出，使 printf() 走串口
 * @param  ch 待输出字符
 * @param  f  文件流（未使用）
 * @return 输出的字符
 */
int fputc(int ch, FILE *f)
{
    uint8_t data = (uint8_t)ch;
    HAL_UART_Transmit(&huart2, &data, 1, HAL_MAX_DELAY);
    return ch;
}

/**
 * @brief  格式化发送
 * @param  format 格式串（只读）
 * @note   原来是 vsprintf(String, format, arg)，写进 100 字节的栈数组却
 *         没有任何长度限制 —— 一旦格式化结果超过 100 字节就会踩坏栈
 *         （OV2640 初始化失败那条打印就是格式化字符串，属于可达路径）。
 *         现改为 vsnprintf，最多写 sizeof(String)-1 字节并保证结尾 '\0'。
 */
void Serial_Printf(const char *format, ...)
{
    char String[100];
    va_list arg;
    va_start(arg, format);
    (void)vsnprintf(String, sizeof(String), format, arg);
    va_end(arg);
    Serial_SendString(String);
}

//接收部分
void Serial_StartReceive(void)
{
    HAL_UART_Receive_IT(&huart2, &Serial_RxData, 1);
}

uint16_t Serial_GetRxCount(void)
{
    if (Serial_RxHead >= Serial_RxTail)
    {
        return (uint16_t)(Serial_RxHead - Serial_RxTail);
    }
    return (uint16_t)(SERIAL_RX_BUFFER_SIZE - Serial_RxTail + Serial_RxHead);
}
uint8_t Serial_GetRxFlag(void)
{
    if (Serial_RxFlag == 1)
    {
        Serial_RxFlag = 0;
        return 1;
    }
    return 0;
}
uint8_t Serial_GetRxData(void)
{
    return Serial_RxData;
}
uint8_t Serial_ReadByte(void)
{
    uint8_t data = 0U;
    if (Serial_GetRxCount() != 0U)
    {
        data = Serial_RxBuffer[Serial_RxTail];
        Serial_RxTail = (Serial_RxTail + 1U) % SERIAL_RX_BUFFER_SIZE;
    }
    return data;
}
uint8_t ReadLine(char *out, uint8_t max_len, uint32_t timeout_ms)
{
  uint8_t idx = 0U;
  uint32_t start = HAL_GetTick();
  char ch = '\0';

  while ((HAL_GetTick() - start) < timeout_ms)
  {
    while (Serial_GetRxCount() > 0U)
    {
      ch = (char)Serial_ReadByte();
      if (ch == '\n' || ch == '\r')
      {
        if (idx > 0U)
        {
          out[idx] = '\0';
          return 1U;
        }
      }
      else if (idx < max_len - 1U)
      {
        out[idx++] = ch;
      }
    }
  }
  return 0U;
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart != &huart2)
    {
        return;
    }

    Serial_RxBuffer[Serial_RxHead] = Serial_RxData;
    Serial_RxHead = (Serial_RxHead + 1U) % SERIAL_RX_BUFFER_SIZE;
    if (Serial_RxHead == Serial_RxTail)
    {
        Serial_RxTail = (Serial_RxTail + 1U) % SERIAL_RX_BUFFER_SIZE;
    }
    Serial_RxFlag = 1U;
    HAL_UART_Receive_IT(&huart2, &Serial_RxData, 1U);
}
