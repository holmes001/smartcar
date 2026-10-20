/**
 * @file    telemetry.h
 * @brief   上位机波形显示（VOFA+）数据帧接口
 * @details 使用 VOFA+ 的 JustFloat 协议：把若干 float 按小端原样发出，
 *          最后附上固定的帧尾 0x00 0x00 0x80 0x7F，上位机据此切分通道。
 *
 * @note    分层约束
 *          本模块属于服务层（L2），只依赖串口驱动（Serial.h）。
 *          它不认识电机、摄像头或控制算法，接收什么就发什么。
 *
 *          原来这个函数叫 VOFA_SendJustFloat()，和电机速度控制、LCD 显示、
 *          按键扫描一起塞在 app.c 里。现在单独成文件，职责单一。
 */
#ifndef TELEMETRY_H
#define TELEMETRY_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/** @brief JustFloat 协议帧尾（对应 float 的 +inf 位模式） */
#define TELEMETRY_JUSTFLOAT_TAIL_SIZE   (4U)

/**
 * @brief  按 JustFloat 协议发送一组 float
 * @param  data  数据首地址（只读）
 * @param  count 通道数（float 个数）
 * @note   单帧总长度 = count*4 + 4 字节。上位机按顺序对齐到各个通道，
 *         因此**通道顺序一旦确定就不要随意调整**，否则上位机配置会错位。
 */
void Telemetry_SendJustFloat(const float *data, uint8_t count);

/**
 * @brief  发送单个通道（常用场景的简写）
 * @param  value 通道数值
 */
void Telemetry_SendScalar(float value);

#ifdef __cplusplus
}
#endif

#endif /* TELEMETRY_H */
