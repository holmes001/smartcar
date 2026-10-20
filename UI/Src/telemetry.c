/**
 * @file    telemetry.c
 * @brief   VOFA+ JustFloat 数据帧实现
 */
#include "telemetry.h"

#include "Serial.h"

/* ========================================================================== */
/*                                  接口实现                                   */
/* ========================================================================== */

void Telemetry_SendJustFloat(const float *data, uint8_t count)
{
    const uint8_t  tail[TELEMETRY_JUSTFLOAT_TAIL_SIZE] = { 0x00U, 0x00U, 0x80U, 0x7FU };
    const uint8_t *raw;
    uint16_t       total;
    uint16_t       i;

    if ((data == 0) || (count == 0U))
    {
        return;
    }

    /* 逐字节发出，保持小端字节序，与 VOFA+ 的解析方式一致 */
    raw   = (const uint8_t *)data;
    total = (uint16_t)((uint16_t)count * 4U);

    for (i = 0U; i < total; i++)
    {
        Serial_SendByte(raw[i]);
    }

    for (i = 0U; i < TELEMETRY_JUSTFLOAT_TAIL_SIZE; i++)
    {
        Serial_SendByte(tail[i]);
    }
}

void Telemetry_SendScalar(float value)
{
    Telemetry_SendJustFloat(&value, 1U);
}
