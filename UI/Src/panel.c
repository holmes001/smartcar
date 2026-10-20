/**
 * @file    panel.c
 * @brief   人机面板实现：按键去抖扫描 + LCD 显示
 *
 * @note    硬件按键（与 panel.h 的说明一致）
 *            PD8  阈值增加
 *            PD9  阈值减少
 *            PD10 模式切换
 *          三个按键都是"另一端接地、按下为低电平"，且都挂在 GPIOD 上。
 *
 * @note    本次修改
 *          去掉了 Panel_WaitForStart()：它依赖已经不存在的"启动键"概念，
 *          而且从未被调用（会被 --gc-sections 回收）。开机等待的功能
 *          已由状态机取代 —— 上电默认处于"调节"态，本来就不会动。
 */
#include "panel.h"

#include "main.h"
#include "encoder.h"        /* Encoder_GetSpeed / Encoder_GetPulse，以及 motor_id_t */
#include "lcd_spi_200.h"
#include "vision_types.h"   /* 图像尺寸常量 VISION_IMG_WIDTH / VISION_IMG_HEIGHT */

/* ========================================================================== */
/*                                内部常量与状态                                */
/* ========================================================================== */

/** @brief 按键去抖时间（毫秒）：电平需要稳定这么久才被认可 */
#define PANEL_DEBOUNCE_MS       (15U)

/** @brief LCD 刷新节流周期（毫秒） */
#define PANEL_LCD_REFRESH_MS    (100U)

/** @brief 各显示项的固定坐标（图像占 0~119 行，信息显示在其下方） */
#define PANEL_MODE_X            (0U)
#define PANEL_MODE_Y            (134U)
#define PANEL_MODE_TEXT_X       (48U)
#define PANEL_THRESHOLD_Y       (150U)
#define PANEL_SPEED_Y           (166U)
#define PANEL_CAPTURE_Y         (182U)
#define PANEL_COL_R_X           (160U)
#define PANEL_COL_VALUE_L_X     (64U)
#define PANEL_COL_VALUE_R_X     (224U)

/**
 * @brief 按键 -> GPIO 端口映射表
 * @note  索引与 panel_key_t 一致。三个按键都在 GPIOD 上。
 */
static GPIO_TypeDef *const s_key_port[PANEL_KEY_COUNT] =
{
    GPIOD,      /* PANEL_KEY_UP    PD8  */
    GPIOD,      /* PANEL_KEY_DOWN  PD9  */
    GPIOD       /* PANEL_KEY_MODE  PD10 */
};

/** @brief 按键 -> GPIO 引脚映射表，索引与 panel_key_t 一致 */
static const uint16_t s_key_pin[PANEL_KEY_COUNT] =
{
    GPIO_PIN_8,     /* PANEL_KEY_UP    */
    GPIO_PIN_9,     /* PANEL_KEY_DOWN  */
    GPIO_PIN_10     /* PANEL_KEY_MODE  */
};

/** @brief 上一次采样到的原始电平（1 = 按下） */
static uint8_t  s_key_raw[PANEL_KEY_COUNT];

/** @brief 去抖之后的稳定电平（1 = 按下） */
static uint8_t  s_key_stable[PANEL_KEY_COUNT];

/** @brief 原始电平发生变化的时间戳，用于计算去抖时间 */
static uint32_t s_key_change_tick[PANEL_KEY_COUNT];

/* ========================================================================== */
/*                                  内部函数                                   */
/* ========================================================================== */

/**
 * @brief  读取按键原始电平
 * @param  key 按键编号
 * @retval 1 按下（低电平有效）；0 松开
 * @note   硬件上按键另一端接地，所以读到 RESET 表示按下。
 */
static uint8_t Panel_ReadRaw(panel_key_t key)
{
    return (HAL_GPIO_ReadPin(s_key_port[key], s_key_pin[key]) == GPIO_PIN_RESET) ? 1U : 0U;
}

/**
 * @brief  判断按键编号是否合法
 * @param  key 待检查的编号
 * @retval 1 合法；0 非法
 */
static uint8_t Panel_IsValid(panel_key_t key)
{
    return ((uint32_t)key < (uint32_t)PANEL_KEY_COUNT) ? 1U : 0U;
}

/* ========================================================================== */
/*                                  接口实现                                   */
/* ========================================================================== */

void Panel_Init(void)
{
    uint32_t now = HAL_GetTick();

    for (uint32_t i = 0U; i < (uint32_t)PANEL_KEY_COUNT; i++)
    {
        uint8_t raw = Panel_ReadRaw((panel_key_t)i);

        s_key_raw[i]         = raw;
        s_key_stable[i]      = raw;
        s_key_change_tick[i] = now;
    }
}

panel_key_event_t Panel_PollKey(panel_key_t key)
{
    uint8_t  raw;
    uint32_t now;

    if (Panel_IsValid(key) == 0U)
    {
        return PANEL_KEY_EVENT_NONE;
    }

    raw = Panel_ReadRaw(key);
    now = HAL_GetTick();

    if (raw != s_key_raw[key])
    {
        /* 电平刚变化，重新开始计时，等它稳定下来再判定 */
        s_key_raw[key]         = raw;
        s_key_change_tick[key] = now;
        return PANEL_KEY_EVENT_NONE;
    }

    if (((now - s_key_change_tick[key]) >= PANEL_DEBOUNCE_MS) &&
        (raw != s_key_stable[key]))
    {
        s_key_stable[key] = raw;
        return (raw != 0U) ? PANEL_KEY_EVENT_PRESSED : PANEL_KEY_EVENT_RELEASED;
    }

    return PANEL_KEY_EVENT_NONE;
}

uint8_t Panel_IsKeyDown(panel_key_t key)
{
    if (Panel_IsValid(key) == 0U)
    {
        return 0U;
    }
    return s_key_stable[key];
}

void Panel_ShowMode(const char *name)
{
    if (name == 0)
    {
        return;
    }

    LCD_SetBackColor(LCD_BLACK);
    LCD_SetColor(LCD_GREEN);
    LCD_DisplayString(PANEL_MODE_X, PANEL_MODE_Y, "MODE:");
    /* LCD 驱动只读该字符串，这里做一次去 const 转换 */
    LCD_DisplayString(PANEL_MODE_TEXT_X, PANEL_MODE_Y, (char *)name);
}

void Panel_ShowMotorSpeeds(void)
{
    static uint32_t last_update = 0U;
    uint32_t        now         = HAL_GetTick();

    /* 刷新节流，避免刷屏拖慢控制周期 */
    if ((uint32_t)(now - last_update) < PANEL_LCD_REFRESH_MS)
    {
        return;
    }
    last_update = now;

    LCD_SetBackColor(LCD_BLACK);
    LCD_SetColor(LCD_YELLOW);
    LCD_DisplayString(0U, PANEL_SPEED_Y, "L mm/s:");
    LCD_DisplayNumber(PANEL_COL_VALUE_L_X, PANEL_SPEED_Y, (int32_t)Encoder_GetSpeed(MOTOR_L), 6U);
    LCD_DisplayString(PANEL_COL_R_X, PANEL_SPEED_Y, "R mm/s:");
    LCD_DisplayNumber(PANEL_COL_VALUE_R_X, PANEL_SPEED_Y, (int32_t)Encoder_GetSpeed(MOTOR_R), 6U);

    LCD_SetColor(LCD_CYAN);
    LCD_DisplayString(0U, PANEL_CAPTURE_Y, "L cap:");
    LCD_DisplayNumber(PANEL_COL_VALUE_L_X, PANEL_CAPTURE_Y, (int32_t)Encoder_GetPulse(MOTOR_L), 6U);
    LCD_DisplayString(PANEL_COL_R_X, PANEL_CAPTURE_Y, "R cap:");
    LCD_DisplayNumber(PANEL_COL_VALUE_R_X, PANEL_CAPTURE_Y, (int32_t)Encoder_GetPulse(MOTOR_R), 6U);
}

void Panel_ShowVisionImage(const uint8_t *image)
{
    if (image == 0)
    {
        return;
    }

    show_ov2640_image_int8(0U, 0U, image,
                           (uint16_t)VISION_IMG_WIDTH, (uint16_t)VISION_IMG_HEIGHT,
                           (uint16_t)VISION_IMG_WIDTH, (uint16_t)VISION_IMG_HEIGHT);
}

void Panel_ShowString(uint16_t x, uint16_t y, const char *text)
{
    if (text == 0)
    {
        return;
    }
    /* LCD 驱动只读该字符串，这里做一次去 const 转换 */
    LCD_DisplayString(x, y, (char *)text);
}

void Panel_ShowNumber(uint16_t x, uint16_t y, int32_t value, uint8_t len)
{
    LCD_DisplayNumber(x, y, value, len);
}

void Panel_ShowThreshold(uint8_t threshold, int8_t delta)
{
    LCD_SetBackColor(LCD_BLACK);
    LCD_SetColor(LCD_WHITE);
    LCD_DisplayString(0U, PANEL_THRESHOLD_Y, "thr:");
    LCD_DisplayNumber(40U, PANEL_THRESHOLD_Y, (int32_t)threshold, 3U);
    LCD_DisplayString(80U, PANEL_THRESHOLD_Y, "d:");
    LCD_DisplayNumber(104U, PANEL_THRESHOLD_Y, (int32_t)delta, 4U);
}
