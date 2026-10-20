/**
 * @file    panel.h
 * @brief   人机面板接口：按键扫描 + LCD 显示
 * @details 把"输入"（3 个按键）和"输出"（LCD）集中在一个模块里，
 *          应用层只调用语义化的函数，不直接操作 GPIO 或 LCD 驱动。
 *
 * @note    分层约束
 *          本模块属于服务层（L2），向下可以依赖驱动层（lcd_spi_200.h）。
 *          它不认识控制算法，也不 include app.h。
 *          **它也不认识状态机** —— 状态机在 Base/app_fsm.c。
 *          本模块只做两件事：报告按键事件、把内容画到屏上。
 *
 * @note    硬件按键定义（按实际电路板修正）
 *          | 按键 | 引脚 | 功能 |
 *          |------|------|------|
 *          | PANEL_KEY_UP   | PD8  | 阈值增加 |
 *          | PANEL_KEY_DOWN | PD9  | 阈值减少 |
 *          | PANEL_KEY_MODE | PD10 | 模式切换（调节 <-> 运行） |
 *
 *          电路板上**没有 PE8**。原先的 PANEL_KEY_START 语义已升级为
 *          PANEL_KEY_MODE：它不再只是"启动"，而是"在调节与运行之间切换"。
 */
#ifndef PANEL_H
#define PANEL_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ========================================================================== */
/*                                  类型定义                                   */
/* ========================================================================== */

/**
 * @brief 面板按键编号
 * @note  与硬件引脚的对应关系见 panel.c 的 s_key_port / s_key_pin 表。
 */
typedef enum
{
    PANEL_KEY_UP = 0,   /**< PD8  阈值增加 */
    PANEL_KEY_DOWN,     /**< PD9  阈值减少 */
    PANEL_KEY_MODE,     /**< PD10 模式切换：调节态按下去开始运行，运行态按下去结束运行 */
    PANEL_KEY_COUNT     /**< 按键数量 */
} panel_key_t;

/** @brief 按键事件 */
typedef enum
{
    PANEL_KEY_EVENT_NONE = 0,   /**< 无事件 */
    PANEL_KEY_EVENT_PRESSED,    /**< 按下沿（本次采样刚刚按下） */
    PANEL_KEY_EVENT_RELEASED    /**< 松开沿 */
} panel_key_event_t;

/* ========================================================================== */
/*                                  接口函数                                   */
/* ========================================================================== */

/**
 * @brief  初始化面板
 * @details 记录各按键的初始电平，避免上电瞬间被误判为"按下"。
 * @note   必须在 MX_GPIO_Init() 与 LCD_Init() 之后调用。
 */
void Panel_Init(void);

/**
 * @brief  轮询一个按键（非阻塞）
 * @param  key 按键编号
 * @return 本次轮询产生的事件；无事件返回 PANEL_KEY_EVENT_NONE
 * @note   需要在主循环里以固定周期（建议 5~10 ms 一次）调用；
 *         函数内部用 HAL_GetTick() 做去抖，不含任何阻塞等待。
 */
panel_key_event_t Panel_PollKey(panel_key_t key);

/**
 * @brief  查询按键当前是否处于按下状态（去抖后）
 * @param  key 按键编号
 * @retval 1 按下；0 松开
 */
uint8_t Panel_IsKeyDown(panel_key_t key);

/**
 * @brief  在 LCD 上显示当前工作模式
 * @param  name 模式名称，ASCII 字符串（字库只支持 ASCII）
 * @note   显示在图像下方固定位置，供现场确认当前处于哪个状态。
 */
void Panel_ShowMode(const char *name);

/**
 * @brief  在 LCD 上显示两轮速度与编码器捕获值
 * @note   内部做了 100 ms 刷新节流，调用频率高于 10 Hz 也不会闪烁。
 *         显示位置沿用原来的布局（屏幕下方两行）。
 */
void Panel_ShowMotorSpeeds(void);

/**
 * @brief  在 LCD 左上角显示一帧视觉处理结果
 * @param  image 图像缓冲，逐行连续存放，尺寸为 VISION_IMG_WIDTH x VISION_IMG_HEIGHT
 */
void Panel_ShowVisionImage(const uint8_t *image);

/**
 * @brief  在 LCD 指定位置显示字符串
 * @param  x    起始列
 * @param  y    起始行
 * @param  text 以 '\0' 结尾的字符串（只读）
 */
void Panel_ShowString(uint16_t x, uint16_t y, const char *text);

/**
 * @brief  在 LCD 指定位置显示整数
 * @param  x     起始列
 * @param  y     起始行
 * @param  value 待显示的整数
 * @param  len   显示宽度，不足补空格
 */
void Panel_ShowNumber(uint16_t x, uint16_t y, int32_t value, uint8_t len);

/**
 * @brief  显示当前二值化阈值与偏移量，便于现场调参
 * @param  threshold 自动算出的阈值
 * @param  delta     人工叠加的偏移量
 */
void Panel_ShowThreshold(uint8_t threshold, int8_t delta);

#ifdef __cplusplus
}
#endif

#endif /* PANEL_H */
