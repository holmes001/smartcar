/**
 * @file    dcmi_ov2640.h
 * @brief   OV2640 摄像头驱动（DCMI + DMA + SCCB）接口
 * @details 负责三件事：
 *            1. 通过 SCCB（类 I2C）配置 OV2640 的寄存器；
 *            2. 通过 DCMI + DMA 把图像连续搬运到 Camera_Buffer；
 *            3. 在帧完成中断里置位 DCMI_FrameState，通知上层取帧。
 *
 * @note    分层约束（重构后）
 *          本文件属于驱动层（L1）。重构前它 include 了 lcd_spi_200.h
 *          （LCD 驱动），也就是说"摄像头驱动依赖显示屏驱动"——这是分层违规：
 *          摄像头只负责产图，显示是 UI 层的事。现已去掉该依赖。
 *
 *          图像帧缓冲（mt9v03x_image / DCMI_FrameState / OV2640_FPS）的
 *          声明已移到 camera_process/Inc/vision_types.h，因为它们是
 *          **摄像头与视觉层之间的接口**，两边都应当看到同一份定义，
 *          而不应该让视觉层反过来去 include 摄像头驱动的头文件。
 */
#ifndef __DCMI_OV2640_H
#define __DCMI_OV2640_H

#include "main.h"
#include "sccb.h"
#include "dcmi.h"
#include "vision_types.h"   /* 帧缓冲与就绪标志的声明 */

#ifdef __cplusplus
extern "C" {
#endif

/* ========================================================================== */
/*                                  状态与配置                                 */
/* ========================================================================== */

/** @brief 通讯成功标志 */
#define  OV2640_Success   0
/** @brief 通讯错误标志 */
#define  OV2640_Error     -1

/** @brief 使能 */
#define  OV2640_Enable    1
/** @brief 禁止 */
#define  OV2640_Disable   0

/** @brief 输出格式：RGB565，被 OV2640_Set_Pixformat() 引用 */
#define Pixformat_RGB565   0
/** @brief 输出格式：JPEG，被 OV2640_Set_Pixformat() 引用 */
#define Pixformat_JPEG     1

/** @brief 特效：正常 */
#define  OV2640_Effect_Normal       0
/** @brief 特效：负片（颜色全部取反） */
#define  OV2640_Effect_Negative     1
/** @brief 特效：黑白 */
#define  OV2640_Effect_BW           2
/** @brief 特效：黑白 + 负片 */
#define  OV2640_Effect_BW_Negative  3

/*
 * 1. 定义 OV2640 实际输出的图像大小，可根据应用或显示屏调整
 *    （同时也要修改配置参数里的时钟分频）
 * 2. 这两个参数不影响帧率，且不能超过对应模式的最大尺寸
 * 3. SVGA 模式下输出图像最大 800*600，最大帧率 30 帧
 * 4. UXGA 模式下输出图像最大 1600*1200，最大帧率 15 帧
 * 5. 要设置的图像长、宽必须能被 4 整除！
 * 6. 长宽比必须满足 4:3，否则画面会被拉伸畸变
 */
/** @brief 传感器实际输出宽度（4:3 原始画面） */
#define OV2640_Width          192
/** @brief 传感器实际输出高度（4:3 原始画面） */
#define OV2640_Height         144

/*
 * 1. 定义要显示的画面大小，数值一定要能被 4 整除！
 * 2. RGB565 格式下，最终会由 DCMI 将 OV2640 输出的 4:3 图像裁剪为适应屏幕的比例
 * 3. 此处的分辨率不能超过 OV2640_Width 和 OV2640_Height
 * 4. 分辨率太高时需要修改 PCLK 时钟速度，详见 dcmi_ov2640_cfg.h 里 0xd3 寄存器
 *
 * 注意：这里不再重复写死 188/120，而是直接引用视觉层的图像几何常量，
 *       避免"驱动一套尺寸、算法一套尺寸"两边对不上。
 */
/** @brief 送去做视觉处理的图像宽度（列数） */
#define Display_Width         VISION_IMG_WIDTH
/** @brief 送去做视觉处理的图像高度（行数） */
#define Display_Height        VISION_IMG_HEIGHT

/** @brief 摄像头图像缓冲区基址（DCMI1 对应的 AXI SRAM 区域） */
#define Camera_Buffer         0x24000000

/*
 * 1. RGB565 模式下需要 图像分辨率*2 的大小
 * 2. JPG 模式下缓冲区大小并不固定，例如 640*480 分辨率 JPG 大概占 30K，
 *    预留 2 倍左右即可，用户可根据实际情况调整
 */
/** @brief DMA 传输数据大小（32 位宽） */
#define OV2640_BufferSize     Display_Width * Display_Height * 2 / 4

/* ========================================================================== */
/*                                OV2640 寄存器                                */
/* ========================================================================== */

/** @brief 寄存器组选择寄存器 */
#define  OV2640_SEL_Registers       0xFF
/** @brief 0x00 选择 DSP 寄存器组 */
#define  OV2640_SEL_DSP             0x00
/** @brief 0x01 选择 SENSOR 寄存器组 */
#define  OV2640_SEL_SENSOR          0x01

/* DSP 寄存器组 (0xFF = 0x00) */
/** @brief 复位控制器、SCCB 单元、JPEG 单元、DVP 接口单元等 */
#define  OV2640_DSP_RESET           0xE0
/** @brief 间接寄存器访问：地址 */
#define  OV2640_DSP_BPADDR          0x7C
/** @brief 间接寄存器访问：数据 */
#define  OV2640_DSP_BPDATA          0x7D

/* SENSOR 寄存器组 (0xFF = 0x01) */
/** @brief 公共控制：系统复位、分辨率选择、缩放模式、彩条设置 */
#define  OV2640_SENSOR_COM7         0x12
/** @brief 寄存器组 4，可设置扫描方向等 */
#define  OV2640_SENSOR_REG04        0x04
/** @brief 器件 ID 高字节 */
#define  OV2640_SENSOR_PIDH         0x0a
/** @brief 器件 ID 低字节 */
#define  OV2640_SENSOR_PIDL         0x0b

/* ========================================================================== */
/*                                  接口函数                                   */
/* ========================================================================== */

/**
 * @brief  初始化 SCCB、DCMI、DMA 并配置 OV2640
 * @return OV2640_Success 成功；OV2640_Error 失败
 */
int8_t   OV2640_Init(void);

/**
 * @brief  启动 DMA 连续传输
 * @param  DMA_Buffer      缓冲区基址
 * @param  DMA_BufferSize  传输数据量（32 位为单位）
 */
void     OV2640_DMA_Transmit_Continuous(uint32_t DMA_Buffer, uint32_t DMA_BufferSize);

/**
 * @brief  启动 DMA 快照传输（传完一帧就停）
 * @param  DMA_Buffer      缓冲区基址
 * @param  DMA_BufferSize  传输数据量（32 位为单位）
 */
void     OV2640_DMA_Transmit_Snapshot(uint32_t DMA_Buffer, uint32_t DMA_BufferSize);

/** @brief 挂起 DCMI，停止捕获数据 */
void     OV2640_DCMI_Suspend(void);

/** @brief 恢复 DCMI，开始捕获数据 */
void     OV2640_DCMI_Resume(void);

/** @brief 禁止 DCMI 的 DMA 请求，停止捕获并关闭外设 */
void     OV2640_DCMI_Stop(void);

/**
 * @brief  裁剪画面
 * @param  Displey_XSize 目标显示宽度
 * @param  Displey_YSize 目标显示高度
 * @param  Sensor_XSize  传感器输出宽度
 * @param  Sensor_YSize  传感器输出高度
 * @return 0 成功；非 0 失败
 */
int8_t   OV2640_DCMI_Crop(uint16_t Displey_XSize, uint16_t Displey_YSize,
                          uint16_t Sensor_XSize, uint16_t Sensor_YSize);

/** @brief 执行软件复位 */
void     OV2640_Reset(void);

/** @brief 读取器件 ID */
uint16_t OV2640_ReadID(void);

/**
 * @brief  批量写入寄存器配置表
 * @param  ConfigData 配置表，每项为 {寄存器, 值}
 */
void     OV2640_Config(const uint8_t (*ConfigData)[2]);

/** @brief 设置图像输出格式（Pixformat_RGB565 / Pixformat_JPEG） */
void     OV2640_Set_Pixformat(uint8_t pixformat);

/**
 * @brief  设置传感器实际输出的图像大小
 * @param  width  宽度
 * @param  height 高度
 * @return OV2640_Success 成功；OV2640_Error 失败
 */
int8_t   OV2640_Set_Framesize(uint16_t width, uint16_t height);

/**
 * @brief  设置是否水平镜像
 * @param  ConfigState OV2640_Enable / OV2640_Disable
 * @return OV2640_Success 成功；OV2640_Error 失败
 */
int8_t   OV2640_Set_Horizontal_Mirror(int8_t ConfigState);

/**
 * @brief  设置是否垂直翻转
 * @param  ConfigState OV2640_Enable / OV2640_Disable
 * @return OV2640_Success 成功；OV2640_Error 失败
 */
int8_t   OV2640_Set_Vertical_Flip(int8_t ConfigState);

/** @brief 设置饱和度 */
void     OV2640_Set_Saturation(int8_t Saturation);

/** @brief 设置亮度 */
void     OV2640_Set_Brightness(int8_t Brightness);

/** @brief 设置对比度 */
void     OV2640_Set_Contrast(int8_t Contrast);

/** @brief 设置特效（正常 / 负片 / 黑白 / 黑白+负片） */
void     OV2640_Set_Effect(uint8_t effect_Mode);

/* ========================================================================== */
/*                                  引脚配置                                   */
/* ========================================================================== */

/** @brief PWDN 引脚 */
#define OV2640_PWDN_PIN                 GPIO_PIN_14
/** @brief PWDN 端口 */
#define OV2640_PWDN_PORT                GPIOD
/** @brief PWDN 端口时钟使能 */
#define GPIO_OV2640_PWDN_CLK_ENABLE     __HAL_RCC_GPIOD_CLK_ENABLE()

/** @brief 低电平：退出掉电模式，摄像头正常工作 */
#define OV2640_PWDN_OFF   HAL_GPIO_WritePin(OV2640_PWDN_PORT, OV2640_PWDN_PIN, GPIO_PIN_RESET)

/** @brief 高电平：进入掉电模式，功耗最低 */
#define OV2640_PWDN_ON    HAL_GPIO_WritePin(OV2640_PWDN_PORT, OV2640_PWDN_PIN, GPIO_PIN_SET)

#ifdef __cplusplus
}
#endif

#endif /* __DCMI_OV2640_H */
