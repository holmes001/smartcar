# STM32H750 循迹车 · 代码 API 手册

| 项目 | 内容 |
|---|---|
| 适用工程 | `stm32_HAL/code_v3.1`（由 `code_v2.1` 更名而来） |
| 目标芯片 | STM32H750VBT6（LQFP100，128 KB Flash，1 MB RAM） |
| 核心板 | 反客 FK750M1-VBT6 |
| 框架 | STM32Cube HAL + CMake + `arm-none-eabi-gcc` |
| 调试器 | ST-Link V2（`OpenOCD`，SWD） |
| 源文件数 | 68 个 `.c` / `.h`（不含 `Drivers/`） |
| 代码规模 | 应用相关约 12 900 行 |
| 固件占用 | **FLASH 88 856 B / 128 KB (67.79%)**，**DTCMRAM 58 904 B / 128 KB (44.94%)** |

> 本文档描述的是**当前代码的真实状态**，包括仍在使用的旧路径（占空比档）与新建路径（串级档）并存的情况。

---

# 1. 架构总览

## 1.1 五层结构

```
┌───────────────────────────────────────────────────────────────────────────┐
│ L4  应用层  Base/                                                          │
│       app.c        —— 初始化编排 / 主循环 / 按键→事件→动作                 │
│       app_fsm.c    —— 表驱动两态状态机（ADJUST / RUN）                     │
│       type_def.h   —— 移植代码的整型别名                                   │
│                    职责：**知道一切，但只做编排，不做算法**                 │
├───────────────────────────────────────────────────────────────────────────┤
│ L3  服务层  UI/                                                            │
│       panel.c      —— LCD 显示 + 按键扫描（人机接口）                      │
│       telemetry.c  —— VOFA+ JustFloat 帧封装（上位机通信）                 │
│                    职责：面向"人和上位机"的服务，不含控制逻辑              │
├───────────────────────────────────────────────────────────────────────────┤
│ L2  算法层  camera_process/  +  Control/                                   │
│       vision_types.h        —— 视觉全局类型与常量                          │
│       Binarization.c        —— 大津法阈值 / 二值化 / 边线可视化            │
│       scan_line.c           —— 逐行扫线（边线搜索、补线、斑马线）          │
│       Element_recognition.c —— 路径中心与横向偏差                        │
│       inner_parameter.c     —— 内参（当前为空壳）                          │
│       pid.c                 —— 通用位置式 PID                              │
│       chassis.c             —— 底盘运动学 + 三级串级控制                   │
│                    职责：**纯算法，可脱离硬件在 PC 上验证**                │
├───────────────────────────────────────────────────────────────────────────┤
│ L1  驱动层  HardWare/                                                      │
│       motor/      motor.c  encoder.c     —— 电机 PWM / 编码器测速          │
│       LCD/        lcd_spi_200.c lcd_fonts.c lcd_image.c                    │
│       OV2640/     sccb.c dcmi_ov2640.c dcmi_ov2640_cfg.h                   │
│       protocol/   Serial.c bluetooth.c                                     │
│       Buzzer/     buzzer.c                                                │
│                    职责：把"器件"抽象成函数，认识 HAL，不认识控制/应用      │
├───────────────────────────────────────────────────────────────────────────┤
│ L0  硬件层  Core/  +  Drivers/                                             │
│       main.c  tim.c  gpio.c  usart.c  spi.c  dcmi.c  dma.c  i2c.c         │
│       stm32h7xx_it.c  stm32h7xx_hal_msp.c  syscalls.c  sysmem.c           │
│       Drivers/STM32H7xx_HAL_Driver  +  Drivers/CMSIS                      │
│                    职责：CubeMX 生成，**原则上不允许手改**（只写 USER CODE）│
└───────────────────────────────────────────────────────────────────────────┘
```

## 1.2 分层规则

| 规则 | 说明 | 违反的后果 |
|---|---|---|
| **只能向上依赖** | L4→L3→L2→L1→L0，反向禁止 | 循环依赖、无法单独测试 |
| **算法层不 include HAL** | `camera_process/` 全部文件不含任何 `stm32*` 头 | 无法在 PC 上验证 |
| **驱动层不认识控制/应用** | 驱动只 `#include` 自己的头 + `main.h` + 对应 `Core/` 头 | 改控制层要重编驱动 |
| **同层不互相调用** | 需要跨模块协作时由 L4（`app.c`）编排 | 隐式耦合 |
| **L0 只写 `USER CODE`** | CubeMX 重新生成会覆盖其它部分 | 改动丢失 |

### 已存在的两处例外（已知、可接受，但要注意）

1. **`dcmi_ov2640.h` → `vision_types.h`（L1 依赖 L2）**
   - 原因：`Camera_Buffer` 的地址和 `Display_Width/Height` 需要与视觉层的图像尺寸一致，头文件里直接复用了 `VISION_IMG_WIDTH/HEIGHT`。
   - 建议：如果以后要换分辨率，这个反向依赖是最容易出错的地方。

2. **`app.c` 直接 `#include` 了 17 个下层头文件**
   - 这是应用层"编排者"的正常职责，不算越界，但也意味着 `app.c` 是最容易变成"上帝文件"的地方。

## 1.3 全局数据流

```
 OV2640 ──DCMI+DMA2_Stream7──► Camera_Buffer (0x24000000, 45120 B)
                                      │
                                      ▼
                       Vision_OtsuThreshold()  ──► threshold
                                      │
                       Vision_Binarize(delta)  ──► Grayscale[] / imo[][]
                                      │
                       Vision_ScanLine()       ──► lineinfo[120]
                                      │
                       Vision_ComputePathCenter() ──► path_center[120] / path_valid[120]
                                      │
                    Vision_IsPathValid()  ──┬── 否 ──► Chassis_Stop()
                    Vision_GetLateralErrorNorm()        (硬停，duty=0 滑行)
                                      │ 是
                                      ▼
              ┌─────────────── 外环 (30 Hz，帧率驱动) ───────────────┐
              │  Chassis_UpdateOuter(e_norm, dt)                     │
              │    ① 偏航 PID      e_norm ──► ω_ref [rad/s]          │
              │    ② 运动学        ω_ref × 90mm ──► Δv [mm/s]        │
              │    ③ 差速分配      base ± Δv ──► s_wheel_target[L/R] │
              └───────────────────────────┬──────────────────────────┘
                                          │ (写全局目标，浮点 32bit 原子)
              ┌───────────────────────────▼──────────────────────────┐
              │  Chassis_UpdateInner(dt)   500 Hz，TIM6 ISR 分频触发 │
              │    ④ 轮速 PID  目标 − 编码器实测 ──► 占空比 [%]      │
              │                  ──► Motor_SetDuty()                 │
              └──────────────────────────────────────────────────────┘
                                          │
                                     TB6612 ×2 ──► 左右直流电机
                                          ▲
                       编码器 ──TIM4/TIM5 输入捕获──► Encoder_Update() 10 kHz
```

---

# 2. 硬件资源占用

## 2.1 MCU 与存储器

| 项目 | 规格 |
|---|---|
| 型号 | STM32H750VBT6 |
| 内核 | Cortex-M7 @ 480 MHz，FPU (double precision)，16 KB D-Cache / 16 KB I-Cache |
| 片内 Flash | **128 KB 单扇区**（`0x08000000`，擦除即整片擦除） |
| DTCM | 128 KB @ `0x20000000`（主用 RAM，放栈/堆/全局变量） |
| AXI SRAM (D1) | 512 KB @ `0x24000000`（摄像头帧缓冲固定占前 45120 B） |
| SRAM D2/D3 | 288 KB / 64 KB（未使用） |
| ITCM | 64 KB @ `0x00000000`（未使用） |

## 2.2 时钟树

| 节点 | 配置 | 结果频率 |
|---|---|---|
| HSE | 外部晶振 | 25 MHz |
| PLL1 | M=5，N=192，P=2，Q=40，R=2，VCIRANGE_2，VCOWIDE | VCO = 960 MHz |
| SYSCLK | `PLL1P` | **480 MHz** |
| AHB (HCLK) | `RCC_HCLK_DIV2` | **240 MHz** |
| APB1 | `RCC_APB1_DIV2` | 120 MHz（定时器时钟 **240 MHz**） |
| APB2 | `RCC_APB2_DIV2` | 120 MHz（定时器时钟 **240 MHz**） |

## 2.3 定时器分配

| 定时器 | 用途 | PSC | ARR | 实际频率 | 配置要点 |
|---|---|---|---|---|---|
| **TIM4** | 编码器 1（左）输入捕获 | 23 | 65535 | 计数 **10 MHz** | 从模式复位，CH2 = PD13 |
| **TIM5** | 编码器 2（右）输入捕获 | 23 | 65535 | 计数 **10 MHz** | 从模式复位，CH2 = PA1 |
| **TIM6** | 系统 10 kHz 时基 | 239 | 99 | **10 kHz** | 中断里跑 `Encoder_Update()` + 分频出 500 Hz 内环节拍 |
| **TIM12** | 电机 A PWM（PB14/PB15） | 48 | **2000** | **2.45 kHz** | 2 通道，占空比分辨率 **0.05 %** |
| **TIM15** | 电机 B PWM（PA2/PA3） | 48 | **2000** | **2.45 kHz** | 2 通道，占空比分辨率 **0.05 %** |
| **TIM14** | 蜂鸣器 PWM（PA7） | 4800 | 100 | **≈495 Hz** | 单通道，音量调占空比 |

> **TIM12/TIM15 的 ARR 从 100 改成 2000 是本工程最关键的一次硬件改动**：
> 占空比分辨率从 1 % 提升到 0.05 %，偏航角速度的最小台阶从 27.3 °/s 降到 1.36 °/s。
> 代价：PWM 频率从 48.9 kHz 降到 2.45 kHz（进入音频范围，可能有可听啸叫）。

### TIM6 的两个职责

```
TIM6 中断 (10 kHz)
├── Encoder_Update()                       ← 每 10 kHz
└── 分频计数 / 20 ──► 上层注册的回调        ← 500 Hz = CHASSIS_INNER_LOOP_HZ
```

## 2.4 GPIO 分配

| 引脚 | 信号名 | 方向/模式 | 用途 |
|---|---|---|---|
| PA0 | `Encoder2_DIR` | 输入 | 右轮编码器方向 |
| PA1 | `TIM5_CH2` | 复用输入 | 右轮编码器脉冲 |
| PA2 / PA3 | `TIM15_CH1/CH2` | 复用推挽 | 电机 B 正/反转 PWM |
| PA4 | `DCMI_HSYNC` | 复用 | 摄像头行同步 |
| PA6 | `DCMI_PIXCLK` | 复用 | 摄像头像素时钟 |
| PA7 | `TIM14_CH1` | 复用推挽 | 蜂鸣器 |
| PB7 | `DCMI_VSYNC` | 复用 | 摄像头场同步 |
| PB8 / PB9 | `SCCB_SCL/SDA` | **开漏 + 上拉** | 摄像头寄存器配置（软件 I²C） |
| PB10 / PB11 | `I2C2_SCL/SDA` | 复用开漏 | I²C2（已初始化，当前未使用） |
| PB14 / PB15 | `TIM12_CH1/CH2` | 复用推挽 | 电机 A 正/反转 PWM |
| PC6 / PC7 | `DCMI_D0/D1` | 复用 | 摄像头数据 |
| PD3 | `DCMI_D5` | 复用 | 摄像头数据 |
| PD5 / PD6 | `USART2_TX/RX` | 复用 | 串口（VOFA+ / 调试） |
| **PD8** | 按键 UP | 输入 | 调节态：阈值 +；运行态：右轮补偿 + |
| **PD9** | 按键 DOWN | 输入 | 调节态：阈值 −；运行态：右轮补偿 − |
| **PD10** | 按键 MODE | 输入 | 运行/停止切换 |
| PD12 | `Encoder1_DIR` | 输入 | 左轮编码器方向 |
| PD13 | `TIM4_CH2` | 复用输入 | 左轮编码器脉冲 |
| PD14 | `CAMREA_PWDN` | 输出 | 摄像头掉电控制 |
| PD15 | `LCD_BL` | 输出 | LCD 背光 |
| PE0 / PE1 | `DCMI_D2/D3` | 复用 | 摄像头数据 |
| PE4 / PE5 / PE6 | `DCMI_D4/D6/D7` | 复用 | 摄像头数据 |
| PE11 | `LCD_CS` (SPI4_NSS) | 复用推挽 | LCD 片选 |
| PE12 | `LCD_SCL` (SPI4_SCK) | 复用推挽 | LCD 时钟 |
| PE14 | `LCD_SDA` (SPI4_MOSI) | 复用推挽 | LCD 数据 |
| PE15 | `LCD_DC` | 输出 | LCD 数据/命令选择 |

> `GPIOD` 的 `PIN_8/9/10` 在 `gpio.c` 里统一配置为 **输入 + 上拉**（三个按键，低电平有效）。
> `GPIOB` 的 `PIN_8/9` 为**开漏输出 + 上拉**（SCCB 软件 I²C）。

## 2.5 外设与 DMA

| 外设 | 句柄 | 关键参数 | 用途 |
|---|---|---|---|
| **USART2** | `huart2` | 115200-8-N-1，TX/RX | 调试输出 + VOFA+ 遥测（`app.c` 可在运行时改成别的波特率） |
| **SPI4** | `hspi4` | Master，2Lines TXOnly，**预分频 2 → 60 MHz** | LCD 刷屏 |
| **DCMI** | `hdcmi` | 8 bit，硬件同步，连续模式 | 摄像头采集 |
| **DMA2_Stream7** | `hdma_dcmi` | P2M，WORD/WORD，**CIRCULAR**，LOW | DCMI → `Camera_Buffer` |
| **I2C2** | `hi2c2` | — | 已初始化，当前**未使用** |

## 2.6 内存占用（实测）

| 区域 | 起始地址 | 容量 | 已用 | 占比 |
|---|---|---|---|---|
| FLASH | `0x08000000` | 128 KB | **88 856 B** | **67.79 %** |
| DTCMRAM | `0x20000000` | 128 KB | **58 904 B** | **44.94 %** |
| RAM (AXI) | `0x24000000` | 512 KB | 0 B（链接器统计） | 0 % |
| RAM_D2 | `0x30000000` | 288 KB | 0 B | 0 % |
| RAM_D3 | `0x38000000` | 64 KB | 0 B | 0 % |
| ITCMRAM | `0x00000000` | 64 KB | 0 B | 0 % |

**注意：AXI SRAM 显示 0 B，但摄像头帧缓冲是硬编码地址 `0x24000000`，占 45120 B（≈44 KB）**，链接器不知道这块占用。

| 硬编码缓冲 | 地址 | 大小 | 定义处 |
|---|---|---|---|
| `Camera_Buffer` | `0x24000000` | `188 × 120 × 2 = 45 120 B` | `dcmi_ov2640.h` |

栈顶 `_estack = 0x20020000`（DTCM 顶端）。

---

# 3. L0 硬件层（HAL）

> 全部由 STM32CubeMX 生成。**除 `USER CODE` 区域外不允许手改**，否则重新生成会丢失。

## 3.1 文件清单

| 文件 | 行数 | 职责 |
|---|---|---|
| `Core/Src/main.c` | 321 | `SystemClock_Config()`、`Error_Handler()`、`main()`、`MPU_Config()`、缓存使能 |
| `Core/Inc/main.h` | 89 | 引脚别名宏（`*_Pin` / `*_GPIO_Port`）、`Error_Handler` 声明 |
| `Core/Src/tim.c` | 618 | TIM4/5/6/12/14/15 初始化与 MSP |
| `Core/Src/gpio.c` | 115 | GPIO 时钟与引脚模式 |
| `Core/Src/usart.c` | 146 | USART2 + 中断 |
| `Core/Src/spi.c` | 146 | SPI4 |
| `Core/Src/dcmi.c` | 210 | DCMI + DMA2_Stream7 + 中断 |
| `Core/Src/dma.c` | 55 | DMA 控制器使能 |
| `Core/Src/i2c.c` | 150 | I2C2 |
| `Core/Src/stm32h7xx_it.c` | 407 | 全部中断服务函数（含 DCMI/DMA/UART 回调） |
| `Core/Src/stm32h7xx_hal_msp.c` | 88 | HAL MSP 底层初始化 |
| `Core/Src/syscalls.c` / `sysmem.c` | 244 / 87 | newlib 系统调用桩、堆管理 |
| `Core/Src/system_stm32h7xx.c` | 450 | `SystemInit()`、`SystemCoreClock` |
| `Core/Inc/stm32h7xx_hal_conf.h` | 514 | HAL 模块裁剪开关 |
| `Drivers/STM32H7xx_HAL_Driver/` | — | ST HAL 驱动库 |
| `Drivers/CMSIS/` | — | 内核头 + 启动文件 |

## 3.2 对上层暴露的接口

**`main.h` 提供的引脚别名**（驱动层全靠它拿引脚）：

```c
/* 编码器方向 */
#define Encoder1_DIR_Pin        GPIO_PIN_12   /* GPIOD */
#define Encoder2_DIR_Pin        GPIO_PIN_0    /* GPIOA */
/* LCD */
#define LCD_CS_Pin / LCD_SCL_Pin / LCD_SDA_Pin    /* GPIOE: PIN_11/12/14 */
#define LCD_DC_Pin              GPIO_PIN_15   /* GPIOE */
#define LCD_BL_Pin              GPIO_PIN_15   /* GPIOD */
/* 摄像头 */
#define CAMREA_PWDN_Pin         GPIO_PIN_14   /* GPIOD */
/* SCCB 软件 I2C */
#define SCCB_SCL_Pin / SCCB_SDA_Pin               /* GPIOB: PIN_8/9 */
```

**`tim.h` 提供的句柄**：

```c
extern TIM_HandleTypeDef htim4, htim5, htim6, htim12, htim14, htim15;
```

**`usart.h` / `spi.h` / `dcmi.h` / `dma.h` / `i2c.h`**：

```c
extern UART_HandleTypeDef huart2;
extern SPI_HandleTypeDef  hspi4;
extern DCMI_HandleTypeDef hdcmi;
extern DMA_HandleTypeDef  hdma_dcmi;
extern I2C_HandleTypeDef  hi2c2;
```

**`main.c` 关键函数**：

| 函数 | 说明 |
|---|---|
| `void SystemClock_Config(void)` | PLL1 配置到 480 MHz，见 §2.2 |
| `void MPU_Config(void)` | MPU region 0 = AXI SRAM 512 KB，Normal/Write-back |
| `void Error_Handler(void)` | 死循环关中断，应用层可重写（本工程用 `App_OnFatalError` 包装） |

**缓存**：`main()` 里在 `SystemClock_Config()` 之后调用 `SCB_EnableICache()` / `SCB_EnableDCache()`（`main.c:94` 附近），所以 **D-Cache 是开启的**。

---

# 4. L1 驱动层

## 4.1 `motor.h` / `motor.c`

| 项 | 内容 |
|---|---|
| **层** | L1 驱动层 |
| **职责** | 两个直流电机的 PWM 占空比输出（含方向），通过 TIM12 / TIM15 双通道实现正反转 |
| **依赖** | `motor.h` → `<stdint.h>`；`motor.c` → `motor.h`、`main.h`、`tim.h` |

**自定义类型**

```c
typedef enum
{
    MOTOR_L = 0,        /* 左轮，TIM12（PB14/PB15） */
    MOTOR_R,            /* 右轮，TIM15（PA2/PA3）  */
    MOTOR_COUNT
} motor_id_t;
```

**常量**

| 常量 | 值 | 含义 |
|---|---|---|
| `MOTOR_DUTY_MAX` | `(100.0f)` | 占空比上限（%） |
| `MOTOR_DUTY_MIN` | `(-100.0f)` | 占空比下限（%） |

**接口**

```c
void  Motor_Init(void);                          /* 启动两路 PWM，初始占空比 0 */
void  Motor_SetDuty(motor_id_t id, float duty_pct);
void  Motor_Stop(motor_id_t id);                 /* = Motor_SetDuty(id, 0) */
void  Motor_StopAll(void);
float Motor_GetDuty(motor_id_t id);
```

**`Motor_SetDuty` 的行为约定**

| 参数 | 输出 | 说明 |
|---|---|---|
| `duty > 0` | CH1 = compare，CH2 = 0 | 正转 |
| `duty < 0` | CH1 = 0，CH2 = compare | 反转（取绝对值） |
| `duty == 0` | CH1 = CH2 = 0 | **滑行（coast）**，不是刹车 |

`compare = (uint32_t)(|duty| × ARR / 100)`，**对小数截断**，所以有效分辨率 = `100/ARR %` = 0.05 %（ARR=2000）。

## 4.2 `encoder.h` / `encoder.c`

| 项 | 内容 |
|---|---|
| **层** | L1 驱动层 |
| **职责** | 增量式编码器测速（TIM4/TIM5 输入捕获 + 从模式复位），并提供高频节拍钩子 |
| **依赖** | `encoder.h` → `<stdint.h>`、`motor.h`（复用 `motor_id_t`）；`encoder.c` → `encoder.h`、`main.h`、`tim.h` |

**常量（`encoder.c` 内部）**

| 常量 | 值 | 含义 |
|---|---|---|
| `ENCODER_TIMER_TICK_HZ` | `(1.0E7f)` | 捕获定时器计数频率 10 MHz（240 MHz / 24） |
| `ENCODER_STALL_TIMEOUT_TICKS` | `(7000U)` | 超过此捕获值认为堵转，速度清 0（10 MHz 下 = 0.7 ms） |
| `ENCODER_TICK_BASE_HZ` | `(10000U)` | TIM6 中断频率 10 kHz，用作分频基准 |
| `s_encoder_resolution` | `10000U` | 编码器线数（`static const`） |
| `s_wheel_circumference` | `75.0f` | **轮周长 75 mm**（`static const`） |

**接口**

```c
void     Encoder_Init(void);
void     Encoder_Update(void);                      /* 由 TIM6 中断调用，10 kHz */
float    Encoder_GetSpeed(motor_id_t id);           /* mm/s，带符号 */
uint32_t Encoder_GetPulse(motor_id_t id);           /* 最近一次捕获计数 */
uint32_t Encoder_GetSampleCount(void);              /* 采样次数（判断编码器活着） */
void     Encoder_SetTickHandler(void (*handler)(void), uint32_t divider);
float    Encoder_GetTickPeriod(void);               /* 秒，如 500 Hz 返回 0.002f */
```

**速度换算公式**

```
speed[mm/s] = wheel_circumference × ENCODER_TIMER_TICK_HZ / (resolution × pulse)
            = 75 × 1.0e7 / (10000 × pulse)
            = 75000 / pulse
```

| 捕获值 `pulse` | 换算速度 |
|---|---|
| 50 | 1500 mm/s |
| 75 | 1000 mm/s |
| 150 | 500 mm/s |
| 250 | 300 mm/s |
| > 7000 | 判为堵转，速度清 0 |

**节拍钩子（给串级内环用）**

```c
/* 在 app.c 的 App_Init 里注册 */
Encoder_SetTickHandler(App_InnerLoopTick, 10000U / CHASSIS_INNER_LOOP_HZ);  /* = 20 → 500 Hz */
```

回调在 **TIM6 中断上下文**执行，必须短小、不可阻塞。

## 4.3 `buzzer.h` / `buzzer.c`

| 项 | 内容 |
|---|---|
| **层** | L1 驱动层 |
| **职责** | TIM14 CH1（PA7）输出 PWM 驱动无源蜂鸣器 |
| **依赖** | `buzzer.h` → `<stdint.h>`；`buzzer.c` → `buzzer.h`、`stdint.h`、`tim.h` |

**接口**

```c
void buzzer_init(void);                              /* 启动 TIM14 PWM */
void buzzer_beep(uint8_t volume, uint8_t duration_ms); /* volume = 占空比 % */
```

## 4.4 LCD 驱动

### `lcd_spi_200.h` / `lcd_spi_200.c`（1696 行）

| 项 | 内容 |
|---|---|
| **层** | L1 驱动层 |
| **职责** | 240×320 SPI LCD 的基础绘图、字符串、图片、OV2640 图像显示 |
| **依赖** | `lcd_spi_200.h` → `<stdio.h>`、`stm32h7xx_hal.h`、`lcd_fonts.h`、`lcd_image.h`；`lcd_spi_200.c` → `spi.h`、`lcd_spi_200.h` |

**常量**

| 常量 | 值 | 含义 |
|---|---|---|
| `LCD_Width` / `LCD_Height` | 240 / 320 | 屏幕像素尺寸 |
| `Direction_H_Flip` / `Direction_V` / `Direction_V_Flip` | 1 / 2 / 3 | 显示方向 |
| `Fill_Zero` / `Fill_Space` | 0 / 1 | 数字补位方式 |
| `LCD_WHITE` … `DARK_GREY` | 24 个颜色宏 | `0xRRGGBB` 格式 |

**背光与数据/命令控制（宏）**

```c
LCD_Backlight_ON / LCD_Backlight_OFF     /* PD15 */
LCD_DC_Command   / LCD_DC_Data           /* PE15 */
```

**接口（全部 29 个）**

```c
/* ---- 初始化与设置 ---- */
void LCD_Init(void);                                        /* 屏与 SPI 初始化 */
void LCD_Clear(void);                                       /* 清屏 */
void LCD_ClearRect(uint16_t x, uint16_t y, uint16_t w, uint16_t h);
void LCD_SetAddress(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2);
void LCD_SetColor(uint32_t Color);                          /* 画笔色 */
void LCD_SetBackColor(uint32_t Color);                      /* 背景色 */
void LCD_SetDirection(uint8_t direction);
void LCD_SetAsciiFont(pFONT *fonts);
void LCD_SetTextFont(pFONT *fonts);
void LCD_ShowNumMode(uint8_t mode);                         /* Fill_Space / Fill_Zero */

/* ---- 字符与数字 ---- */
void LCD_DisplayChar(uint16_t x, uint16_t y, uint8_t c);
void LCD_DisplayString(uint16_t x, uint16_t y, char *p);
void LCD_DisplayChinese(uint16_t x, uint16_t y, char *pText);
void LCD_DisplayText(uint16_t x, uint16_t y, char *pText);
void LCD_DisplayNumber(uint16_t x, uint16_t y, int32_t number, uint8_t len);
void LCD_DisplayDecimals(uint16_t x, uint16_t y, double number, uint8_t len, uint8_t decs);

/* ---- 基础几何 ---- */
void LCD_DrawPoint(uint16_t x, uint16_t y, uint32_t color);
void LCD_DrawLine_V(uint16_t x, uint16_t y, uint16_t height);
void LCD_DrawLine_H(uint16_t x, uint16_t y, uint16_t width);
void LCD_DrawLine(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2);
void LCD_DrawRect(uint16_t x, uint16_t y, uint16_t w, uint16_t h);
void LCD_DrawCircle(uint16_t x, uint16_t y, uint16_t r);
void LCD_DrawEllipse(int x, int y, int r1, int r2);
void LCD_FillRect(uint16_t x, uint16_t y, uint16_t w, uint16_t h);
void LCD_FillCircle(uint16_t x, uint16_t y, uint16_t r);

/* ---- 图像 ---- */
void LCD_DrawImage(uint16_t x, uint16_t y, uint16_t w, uint16_t h, const uint8_t *pImage);
void LCD_CopyBuffer(uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint16_t *DataBuff);
void show_ov2640_image(uint16_t x, uint16_t y, const uint16_t *image,
                       uint16_t width, uint16_t height,
                       uint16_t dis_width, uint16_t dis_height, uint8_t threshold);
void show_ov2640_image_int8(uint16_t x, uint16_t y, const uint8_t *image,
                            uint16_t width, uint16_t height,
                            uint16_t dis_width, uint16_t dis_height);
```

> `show_ov2640_image()` 是最耗时的路径：188×120 个 16 bit 像素 = 45 120 字节，
> SPI4 在 60 MHz 下光传输约 6 ms，加逐像素格式转换约 **8 ms**。
> `app.c` 用 `APP_IMAGE_DISPLAY_DIV = 3` 把它节流到每 3 帧一次。

### `lcd_fonts.h` / `lcd_fonts.c`（988 行）

**自定义类型**

```c
typedef struct _pFont
{
    const uint8_t *pData;   /* 字模数据 */
    uint8_t  Width;         /* 字符宽 */
    uint8_t  Height;        /* 字符高 */
    uint8_t  Sizes;         /* 字节数 */
} pFont;
```

**常量对象**：`ASCII_Font12`、`ASCII_Font16`、`ASCII_Font24`、`ASCII_Font32`（由 `lcd_fonts.c` 定义）

### `lcd_image.h` / `lcd_image.c`（79 行）

**常量对象**：图片数组（`extern const uint8_t ...`），当前**未被引用**。

## 4.5 OV2640 摄像头驱动

### `sccb.h` / `sccb.c`（456 行）

| 项 | 内容 |
|---|---|
| **层** | L1 驱动层 |
| **职责** | 用 GPIO 模拟 SCCB（类 I²C）读写摄像头寄存器 |
| **依赖** | `sccb.h` → `main.h`；`sccb.c` → `sccb.h` |

**常量**

| 常量 | 值 | 含义 |
|---|---|---|
| `OV2640_DEVICE_ADDRESS` | `0x60` | OV2640 从地址 |
| `OV5640_DEVICE_ADDRESS` | `0x78` | OV5640 从地址 |
| `SCCB_SCL/SDA_PORT/PIN` | PB8 / PB9 | 引脚 |
| `ACK_OK` / `ACK_ERR` | 1 / 0 | 应答结果 |
| `SCCB_ERROR_NONE` … `SCCB_ERROR_READ_ADDRESS` | 0~4 | 错误码（`SCCB_LastError` 全局变量记录） |
| `SCCB_DelayVaule` | 8 | 位延时 |

**全局变量**：`uint8_t SCCB_LastError;`（供 `app.c` 打印诊断信息）

### `dcmi_ov2640.h` / `dcmi_ov2640.c`（710 行）

| 项 | 内容 |
|---|---|
| **层** | L1 驱动层 |
| **职责** | OV2640 初始化、寄存器表下发、DCMI+DMA 连续采集控制 |
| **依赖** | `dcmi_ov2640.h` → `main.h`、`sccb.h`、`dcmi.h`、**`vision_types.h`**（反向依赖，见 §1.2）；`dcmi_ov2640.c` → `dcmi_ov2640.h`、`dcmi_ov2640_cfg.h`、`Serial.h` |

**常量**

| 常量 | 值 | 含义 |
|---|---|---|
| `OV2640_Success` / `OV2640_Error` | 0 / −1 | 返回值 |
| `OV2640_Enable` / `OV2640_Disable` | 1 / 0 | 功能开关 |
| `Pixformat_RGB565` / `Pixformat_JPEG` | 0 / 1 | 像素格式 |
| `OV2640_Effect_*` | 0~3 | Normal / Negative / BW / BW_Negative |
| `OV2640_Width` / `OV2640_Height` | 192 / 144 | 传感器输出尺寸 |
| `Display_Width` / `Display_Height` | `VISION_IMG_WIDTH` / `VISION_IMG_HEIGHT` | = 188 × 120 |
| **`Camera_Buffer`** | **`0x24000000`** | 帧缓冲起始地址（AXI SRAM） |
| `OV2640_BufferSize` | `188 × 120 × 2 / 4 = 11280` | DMA 传输字数（32 bit 为单位） |
| `OV2640_PWDN_PIN/PORT` | PD14 | 掉电控制 |

**接口**

```c
int8_t   OV2640_Init(void);
void     OV2640_DMA_Transmit_Continuous(uint32_t DMA_Buffer, uint32_t DMA_BufferSize);
void     OV2640_DMA_Transmit_Snapshot(uint32_t DMA_Buffer, uint32_t DMA_BufferSize);
void     OV2640_DCMI_Suspend(void);
void     OV2640_DCMI_Resume(void);
void     OV2640_DCMI_Stop(void);
void     OV2640_Reset(void);
uint16_t OV2640_ReadID(void);
void     OV2640_Config(const uint8_t (*ConfigData)[2]);
void     OV2640_Set_Pixformat(uint8_t pixformat);
int8_t   OV2640_Set_Framesize(uint16_t width, uint16_t height);
int8_t   OV2640_Set_Horizontal_Mirror(int8_t ConfigState);
int8_t   OV2640_Set_Vertical_Flip(int8_t ConfigState);
void     OV2640_Set_Saturation(int8_t Saturation);
void     OV2640_Set_Brightness(int8_t Brightness);
void     OV2640_Set_Contrast(int8_t Contrast);
void     OV2640_Set_Effect(uint8_t effect_Mode);
```

> `OV2640_BufferSize` 除以 4 是因为 DMA 用 **WORD 对齐**（`DMA_PDATAALIGN_WORD` / `DMA_MDATAALIGN_WORD`）。

### `dcmi_ov2640_cfg.h`（275 行）

纯数据文件：两张寄存器配置表（RGB565 / JPEG），格式 `const uint8_t [N][2]`，由 `OV2640_Config()` 下发。**无函数、无类型**。

## 4.6 通信协议驱动

### `Serial.h` / `Serial.c`（210 行）

| 项 | 内容 |
|---|---|
| **层** | L1 驱动层 |
| **职责** | USART2 收发封装（阻塞发送 + 中断接收缓冲 + `printf` 风格） |
| **依赖** | `Serial.h` → `<stdint.h>`、`<stdio.h>`；`Serial.c` → `main.h`、`Serial.h` |

**常量**

| 常量 | 值 | 含义 |
|---|---|---|
| `SERIAL_RX_BUFFER_SIZE` | `256U` | 接收环形缓冲大小 |

**接口**

```c
void     Serial_ClearRxBuffer(void);
void     Serial_StartReceive(void);                     /* 启动中断接收 */
void     Serial_SendByte(uint8_t Byte);                 /* 阻塞 */
void     Serial_SendArray(const uint8_t *Array, uint16_t Length);
void     Serial_SendString(const char *String);
void     Serial_SendNumber(uint32_t Number, uint8_t Length);
void     Serial_Printf(const char *format, ...);
uint8_t  Serial_GetRxFlag(void);
uint8_t  Serial_GetRxData(void);
uint16_t Serial_GetRxCount(void);
uint8_t  Serial_ReadByte(void);
uint8_t  ReadLine(char *out, uint8_t max_len, uint32_t timeout_ms);
```

**阻塞代价（重要）**：`Serial_SendByte` 内部 `HAL_UART_Transmit(..., HAL_MAX_DELAY)`，逐字节阻塞。

| 波特率 | 1 字节耗时 | 一帧 20 字节（VOFA 4 通道） |
|---|---|---|
| 9600 | 1.04 ms | **20.8 ms** |
| 115200 | 0.087 ms | **1.74 ms** |
| 460800 | 0.022 ms | **0.43 ms** |

### `bluetooth.h` / `bluetooth.c`（195 行）

| 项 | 内容 |
|---|---|
| **层** | L1 驱动层 |
| **职责** | JDY-33 蓝牙模块 AT 指令回包解析，判断连接状态 |
| **依赖** | `bluetooth.h` → `<stdint.h>`；`bluetooth.c` → `bluetooth.h`、`stdint.h` |

**自定义类型**

```c
typedef enum
{
    JDY33_NO_STATE = 0,     /* 未知 */
    JDY33_SPP_CONNECT,      /* 经典蓝牙串口已连 */
    JDY33_BLE_CONNECT,      /* BLE 已连 */
    JDY33_DISCONNECT        /* 已断开 */
} jdy33_state_t;
```

**接口**

```c
void show_bt_state(void);   /* 阻塞：内部带超时等待，禁止在实时路径调用 */
```

> 当前 `app.c` **没有调用**它（预留）。

---

# 5. L2 算法层

## 5.1 `vision_types.h`（155 行）—— 视觉全局约定

| 项 | 内容 |
|---|---|
| **层** | L2 算法层（但是**被 L1 的 `dcmi_ov2640.h` 引用**） |
| **职责** | 图像尺寸、控制行、扫线范围等全局常量；`lineinfo_s` / `watch_o`；全局实例声明 |
| **依赖** | `<stdint.h>`、`type_def.h` |

**常量**

| 常量 | 值 | 含义 |
|---|---|---|
| `VISION_IMG_WIDTH` | `(188)` | 图像宽（列数） |
| `VISION_IMG_HEIGHT` | `(120)` | 图像高（行数） |
| `VISION_IMG_CX` | `(94)` | 图像中心列 = 188/2 |
| `VISION_CONTROL_ROW` | `(60)` | **控制行**：取偏差的那一行 |
| `VISION_EDGE_STORE_SIZE` | `(22)` | 每行最多保存的跳变沿数 |
| `VISION_MIN_EDGE_WIDTH` | `(20)` | 认定赛道线的最小宽度（像素） |
| `VISION_BASE_LINE` | `(5)` | 扫线起始行（离车最近） |
| `VISION_FORWARD_NEAR` | `(0)` | 前视近端 |
| `VISION_FORWARD_FAR` | `(115)` | 扫线终止行（最远） |

> **行号方向**：`line_single()` 内部读 `Grayscale[119 - y]`，所以
> **算法行号 0 = 图像底部 = 离车最近**，行号越大越远。`VISION_CONTROL_ROW = 60` 是折中位置。

**自定义类型**

```c
struct lineinfo_s
{
    int16_t  y;                                   /* 行号 */
    int16_t  left;                                /* 左边界列 */
    int16_t  right;                               /* 右边界列 */
    int16_t  edge_count;                          /* 本行跳变沿总数 */
    uint8_t  edge_store[VISION_EDGE_STORE_SIZE];  /* 本行所有跳变沿列号 */
    int16_t  left_adjust;                         /* 补线后的左线 */
    int16_t  right_adjust;                        /* 补线后的右线 */
    int      left_lost;                           /* 左线丢失标志 */
    int      right_lost;                          /* 右线丢失标志 */
    uint8_t  zebra_flag;                          /* 斑马线标志 */
    int16_t  persp_lx, persp_ly;                  /* 左线逆透视后坐标 */
    int16_t  persp_rx, persp_ry;                  /* 右线逆透视后坐标 */
    int16_t  angel_left, angel_right;             /* 左右侧倾角 */
    int16_t  whole_lost;                          /* 1 = 本行两侧全线全丢 */
};
typedef struct lineinfo_s lineinfo_s;

struct watch_o
{
    uint8_t threshold;        /* 当前二值化阈值 */
    int     watch_line;       /* 能看到的最远行 */
    int     watch_lost;       /* 看丢的行数 */
    int     cross;            /* 左右线同时丢失的行数 */
    int     left_lost;        /* 左线丢失行数 */
    int     right_lost;       /* 右线丢失行数 */
    int     left_near_lost;   /* 左线开始丢失的行号 */
    int     right_near_lost;  /* 右线开始丢失的行号 */
    int     ZebraInLine;      /* 斑马线计数 */
};
typedef struct watch_o watch_o;
```

**全局实例（`extern`）**

```c
extern struct lineinfo_s lineinfo[VISION_IMG_HEIGHT];   /* 120 行边线信息，定义在 scan_line.c */
extern struct watch_o     watch;                        /* 整车视觉状态，定义在 Element_recognition.c */
```

## 5.2 `Binarization.h` / `Binarization.c`（188 行）

| 项 | 内容 |
|---|---|
| **层** | L2 算法层 |
| **职责** | 大津法（Otsu）自动阈值、二值化、边线可视化叠加 |
| **依赖** | 只有 `Binarization.h` → `vision_types.h`。**不含任何 HAL** |

**内部常量**

| 常量 | 值 | 含义 |
|---|---|---|
| `_GRAY_SCALE` | 64 | 灰度量化级数 |
| `_SHIFT_FOR_GRAYSCALE` | 10 | 灰度直方图移位 |
| `_Thresh_Mult` | 2 | 阈值倍数 |

**接口**

```c
int  Vision_OtsuThreshold(uint16_t *img, uint8_t img_v, uint8_t img_h, uint8_t step);
void Vision_Binarize(int8_t delta);
void Vision_DrawEdges(void);
```

| 函数 | 参数 | 返回/副作用 |
|---|---|---|
| `Vision_OtsuThreshold` | `img` 起始行指针；`img_v` 行数；`img_h` 列数；`step` 取样步长 | 灰度阈值（整数） |
| `Vision_Binarize` | `delta` 人工叠加的阈值偏移 | 生成 `Grayscale[]` 与 `imo[][]` |
| `Vision_DrawEdges` | — | 把扫线结果叠加到 `imo[][]`（可视化，供 LCD 显示） |

## 5.3 `scan_line.h` / `scan_line.c`（308 行）

| 项 | 内容 |
|---|---|
| **层** | L2 算法层 |
| **职责** | 从二值图像逐行搜索左右边线，补线，识别斑马线 |
| **依赖** | 只有 `scan_line.h` → `vision_types.h` |

**接口**

```c
/* 底层工具（一般不直接调用） */
uint8_t get_orign_edges(uint8_t *inputimg, uint8_t *edge_store);
int     get_best_edge(struct lineinfo_s *lineinfo, uint8_t *edge_store, struct lineinfo_s *lineinfo_ref);
int     get_max_edge(struct lineinfo_s *lineinfo, uint8_t *edge_store);
int     line_single(struct lineinfo_s *lineinfo, unsigned char *inputimg);
int     line_findnext(struct lineinfo_s *lineinfo, uint8_t *inputimg, struct lineinfo_s *lineinfo_ref);
int     zebra_detect(struct lineinfo_s *lineinfo, uint8_t *edge_store, uint8_t *inputimg);

/* 对外唯一入口 */
void    Vision_ScanLine(void);
```

`Vision_ScanLine()` 从 `VISION_BASE_LINE`（5）扫到 `VISION_FORWARD_FAR`（115），结果写入全局 `lineinfo[120]`。

## 5.4 `Element_recognition.h` / `Element_recognition.c`（69 行）

| 项 | 内容 |
|---|---|
| **层** | L2 算法层 |
| **职责** | 边线 → 路径中心；对外输出横向偏差 |
| **依赖** | 只有 `Element_recognition.h` → `vision_types.h` |

**内部状态（`static`）**

```c
static int16_t s_path_center[VISION_IMG_HEIGHT];   /* 每行路径中心列；无效行为 VISION_IMG_CX */
static uint8_t s_path_valid [VISION_IMG_HEIGHT];   /* 每行路径是否有效 */
```

**接口**

```c
void    Vision_ComputePathCenter(void);
int16_t Vision_GetLateralError(void);       /* 像素偏差，范围约 ±94 */
float   Vision_GetLateralErrorNorm(void);   /* 归一化偏差，-1.0 ~ +1.0 */
uint8_t Vision_IsPathValid(void);           /* 1 = 控制行有效 */
```

**判定逻辑（关键限制）**

```c
/* 某行有效的条件：左右边线都找到了 */
if (lineinfo[row].whole_lost || lineinfo[row].left_lost || lineinfo[row].right_lost)
    s_path_valid[row] = 0;

/* 下面三个函数**只看 VISION_CONTROL_ROW（60）这一行** */
Vision_GetLateralError()     → s_path_center[60] - 94
Vision_GetLateralErrorNorm() → (s_path_center[60] - 94) / 94
Vision_IsPathValid()         → s_path_valid[60]
```

> **这就是直角弯丢线的根因**：判定只依赖单行，第 60 行失效就整体判为丢线。

## 5.5 `inner_parameter.h` / `inner_parameter.c`（11 行）

**空壳占位模块**，接口已声明但**函数体为空**：

```c
void inner_parameter_init(void);      /* {} */
void update_inner_parameters(void);   /* {} */
```

当前**没有任何地方调用**。

## 5.6 `pid.h` / `pid.c`（98 行）

| 项 | 内容 |
|---|---|
| **层** | L2 算法层 |
| **职责** | 通用位置式 PID，带积分限幅与输出限幅 |
| **依赖** | 只有 `pid.h` |

**自定义类型**

```c
typedef struct
{
    float kp, ki, kd;
    float integral;          /* 积分累加量 */
    float last_error;        /* 上次误差（用于微分） */
    float integral_limit;    /* 积分限幅，0 = 不限幅 */
    float output_min, output_max;
} pid_t;
```

**接口**

```c
void  Pid_Init(pid_t *pid, float kp, float ki, float kd,
               float output_min, float output_max, float integral_limit);
void  Pid_SetGains(pid_t *pid, float kp, float ki, float kd);
void  Pid_Reset(pid_t *pid, float initial_error);   /* 清积分 + 设 last_error */
float Pid_Update(pid_t *pid, float error, float dt_s);
```

**`Pid_Update` 算法**

```c
if (dt_s <= 0) return 0;                          /* ← 注意：dt 非法直接返回 0 输出 */

integral += error * dt_s;
if (integral_limit > 0)  clamp(integral, ±integral_limit);

derivative = (error - last_error) / dt_s;         /* 误差差分，避免设定值突变冲击 */
last_error = error;

output = kp*error + ki*integral + kd*derivative;
clamp(output, output_min, output_max);
```

> **没有抗积分饱和（conditional integration）**：输出饱和时积分仍继续累积，只靠 `integral_limit` 兜底。
> 因此 `ki × integral_limit` 不应超过输出限幅范围，否则积分单独就能顶满输出。

## 5.7 `chassis.h` / `chassis.c`（400 行）—— 底盘控制核心

| 项 | 内容 |
|---|---|
| **层** | L2 算法层（依赖 L1 的 `motor.h` / `encoder.h`） |
| **职责** | 差速运动学 + 三级串级控制；提供**两套并行的控制档位** |
| **依赖** | `chassis.h` → `<stdint.h>`、`motor.h`；`chassis.c` → `chassis.h`、`encoder.h`、`motor.h`、`pid.h` |

### 常量

| 常量 | 值 | 含义 |
|---|---|---|
| `CHASSIS_TRACK_WIDTH_MM` | `(180.0f)` | 轮间距 180 mm |
| `CHASSIS_HALF_TRACK_MM` | `(180.0f × 0.5f)` = 90 | 差速运动学半径 |
| `CHASSIS_INNER_LOOP_HZ` | `(500U)` | 内环建议频率 |

### 自定义类型

```c
typedef enum
{
    CHASSIS_PROFILE_DUTY    = 0,   /* 占空比档：base/差速都当占空比 %，轮速环不参与 */
    CHASSIS_PROFILE_CASCADE = 1    /* 串级档：base/差速都是 mm/s，轮速环闭环 */
} chassis_profile_t;

typedef struct
{
    /* 基础速度 */
    float base_speed_mm_s;          /* 量纲随档位变化！占空比档 = %，串级档 = mm/s */

    /* 右轮速度补偿 */
    float right_comp_mm_s;          /* 右轮额外叠加量（量纲同上） */
    float right_comp_limit_mm_s;    /* 补偿绝对值上限 */

    /* ① 偏航角速度环 */
    float yaw_kp, yaw_ki, yaw_kd;   /* rad/s per 归一化误差 */
    float yaw_integral_limit;       /* 积分限幅（归一化误差·s） */
    float yaw_output_ratio;         /* 角速度限幅 = base × ratio / (L/2) */

    /* ③ 轮速环 */
    float wheel_kp, wheel_ki, wheel_kd;  /* % 占空比 per (mm/s) */
    float wheel_integral_limit;          /* mm/s·s */
    float wheel_output_limit_pct;        /* 输出限幅，% */
} chassis_config_t;
```

### 两套默认参数

**`CHASSIS_DEFAULT_CONFIG`（占空比档，`CHASSIS_PROFILE_DUTY`）**

| 字段 | 当前值 | 说明 |
|---|---|---|
| `base_speed_mm_s` | `40.0f` | **实际是 40 % 占空比** |
| `right_comp_mm_s` | `0.0f` | |
| `right_comp_limit_mm_s` | `100.0f` | |
| `yaw_kp` | `0.8f` | |
| `yaw_ki` | `0.2f` | |
| `yaw_kd` | `0.0f` | |
| `yaw_integral_limit` | `1.0f` | |
| `yaw_output_ratio` | `0.5f` | |
| `wheel_kp` | `5.0f` | 该档不使用 |
| `wheel_ki` | `2.0f` | 该档不使用 |
| `wheel_kd` | `0.0f` | |
| `wheel_integral_limit` | `20.0f` | |
| `wheel_output_limit_pct` | `100.0f` | |

**`CHASSIS_CASCADE_CONFIG`（串级档，`CHASSIS_PROFILE_CASCADE`）**

| 字段 | 当前值 | 说明 |
|---|---|---|
| `base_speed_mm_s` | `300.0f` | mm/s |
| `right_comp_mm_s` | `0.0f` | mm/s |
| `right_comp_limit_mm_s` | `400.0f` | mm/s |
| `yaw_kp` | `2.5f` | |
| `yaw_ki` | `2.0f` | |
| `yaw_kd` | `0.0f` | |
| `yaw_integral_limit` | `1.0f` | |
| `yaw_output_ratio` | `0.5f` | |
| `wheel_kp` | `1.2f` | 环路增益 ≈ 1.2 × 21.4 ≈ 25.7 |
| `wheel_ki` | `0.0f` | |
| `wheel_kd` | `0.0f` | |
| `wheel_integral_limit` | `25.0f` | |
| `wheel_output_limit_pct` | `100.0f` | |

### 接口

```c
/* ---- 初始化 / 参数 ---- */
void  Chassis_Init(chassis_profile_t profile);
const chassis_config_t *Chassis_GetConfig(void);
void  Chassis_SetBaseSpeed(float mm_s);
float Chassis_GetBaseSpeed(void);

/* ---- 右轮补偿（运行态下 PD8/PD9 调） ---- */
float Chassis_GetRightComp(void);
void  Chassis_AdjustRightComp(float delta_mm_s);
void  Chassis_SetRightComp(float mm_s);

/* ---- 在线改增益 ---- */
void  Chassis_SetYawGains(float kp, float ki, float kd);
void  Chassis_SetWheelGains(float kp, float ki, float kd);

/* ---- 控制周期（四种，见下表） ---- */
void  Chassis_Update(float lateral_error_norm, float dt_s);            /* 占空比档，单速率 */
void  Chassis_UpdateCascade(float lateral_error_norm, float dt_s);     /* 串级档，单速率 */
void  Chassis_UpdateOuter(float lateral_error_norm, float dt_s);       /* 串级档，外环 */
void  Chassis_UpdateInner(float dt_s);                                 /* 串级档，内环 */

/* ---- 观测 / 调试 ---- */
float Chassis_GetYawCommandRadS(void);
void  Chassis_SetWheelTarget(motor_id_t id, float mm_s);
float Chassis_GetWheelTarget(motor_id_t id);
void  Chassis_UpdateWheel(motor_id_t id, float dt_s);                  /* 单轮手动闭环 */

/* ---- 停车 ---- */
void  Chassis_Stop(void);
```

### 四个控制函数的分工（**这是最容易用错的地方**）

| 函数 | 档位 | 执行频率 | 做什么 | 不下发 |
|---|---|---|---|---|
| `Chassis_Update` | `DUTY` | 帧率 | ①角速度环 → ②`× L/2 × 0.05` → ③分配 → ④**直接当占空比** | 无 |
| `Chassis_UpdateCascade` | `CASCADE` | 帧率 | = `UpdateOuter` + `UpdateInner`（一键跑完） | — |
| `Chassis_UpdateOuter` | `CASCADE` | **帧率（30 Hz）** | ①→②→③，只写 `s_wheel_target[]`，并置 `s_inner_enabled = 1` | 电机 |
| `Chassis_UpdateInner` | `CASCADE` | **500 Hz（TIM6 ISR）** | ④ 轮速闭环 → `Motor_SetDuty()` | — |

> **档位与函数必须成对使用**：
> `Chassis_Init(0)` ↔ `Chassis_Update()`
> `Chassis_Init(1)` ↔ `Chassis_UpdateOuter()`（+ 中断里的 `Chassis_UpdateInner()`）
>
> 混用后果：把 mm/s 的 `base` 喂给占空比档 → 电机立刻满占空比。

### 内部状态（`static`）

```c
static chassis_config_t s_cfg;                        /* 当前生效参数 */
static pid_t   s_yaw_pid;                             /* ① 偏航环 */
static pid_t   s_wheel_pid[MOTOR_COUNT];              /* ③ 左右轮速环 */
static float   s_wheel_target[MOTOR_COUNT];           /* 目标速度 */
static float   s_yaw_command;                         /* 最近一次 ω_ref（调参观察） */
static uint8_t s_inner_enabled;                       /* 内环是否允许输出 */
```

### `Chassis_Stop()` 的行为

```c
s_inner_enabled = 0;        /* 先关内环，避免中断里把电机又拉起来 */
Motor_StopAll();            /* 占空比置 0 → 滑行 */
s_wheel_target[i] = 0;
Pid_Reset(&s_wheel_pid[i], 0);
Pid_Reset(&s_yaw_pid, 0);
s_yaw_command = 0;
```

---

# 6. L3 服务层

## 6.1 `panel.h` / `panel.c`（222 行）—— 人机接口

| 项 | 内容 |
|---|---|
| **层** | L3 服务层 |
| **职责** | LCD 分栏显示 + 三个按键的消抖与边沿检测 |
| **依赖** | `panel.h` → `<stdint.h>`；`panel.c` → `panel.h`、`main.h`、`encoder.h`、`lcd_spi_200.h`、`vision_types.h` |

**自定义类型**

```c
typedef enum
{
    PANEL_KEY_UP = 0,       /* PD8  */
    PANEL_KEY_DOWN,         /* PD9  */
    PANEL_KEY_MODE,         /* PD10 */
    PANEL_KEY_COUNT
} panel_key_t;

typedef enum
{
    PANEL_KEY_EVENT_NONE = 0,   /* 无 */
    PANEL_KEY_EVENT_PRESSED,    /* 按下沿（本次轮询新出现） */
    PANEL_KEY_EVENT_RELEASED    /* 释放沿 */
} panel_key_event_t;
```

**屏幕布局常量（`panel.c` 内部）**

| 常量 | 值 | 含义 |
|---|---|---|
| `PANEL_DEBOUNCE_MS` | `(15U)` | 按键消抖时间 |
| `PANEL_LCD_REFRESH_MS` | `(100U)` | 速度显示刷新节流 |
| `PANEL_MODE_X` / `PANEL_MODE_Y` | 0 / 134 | 模式行位置 |
| `PANEL_MODE_TEXT_X` | 48 | 模式文字起始列 |
| `PANEL_THRESHOLD_Y` | 150 | 阈值行 |
| `PANEL_SPEED_Y` | 166 | 速度行 |
| `PANEL_CAPTURE_Y` | 182 | 捕获率行 |
| `PANEL_COL_R_X` | 160 | 右轮列表头 |
| `PANEL_COL_VALUE_L_X` | 64 | 左轮数值列 |
| `PANEL_COL_VALUE_R_X` | 224 | 右轮数值列 |

**实际屏幕分区（240 × 320）**

```
 y=0~119    视觉图像（188×120，左上角）
 y=134      模式    MODE:ADJUST / RUN
 y=150      阈值    thr:xxx d:xxx
 y=166      速度    L:xxx  R:xxx  (mm/s)
 y=182      捕获率  L:xx%   R:xx%
 y=198      右轮补偿 comp:xxxx
 y=214      诊断    fps:xx  dtmax:xx
```

**接口**

```c
void               Panel_Init(void);
panel_key_event_t  Panel_PollKey(panel_key_t key);
uint8_t            Panel_IsKeyDown(panel_key_t key);
void               Panel_ShowMode(const char *name);
void               Panel_ShowMotorSpeeds(void);
void               Panel_ShowVisionImage(const uint8_t *image);
void               Panel_ShowString(uint16_t x, uint16_t y, const char *text);
void               Panel_ShowNumber(uint16_t x, uint16_t y, int32_t value, uint8_t len);
void               Panel_ShowThreshold(uint8_t threshold, int8_t delta);
```

**`Panel_PollKey` 语义**：内部维持每个键的消抖状态，只在状态变化的那一次轮询返回 `PRESSED` / `RELEASED`，否则返回 `NONE`。**必须在主循环里周期调用**（依赖调用间隔做消抖）。

## 6.2 `telemetry.h` / `telemetry.c`（42 行）—— 上位机通信

| 项 | 内容 |
|---|---|
| **层** | L3 服务层 |
| **职责** | 把浮点数组打包成 VOFA+ JustFloat 帧 |
| **依赖** | `telemetry.h` → `<stdint.h>`；`telemetry.c` → `telemetry.h`、`Serial.h` |

**常量**

| 常量 | 值 | 含义 |
|---|---|---|
| `TELEMETRY_JUSTFLOAT_TAIL_SIZE` | `(4U)` | 帧尾字节数（`0x00 0x00 0x80 0x7f`） |

**接口**

```c
void Telemetry_SendJustFloat(const float *data, uint8_t count);
void Telemetry_SendScalar(float value);
```

**帧格式**

```
[float0][float1]...[floatN-1] [0x00 0x00 0x80 0x7F]
                               └── JustFloat 帧尾 ──┘
帧长 = 4 × count + 4 字节
```

**本工程的通道分配（`app.c`）**

| 通道 | 内容 |
|---|---|
| ch0 | 右轮目标速度 `Chassis_GetWheelTarget(MOTOR_R)` |
| ch1 | 右轮实测速度 `Encoder_GetSpeed(MOTOR_R)` |
| ch2 | 左轮目标速度 |
| ch3 | 左轮实测速度 |

发送周期 `APP_TELEMETRY_PERIOD_MS = 100 ms`。

---

# 7. L4 应用层

## 7.1 `app.h` / `app.c`（498 行）—— 总编排

| 项 | 内容 |
|---|---|
| **层** | L4 应用层 |
| **职责** | 外设初始化顺序、主循环、视觉流水线调度、按键→事件→动作、遥测、诊断统计 |
| **依赖** | `app.h` → 无；`app.c` → **17 个下层头文件**（见 §8.1） |

**可调参数（`app.c` 内部宏）**

| 宏 | 当前值 | 含义 |
|---|---|---|
| `APP_OTSU_ROW` | `(30U)` | 大津法取样起始行 |
| `APP_OTSU_ROWS` | `(60U)` | 取样行数 |
| `APP_OTSU_STEP` | `(10U)` | 取样步长（越大越快越粗糙） |
| `APP_THRESHOLD_MAX` | `(120)` | 自动阈值上限（太亮会把白底当赛道） |
| `APP_THRESHOLD_MIN` | `(80)` | 自动阈值下限（太暗会丢线） |
| `APP_DELTA_STEP_UP` | `(10)` | PD8 单次阈值增量 |
| `APP_DELTA_STEP_DOWN` | `(5)` | PD9 单次阈值减量（不对称，防止调过头） |
| `APP_DELTA_LIMIT` | `(120)` | 阈值偏移绝对值上限 |
| `APP_COMP_STEP_MM_S` | `(5.0f)` | **运行态下 PD8/PD9 每次右轮补偿增量 mm/s** |
| `APP_TELEMETRY_PERIOD_MS` | `(100U)` | 遥测周期 |
| `APP_STARTUP_DELAY_MS` | `(1000U)` | 上电等待（给操作者放车的时间） |
| `APP_MIN_DT_MS` | `(1U)` | 控制周期下限，小于它跳过控制避免除零 |
| `APP_UART_BAUDRATE` | `(115200U)` | 运行时重配的波特率（与 `usart.c` 一致） |
| `APP_IMAGE_DISPLAY_DIV` | `(3U)` | 刷屏节流：每 3 帧刷一次 LCD |
| `APP_STAT_WINDOW_MS` | `(1000U)` | 帧率统计窗口 |

**外部接口**

```c
void App_Init(void);          /* 上电初始化（被 main.c 调用） */
void App_Loop(void);          /* 主循环体（被 main.c 的 while(1) 调用） */
void App_OnFatalError(void);  /* 致命错误处理（被 Error_Handler 调用） */
```

**内部静态状态**

```c
static int8_t   s_threshold_delta;      /* 人工阈值偏移，仅调节态改 */
static uint32_t s_last_control_tick;    /* 上一控制周期时间戳（算 dt） */
static uint8_t  s_image_frame_counter;  /* 刷屏节流计数器 */

/* 诊断统计（可直接删掉不影响功能） */
static uint32_t s_stat_frame_count, s_stat_tick;
static uint16_t s_stat_fps;
static uint32_t s_stat_dt_max_ms, s_stat_dt_peak_ms;
```

**`App_Init()` 初始化顺序**

```
0. AppFsm_Init()                     状态机 → ADJUST
1. Chassis_Init(1) → Chassis_Stop()  底盘（含 Motor_Init + Encoder_Init）
1b. Encoder_SetTickHandler(...)      注册 500 Hz 内环节拍
2. App_SetBaudrate(115200) → Serial_StartReceive()
3. OV2640_Init() → OV2640_DMA_Transmit_Continuous(Camera_Buffer, OV2640_BufferSize)
4. LCD_Init() → LCD_SetAsciiFont(&ASCII_Font16) → LCD_ShowNumMode(Fill_Space)
5. buzzer_init() → Panel_Init()
6. 串口打印 ready → 显示 → 蜂鸣 → HAL_Delay(1000)
```

**`App_Loop()` 每轮做的事**

```
1. App_PollKeys()          按键 → 事件 → 状态机 → 动作
2. App_HandleFrame()       若新帧就绪：大津法 → 二值化 → 扫线 → 路径中心 → 控制 → 显示
3. App_UpdateTelemetry()   每 100 ms 发一次 JustFloat
4. Panel_ShowMode()        模式显示
5. Panel_ShowMotorSpeeds() 速度显示（内部 100 ms 节流）
6. App_ShowRightComp()     右轮补偿值
7. App_ShowPerfStats()     fps / dtmax
```

**`App_HandleFrame()` 的控制段（当前版本）**

```c
dt = App_ComputeDt();
App_UpdatePerfStats(dt);

if (AppFsm_IsRunning() && dt > 0)
{
    if (Vision_IsPathValid())
        Chassis_UpdateOuter(Vision_GetLateralErrorNorm(), dt);
    else
        Chassis_Stop();                   /* 丢线 = 硬停（滑行） */
}
```

## 7.2 `app_fsm.h` / `app_fsm.c`（115 行）—— 状态机

| 项 | 内容 |
|---|---|
| **层** | L4 应用层 |
| **职责** | 纯决策逻辑：两态（ADJUST / RUN）+ 表驱动转移；**不碰电机、不持有阈值** |
| **依赖** | 只有 `app_fsm.h` → `<stdint.h>` |

**自定义类型**

```c
typedef enum { APP_STATE_ADJUST = 0, APP_STATE_RUN, APP_STATE_COUNT } app_state_t;

typedef enum
{
    APP_EVENT_NONE = 0,
    APP_EVENT_THRESHOLD_UP,     /* 来自 PD8 */
    APP_EVENT_THRESHOLD_DOWN,   /* 来自 PD9 */
    APP_EVENT_TOGGLE_RUN,       /* 来自 PD10 */
    APP_EVENT_COUNT
} app_event_t;

typedef enum
{
    APP_ACTION_NONE = 0,
    APP_ACTION_THRESHOLD_UP,
    APP_ACTION_THRESHOLD_DOWN,
    APP_ACTION_START_RUN,
    APP_ACTION_STOP_RUN,
    APP_ACTION_RIGHT_COMP_UP,
    APP_ACTION_RIGHT_COMP_DOWN,
    APP_ACTION_COUNT
} app_action_t;
```

**状态转移表（`app_fsm.c` 里的 `s_transition[]`，共 6 条）**

| 当前状态 | 事件 | 次态 | 动作 |
|---|---|---|---|
| ADJUST | THRESHOLD_UP | ADJUST | `THRESHOLD_UP`（阈值 +10） |
| ADJUST | THRESHOLD_DOWN | ADJUST | `THRESHOLD_DOWN`（阈值 −5） |
| ADJUST | TOGGLE_RUN | **RUN** | `START_RUN`（清积分、对齐时钟） |
| **RUN** | THRESHOLD_UP | RUN | **`RIGHT_COMP_UP`（右轮补偿 +5 mm/s）** |
| **RUN** | THRESHOLD_DOWN | RUN | **`RIGHT_COMP_DOWN`（右轮补偿 −5 mm/s）** |
| RUN | TOGGLE_RUN | **ADJUST** | `STOP_RUN`（停车） |

> **PD8/PD9 的含义随状态改变**：调节态调阈值，运行态调右轮补偿。

**接口**

```c
void          AppFsm_Init(void);
app_action_t  AppFsm_Handle(app_event_t event);   /* 返回要执行的动作，并更新状态 */
app_state_t   AppFsm_GetState(void);
uint8_t       AppFsm_IsRunning(void);
const char   *AppFsm_GetStateName(void);          /* "ADJUST" / "RUN" */
```

## 7.3 `type_def.h`（12 行）

移植代码的整型别名兼容层：

```c
typedef int16_t  int16;      /* 由 stdint 映射 */
typedef uint8_t  uint8;
typedef uint16_t uint16;
typedef int8_t   int8;
typedef uint32_t uint32;
```

被 `vision_types.h` 引用。

## 7.4 `tests/test_app_fsm.c`（110 行）

**PC 端测试程序**，不参与固件构建。用 `x86_64-w64-mingw32-gcc` 编译 `app_fsm.c` 后验证转移矩阵：

```bash
x86_64-w64-mingw32-gcc -std=gnu11 -Wall -Wextra \
  -I Base/Inc -o test_fsm.exe tests/test_app_fsm.c Base/Src/app_fsm.c
```

输出 `ALL PASS` / `TEST FAILED`。

---

# 8. 依赖关系

## 8.1 依赖矩阵（行 = 依赖方，列 = 被依赖）

```
                 v  = 通过 .c 引用
                 h  = 通过 .h 引用（会传染）
```

| 文件 | 依赖的本项目头文件 |
|---|---|
| `Base/app.c` | app.h, main.h, app_fsm.h, buzzer.h, dcmi_ov2640.h, encoder.h, lcd_spi_200.h, motor.h, Serial.h, usart.h, chassis.h, Binarization.h, Element_recognition.h, scan_line.h, vision_types.h, panel.h, telemetry.h |
| `Base/app_fsm.c` | app_fsm.h |
| `Base/app.h` | *（无）* |
| `Base/app_fsm.h` | *（无）* |
| `Base/type_def.h` | *（无）* |
| `UI/panel.c` | panel.h, main.h, encoder.h, lcd_spi_200.h, vision_types.h |
| `UI/telemetry.c` | telemetry.h, Serial.h |
| `UI/panel.h` | *（无）* |
| `UI/telemetry.h` | *（无）* |
| `Control/chassis.c` | chassis.h, encoder.h, motor.h, pid.h |
| `Control/pid.c` | pid.h |
| `Control/chassis.h` | motor.h |
| `Control/pid.h` | *（无）* |
| `camera_process/Binarization.c` | Binarization.h（→ vision_types.h → type_def.h） |
| `camera_process/scan_line.c` | scan_line.h（→ vision_types.h） |
| `camera_process/Element_recognition.c` | Element_recognition.h（→ vision_types.h） |
| `camera_process/inner_parameter.c` | inner_parameter.h |
| `camera_process/*.h` | vision_types.h（→ type_def.h） |
| `HardWare/motor/motor.c` | motor.h, main.h, tim.h |
| `HardWare/motor/encoder.c` | encoder.h, main.h, tim.h |
| `HardWare/motor/encoder.h` | motor.h（复用 `motor_id_t`） |
| `HardWare/LCD/lcd_spi_200.c` | spi.h, lcd_spi_200.h（→ stm32h7xx_hal.h / lcd_fonts.h / lcd_image.h） |
| `HardWare/LCD/lcd_fonts.c` | lcd_fonts.h |
| `HardWare/LCD/lcd_image.c` | lcd_image.h |
| `HardWare/OV2640/sccb.c` | sccb.h（→ main.h） |
| `HardWare/OV2640/dcmi_ov2640.c` | dcmi_ov2640.h, dcmi_ov2640_cfg.h, Serial.h |
| `HardWare/OV2640/dcmi_ov2640.h` | main.h, sccb.h, dcmi.h, **vision_types.h** ← 反向依赖 |
| `HardWare/protocol/Serial.c` | main.h, Serial.h |
| `HardWare/protocol/bluetooth.c` | bluetooth.h, stdint.h |
| `HardWare/Buzzer/buzzer.c` | buzzer.h, stdint.h, tim.h |

## 8.2 依赖图（自上而下）

```
                        ┌─────────────┐
                        │  main.c     │
                        └──────┬──────┘
                               │ App_Init / App_Loop
                        ┌──────▼──────────────┐
                        │      app.c          │  ← 唯一的"知道一切"的文件
                        └──┬────┬────┬────┬───┘
              ┌────────────┘    │    │    └──────────────┐
              ▼                 ▼    ▼                   ▼
     ┌────────────────┐  ┌──────────┐  ┌───────────┐ ┌──────────────┐
     │ app_fsm.c      │  │ panel.c  │  │telemetry.c│ │ 视觉流水线   │
     │（纯逻辑，无依赖）│  │  ↓       │  │   ↓       │ │  Binarization│
     └────────────────┘  │ lcd_*.c  │  │ Serial.c  │ │  scan_line   │
                         │ encoder  │  └───────────┘ │  Element_rec │
                         └──────────┘                └──────┬───────┘
                                                            │
     ┌──────────────────────────────────────────────────────▼───────┐
     │                       chassis.c                              │
     │   ┌────────┐   ┌──────────┐   ┌──────────┐                   │
     │   │ pid.c  │   │ motor.c  │   │ encoder.c│                   │
     │   └────────┘   └────┬─────┘   └────┬─────┘                   │
     └─────────────────────┼──────────────┼─────────────────────────┘
                           ▼              ▼
     ┌──────────────────────────────────────────────────────────────┐
     │              Core/  (tim.c  gpio.c  spi.c  dcmi.c ...)        │
     │              Drivers/STM32H7xx_HAL_Driver + CMSIS             │
     └──────────────────────────────────────────────────────────────┘
```

## 8.3 依赖健康度

| 检查项 | 结果 |
|---|---|
| 算法层是否 include HAL | ✔ 干净（`camera_process/` 与 `Control/` 都不含 `stm32*`） |
| 是否存在循环依赖 | ✔ 无 |
| 驱动层是否依赖应用层 | ✔ 无 |
| 反向依赖（L1→L2） | ⚠ 1 处：`dcmi_ov2640.h` → `vision_types.h` |
| 单文件依赖数最多的 | `app.c`（17 个）—— 符合编排者定位，但要警惕继续膨胀 |

---

# 9. 常量速查表

## 9.1 几何与车辆参数

| 常量 | 值 | 位置 |
|---|---|---|
| `CHASSIS_TRACK_WIDTH_MM` | 180.0f | `chassis.h` |
| `CHASSIS_HALF_TRACK_MM` | 90.0f | `chassis.h` |
| `s_wheel_circumference` | **75.0f** mm | `encoder.c` |
| `s_encoder_resolution` | **10000** 线 | `encoder.c` |

## 9.2 时间与频率

| 常量 | 值 | 位置 |
|---|---|---|
| `CHASSIS_INNER_LOOP_HZ` | 500 | `chassis.h` |
| `ENCODER_TIMER_TICK_HZ` | 1.0e7 | `encoder.c` |
| `ENCODER_TICK_BASE_HZ` | 10000 | `encoder.c` |
| `ENCODER_STALL_TIMEOUT_TICKS` | 7000 | `encoder.c` |
| `APP_TELEMETRY_PERIOD_MS` | 100 | `app.c` |
| `APP_STARTUP_DELAY_MS` | 1000 | `app.c` |
| `APP_MIN_DT_MS` | 1 | `app.c` |
| `APP_STAT_WINDOW_MS` | 1000 | `app.c` |
| `PANEL_DEBOUNCE_MS` | 15 | `panel.c` |
| `PANEL_LCD_REFRESH_MS` | 100 | `panel.c` |

## 9.3 阈值与调参

| 常量 | 值 | 位置 |
|---|---|---|
| `APP_THRESHOLD_MAX` / `MIN` | 120 / 80 | `app.c` |
| `APP_DELTA_STEP_UP` / `DOWN` | 10 / 5 | `app.c` |
| `APP_DELTA_LIMIT` | 120 | `app.c` |
| `APP_COMP_STEP_MM_S` | 5.0f | `app.c` |
| `VISION_MIN_EDGE_WIDTH` | 20 | `vision_types.h` |
| `VISION_EDGE_STORE_SIZE` | 22 | `vision_types.h` |

## 9.4 缓冲区与尺寸

| 常量 | 值 | 位置 |
|---|---|---|
| `VISION_IMG_WIDTH` / `HEIGHT` | 188 / 120 | `vision_types.h` |
| `VISION_IMG_CX` | 94 | `vision_types.h` |
| `VISION_CONTROL_ROW` | 60 | `vision_types.h` |
| `VISION_BASE_LINE` / `FORWARD_FAR` | 5 / 115 | `vision_types.h` |
| `Camera_Buffer` | 0x24000000 | `dcmi_ov2640.h` |
| `OV2640_BufferSize` | 11280（word） | `dcmi_ov2640.h` |
| `SERIAL_RX_BUFFER_SIZE` | 256 | `Serial.h` |
| `LCD_Width` / `Height` | 240 / 320 | `lcd_spi_200.h` |
| `TELEMETRY_JUSTFLOAT_TAIL_SIZE` | 4 | `telemetry.h` |

## 9.5 硬件相关

| 常量 | 值 | 位置 |
|---|---|---|
| `OV2640_DEVICE_ADDRESS` | 0x60 | `sccb.h` |
| `OV5640_DEVICE_ADDRESS` | 0x78 | `sccb.h` |
| `SCCB_DelayVaule` | 8 | `sccb.h` |
| `MOTOR_DUTY_MAX` / `MIN` | ±100.0f | `motor.h` |
| `APP_UART_BAUDRATE` | 115200 | `app.c` |

---

# 10. 典型调用序列

## 10.1 上电初始化

```c
main()
├── HAL_Init()
├── MPU_Config()
├── SystemClock_Config()           /* 480 MHz */
├── MX_GPIO_Init / MX_DMA_Init / MX_DCMI_Init / MX_I2C2_Init
├── MX_SPI4_Init / MX_TIM4..15_Init / MX_USART2_UART_Init
├── SCB_EnableICache() / SCB_EnableDCache()
└── App_Init()
    ├── AppFsm_Init()                          → ADJUST
    ├── Chassis_Init(1)                        → 串级档 + Motor_Init + Encoder_Init
    ├── Chassis_Stop()
    ├── Encoder_SetTickHandler(App_InnerLoopTick, 20)   → 500 Hz 内环
    ├── App_SetBaudrate(115200) + Serial_StartReceive()
    ├── OV2640_Init() + OV2640_DMA_Transmit_Continuous(0x24000000, 11280)
    ├── LCD_Init() + LCD_SetAsciiFont(&ASCII_Font16)
    ├── buzzer_init() + Panel_Init()
    └── HAL_Delay(1000)
while (1) App_Loop();
```

## 10.2 一帧图像的完整路径

```c
App_Loop()
└── App_HandleFrame()                 /* DCMI_FrameState 置位时进入 */
    ├── Vision_OtsuThreshold(...)     → threshold（并夹到 80~120）
    ├── Vision_Binarize(delta)        → Grayscale[] / imo[][]
    ├── Vision_ScanLine()             → lineinfo[0..115]
    ├── Vision_ComputePathCenter()    → path_center[] / path_valid[]
    ├── dt = App_ComputeDt()
    ├── App_UpdatePerfStats(dt)
    ├── if (IsRunning)
    │   ├── Vision_IsPathValid() ?
    │   │   ├── 是 → Chassis_UpdateOuter(e_norm, dt)
    │   │   │        ├── ω_ref = Pid_Update(&s_yaw_pid, e_norm, dt)
    │   │   │        ├── Δv    = ω_ref × 90
    │   │   │        ├── s_wheel_target[L] = base − Δv
    │   │   │        ├── s_wheel_target[R] = base + comp + Δv
    │   │   │        └── s_inner_enabled = 1
    │   │   └── 否 → Chassis_Stop()
    └── 显示：DrawEdges / ShowVisionImage(每3帧) / ShowThreshold
```

## 10.3 内环（中断上下文，500 Hz）

```c
TIM6_IRQHandler
└── HAL_TIM_PeriodElapsedCallback(TIM6)
    ├── Encoder_Update()                    /* 10 kHz：堵转判断 */
    └── s_tick_count++ == 20 ?
        └── App_InnerLoopTick()             /* 500 Hz */
            └── Chassis_UpdateInner(Encoder_GetTickPeriod() = 0.002f)
                ├── if (!s_inner_enabled) return;
                └── for each wheel:
                    ├── measured = Encoder_GetSpeed(i)
                    ├── duty = Pid_Update(&s_wheel_pid[i], target − measured, dt)
                    └── Motor_SetDuty(i, duty)
```

## 10.4 按键 → 动作

```c
App_PollKeys()
├── Panel_PollKey(PANEL_KEY_UP) == PRESSED ?
│   └── App_ApplyAction(AppFsm_Handle(APP_EVENT_THRESHOLD_UP))
│       ├── ADJUST 态 → s_threshold_delta += 10（夹到 ±120）
│       └── RUN 态    → Chassis_AdjustRightComp(+5.0f)
├── Panel_PollKey(PANEL_KEY_DOWN) == PRESSED ? → 同理，−5 / −5
└── Panel_PollKey(PANEL_KEY_MODE) == PRESSED ?
    ├── ADJUST → RUN：Chassis_Stop() + 对齐 s_last_control_tick
    └── RUN → ADJUST：Chassis_Stop()
```

---

# 11. 注意事项与已知限制

## 11.1 必须成对使用的接口

| 组合 | 说明 |
|---|---|
| `Chassis_Init(0)` ↔ `Chassis_Update()` | 占空比档 |
| `Chassis_Init(1)` ↔ `Chassis_UpdateOuter()` + 中断里 `Chassis_UpdateInner()` | 串级档 |
| `OV2640_DMA_Transmit_Continuous` ↔ `Camera_Buffer` + `OV2640_BufferSize` | 缓冲区地址/长度必须一致 |

## 11.2 中断上下文的约束

| 函数 | 在中断里被调用 | 限制 |
|---|---|---|
| `Encoder_Update()` | ✔ TIM6 10 kHz | 不得阻塞 |
| `Encoder_SetTickHandler` 注册的回调 | ✔ 分频后 | 不得阻塞、不得 `HAL_Delay`、不得串口阻塞发送 |
| `Chassis_UpdateInner()` | ✔ 500 Hz | 同上，含 2 次 `Pid_Update` + 2 次 `Motor_SetDuty` |

## 11.3 数据竞争（已知、影响可忽略）

| 变量 | 写方 | 读方 | 后果 |
|---|---|---|---|
| `s_wheel_target[2]` | 外环（30 Hz，线程上下文） | 内环（500 Hz，ISR） | 32 位浮点写入是原子的，最坏情况是内环用到上一帧的目标 |
| `s_inner_enabled` | `Chassis_UpdateOuter` / `Chassis_Stop` | `Chassis_UpdateInner` | 单字节写原子 |
| `pid_t s_wheel_pid[2]` | `Chassis_Stop`（复位） | `Chassis_UpdateInner` | 复位瞬间可能与一次更新交错，最坏丢一个周期的状态 |

## 11.4 直角弯丢线（**当前未解决的核心问题**）

| 现象 | 原因 | 状态 |
|---|---|---|
| 90° 弯稳定丢线 | `Vision_IsPathValid()` **只判定第 60 行**，拐角处该行失效 | 未解决 |
| 丢线判定会闪烁 | 同上，第 60 行在拐角处时有时无 | 未解决 |
| 丢线后立即硬停 | `app.c` 走 `Chassis_Stop()`（duty=0 滑行） | 当前行为 |
| 300 mm/s 下最小转弯半径 180 mm | `Δv_max = base × yaw_output_ratio = 150`，`R = v/ω = 180 mm` | 需提高 `yaw_output_ratio` 到 0.8~1.0 才能到几何极限 90 mm |

**改进方向（尚未实现）**：对有效行做加权最小二乘拟合 `x(r) = a + b·r`，在控制行（或前视行）处**外推求值**——既能在第 60 行失效时仍有输出，又天然得到斜率（行进方向）。

## 11.5 阻塞点

| 位置 | 代价 | 说明 |
|---|---|---|
| `Serial_SendByte` | 1 字节 = 1.04 ms @ 9600 / 0.087 ms @ 115200 | 逐字节 `HAL_MAX_DELAY` 阻塞 |
| `App_UpdateTelemetry` | 20 字节 @ 100 ms 周期 | 115200 下 1.74 ms，占 1.7 % |
| `Panel_ShowVisionImage` | 约 8 ms | 已用 `APP_IMAGE_DISPLAY_DIV=3` 节流到 1/3 |
| `show_bt_state()` | 带超时等待 | **禁止在实时路径调用**（当前未调用） |

## 11.6 其它

| 项 | 说明 |
|---|---|
| `Pid_Update(dt ≤ 0)` | **直接返回 0**（输出占空比 0 = 滑行），不是保持上次值 |
| `Pid_Update` 无抗积分饱和 | 输出饱和时积分仍累积，只有 `integral_limit` 兜底 |
| `Pid_Reset(pid, initial_error)` | 会同时设置 `last_error`，用于避免重启时的微分冲击 |
| `inner_parameter.c` | 空壳，未接入 |
| `bluetooth.c` | 已实现但未接入主流程 |
| `i2c.c` | 已初始化但未使用 |
| TIM12/15 PWM 2.45 kHz | 在音频范围，可能有啸叫；需要时可把 `Period` 改回 1000（4.89 kHz，分辨率 0.1 %） |
| `code_v2.1` | 已打包为 `code_v2.1.zip`，当前活动工程是 `code_v3.1` |

---

# 12. 变更记录

## 12.1 中心线二次拟合 + 彩色可视化（本文档之后新增）

> **本节记录的是文档正文之后的一次改动，凡与 §5.1 / §5.2 / §5.3 / §5.4 / §11.4 冲突处，以本节为准。**

### 改动动机

原来的横向偏差取 `path_center[VISION_CONTROL_ROW]`，**只依赖第 60 行一个采样点**：

- 直角弯处第 60 行失效 → 整帧判丢线 → 车在弯里被硬停
- 该判定在拐角处**会闪烁**（时有时无）→ 反复刹停/起步
- 单点噪声直接进控制环

### 新增接口

| 符号 | 位置 | 说明 |
|---|---|---|
| `#define VISION_LOOKAHEAD_ROW` | `vision_types.h` | **80**，前视行 |
| `#define VISION_FIT_MIN_ROWS` | `vision_types.h` | **8**，最少有效行数 |
| `#define VISION_FIT_OUTLIER_PX` | `vision_types.h` | **12.0f**，离群阈值 |
| `struct path_fit_s` + `extern path_fit` | `vision_types.h` / `scan_line.c` | 拟合结果 |
| `extern path_center[]` / `path_valid[]` | `vision_types.h` / `Element_recognition.c` | 原来是 static，现改为全局供绘图用 |
| `static inline Vision_EvalPathFit(row)` | `vision_types.h` | 任意行求值，拟合无效返回 `VISION_IMG_CX` |
| `void Vision_FitPath(void)` | `scan_line.h` / `.c` | 加权最小二乘二次拟合（两遍法） |
| `float Vision_GetFitX(float row)` | `scan_line.h` / `.c` | 求值包装 |
| `float Vision_GetLateralErrorLookaheadNorm(void)` | `Element_recognition.h` / `.c` | 前视行偏差（实验用，当前未被调用） |

### 变更的语义

| 接口 | 旧语义 | 新语义 |
|---|---|---|
| `Vision_IsPathValid()` | `path_valid[60]`（单行） | `path_fit.valid`（≥8 行有效） |
| `Vision_GetLateralError()` | `path_center[60] − 94` | `path_fit.x_ctrl − 94`（可外推） |
| `Vision_GetLateralErrorNorm()` | 同上 ÷ 94 | 同上 ÷ 94（量纲不变，`yaw_kp` 不用重调） |
| `Vision_ComputePathCenter()` | 控制误差的数据源 | **仅服务于绘图**（填 `path_center/path_valid`） |

### 调用顺序（`Vision_ScanLine()` 内部）

```
Vision_ScanLine()
├── 扫线填 lineinfo[]
└── Vision_FitPath()            ← 自动调用，填 path_fit
        ↓
Vision_ComputePathCenter()      ← 填 path_center / path_valid（供绘图）
        ↓
Vision_DrawEdges()              ← 画二值图 + 边线 + 中心线 + 拟合曲线 + 十字
```

### 拟合模型与算法

```
t    = (row − 60) / 60          ∈ [-1, +1]     （归一化，保证数值条件良好）
x(t) = a + b·t + c·t²
权重  w = 1/(1 + 0.6·|t|)                      （端点保留 62.5%）
```

两遍法：第一遍解 3×3 正规方程 → 算残差 → `|残差| > 12px` 的行权重清零 →
第二遍重解。第二遍若内点不足则保留第一遍结果。

### 显示配色（`Binarization.c` 的 `Vision_DrawEdges`）

`imo` 值经 `show_ov2640_image_int8()` 映射：`0`→深灰、`1`→绿、`2`→红、`3`→蓝、其它→白。

| 内容 | imo 值 | 颜色 | 粗细 |
|---|---|---|---|
| 二值图底色 | 0 / 255 | 深灰 / 白 | — |
| **左右道路边线** | 1 | **绿** | 加粗 3px |
| **道路中心线** | 2 | **红** | 加粗 3px |
| **二次拟合曲线** | 3 | **蓝** | 加粗 3px |
| 目标参考十字（主点列 × 控制行） | 2 | 红 | 细 1px |
| **主点十字**（拟合曲线在控制行的位置） | 2 | **红** | 加粗 + 臂长 7px |

> 注意：**右边界颜色由原来的红改为绿**，让红色专属于"中心线 / 主点"。
> 同时修正了原实现里十字标记未做上下翻转的 1 行错误。

### 验证

`tests/test_path_fit.c`（PC 端，`ALL PASS`）：

| 用例 | 结果 |
|---|---|
| 无噪声合成曲线 | 系数误差 < 0.11 px |
| ±1 像素噪声 | 系数误差 < 0.09 px |
| 每 7 行一个 +45 px 离群点 | 110 行 → 剔除 13 行 → 97 行，误差 < 0.13 px |
| **控制行 50~70 行全丢线** | **仍输出 `x_ctrl`，误差 0.05 px** |
| 只有 5 行有效 | `valid = 0`，`x_ctrl` 回退到图像中心 |
| `GetFitX` vs `EvalPathFit` | 完全一致 |

编译方式：

```bash
x86_64-w64-mingw32-gcc -std=gnu11 -Wall -Wextra \
  -I camera_process/Inc -I Base/Inc \
  -o test_path_fit.exe tests/test_path_fit.c camera_process/Src/scan_line.c -lm
```

### 资源变化

| 项 | 改动前 | 改动后 |
|---|---|---|
| FLASH | 88 856 B (67.79 %) | **90 880 B (69.34 %)** |
| DTCMRAM | 58 904 B (44.94 %) | **60 016 B (45.79 %)** |
| 计算开销 | — | ≈ 2300 次浮点运算 / 帧（< 10 µs @480 MHz） |

### 尚未处理

- `app.c` 丢线仍走 `Chassis_Stop()`（硬停 + 滑行），未采用柔和停车
- `yaw_output_ratio = 0.5` 导致最小转弯半径 180 mm，真 90° 弯（90~120 mm）仍转不过来
- `Vision_GetLateralErrorLookaheadNorm()` 已提供但未接入 —— 接入需同步下调 `yaw_kp`

## 12.2 门限方案修正：从自适应倍数改为固定 25 px

> §12.1 里写的"两遍法（用二次曲线自己的残差筛）"已被 §12.2 取代，以本节为准。

### 为什么要改

§12.1 的实现用**二次曲线自己的残差**筛坏行。实测发现这是**掩蔽（masking）**：
一段连续坏行（直角弯的垂直赛道、阴影、缺口，通常 10~20 行）会把第一遍曲线"拉弯"，
曲线恰好穿过它们、残差反而变小，于是筛不掉 —— 表现就是"曲率非常大的抛物线"。

### 试过的方案与实测结论

| 方案 | 结果 |
|---|---|
| 用二次曲线自己的残差（§12.1） | ✘ 掩蔽，连续坏块一个都筛不掉 |
| 用直线残差 + `3 × 平均\|残差\|` | ✘ 平均残差被坏行自己抬高（0.8→12），门限涨到 36，坏块漏过 |
| 用直线残差 + `K × **中位数**\|残差\|` | ✘ 中位数不被稀释了，但**没有任何 K 能同时满足两个场景**（见下表） |
| **用直线残差 + 固定 25 px** | ✔ 两个场景都有余量 |

**为什么自适应倍数不可能成功**（实测数据）：

| 场景 | 残差中位数 | 合法弯最大残差 | 坏块残差 |
|---|---|---|---|
| 强弯(c=34) + 12 个稀疏离群点 | ≈ 6 | **19** | 45 |
| 直道 + 21 行 40px 连续坏块 | ≈ 11 | 11 | **32** |

- 要保住强弯的行 → 门限必须 **> 19** → `K > 3.2`
- 要抓住坏块 → 门限必须 **< 32** → `K < 2.9`

两个要求方向相反，**不存在可用的 K**。固定 25 px 则 19 < 25 < 32，两边都有余量。

### 最终实现（`scan_line.c` 的 `Vision_FitPath`）

```
1. 收集有效行 + 权重 w = 1/(1+0.6|t|)
2. 加权**直线**拟合 x = a + b·t                    ← 2×2，坏块无法把直线"弯折"
3. 缓存每行对直线的绝对残差
4. 残差 > VISION_FIT_OUTLIER_PX(25) 的行权重清零   ← 第二层防御
5. 用内点拟合**二次**曲线                          ← 3×3
   解不出就退回直线结果（c = 0）
6. |c| 夹到 VISION_FIT_MAX_SAGITTA_PX(60)          ← 第一层防御
7. 填 path_fit
```

### 参数

| 常量 | 值 | 作用 |
|---|---|---|
| `VISION_FIT_OUTLIER_PX` | **25.0f** | 直线残差门限（固定阈值） |
| `VISION_FIT_MAX_SAGITTA_PX` | **60.0f** | 曲率上限（最后一层保险） |

> **副作用（有意取舍）**：25 px 的门限把可表达的曲率限制在 `c ≈ 37` 左右
> （抛物线与直线的最大偏差 ≈ 2c/3）。超过的行会被判坏行，曲率因此**偏保守**。
> 宁可预瞄不足，也不要让荒谬的抛物线污染偏移。

### 验证（`tests/test_path_fit.c`，ALL PASS）

| 用例 | 结果 |
|---|---|
| 无噪声合成曲线 | a/b/c 误差 < 0.11 px |
| ±1 像素噪声 | 误差 < 0.09 px |
| **强弯 c=34 + 12 个稀疏离群点** | `a=94.000`、`c=34.011`、`rows_used=97` ✔ |
| **直道 + 21 行连续坏块** | **`rows_used=89`、`a=94.00`、`c=0.000`（坏块全剔除）** ✔ |
| **控制行 50~70 行全丢线** | 仍输出 `x_ctrl`，误差 0.05 px ✔ |
| 只有 5 行有效 | `valid=0`，回退图像中心 ✔ |
| 真实 c=200（荒谬曲率） | 被夹到 60，`a` 不受影响 ✔ |

### 仍然存在的极限

**"强弯 + 中等坏块"同时出现时无法分离**：强弯本身相对直线的偏差就到 19 px，
而 40 px 坏块的残差只有 21~25 px，两者重叠。任何全局阈值都无能为力。
要彻底解决只能做 **Layer 4**：把"偏移"（近场直线）和"预瞄"（远场斜率）拆成两路输出。

## 12.3 拟合内核改为"离散正交多项式 + 内积投影"

> 取代 §12.2 里的"3×3 正规方程 + Cramer 展开"。**流水线、参数、语义都没变**，
> 变的只是求解内核与退化判据。完整分析见 `docs/循迹拟合分析与正交多项式方案.md`。

### 改了什么

| 项 | 改动 |
|---|---|
| 求解方式 | 不再解 `G·[a b c]ᵀ = r`（Hankel 正规方程 + Cramer + `det` 判奇异），改为先把 `{1, t, t²}` 对加权离散内积正交化为 `p0/p1/p2`（三项递推），系数直接取投影 `c_k = <x,p_k>/<p_k,p_k>` |
| 退化判据 | 从 `|det| < 1e-6`（**有量纲**的绝对阈值）改为 `<p_k,p_k>/<p_0,p_0>`（**无量纲**相对阈值 `VISION_FIT_ORTHO_MIN_REL`） |
| 降阶 | 新增：跨度不足时二次项直接降为直线（原先靠 `det` 判奇异，实测几乎从不触发） |
| 新接口 | `Vision_FitPoly(t[], x[], w[], n, order, out)` —— 纯数学、无全局状态、可在 PC 上单独验证 |
| 新字段 | `path_fit.resid_rms`（内点 RMS）、`path_fit.t_span`（数据跨度）、`path_fit.order`（实际阶数） |
| 内存 | 采样缓存从"按行 3 个数组 + 残差数组"改为"按有效行压缩的 t/x/w"，`.bss` **−116 B** |
| 代码 | 删除 `Fit_SolveOnce()` / `Fit_SolveLineOnce()`（135 行），新增 `Vision_FitPoly()` + `Fit_ResetInvalid()`；`.text` **+224 B** |

### 结论（必须说清，避免误解）

- **估计量没变**：正交基与正规方程是同一个最小二乘解，实测系数差 < 1e-4 px。
  所以"换成正交多项式"**不会**让满幅正常帧更准。
- **它解决的是退化帧**：`<p2,p2>` 精确反映"这一阶有没有被数据撑住"。
  只剩近端 1/4 行时（直角弯典型帧），旧实现硬解二次项，`x_ctrl` 抖动 6.18 px、
  最大偏差 15.43 px；新实现自动降阶，抖动 **1.05 px**、最大偏差 **2.90 px**（200 次蒙特卡洛）。
- 只有 2 个不同行号时，旧实现因 `det≈1e-5 > 1e-6` 仍照解，`c=-91.4`、`x_ctrl` 偏 11 px；
  新实现 `order=1`、`c=0`、`x_ctrl=94.000`。

### 验证（`tests/test_path_fit.c`，11/11 ALL PASS）

| 用例 | 结果 |
|---|---|
| 1~8（原有：精确还原 / 噪声 / 稀疏离群点 / 控制行全丢 / 行数不足 / 求值一致 / 连续坏块 / 曲率钳位） | 全部保持原判据通过（`rows_used=97`、`rows_used=89`、`c=60` 等数字不变） |
| 9 满幅等价性 | 正交 vs 正规方程：a/b/c 差 < 1e-4，`order=2` |
| 10 窄跨度抖动 | 1.05 px vs 6.18 px（标准差），降阶 200/200 |
| 11 只有 2 个不同行号 | `order=1, c=0` vs `c=-91.4` |

编译：`gcc -std=gnu11 -Wall -Wextra -I camera_process/Inc -I Base/Inc -o test_path_fit.exe tests/test_path_fit.c camera_process/Src/scan_line.c -lm`

### 尚未处理（仍以 §12.2 与 `循迹拟合分析…md` §5 为准）

`slope_ctrl` / `x_look` 仍未被控制器使用；`yaw_output_ratio=0.5` 导致最小转弯半径 180 mm；
扫线层的边线配对与压缩循环问题未动。这些都是比拟合更大的收益点。

---

*文档基于 `code_v3.1` 的实际代码生成，所有数值、签名、依赖关系均来自源码提取，非推测。
§12.3 基于 `code_v3.1` + 正交拟合改动，数值来自 `tests/test_path_fit.c` 实跑输出。*
