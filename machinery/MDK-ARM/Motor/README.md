# 电机库与 CubeMX 设置

自定义库只保留 `motor.c`、`motor.h`。CubeMX 负责 GPIO、时钟、TIM2 PWM 和 TIM3/TIM4 编码器初始化；库负责启动 PWM、设置占空比和计算编码器反馈。main.c 已接入，默认两台电机停止。

## 1. 系统和时钟

- System Core → SYS → Debug：Serial Wire，释放 PA15/PB3 并保留下载调试。
- System Core → RCC → HSE：Crystal/Ceramic Resonator。
- Clock Configuration：HSE 8MHz，PLL ×9，SYSCLK/HCLK 72MHz，APB1 /2，确认 APB1 timer clocks = 72MHz。

## 2. TIM2：两台电机的 PWM

Timers → TIM2：勾选 Internal Clock，Channel1～4 分别选择 PWM Generation CH1～CH4。

| 参数 | 设置 |
|---|---|
| Prescaler | 0 |
| Counter Mode | Up |
| Counter Period | 3599 |
| Clock Division | No Division |
| PWM Mode | PWM mode 1 |
| 各通道 Pulse | 0 |
| 各通道 Output Polarity | High |

72MHz / (0+1) / (3599+1) = 20kHz。确认实际引脚如下，不要使用 TIM2 默认 PA0/PA1：

| 引脚 | 功能 | 原理图连接 |
|---|---|---|
| PA15 | TIM2_CH1 | CN1 的 AT8236 IN1 |
| PB3 | TIM2_CH2 | CN1 的 AT8236 IN2 |
| PA2 | TIM2_CH3 | CN2 的 AT8236 IN2 |
| PA3 | TIM2_CH4 | CN2 的 AT8236 IN1 |

在 Pinout 上为 PA15/PB3 指定上述功能。生成代码应包含 `__HAL_AFIO_REMAP_TIM2_PARTIAL_1()`。

## 3. TIM3/TIM4：霍尔编码器输入

分别打开 TIM3、TIM4，Combined Channels 选择 Encoder Mode。CH1/CH2 由该模式占用，不单独选 PWM；不要选择 Internal Clock 作为编码器计数源。

| 参数 | 两个定时器均设置 |
|---|---|
| Encoder Mode | Encoder Mode TI1 and TI2 |
| Prescaler | 0 |
| Counter Period | 65535 |
| Clock Division | No Division |
| IC1/IC2 Polarity | Rising Edge |
| IC1/IC2 Selection | Direct TI |
| IC1/IC2 Prescaler | DIV1 |
| IC1/IC2 Filter | 6 |

TIM3：PA6=CH1、PA7=CH2；TIM4：PB6=CH1、PB7=CH2。GPIO 为输入。当前生成代码是 No Pull；若实际霍尔输出为开漏且电机板没有上拉，需要在 GPIO Settings 设置 Pull-up（上拉到 MCU 3.3V）。
TI1 and TI2 使用 A/B 两相解码，四倍频计数；这里的 Rising Edge 是输入极性设置，不代表只计每相上升沿。
本方案轮询计数，不需要开启 TIM2/3/4 的 NVIC 中断或 DMA。

## 4. 生成与接入

点击 Generate Code，确认 main.c 中按顺序调用：

```c
MX_GPIO_Init();
MX_TIM2_Init();
MX_TIM3_Init();
MX_TIM4_Init();
```

Project Manager → Code Generator 勾选保留用户代码（Keep User Code when re-generating）。
本工程已经生成上述正确配置；库的调用放在 USER CODE 区域：

```c
#include "motor.h"
static Motor_HandleTypeDef motorA, motorB;
static Encoder_State encoderA, encoderB;

/* MX_TIMx_Init 之后，实际 main.c 中还检查了返回值 */
Motor_Init(&motorA, &htim2, TIM_CHANNEL_1, TIM_CHANNEL_2);
Motor_Init(&motorB, &htim2, TIM_CHANNEL_4, TIM_CHANNEL_3);
HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);
HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL);
Encoder_Reset(&encoderA, (uint16_t)__HAL_TIM_GET_COUNTER(&htim3));
Encoder_Reset(&encoderB, (uint16_t)__HAL_TIM_GET_COUNTER(&htim4));
```

主循环已经每隔至少 10ms 读取一次 TIM3/TIM4 计数并调用 Encoder_Sample，使用实际经过的毫秒数。

## 5. 使用接口

```c
Motor_SetSpeed(&motorA, 300);  /* 正方向 30%，不是 300 RPM */
Motor_SetSpeed(&motorB, -300); /* 反方向 30% */
Motor_Stop(&motorA);          /* 滑行停止 */
Motor_Brake(&motorB);         /* AT8236 刹车 */
```

占空比参数范围 -1000～1000，超出限幅；0 停止。运动中换向前先停止，等待负载降速。初始化本身不让电机转动。所有函数在同一个主循环上下文使用。

`encoderA/encoderB` 保存反馈：delta=本次增量，position=累计计数，counts_per_second=计数/秒，rpm=标定后的输出轴转速，valid=本次采样是否有效。
默认 counts_per_turn=0，表示未标定，rpm 的 0 没有测量意义。获得实际 CPR 后，在启动测量前设置 `encoderA.counts_per_turn = cpr;`，B 同理；需反转反馈方向时设置 sign=-1。
输出轴 CPR 是四倍频后每圈计数。可以缓慢转动输出轴数圈进行测量；不要给所有 N20 假定同一个减速比或 CPR。

每次采样变化应小于 32768 计数。间隔超过 100ms、恰好半个计数范围或累计溢出时，该次采样无效；超时丢失区间不计入累计位置。避免长时间阻塞主循环，读取反馈时检查 valid。

## 文件和验证

- 参与编译的自定义库：motor.c、motor.h。原 n20/encoder 文件及过时接入脚本已移至 build/motor-simplify/old-library 备份。
- main.c 修改前备份：build/motor-simplify/main.c.before。
- tim.c/tim.h 是 CubeMX 定时器代码；stm32f1xx_hal_tim.c、stm32f1xx_hal_tim_ex.c 是 ST 官方 HAL，继续保留。
- ARMCC 全工程 21 个源文件编译链接通过；驱动与编码器逻辑测试通过。没有烧录或电机实测。
- 测试文件在 Motor/tests，仅用于电脑测试；其模拟 HAL 头文件不能加到固件包含路径。
- 构建检查：`powershell -NoProfile -ExecutionPolicy Bypass -File Motor/build_check.ps1`。

电机资料：[立创 N20 霍尔编码器电机](https://wiki.lckfb.com/zh-hans/tjx-tms320f28p550/module/control/n20-hall-encoder-motor.html)。本板 CN1/CN2 第 1、6 脚为 AT8236 电机功率输出，第 2、5 脚为编码器电源和地，第 3、4 脚为编码器反馈。实际线束须按信号核对脚序，VIN 应匹配所用电机额定电压。
