# 电机库与 CubeMX 设置

当前入口：`CSGO(CN1速度百分比, CN2速度百分比)`，范围0～100；零停止。不是RPM，也不是PWM占空比。PID参数与100%参考速度已集中到 `pid.c`；详见 [PID说明.md](PID说明.md)。四字符串口已接入，见 [串口通信.md](../串口通信.md)；历史实测见 [PID.md](../PID.md)。

## 当前硬件配置

- SYS Debug=Serial Wire，保留SWD、释放PA15/PB3；HSE8MHz、PLL×9、SYSCLK72MHz、APB1/2，TIM2时钟72MHz。
- TIM2：内部时钟、向上计数、PSC19、ARR3599，即1kHz；四路PWM mode1、active high、初始Pulse0，TIM2 Partial Remap1。
- TIM3/TIM4：Encoder TI1 and TI2、PSC0、ARR65535、输入Direct TI、DIV1、Rising极性、IC1/IC2 Filter6；PA6/PA7/PB6/PB7均上拉输入。四倍频计数，不是只计单相上升沿。
- `Core/tim.c` 与 `.ioc` 已同步上述定时器和上拉配置。USART1当前由自定义motor_uart.c初始化，不在CubeMX中重复生成。

| 信号 | MCU引脚 | 用途 |
| --- | --- | --- |
| TIM2_CH1 / CH2 | PA15 / PB3 | CN1的AT8236 IN1 / IN2 |
| TIM2_CH4 / CH3 | PA3 / PA2 | CN2的AT8236 IN1 / IN2 |
| TIM3_CH1 / CH2 | PA6 / PA7 | CN1编码器A/B |
| TIM4_CH1 / CH2 | PB6 / PB7 | CN2编码器A/B |
| USART1_TX / RX | PA9 / PA10 | U9第3/4脚；TX网名USART2_TX为误标 |

正常电机输出使用快衰减：IN1 PWM/IN2低为正向，反之为负向；两输入低为滑行，两输入高为AT8236刹车。电机VIN与编码器3.3V必须正常供电，不能仅依靠下载器或USB-TTL。

## 主程序与工程

main.c顺序执行CubeMX GPIO/TIM2/3/4初始化、Motor_SystemInit、Motor_UARTInit、CSGO(0,0)。主循环执行Motor_UARTPoll，它内部负责Motor_Update。上电不运动，接收有效指令才启用闭环。

EIDE源文件包括motor.c、pid.c、motor_command.c、motor_uart.c和HAL UART驱动，编译宏包含HAL_UART_MODULE_ENABLED。Keil源文件/宏同步添加；当前实际验证采用EIDE+ARMCC5，原uVision工程其他旧驱动绝对路径未在本次统一迁移。CubeMX重生成后须保留USER CODE并检查自定义文件、Motor包含路径、HAL UART宏。

不要同时生成另一个USART1初始化/IRQ/回调；若改成CubeMX管理串口，应迁移motor_uart.c的适配层并确保单一硬件所有者。不要让USART2占用PA2/PA3，也不要重映射USART1到PB6/PB7。

## 常用接口

- `CSGO(50,0)`：CN1为满量程的50%，当前3000计数/秒；CN2停止。
- `CSGO(0,0)`：两路停止并退出闭环。
- `Motor_SetTargetsRPM`：旧CSGO的RPM接口；必须标定真实CPR并换算增益。
- `Motor_SetDutyPercent(30,30)`：手动30% PWM，退出两路PID。
- `Motor_SetSpeed(&motorA,300)`：手动PWM千分比，即30%，不是速度。
- `Motor_Stop/Brake/DeInit`：停止、刹车、释放资源。

所有电机API由一个主循环上下文调用。首次使用自定义句柄前清零，底层初始化要求TIM2/3/4、向上内部计时、ARR1～65534、两个不同且空闲的PWM通道。

encoderA/B提供delta、64位累计position、counts_per_second、rpm与valid。CPR为输出轴每圈四倍频计数；CPR=0表示未标定，rpm的0没有测量意义。不要为所有N20假设同一减速比或CPR。

## 验证与限制

运行 `.\Motor\tests\run_msvc.cmd`：五组主机测试通过；EIDE+ARMCC5全工程构建通过。本轮未烧录/实物UART联调。历史单路正向3000、6000计数/秒实测不能代替低速、双路同时或长期工况的验收。

速度比例的100%参考为6000计数/秒，PID最高输出30% PWM。串口失联500ms、已有非零PWM无反馈300ms、非法帧或队列溢出都会停机。故障后串口先发0000再运动。软件超时不是硬件急停、独立看门狗或电流保护。
