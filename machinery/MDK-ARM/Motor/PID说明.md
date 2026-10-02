# PID 与速度接口

## 参数统一放在 pid.c

`Motor_MeasuredCountsPID` 定义在 `pid.c`、声明在 `pid.h`：Kp=0.05、Ki=0.4、Kd=0，输出及积分范围0～300 PWM千分比，derivative_tau=0.03s。速度输入为四倍频编码器计数/秒；Ki在算法内乘实际dt，不要再预乘10ms。

`Motor_FullScaleCountsPerSecond[2]` 同样在 pid.c，当前两路均6000计数/秒。它是百分比控制的100%参考速度，来自已测试工作点，不是最大速度的物理标定。修改范围后需要重新验证可达性和响应。

实测数据见 [PID.md](../PID.md)。正式工程已把TIM2改为1kHz（时钟72MHz、PSC19、ARR3599），编码器输入上拉，sign=+1；快衰减、最大30% PWM。此前只验证单路正向3000/6000、每轮2秒，未证明其他速度、双路同时或长期运行性能。

## CSGO 现在表示速度百分比

按用户最新要求，`CSGO(first, second)` 接受两个有限的0～100浮点数：

```text
速度百分比 = 期望计数速度 / 满量程计数速度 × 100
目标计数速度 = 百分比 × 满量程计数速度 / 100
```

```c
CSGO(50, 0);
CSGO(0, 100);
CSGO(0, 0);
```

依次代表CN1目标3000/CN2停止，CN1停止/CN2目标6000，两路停止。这里100%速度不等于100% PWM，输出仍限30%。越界、负数、NaN/Inf或任一路未配置时返回HAL_ERROR，先校验两路再修改目标。重复同向目标保留积分。

四字符串口把两位编码00～99映射为0～100%后调用CSGO；因此串口 `3060` 不是直接调用CSGO(30,60)。编码限制、回执和接线见 [串口通信.md](../串口通信.md)。

## 初始化与主循环

当前 main.c 已接入以下顺序，上电不自动运行：

```c
if (Motor_SystemInit() != HAL_OK) { Error_Handler(); }
if (Motor_UARTInit() != HAL_OK) { Error_Handler(); }
if (CSGO(0, 0) != HAL_OK) { Error_Handler(); }
```

`Motor_UARTInit()` 配置两路计数PI并初始化USART1。主循环持续执行 `Motor_UARTPoll()`；它处理通信、调用Motor_Update并在控制故障时锁存停机，不要再重复调用Motor_Update。

不使用串口时，在Motor_SystemInit之后分别用 `Motor_PIDConfigureCounts(&motorA, 1, &Motor_MeasuredCountsPID)`、motorB对应调用配置两路，然后主循环调用Motor_Update、由应用处理其错误。所有电机API只能在同一个前台上下文调用，中断仅放入接收队列。

## 其他接口与兼容性

| 接口 | 单位及作用 |
| --- | --- |
| `CSGO` | 两路0～100%参考速度；新语义，不能再当RPM调用 |
| `Motor_SetTargetsRPM` | 旧CSGO的双路有符号RPM行为，需真实CPR与RPM参数 |
| `Motor_PIDConfigure` / `Motor_SetTargetRPM` | 配置和控制单路RPM，零CPR不允许配置 |
| `Motor_PIDConfigureCounts` | 配置计数闭环，自动停止该路、清积分、CPR置0，默认300ms无反馈保护 |
| `Motor_SetTargetCountsPerSecond` / `Motor_SetTargetsCountsPerSecond` | 直接设置有符号计数速度；无百分比缩放 |
| `Motor_SetFeedbackTimeout` | 停止闭环时设置无反馈保护，10～60000ms，0禁用；运行中返回HAL_BUSY |
| `Motor_SetDutyPercent` / `Motor_SetSpeed` | 开环PWM百分比 / 千分比，退出PID；不是目标转速 |
| `Motor_Stop` / `Motor_Brake` / `Motor_DeInit` | 关闭PID并滑行停止 / AT8236刹车 / 释放资源 |
| `Motor_PIDIsEnabled` | 查询该路是否启用闭环 |

历史调用代码需迁移：原 `CSGO(30,30)` 若意图为30RPM，现在应写 `Motor_SetTargetsRPM(30,30)`。计数模式与RPM模式的非零目标不能混用；零目标可以停止任意模式。

若以后得到真实输出轴四倍频CPR，可用 `PID_MakeRPMConfig(cpr, &config)` 按CPR/60换算Kp/Ki/Kd，输出和积分限幅保持不变，再配置RPM接口；零CPR或空输出指针返回0。当前串口百分比控制不需要CPR，不输出伪造RPM。

## 控制与故障边界

每路PID独立：位置式P/I/D、按实际dt积分、测量微分及低通滤波、条件积分抗饱和、输出/积分限幅和非有限数值检查。更新周期至少10ms，计数窗口1个计数对应约100计数/秒的量化台阶。

控制只按目标方向输出单向PWM，超速可降至零但不反向刹车。有符号接口换向必须先置零并等待负载降速；没有机械停稳检测。百分比接口只允许正向。

采样间隔超过100ms、16位计数差恰好半范围、累计溢出、CPR/sign被外部篡改或PID计算错误时，停止活动闭环。每次实际位移应小于32768计数，否则无法唯一还原回绕。

有非零PWM而持续零净计数时累计无反馈时间；达到阈值返回HAL_TIMEOUT并停止活动闭环。非零计数重置时间；超时后不自动重启。RPM模式默认禁用这项保护以兼容极低速，可按需开启。

该保护不能区分堵转、断线或低速，也不能检测所有噪声和反向反馈；没有电流保护。USART应用另有500ms失联保护与故障锁存，恢复先发0000。所有软件超时依赖主循环，正式固件未新增硬件看门狗。

实际反馈读 `encoderA/B.counts_per_second`，并检查 `valid`；未标定CPR时 `rpm=0` 没有真实RPM测量意义。串口ACK回报的是设置值，不是实际反馈。

## 验证

`.\Motor\tests\run_msvc.cmd`：MSVC C11 /W4 /WX，PID、电机、编码器、协议和UART适配五组测试通过。覆盖比例映射、参数换算、限幅、正负底层目标、单位隔离、换向限制、半帧/超长/非法帧、失联锁存、UART错误/溢出、TX缓冲寿命与背压等。

EIDE+ARMCC5全工程24个C文件、1个汇编文件构建通过。本次新固件尚未烧录；既有实机PID结果不等同于新串口控制已完成硬件验收。
