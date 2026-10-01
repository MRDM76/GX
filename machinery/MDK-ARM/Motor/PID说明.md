# PID 转速闭环使用说明

## 文件与职责

- `pid.c/pid.h`：独立于 HAL 的位置式 PID，可单独测试或用于其他控制对象。
- `motor.c/motor.h`：将 PID 输出送到现有电机 PWM，在 Motor_Update 中利用编码器 RPM 闭环。仍由 CubeMX 配置 GPIO 和定时器，没有恢复 n20 包装层。
- EIDE、Keil、构建检查脚本已加入 pid.c；包含路径仍为 Motor。

CSGO(CN1_RPM, CN2_RPM) 现在直接设置两路 PID 目标转速，支持小数。main.c 暂用 CSGO(0,0) 保持停止；完成 CPR/方向标定和增益配置后，可改为 CSGO(30,30)，表示两路目标 30 转/分钟。

## 控制过程

```text
目标 RPM → 与编码器 RPM 比较 → PID → 限幅后的 PWM → AT8236 → 电机
                 ↑                                  ↓
                 └────────── 霍尔编码器反馈 ─────────┘
```

每路有独立 PID 状态，主循环仍只需调用 Motor_Update。内部每隔至少 10ms 读取编码器，以实际经过的秒数作为 dt，计算 PWM 千分比。

PID 使用 P=Kp×误差、I 累加 Ki×误差×dt、D 为 -Kd×测量变化/dt。D 针对测量值而非目标值，目标突变不会产生微分冲击；derivative_tau 是 D 的一阶低通时间常数（秒）。
积分值独立限幅，并在输出饱和且继续积分会加重饱和时暂停该方向积分。参数和计算输入出现 NaN/Inf、无效 dt 或算术溢出时返回失败。

## 必要的实测参数

1. CPR：输出轴每转一圈的四倍频计数，与电机具体减速比有关。当前没有这个数值，不能随意填通用 N20 数字。
2. 编码器方向：低占空比正向运动时，反馈应为正；否则 Configure 的 sign 设为 -1。
3. Kp、Ki、Kd：与具体电机、负载、采样及 CPR 有关，没有未经实测即可保证稳定的固定参数。

这里输出单位为千分比：600 表示最高 60% PWM。Kp 单位为千分比/RPM，Ki 为千分比/(RPM·秒)，Kd 为千分比·秒/RPM。不能把按百分比计算的增益原样混用。

## 初始化和启用

CubeMX 初始化与 Motor_SystemInit 调用保持不变。配置在 Motor_SystemInit 成功之后执行，Configure 会先停止对应电机。以下是调用模板，`measured_cpr`、`tuned_kp/ki/kd` 等由实际标定和调参得到，不是项目中已定义的常量：

```c
PID_Config speed_pid = {
    .kp = tuned_kp,
    .ki = tuned_ki,
    .kd = tuned_kd,
    .output_min = 0.0f,
    .output_max = max_duty_percent * 10.0f,
    .integral_min = 0.0f,
    .integral_max = max_duty_percent * 10.0f,
    .derivative_tau = derivative_filter_seconds
};

if (Motor_PIDConfigure(&motorA, measured_cpr_A, encoder_sign_A, &speed_pid) != HAL_OK ||
    Motor_PIDConfigure(&motorB, measured_cpr_B, encoder_sign_B, &speed_pid) != HAL_OK)
{
    Error_Handler();
}

/* 两路可配置不同增益。目标单位为输出轴 RPM。 */
if (CSGO(target_rpm_A, target_rpm_B) != HAL_OK)
{
    Motor_PIDDisable(&motorA);
    Motor_PIDDisable(&motorB);
    Error_Handler();
}

/* while(1) 中持续调用；CSGO 在目标改变时调用即可 */
(void)Motor_Update();
```

没有标定 CPR、sign 不是 ±1、全零 P/I 参数、output_min 不是 0 或输出上限不在 (0,1000] 时，电机闭环配置失败。泛用 PID 内核允许单独 D，电机转速接口要求至少有 P 或 I。

## 对外接口

| 接口 | 行为 |
|---|---|
| Motor_PIDConfigure | 设置 CPR、方向和 PID 参数，停止选中的电机，准备闭环 |
| Motor_SetTargetRPM | 设置有符号输出轴 RPM，并启用闭环；0 退出闭环并滑行停止 |
| Motor_PIDDisable | 退出闭环、清空 PID 状态并滑行停止 |
| Motor_PIDIsEnabled | 查询该电机闭环是否启用 |
| Motor_Update | 采样并控制，HAL_BUSY 尚未到周期，HAL_TIMEOUT 采样无效，HAL_ERROR 配置/计算/输出错误 |
| CSGO | 同时设置 CN1/CN2 目标 RPM；先验证两路，任一路无效则两路原目标保持；0 停止 |
| Motor_SetDutyPercent、Motor_SetSpeed | 退出 PID，切回手动占空比；前者控制两路百分比 |
| Motor_Stop/Brake/DeInit | 关闭该电机 PID，防止下一次采样覆盖停止/刹车命令 |

motorA=CN1，motorB=CN2。PID 通过内部输出函数写 PWM，不会关闭自己。所有调用从同一个主循环上下文执行。
RPM 正负选择转向。控制器在目标方向上只输出非负 PWM 幅值；超速时减小至零，不通过反向扭矩主动刹车。运行中直接改变目标符号会返回 HAL_ERROR，原目标保持；应先目标置 0、等待负载降速，再给反向目标。

## 异常和实际边界

- 采样间隔超过 100ms、计数差恰好半范围、累计溢出等产生无效样本时，Motor_Update 停止所有当前处于 PID 模式的电机并关闭闭环；不会自动恢复。排查后重新调用目标接口才重新启用。
- 闭环中直接改 encoderA/B.counts_per_turn 或 sign 会被识别为配置变化，下一次采样关闭闭环并停止；应停止后用 Configure 重新配置。
- 每采样实际计数变化必须小于 32768，否则 16 位计数回绕无法唯一判断。
- 超时处理依赖主循环再次调用 Motor_Update；没有独立硬件看门狗，主循环完全卡死时不能承诺自动停机。
- 零计数也可能是静止、堵转或信号断线，本库不能区分，PID 可能提升到配置的占空比上限；尚未实现堵转、断线、电流保护策略。
- 未实现速度斜坡、负载识别或自动调参。当前测量是采样窗内的平均 RPM，低速会有计数量化波动。开始调试可先用 PI（Kd=0），按实际响应小幅调整增益和输出上限。

## 验证

泛用 PID 测试覆盖 P/I、不同 dt、积分/输出限幅、饱和后恢复、微分滤波、目标突变不产生 D 冲击、非法输入，以及模拟一阶对象的目标跟踪与负载变化。这些是软件数值测试，不能证明实机 N20 已调好。
电机联调测试覆盖 CPR/方向校验、两路正负 RPM 输出、超速不反转、零目标停止、手动接管、采样超时停机、CPR 被改动、毫秒计数回绕。
ARMCC 编译链接通过（22 个源文件）。未烧录或上板调参。

```powershell
gcc -std=c99 -Wall -Wextra -Werror -I Motor Motor/tests/test_pid.c Motor/pid.c -lm -o build/motor-check/test_pid.exe
.\build\motor-check\test_pid.exe
gcc -std=c99 -Wall -Wextra -Werror -I Motor/tests -I Motor Motor/tests/test_motor.c Motor/motor.c Motor/pid.c -o build/motor-check/test_motor.exe
.\build\motor-check\test_motor.exe
```
