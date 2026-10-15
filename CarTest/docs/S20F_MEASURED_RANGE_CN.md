# 舵机配置与 DebugSteering 操作

> **历史版本操作说明（2026-10-11 保护版已取代）**：本页记录的是事故前舵机供电操作，不适用于本次事件后的带电机械验收。当前只有 DebugSteering 可输出供 PA8 数字波形检查的舵机信号，测波形时舵机 5 V 与电机 VM 必须物理断开；DebugBench/DebugFollow 的电机输出锁定为 0。请改用[当前单边界与舵机保护验收说明](STEERING_SINGLEEDGE_BRINGUP_CN.md)，不得按本页旧步骤给舵机上电或做机械端点测试。

## 当前配置

| 位置 | 配置脉宽 |
| --- | ---: |
| 左 | 1100 μs |
| 中位 | 1530 μs |
| 右 | 2500 μs |

用户确认这组配置平顺并留有余量。P0.7 映射把归一化视觉误差 -1000、0、+1000 分别转换为 1229、1530、2209 μs。OpenMV 程序和视觉协议不变。配置值与映射值不代表机械止挡已经测定；H743 引脚波形、掉电位置和全机械行程仍需现场记录。

## 运行 DebugSteering

1. 电机 VM 保持物理断开，前轮架空并清空转向机构周围障碍。舵机使用已核对的电源和共地。
2. 从 CarTest 根目录运行 `powershell.exe -NoProfile -ExecutionPolicy Bypass -File .\tools\build.ps1 -Preset DebugSteering -Fresh`，或在 VS Code 选择 `CarTest H743 - DebugSteering (motors off; vision auto-steering)`。目标文件是 `build/DebugSteering/CarTest.elf`。
3. 启动 OpenMV。主控收到三帧新鲜有效视觉数据后自动开始按每个新偏差更新舵机。PE7 不参与本 profile 的普通启停；释放 PE7 不会停止舵机。
4. 只读观察 `car_mode`（运行时为 11）、`car_steering_session_active`、`car_steering_servo_target_us`、`car_servo_applied_pulse_us`、`car_servo_output_active`、`car_fault`、`car_vision_valid` 和 `car_vision_timeout`。运行期间 `car_drive_allowed` 必须为 0；`car_motor_left_in1_output_permille`、`car_motor_left_in2_output_permille`、`car_motor_right_in1_output_permille`、`car_motor_right_in2_output_permille` 必须一直为 0。
5. 视觉无效、超时或故障时，舵机输出应关闭。可恢复的视觉中断后需收到三帧新的有效数据，再自动恢复跟随。硬故障来源消失后仍需按固件要求用实体 PE7 确认；视觉帧不能清除硬故障。
6. 普通停止请停止 OpenMV 输出；需要立即停止舵机供电时，断开舵机电源。PE7 释放不是此 profile 的停止操作。不要通过 Live Watch 写变量控制舵机。

这些 Live Watch 项是软件状态快照，不能替代引脚示波器测量或前轮机械观察。出现卡滞、干涉、持续嗡鸣、异常抖动或电机输出非零时，立即切断舵机电源并停止测试。

## 历史测量

2026-10-07 用户曾报告在 5 V、前轮架空、连杆连接、VM 断开时，1500 μs 向左、1530 μs 居中、1550 μs 向右，动作平顺。该记录保留为当时的短行程观察，已由上方当前配置取代，不是当前编译范围。旧单脉冲试验的构建和主机证据保留在 [2026-10-09 试验报告](../test-data/servo-1400-1660-20261009/BUILD_TEST_REPORT.md) 与 [此前试验报告](../test-data/servo-trial-20261008/BUILD_TEST_REPORT.md)；这些旧 profile 已从当前 CMake preset、build task 和启动菜单移除。

编码器倍率历史值和三圈累计读数保留在[编码器输入诊断记录](ENCODER_INPUT_DIAG_CN.md)。当前 DebugFollow 标定已启用；基本循迹启停及视觉中断恢复步骤见[Follow 恢复操作说明](FOLLOW_BRINGUP_CN.md)。本文件的舵机专项记录不表示 Follow 的实车循迹已验证。
