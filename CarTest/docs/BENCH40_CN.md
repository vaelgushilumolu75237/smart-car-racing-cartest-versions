# DebugBench 双轮架空检查

> **历史版本操作说明（2026-10-11 保护版已取代）**：本页记录的是事故前 Bench 电机输出流程，不适用于当前配置或本次事件后的带电验收。当前 DebugBench 与 DebugFollow 的电机输出均锁定为 0。请改用[当前单边界与舵机保护验收说明](STEERING_SINGLEEDGE_BRINGUP_CN.md)；不要按本页启动电机或进行架空测试。

当前 DebugBench 固定同时驱动左电机 IN2（PA7/TIM3_CH2）和右电机 IN1（PB0/TIM3_CH3）。其他两路 PWM 始终关闭，舵机与 `road_drive_allowed` 保持关闭。专用选择 `CAR_MOTOR_BOTH_FORWARD=3` 只接受 `CAR_MOTOR_IN1=0` 作为合法标记；该标记不会把同一个输入映射到两个电机。

按下 PE7 并保持后，两路共享同一个输出命令：从 0 起步，每 10 ms 增加 10/1000，约 400 ms 到达 400/1000（40%）。首次达到目标时启动独立 5,000 ms 保持计时；从进入 `BENCH_RUNNING` 起有 5,500 ms 绝对上限。先到者停机，异常调度下若绝对上限先到，不保证补足 5 秒保持。主循环间隔超过 50 ms 的保护也会停机。

启动后 PE7 必须保持释放。收到第一组三帧新鲜有效视觉数据后，固件自动选择固定双轮目标并清空 RX/parser；再收到三帧新鲜有效数据、PE7 仍释放且故障和 RX 状态正常时，`car_bench_ready` 才变为 `1`。准备阶段不驱动电机。准备期间按住 PE7 不会启动；看到 ready 后，重新按下一次并保持按下才开始一次性脉冲。

1. 将左右驱动轮都可靠架空，确认轮胎和传动机构不会碰到桌面或线材。断开电机 VM，再刷写 `build/DebugBench/CarTest.elf`。H743 只刷这个 ELF；OpenMV MicroPython 程序仍需单独上传到视觉板。
2. 按现有接线给 H743 和 OpenMV 上电，保持 PE7 释放，并确认 OpenMV 正常发送视觉数据。在 VM 仍断开的状态下等待 `car_bench_ready=1`；如果没有 ready，不要按 PE7。
3. 保持 PE7 释放并再次确认左右车轮都架空后，按现有接线接通电机 VM，确认 ready 仍为 `1`，再按下一次 PE7 并保持。观察左 IN2 和右 IN1 同步斜坡到 40%、保持约 5 秒后归零。想提前停止就立即释放 PE7；两路都会快速停机，松开后不会自动重启。
4. 单独验证视觉超时：复位并重新等待 ready，按下 PE7 开始后在 IDE 中停止 OpenMV 脚本。约 250 ms 视觉超时后两路应同时归零；恢复脚本和视觉数据不会自动重跑脉冲。此一次性运行要再次测试需复位并重新等待 ready。

Live Watch 可观察 `car_mode`、`car_fault`、`car_bench_ready`、`car_drive_allowed`、`car_bench_max_applied_duty_permille`、`car_encoder_left_raw_delta_sum`、`car_encoder_right_raw_delta_sum` 和四路 `car_motor_*_output_permille`。脉冲期间 `car_motor_left_in2_output_permille` 与 `car_motor_right_in1_output_permille` 应同步变化，左 IN1 与右 IN2 应为 `0`；`car_drive_allowed` 应始终为 `0`。用户已分别验证左右轮方向：车辆前进时左编码器原始增量为正、右编码器原始增量为负。增量变量记录原始累计变化，不代表换算距离；本报告不把软件 PWM 命令当作引脚波形或机械运动测量。

40% 是此次双轮架空测试目标和硬上限，不是永久比赛限速。左轮和右轮单独测试已有现场验证记录；本次双轮组合未进行硬件上电或实测。Host 测试及四配置 Fresh 编译只证明软件逻辑、编译和链接结果。验证边界见[双轮 Bench40 构建报告](../DUAL_BENCH40_HOLD5S_BUILD_REPORT.md)。
