# 7.6直道提速完赛版

在当前工作区的 `DebugRingFastLimit2` 上修改；已发布的 `20261114-7.6完赛版` 和 `D:\7.6完赛版` 保持原样。本次按用户要求发布 GitHub 并复制本地完整工程，不操作车辆。

普通直道启动通过原有运行许可、完整舵机PWM帧以及新鲜有效编码器检查后，直接输出原有500/1000启动基准，跳过原先约500ms的10/1000逐步爬升。软件确认该输出实际施加后，现有轮速PI接管并按本档7.6圈/秒目标加速。保留PI输出变化限幅、最高输出限制及原有停止保护；实际轮速不会瞬间达到7.6圈/秒。

仅在 `CARTEST_RING_TUNE == 4`、普通直道阶段且无减速请求、无环岛/绕障接管时启用。普通弯道、环岛、绕障、丢线保持和另外两个档位的启动逻辑不变。出弯恢复直道的原有800ms速度目标过渡保持不变。

修改实现：`Src/main.c` 的 `prepare_follow_motor_targets()`。回归脚本：`tests/straight_start/run.py`，提取实际生产函数，用模拟寄存器检查实际PWM提交、PI接管、停车和各项排除条件。三个档位共48项断言通过。已完成真实ARM交叉构建；未烧录、未实车验证。

固件：`build/DebugRingFastLimit2/CarTest.elf`。相机程序及标定不需要更新。


Current ordinary bend target: 4.4 rps. Preview gain/limit: 0.45/120. Ring: outer 7.6 rps, yaw 282 deg/s, entry straight 700 ms, exit angle 280 deg.
Known limitation retained: wireless START does not start complete runtime/PI history logging. Actual calibration is included locally and omitted from public checkpoints.
