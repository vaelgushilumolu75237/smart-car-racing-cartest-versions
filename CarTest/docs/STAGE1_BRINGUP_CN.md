# Stage 1：当前软件与硬件验证边界

> **历史版本操作说明（2026-10-11 保护版已取代）**：本页所列的 DebugBench/DebugFollow 操作对应事故前输出配置，不适用于当前版本或本次事件后的带电验收。当前 Bench/Follow 电机输出均锁定为 0；PA8 波形检查时舵机 5 V 和电机 VM 必须物理断开。请以[当前单边界与舵机保护验收说明](STEERING_SINGLEEDGE_BRINGUP_CN.md)为准，不要照本页启动电机或舵机机构。

## 当前软件配置

- `Debug`：电机和舵机 PWM 均关闭，用于上电和通信观察。
- `DebugSteering`：三帧新鲜有效视觉数据后自动按视觉偏差驱动舵机；不驱动电机，PE7 不作普通启停。无效视觉、超时或故障会关闭舵机输出。操作和只读变量见[舵机说明](S20F_MEASURED_RANGE_CN.md)。
- `DebugBench`：仅作架空单电机检查；必须保证驱动轮离地。输出斜坡到 40% 后保持 5 秒，正常总时长约 5.4 秒，绝对上限 5.5 秒。
- `DebugFollow`：`CARTEST_FOLLOW_CALIBRATION_COMPLETE=1`，按 PE7 手动启动，视觉软故障自动恢复，最大 duty 40%，没有 5 秒运行时限。必须先完成[Follow 架空恢复检查](FOLLOW_BRINGUP_CN.md)，再进行短程普通直道/弯道验证。

当前构建命令、ELF 位置和 VS Code 启动项见[工程 README](../README.md)。OpenMV MicroPython 程序须单独上传，协议说明见[视觉协议文档](../../../../docs/vision-protocol-v1.md)。PE7 对 FOLLOW/BENCH 的启动、停止和硬故障确认语义见[FOLLOW 恢复边界](BASIC_FOLLOW_RECOVERY_CN.md)；此次逐步测试操作见[Follow 恢复操作说明](FOLLOW_BRINGUP_CN.md)。

## 验证边界

主机测试或交叉编译不能代替 H743 烧录验证、UART 电气验证、PE7 输入确认、舵机波形测量、架空电机检查或赛道测试。`DebugSteering` 的软件状态变量也不能证明实际引脚波形或机械动作。没有对应的现场记录时，相关项目应记为“未测”。

赛道元素、运行一圈后的停止条件以及自主运行要求以[规则复核记录](../../../../docs/rules-review-2026-09-08.md)和原始规则 PDF 为准；当前固件没有完成赛道循迹或完整比赛流程验证。

2026-10-09 前的完整 Stage 1 操作稿已保存在修改前快照：`../test-data/steering-cleanup-20261009/before/files/docs/STAGE1_BRINGUP_CN.md`。当时的构建与测试资料仍在各自的 `test-data` 历史报告中；本文不再列出已经移除的 profile 命令。
