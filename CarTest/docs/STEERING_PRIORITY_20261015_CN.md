# 弯道预瞄服从当前纠偏：2026-10-15

本版相机 BUILD 为 `CACHED_WIDTH_CORRECTION_FIRST_20261015`。本次只处理普通道路上“当前路线要求向左纠偏、远端弯曲预瞄却要求向右”的冲突。

## 控制规则

- 位置误差仍由现有通过拓扑、边界和缓存检查的行数据计算。画面下方 70% 区域占权重 0.8，上方 30% 区域占权重 0.2，中心语义不变。
- 三窗口弯曲检查通过后，原始候选量保存在 `preview_candidate`，实际采用量保存在 `heading_contribution`。在合成普通道路控制量前，只对 `position_error * preview_candidate < 0` 的冲突预瞄处理：当 `0 < abs(position_error) < 20`，按 `abs(preview_candidate) * (20 - abs(position_error)) // 20` 淡出幅值，恢复其原符号，并记录 `preview_rejection_reason=CONFLICT_FADE`；当 `abs(position_error) >= 20` 时实际贡献归零并记录 `OPPOSES_POSITION_CORRECTION`。左右规则镜像，幅值先按正数取整再恢复符号，避免负数整除偏差。
- `position_error == 0` 时乘积为零，预瞄保持原值；同向预瞄始终保留。这样相邻位置值不会触发完整预瞄的阶跃。位置项与实际采用的预瞄贡献相加后仍限制在 ±1000。
- 按当前配置，`reference_error` 只记录同帧 y=40±4 范围内最近、已被现有行权重路径接受的中心误差；没有可用支持点或等距点互相冲突时留空。该字段仅用于复核，不参与控制或路线选择。中心 x=35.0、y=40 时相对 grid 中心 39.5 记为 -114。
- 环岛路径继续使用原来的位置控制。预瞄冲突不会改变帧有效性、路线授权、拓扑、KEEP 或宽度缓存状态，也不会触发通信复位或清缓存。

屏幕完全取消冲突预瞄时显示 `CORR_FIRST`，部分淡出时显示 `CONFLICT_FADE`，同时显示 `POS`、`A`（实际采用量）、`CAN`（候选量）和 `CTRL`；串口诊断中 `preview` 是实际量、`preview_candidate` 是候选量，另保留 `pos_weighted` 与 `raw_ctrl`。`preview_block` 标记取消或淡出。参考诊断以 `rowref` 和 `reference_error` 输出。

## 软件验证范围

主机回归覆盖方向镜像、±1/±10/±19/20/21 边界及整数淡出、同向保留、±1000 截断、连续帧无状态残留、无效帧、环岛原控制，以及 y=40±4 参考诊断（x=35.0 得 -114）。另用实际 `mpy-cross -march=armv7em` 编译生产 `openmv/` 顶层 18 个运行文件。

视频回放只代表软件对所提供图像输入的计算结果，不测量 OpenMV 板端时延、H743 接收、舵机运动或车辆过弯。首次 PWM 检查、1000 ms 丢线保持和 250 ms 通信故障停车均未修改；本轮不构建或烧录新的 H743 ELF。
