# FollowTurn 编码器稳速 PI 试验档

VS Code 启动菜单中的 **PI试验档 - 编码器稳速（日志开启）** 使用独立 preset `DebugFollowSpeedPI`，ELF 为 `build/DebugFollowSpeedPI/CarTest.elf`。现有三个中文差速档和 DebugFollowTurn 仍为原来的开环速度控制；只有这个新档启用 PI。

从项目根目录构建：

```powershell
.\tools\build.ps1 -Preset DebugFollowSpeedPI -Fresh
```

## 控制参数

- 直行目标为每侧 2400 milli-RPS（144 RPM）。转向时按原 FollowTurn 的左右名义占空比同比缩放目标速度；前馈也取自保护后的舵机实际应用脉宽所算出的左右名义目标。
- 试验初值为 `Kp=13107 Q16`（约 0.2 permille/(milli-RPS)）、`Ki=26214 Q16`（约 0.4 permille/(milli-RPS·s)），回算跟踪时间 200 ms。
- PI 请求范围为每侧 0–700‰。每 10 ms 最多向请求值变化 20‰，保持既有 FollowTurn 限速。超速时允许请求降到 0‰；只在 PI 已武装、会话和独立输出保护仍有效时，0‰ 才保持 PWM 通道和资格状态，以便下一次有效编码器采样从实际 0‰ 按 20‰ 步进恢复。
- 轮速目标取自现有平路日志，增益为试验初值，尚未通过实车调参验证。

## 启动、保持和故障

原有启动资格、手动 PE7 会话、舵机首个完整帧和电机从 0 到 500‰ 的启动斜坡照常生效。确认启动资格的周期保持原 500‰ 命令，PI 到后续新鲜有效编码器样本才武装。控制器不在主循环重复积分同一个样本，也不追补错过的采样周期。

有效 PI 运转后，编码器反转、计数/时钟错误、300 ms 零计数停转或速度样本超过 100 ms，会锁存编码器初始化故障并立即关闭输出，不退回开环加速。PI 尚未武装、观察器仍处于正常 `NOT_READY` 时，可等待至观察器最新时间戳后 100 ms；达到 101 ms 仍未就绪即故障停车。观察器明确报告配置、反转、计数、时钟或停转错误则立即停车。独立的 UART、手动停止、会话、道路许可、舵机和电机输出保护继续拥有最终停车权限。

进入已保护的视觉保持后，舵机和两侧实际电机命令保持进入时的精确值，冻结积分和速度更新。恢复后先清除旧样本的跟踪历史，等待新样本再计算；1000 ms 保持到期仍沿用原停车与重新资格流程。

Live Watch 的 `car_speed_pi_compiled`、`car_speed_pi_actuation_enabled`、`car_speed_pi_status`、`car_speed_pi_target_*_millirps`、`car_speed_pi_requested_*_permille`、`car_speed_pi_integral_*_q16` 与 `car_speed_pi_saturated_*` 是只读诊断快照。它们表示软件状态，不是引脚、驱动电流或车轮测量。

板上导出时必须用与本次运行相符的 ELF 解码日志：

```powershell
powershell.exe -NoProfile -ExecutionPolicy Bypass -File .\tools\export_run_log.ps1 -Elf .\build\DebugFollowSpeedPI\CarTest.elf
```

编译和主机测试只能证明软件路径。它们不证明 ELF 已烧录，也不证明传感器比例、PWM、电机、台架响应或实车增益安全；该试验档仍需在独立的物理检查和受控试车后评估。
