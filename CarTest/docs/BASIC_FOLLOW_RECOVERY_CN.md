# DebugFollow 启动、视觉保持与恢复

**当前软件行为（2026-10-13）：** 本页说明本机当前 DebugFollow 源码；尚未由本轮实车验证。输出许可开启只表示固件允许尝试输出，不代表 PWM、电机或车辆已通过验收。

## PE7 手动切换

PE7 低有效。上电后先保持高电平（释放）至少 20 ms。第一次按下沿锁存 `manual_run_request`；按键松开会在高电平稳定 20 ms 后重新武装，但 Follow 继续运行。只有下一次实体按下才取消请求并停车。视觉帧不会创建启动请求。等待相机、反向 KEEP、首个完整舵机 PWM 周期或电机输出准备时，`car_start_pending=1`；它在会话和首个双侧软件输出都 ready 后清零。

## 视觉保持和恢复

仅当已有手动 Follow 会话、首 PWM 周期完整、舵机输出有效且左右电机软件占空比都非零时，新的 CRC/格式正确、序号递增的 `valid=0` 帧会启动视觉保持。舵机脉宽和左右占空比保持为进入时的实际软件应用值，最长 1000 ms，从最后一帧可信有效路线计时。连续三帧新鲜有效路线可恢复。

到达 1000 ms 后输出归零，但手动请求、Follow 会话和 RAM 日志继续等待路线；此时 `car_vision_hold_elapsed_ms=1000`，`car_run_block_reason=11`。之后三帧有效路线也必须重新经过首 PWM 和电机斜坡门。没有新鲜通信帧达到 250 ms、UART/CRC/格式/序号故障仍走独立停车处理，不会延长视觉保持。一次成功保持恢复只让 `car_auto_recovery_count` 增加一次。

再次按 PE7 停车会取消本次自动恢复资格并清理视觉状态；硬故障也会清除手动请求。相机不能清除硬故障，也不能自动启动车辆。不要用 Live Watch 写变量模拟按钮、清故障或启动输出。

## 只读观察

可观察 `car_pe7_raw_high`、`car_button_armed`、`car_manual_run_request`、`car_button_press_event_count`、`car_button_start_event_count`、`car_button_stop_generation`、`car_start_pending`、`car_follow_session_active`、`car_mode`、`car_fault`、`car_run_block_reason`、`car_drive_allowed`、`car_vision_hold_active`、`car_vision_hold_elapsed_ms`、`car_vision_hold_entries`、`car_vision_hold_exit_cause` 和 `car_auto_recovery_count`。这些值是软件状态，不是引脚波形或车辆运动测量。实际逐步检查见[Follow 架空恢复与操作说明](FOLLOW_BRINGUP_CN.md)，RAM 日志冻结和导出条件见[日志导出说明](RUNTIME_LOG_EXPORT_CN.md)。

Bench 仍采用按住运行、释放停止；DebugSteering 不使用 PE7 作普通启停。保持旧版本的报告、基线和旧源码快照未修改。
