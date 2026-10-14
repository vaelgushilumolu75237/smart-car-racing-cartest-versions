# DebugFollow：恢复与短程循迹检查

> **当前版本状态（2026-10-14）**：Follow 软件输出许可已开启，但没有对本版进行板上验证。本页描述的软件行为，不证明 PWM、电机或车辆动作正常。PE7 稳定释放 20 ms 后，第一次实体按下锁存启动请求；松手保持运行，下一次重新武装后的按下才停止。视觉数据不能自行启动。CRC/UART/协议/通信超时和硬故障仍有独立停车路径。

本版保留既有 `DebugFollow`、`DebugFollowDiff5` 和 `DebugFollowDiff10` 行为，并新增独立 `DebugFollowTurn` 第四档 profile。旧档直线目标 500‰、正常目标上限 600‰、独立硬上限 700‰，差速为 0/50/100‰；旧档每侧电机常规步进仍为 10‰/10 ms，舵机常规限速仍为 40 µs/20 ms。第四档直线仍为 500‰，满转向软件目标为外侧 700‰、内侧 300‰（差速 200‰，平均仍为 500‰），正常与硬上限均为 700‰。新档资格建立后每侧最多步进 20‰/10 ms、舵机最多步进 60 µs/20 ms；启动及实际输出停止后的恢复斜坡仍为 10‰/10 ms 和 10 µs/20 ms。所有档位都按已提交的舵机脉宽而非视觉请求值计算差速，且延迟主循环每次只走一步、不补做多步。上述是软件目标与限速设置，尚未经过机械或车辆验证。

运行中，只有新鲜、序号递增、CRC/格式正确的 `valid=0` 帧，且舵机 PWM 和左右电机软件输出此前均有效且非零，才进入视觉保持：保持进入时的舵机脉宽和每侧占空比最多 1000 ms。到期输出归零，但手动请求、会话和运行日志继续等待；三帧连续新鲜有效路线可恢复。短暂保持期内实际输出未停止时，资格与所选档的常规限速保留；1000 ms 保持到期、250 ms 通信超时、UART/协议或硬故障实际停止输出后，恢复须重新通过启动资格并使用 10 步启动斜坡。硬故障不自动恢复，须先排除故障源并按固件流程用实体 PE7 确认。

## 差速 ELF 下拉切换与 F5

VS Code 已有四条中文行驶档菜单。按 `Ctrl+Shift+D` 打开“运行和调试”，在顶部配置下拉框选择目标档位，再按 `F5`。每项会自动运行对应 build task，并让烧录 ELF 与调试符号指向同一个文件；无需编辑 JSON，也无需先单独运行 build task。

| 档位 | 下拉菜单项 | 对应 build task 与 ELF |
| --- | --- | --- |
| 0 档，无差速，50%/50% | `行驶0档 - 无差速 50%/50%（日志开启）` | `CarTest: Build DebugFollow (motors enabled; PE7 toggle; straight 50%; hard cap 70%; diff=0; RAM log ON)` → `${workspaceFolder}/build/DebugFollow/CarTest.elf` |
| 1 档，中差速，最大 55%/45% | `行驶1档 - 中差速 最大55%/45%（日志开启）` | `CarTest: Build DebugFollowDiff5 (motors enabled; d=50; RAM log ON)` → `${workspaceFolder}/build/DebugFollowDiff5/CarTest.elf` |
| 2 档，强差速，最大 60%/40% | `行驶2档 - 强差速 最大60%/40%（日志开启）` | `CarTest: Build DebugFollowDiff10 (motors enabled; d=100; RAM log ON)` → `${workspaceFolder}/build/DebugFollowDiff10/CarTest.elf` |
| 3 档，快速转向，最大 70%/30% | `行驶3档 - 快速转向 70%/30%（日志开启）` | `CarTest: Build DebugFollowTurn (motors enabled; 70/30 turn; RAM log ON)` → `${workspaceFolder}/build/DebugFollowTurn/CarTest.elf` |

排错时注意：只运行 Diff5 或 Diff10 build task 并不会切换启动下拉框；如果当前仍选原 `DebugFollow` 项，随后按 `F5` 就会构建并烧录无差速档。检查下拉框当前选中的名称即可。

`F5` 会复位 H743 并清除之前保存在 RAM 的运行日志。若要保留上一轮记录，先结束运行并在主控仍上电时冻结、导出，再启动下一档；导出命令的 `-Elf` 要使用本表中该轮实际运行的同档 ELF。摄像头程序、路线算法和 UART 协议无需随档位变化；切档选择对应 H743 启动项即可。

## 第一次运行：全部驱动轮架空

1. 按上文从下拉框选择行驶 0、1、2 或 3 档，再按 `F5`；所选启动项会调用匹配的构建任务和 ELF。若需保留上一轮 RAM 日志，先冻结并导出后再按 `F5`。H743 固件烧录与 OpenMV MicroPython 上传是独立操作，摄像头程序无需随档位变化。构建许可只代表软件会尝试输出，不代表 PA8 引脚已有正确波形。
2. 抬起左右驱动轮。复位时保持 PE7 高电平至少 20 ms，运行 OpenMV；Live Watch 应显示 `car_pe7_raw_high=1`、`car_button_armed=1`、`car_fault=0`。只有确认准备启动后才实体按下一次；`car_manual_run_request=1` 表示请求已锁存，等待视觉/反向 KEEP/首 PWM 等门控时 `car_start_pending=1`。待 `car_follow_session_active=1`、`car_drive_allowed=1` 且两路输出就绪后，释放 PE7 不停车。Follow 不使用 `car_bench_ready`，不要等 Bench 的 ready 标志。
3. Live Watch 只读观察 `car_mode`、`car_fault`、`car_manual_run_request`、`car_button_armed`、`car_start_pending`、`car_follow_session_active`、`car_drive_allowed`、`car_run_block_reason`、`car_follow_profile_id`、`car_follow_servo_target_us`、`car_follow_motor_duty_permille`、`car_follow_diff_mode_permille`、`car_follow_inner_diff_permille`、`car_follow_diff_u_permille`、`car_follow_outer_delta_permille`、`car_follow_inner_delta_permille`、`car_follow_normal_cap_permille`、`car_follow_motor_normal_step_permille`、`car_follow_servo_normal_step_us`、`car_follow_output_phase`、`car_follow_startup_qualified`、`car_follow_base_target_permille`、`car_follow_left_target_permille`、`car_follow_right_target_permille`、`car_follow_left_command_permille`、`car_follow_right_command_permille`、`car_servo_frame_ready`、`car_servo_output_active`、`car_servo_applied_pulse_us`，以及四个 `car_motor_*_output_permille`。启动到直线 500‰ 的既有斜坡仍为 10‰/10 ms；资格建立后旧档每侧每 10 ms 最多步进 10‰、常规上限 600‰，新 `DebugFollowTurn` 每侧最多步进 20‰、常规上限 700‰；所有档的硬上限仍为 700‰。仅允许左 IN2/PA7/TIM3_CH2 与右 IN1/PB0/TIM3_CH3 同步输出；左 IN1、右 IN2 始终为 0。
4. 验证视觉保持时，OpenMV 必须继续发送 CRC/格式正确、序号递增的 `valid=0` 帧；停止 OpenMV 脚本会变成通信超时，不是视觉保持测试。确认此前舵机与左右电机软件输出均非零后，保持期内 `car_vision_hold_active=1`，`car_vision_hold_elapsed_ms` 从最后一帧可信路线计时，舵机脉宽和每侧占空比保持不变。不要断开 UART 线制造故障。
5. 在期限前恢复路线时，三帧连续新鲜、递增且有效的数据后自动恢复。若保持到 1000 ms 截止，输出应先归零，`car_manual_run_request` 和日志仍保持活动；`car_vision_hold_elapsed_ms=1000`，`car_run_block_reason=11` 表示等待路线。随后三帧有效数据只会通过完整首 PWM 周期和电机斜坡门重新启动。孤立一两帧不会恢复。
6. 要结束 Follow 并冻结日志，先让 PE7 高电平稳定 20 ms 重新武装，再按第二次。请求应归零、输出停车，日志状态变为 `FROZEN`；松手本身不会停车。保持主控供电后再按[运行日志导出说明](RUNTIME_LOG_EXPORT_CN.md)读取。硬故障可另行冻结日志，但必须先排除故障；不要把视觉保持到期当成完成测试或可导出状态。

完成必要的板上检查后，再决定是否做低风险、短距离的直道和普通弯道检查。发生异常时，若车辆仍响应，应按上述二次按下停止；软件构建和主机测试不替代实体急停与实板验收。

历史记录曾确认舵机端点/中位 1100/1530/2500 μs、P0.7 映射 1229/1530/2209 μs、编码器计数与符号、左 IN2/右 IN1 前进组合、UART 和 PE7 的前序检查。历史现场曾记录一次串口故障码 3，当时未定位；这条记录不表示当前固件正处于该故障。事故后的 PA8 实际 PWM 波形仍须重新确认，拉杆已修复。本次构建和 host 测试没有连接、烧录或操作硬件，也不代替当前版的 10 秒视觉中断/恢复架空检查。

## 本版范围

本版使用现有 Follow 状态机和恢复门控，不新增轮速 PID、元素识别、判圈或 OpenMV 逻辑。它没有十字、环岛、坡道、斑马线的识别与通过策略，也不是完整比赛程序。二次按下、复位或硬故障会结束本次手动运行资格；松开 PE7 不会停止 Follow。Bench 仍使用按住运行、释放停止。不要通过 Live Watch 写变量控制车辆。
