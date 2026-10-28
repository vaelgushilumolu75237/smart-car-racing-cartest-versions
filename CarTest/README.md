# CarTest：H743 车载工程

2026-10-29 PE7 maintained-switch candidate: Follow uses low=RUN and high=immediate STOP. Software validation and a deliberate H743 flash are required; the vehicle firmware has not been updated. See [implementation and validation](docs/PE7-MAINTAINED-SWITCH-20261029.md).

## 2026-10-26 当前配套版本

当前为H750相机三档版本：新A提速无环岛5.6/4.4、新B原速环岛4.8/4.0、新C提速环岛5.6/4.4（单位为驱动轮目标转/秒）。三个新入口位于VS Code调试列表最前面，共用一套320×120相机程序，通过V3档位确认配对；下面旧记录中的占空比、速度和“当前”不代表本轮配置。

交付位于 `deliverables/TrackProfiles_H750_20261026`。操作与日志命令见[三档切换说明](docs/TrackProfiles-使用与切换-20261026.md)，实现和验证边界见[H750实施记录](docs/H750-TrackProfiles-Implementation-20261025.md)。软件与静止相机测试已完成，车辆移动过弯、十字和左右环岛尚待手动验收；保留稳定版回退包。

## 2026-10-20 A/B/C上坡保速试验

当前VS Code入口为A冻结原版、B 65%/35%开环对照、C增量式PI保速试验及关闭执行器的诊断档。B/C采用200ms丢线减速、2000ms续行截止；C直行初始轮速目标2.4转/秒，每轮输出上限70%。相机继续使用未改动的生产`openmv/`目录，保持关闭预瞄及80/20位置权重。

切换与日志命令见[本轮操作说明](docs/UPHILL_ABC_20261019_CN.md)，软件验证见[验证记录](docs/UPHILL_ABC_VALIDATION_20261020.md)。A启动项使用冻结ELF，不重新构建。本轮没有烧录或实车验收，C尚不能视为已经改善上坡表现。

以下保留历史记录；旧段落中的“当前”仅指其标注日期，不用于选择本轮档位或参数。

## 2026-10-16 历史配套版本

当前版本为 `AGGRESSIVE_TURN_20261016`：单边连续段恢复、近端短链和有证据的左右换边；快速档正常舵机120μs/20ms、左右电机各40‰/10ms；3秒最后可靠偏差保持。使用 `build/DebugFollowTurn/CarTest.elf`，相机另行上传本版20个Python运行文件。完整步骤和实测边界见[本版说明](docs/AGGRESSIVE_TURN_20261016_CN.md)，最终文件哈希和软件验证见 `openmv/PACKAGE_MANIFEST.json`。

日志导出默认使用只读HOTPLUG共享连接、固定探针序号和有限降频重试，并在读取RAM前校验板上Flash版本。只读连接、主机测试或编译不能证明新固件已经烧录或完成实车过弯。下方保留历次交付记录；其中旧的“当前构建”、参数及测试结论均以该条日期为准，恢复旧版时使用对应ELF。

本目录是 STM32H743VIT6 GCC/CMake 工程。`openmv/` 中的 MicroPython 程序独立运行，不进入 STM32 构建，也不随 H743 ELF 一起烧录。

> **当前 DebugFollow 操作（2026-10-13）**：DebugFollow 与 DebugFollowNoLog 均保留用户授权的软件输出许可；这不代表实板验收。PE7 先稳定释放 20 ms 后，第一次按下锁存启动请求，松手继续运行，下一次按下才取消请求并停止。有效=0 的短暂丢线仅在舵机和双路电机已有非零软件输出时进入最长 1000 ms 的视觉保持；到期先停车但会话和日志继续等待，三帧连续新鲜有效路线数据可恢复。串口/协议/硬故障路径仍独立停车。Follow 直线基准为 500‰，常规目标上限 600‰、硬上限 700‰；差速档位为 0/50/100‰。舵机 1100–2500 μs 限幅、1530 μs 中位与启动 10 μs/20 ms、正常 Follow 40 μs/20 ms 变化限制保持。没有连接或烧录 H743，也没有做台架或实车验证。

> **2026-10-11 舵机完整脉冲停止修复（当前构建）**：正常停车现在立即关闭电机输出；舵机高电平期间等待捕获比较中断确认下降沿后关闭，帧边界设 128 μs 软件保护窗口。四个 profile 均 Fresh 编译，host 与离散相位模型通过。模型不是板上波形测量；本次未刷写或运行硬件，UART 故障码 3 和舵机突然左转的物理原因仍未确认。详见[本轮停止修复报告](test-data/servo-start-stop-20261011/SERVO_PULSE_STOP_FIX_2026-10-11.md)。

> **2026-10-11 USART1 FIFO 接收修复（当前构建）**：针对现场报告的 ORE=3（FE/NE/PE=0，PE7 release 计数=0），USART1 已启用 16 字节 RX FIFO，并以非空中断和每次最多 16 字节的有界服务排空；UART 错误仍立即停车并清除旧 FIFO 数据。四个 profile 已 Fresh 编译，host、离线解析和 fake-CLI 回归通过。尚未接板验证 ORE 是否消失，不能据此确认现场原因。详见[UART FIFO 修复报告](test-data/uart-fifo-20261011/REPORT.md)。

> **2026-10-12 servo_apply PRIMASK 修复（当前源码）**：Follow/Steering 输出路径在进入共享 PWM 请求区时曾覆盖调用者的 PRIMASK，使主循环后续 IRQ 保持关闭。现已移除重复保存/disable，同时保持 guard、PWM 请求与提交处于同一临界区；真实 servo_apply 提取函数的负对照和正对照通过。构建、测试和硬件未验证边界见[修复报告](test-data/servo-apply-primask-20261012/REPORT.md)。

## 当前交付

当前新增独立的 `DebugFollowSpeedPI` 试验档，原 `DebugFollowTurn` 保持开环。PI 直行轮速目标为 2400 milli-RPS（144 转/分钟），占空比上限为 700‰；增益为待实车调参的初值。九个配置已完成 Fresh 编译，主机测试通过，本轮未连接或烧录 H743。各 ELF 的 SHA-256、ROM Build ID 和测试结果见[PI 构建摘要](docs/FOLLOW_SPEED_PI_BUILD_MANIFEST.json)，操作见[PI 试验说明](docs/FOLLOW_SPEED_PI_TRIAL_CN.md)。完整日志保存在本地 `test-data/speed-pi-trial-20261014/after/`。

用户报告当前环岛视觉版本存在普通直道、弯道回退，尚未确认修复。14号上午的恢复基线保存在[GitHub 归档](https://github.com/vaelgushilumolu75237/smart-car-racing-cartest-versions/releases/tag/v20261014-pre-ring)；当前 PI 作为独立实验版本保存。

## 当前 RAM 运行日志

2026-10-11 起，人工启动的 Follow/Bench 会话可在 H743 RAM 中保存 50 ms 软件遥测采样和状态事件；该日志不持久化。Follow 的 1000 ms 视觉保持到期只是临时停车，日志仍为 ACTIVE；要结束本次记录，需再次按 PE7 取消手动请求并确认日志冻结，然后在板卡仍上电时导出。Bench 仍采用按住运行、释放停止。DebugSteering 不会开启日志，日志代码不参与控制决策。查看[运行日志构建报告](RUNTIME_LOG_BUILD_REPORT_2026-10-11.md)和 [ST-Link Hot Plug CSV 导出说明](docs/RUNTIME_LOG_EXPORT_CN.md)。物理 SWD 导出和硬件运行尚未验证。

本轮差速候选的试车门槛、manifest 填写和分析口径见[差速试车说明](docs/DIFFERENTIAL_TRIALS_CN.md)；2026-10-14 软件证据与空清单分析见[最终验证记录](test-data/differential-20261013/log-analysis/VALIDATION.md)。

本机直接使用此 CarTest 工作副本中的 build/DebugFollow/CarTest.elf 和 tools/export_run_log.ps1，无需下载额外包。冻结、导出步骤见[ST-Link Hot Plug CSV 导出说明](docs/RUNTIME_LOG_EXPORT_CN.md)；已有历史包和基线保持保留。DebugFollowNoLog 不会产生可导出的有效日志头，导出工具会拒绝它。

## 日志 ON/OFF 本机切换与阻塞理由

默认 Follow 项使用日志 ON。先保存 .vscode/launch.json 与 tasks.json 的未保存编辑，在 PowerShell 中运行：
```powershell
Set-Location D:\Codex\smart-car-racing-codex-handoff\smart-car-racing\firmware\vehicle\CarTest
powershell.exe -NoProfile -ExecutionPolicy Bypass -File .\tools\select_follow_logging.ps1 -Variant NoLog
```
然后在 VS Code 启动菜单中选择改名后的 DebugFollowNoLog 并按 F5。切回日志版运行：
```powershell
powershell.exe -NoProfile -ExecutionPolicy Bypass -File .\tools\select_follow_logging.ps1 -Variant Log
```
脚本只切换原 Follow 启动项的任务、ELF 和符号路径并备份 launch.json；三条中文行驶档保持日志开启且不受此切换影响。NoLog ELF 不能导出有效日志。

Live Watch 中 `car_runtime_log_enabled` 显示日志变体；`car_run_block_reason` 只读分类：0=Follow 双路电机软件输出已放行且两侧软件占空比非零（不证明车轮实际转动），1=等待上电释放，2=故障，3=无新鲜有效视觉，4=恢复帧不足，5=相机 RESET/KEEP 门未满足，6=等待新的 PE7 按下沿，7=软件输出许可关闭，8=等待舵机首 PWM 周期，9=舵机停车 pending，10=等待电机斜坡或其他输出保护条件，11=视觉保持已到 1000 ms、输出停止并等待路线。它不代表硬件测量。详情见[日志 A/B 与运行阻塞诊断报告](test-data/runtime-log-ab-20261012/REPORT.md)。
## 当前构建配置

VS Code 启动菜单保留原四项及开环行驶档，并新增独立的编码器 PI 试验档。日常切档按 `Ctrl+Shift+D`，在“运行和调试”顶部下拉框选择启动项后按 `F5`；PI 档参数、状态变量和对应 ELF 导出方式见[FollowTurn 编码器稳速 PI 试验档](docs/FOLLOW_SPEED_PI_TRIAL_CN.md)。

| VS Code 启动项 | 行为 |
| --- | --- |
| CarTest H743 - Debug (all actuators off) | 联调与观察配置；电机和舵机 PWM 均关闭。 |
| CarTest H743 - DebugSteering (motors locked; vision steering diagnostic) | 电机输出锁定；此配置可输出受统一脉宽/变化率保护的舵机诊断波形。测量前须断开舵机 5 V 和 VM。 |
| CarTest H743 - DebugBench (motor outputs locked pending hardware acceptance) | 本次仅保留 profile/ELF；电机输出锁定为 0，舵机 PWM 关闭。 |
| CarTest H743 - DebugFollow (motors enabled; PE7 toggle; straight 50%; hard cap 70%; diff=0; RAM log ON) | 原 Follow 项保留 d=0；直线基准 500‰、常规目标上限 600‰、硬上限 700‰。 |
| 行驶0档 - 无差速 50%/50%（日志开启） | 日志开启，无差速；使用 `build/DebugFollow/CarTest.elf`。 |
| 行驶1档 - 中差速 最大55%/45%（日志开启） | 日志开启，中差速 d=50‰；使用 `build/DebugFollowDiff5/CarTest.elf`。 |
| 行驶2档 - 强差速 最大60%/40%（日志开启） | 日志开启，强差速 d=100‰；使用 `build/DebugFollowDiff10/CarTest.elf`。 |
| 行驶3档 - 快速转向 70%/30%（日志开启） | 原 FollowTurn 转向档，速度仍为开环；使用 `build/DebugFollowTurn/CarTest.elf`。 |
| PI试验档 - 编码器稳速（日志开启） | 独立编码器稳速试验；使用 `build/DebugFollowSpeedPI/CarTest.elf`。参数和物理验证边界见[试验说明](docs/FOLLOW_SPEED_PI_TRIAL_CN.md)。 |

前三个中文差速菜单项仍为原开环控制，差速按已提交舵机脉宽相对 1530 μs 中位计算，左右目标按转向镜像调整；每侧每 10 ms 最多改变 10‰。Turn 和 PI 两个额外档各自绑定同档 build task、ELF 与符号文件；PI 的增益为待实板评估的试验初值。所有行驶项使用 4000 kHz SWD 和 Software system reset。软件构建与主机检查不代表 H743 已烧录或完成实板验收。

## 构建

在本目录的 PowerShell 中构建默认禁用所有执行器的 Debug：

~~~~powershell
.\tools\build.ps1 -Preset Debug
~~~~

转向视觉联调可在 VS Code 选择 DebugSteering，或运行：

~~~~powershell
powershell.exe -NoProfile -ExecutionPolicy Bypass -File .\tools\build.ps1 -Preset DebugSteering -Fresh
~~~~

输出文件为 `build/DebugSteering/CarTest.elf`。`-Fresh` 只让 CMake 重新配置所选 preset，不递归删除 build 目录。构建脚本会从当前用户的 `%LOCALAPPDATA%\stm32cube\bundles` 自动选择 Ninja 与 ARM GCC，并检查现有缓存是否仍匹配当前工具。

2026-10-11 更新后的构建配置已应用于本机工作副本；当前电脑无需另行下载配置包。此前 portable/原电脑版配置包及旧配置包继续保留为历史文件。

已现场验证的单轮基准完整包：[左轮基准](deliverables/CarTest-source-20261010-bench40-hold5s.zip)和[右轮基准](deliverables/CarTest-source-20261010-right-bench40-hold5s.zip)，均包含 40% 保持 5 秒与提前释放停机记录。双轮架空版[源码与四个 profile ELF](deliverables/CarTest-source-20261010-dual-bench40-hold5s.zip)已完成现场架空检查，记录见[双轮构建报告](DUAL_BENCH40_HOLD5S_BUILD_REPORT.md)和[双轮 Bench40 说明](docs/BENCH40_CN.md)。本次基本循迹包：[CarTest-source-20261010-follow40-recovery.zip](deliverables/CarTest-source-20261010-follow40-recovery.zip)，构建/主机测试记录见[Follow 构建报告](FOLLOW_BUILD_REPORT.md)，首次架空检查步骤见[Follow 恢复操作说明](docs/FOLLOW_BRINGUP_CN.md)。此前 500 ms Bench40 包和记录保留为历史版本。

## 历史版本：DebugSteering 操作记录

断开电机 VM，抬起前轮并清理转向机构周围的障碍。启动 OpenMV 程序并调试 DebugSteering。主控收到三帧新鲜有效数据后自动开始跟随后续视觉偏差；无需按下 PE7，释放 PE7 也不会停止此配置。

收到无效视觉、视觉超时或故障时，舵机输出会关闭。可恢复的视觉中断需要三帧新的有效数据，之后自动恢复跟随。硬故障仍须先排除故障源，再按固件要求用实体 PE7 确认；视觉数据不能清除硬故障。普通停止请停止 OpenMV 输出；如需立即切断舵机动作，断开舵机电源。PE7 释放不是此配置的停止操作。

Live Watch 只读观察：运行状态 `car_mode=11`、`car_steering_session_active`、`car_steering_servo_target_us`、`car_servo_applied_pulse_us`、`car_servo_output_active`、`car_fault`、`car_vision_valid` 和 `car_vision_timeout`。同时确认 `car_drive_allowed=0`，并且 `car_motor_left_in1_output_permille`、`car_motor_left_in2_output_permille`、`car_motor_right_in1_output_permille`、`car_motor_right_in2_output_permille` 始终为 0。软件快照不等于引脚波形或机械动作测量；不要通过 Live Watch 写变量控制车辆。

当前转向配置为左 1100 μs、中位 1530 μs、右 2500 μs。P0.7 将归一化视觉误差 -1000、0、+1000 分别映射到 1229、1530、2209 μs。用户确认这组配置平顺并留有余量；这些值不是机械止挡测量。H743 输出波形、掉电位置和完整机械行程仍需现场记录。OpenMV 程序和视觉协议未改。

## 当前版本：DebugFollow 视觉保持与切换操作

Follow 不是 Bench 单次 5 秒运行：PE7 稳定释放 20 ms 后，实体按下沿锁存一次手动请求；释放开关不停车，第二个经过重新武装的按下沿取消请求并停车。摄像头不会自行启动车辆。运行中有效=0 的短暂丢线仅在已有舵机 PWM 与左右电机软件输出时保持进入时的脉宽和两侧占空比，最多 1000 ms；到期输出归零但保留本次请求，三帧连续新鲜有效数据可恢复，超时或 UART/协议故障仍走独立停车路径。硬故障不会由视觉恢复，须先排除故障源并按固件流程确认。操作和观察变量见[Follow 恢复操作说明](docs/FOLLOW_BRINGUP_CN.md)。

Live Watch 中 `car_manual_run_request` 表示锁存请求，`car_button_armed` 表示开关已重新武装，`car_start_pending` 表示请求存在但 Follow 会话或实际软件输出尚未全部就绪；`car_vision_hold_active` 与 `car_vision_hold_elapsed_ms` 表示临时视觉保持及其计时，`car_vision_hold_entries` 和 `car_vision_hold_exit_cause` 记录次数和退出原因。`car_auto_recovery_count` 包含视觉保持后恢复。它们均为软件状态，不等于引脚/车轮测量。

## 固定接口

| 功能 | 接口 |
| --- | --- |
| 左 DRV8871 | PA6/PA7，TIM3_CH1/CH2，AF2 |
| 右 DRV8871 | PB0/PB1，TIM3_CH3/CH4，AF2 |
| 转向舵机 | PA8，TIM1_CH1，AF1，50 Hz |
| 左编码器 | A→PA1/TIM2_CH2，B→PA0/TIM2_CH1，AF1 |
| 右编码器 | A→PC7/TIM8_CH2，B→PC6/TIM8_CH1，AF3 |
| OpenMV UART3 | P4/TX→PA10/RX，P5/RX←PA9/TX，共地；USART1 AF7，115200 8N1，3.3 V TTL |
| PE7 开关 | 低有效；供 FOLLOW/BENCH 操作和硬故障确认使用，不控制 DebugSteering 普通启停。 |

引脚表不证明排针连通、电平、PWM 波形、驱动器禁能行为或机械限位。OpenMV 程序需要单独上传到摄像头；H743 只烧录本工程 ELF。

## 历史验证记录

本轮 profile 构建与主机测试结果见[2026-10-09 构建报告](BUILD_TEST_REPORT.md)。以下报告描述的是当时源码和构建。旧 profile 名称及 ELF 作为历史证据保存，但已从当前 CMake preset、构建任务和启动菜单中移除。

- [2026-10-09 舵机试验构建报告](test-data/servo-1400-1660-20261009/BUILD_TEST_REPORT.md)及[此前舵机试验报告](test-data/servo-trial-20261008/BUILD_TEST_REPORT.md)。
- [2026-10-08 编码器标定构建报告](test-data/encoder-calibration-20261008/BUILD_CHECK_REPORT.md)，以及其中引用的旧线序/GPIO 诊断报告。
- [2026-10-07 S20F 基线构建报告](test-data/s20f-calibration-20261007/BUILD_TEST_REPORT.md)。

这些历史报告描述当时的源码、ELF 和曾启用的输出行为；本次保护版现状以本文顶部提示、[当前构建报告](STEERING_SINGLEEDGE_BUILD_REPORT.md)和[现场验收说明](docs/STEERING_SINGLEEDGE_BRINGUP_CN.md)为准。历史 Follow/Bench 通过记录不得用于证明本次版本已烧录、机构安全、达到 45 次/秒或完成实车循迹。旧操作说明只供追溯历史，不适用于本次事故后的带电验收。
