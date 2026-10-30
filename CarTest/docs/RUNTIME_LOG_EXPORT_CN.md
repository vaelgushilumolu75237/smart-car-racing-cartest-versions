# CarTest 运行日志导出

运行日志保存在 MCU 的 RAM 中。车辆正常停止并冻结日志后，PC 工具通过 ST-Link 的 SWD Hot Plug 读取数据，再检查 ELF 描述符、日志头、记录 CRC 和停止记录，最后生成 CSV。导出期间需要保持主控供电。

## 导出前

从本项目目录选择本次实际构建使用的 `CarTest.elf`。不要从同名的其他工程或旧配置中取 ELF；ROM 描述符中的 build ID 必须与 RAM 日志一致。离线工具依次校验头 schema、header CRC 和保留字节，再比较 build ID，最后报告 `IDLE`、`ACTIVE` 或可导出的 `FROZEN` 状态。build ID 不匹配时，错误会显示 RAM 地址、所选 ELF 的 expected ID 和头中的 observed ID。这个不匹配只说明当前头与所选 ELF 的身份不同；它不能单独证明哪一个固件当时正在运行。工具不会根据头自动切换 ELF；先核对实际使用的 ELF 与记录来源。

当前 DebugFollow 构建示例：

```powershell
Set-Location 'D:\Codex\smart-car-racing-codex-handoff\smart-car-racing\firmware\vehicle\CarTest'
$elf = (Resolve-Path '.\build\DebugFollow\CarTest.elf').Path
```

## 选择本次实际构建

普通三档试跑使用 `DebugFollowTurn` ELF：

```powershell
powershell.exe -NoProfile -ExecutionPolicy Bypass -File .\tools\export_run_log.ps1 -Elf .\build\DebugFollowTurn\CarTest.elf
```

Speed PI 试跑使用 `DebugFollowSpeedPI` ELF：

```powershell
powershell.exe -NoProfile -ExecutionPolicy Bypass -File .\tools\export_run_log.ps1 -Elf .\build\DebugFollowSpeedPI\CarTest.elf
```

每次都按本次实际烧录的配置选择 ELF。若用 PowerShell `$elf` 变量，切换配置后必须重新赋值；不要沿用上一个配置留下的路径。当前 VS Code 工作区没有专用的日志导出 task。ELF 不匹配时工具只报告身份不一致，不会自动换 ELF，也不能恢复或推断一份 `IDLE` / `count=0` 头对应的运行记录。

一次导出必须满足以下状态：

- 本次运行已经由车辆自己的停止路径结束，RAM 日志头为 `FROZEN state=2`。Follow 必须通过 PE7 请求停止，等日志冻结且舵机、电机输出字段为零后再导出；单次视觉保持到 1000 ms 后的临时零输出仍是 `ACTIVE`，会继续记录并等待自动恢复，不能在此时导出。Bench 仍按住运行、释放停止。视觉超时等其他临时故障本身不会冻结日志。
- 车辆和主控继续供电。车辆停止后、导出完成前，不要复位、重启或重新烧录主控，也不要按 F5 启动调试器；这些操作可能清除 RAM 日志。
- 在车辆测试前让 ST-Link 脱离 IDE 调试会话，避免探针被占用。断开 IDE 调试连接不等于车辆已停车；必须由车上 PE7 停止路径结束本次运行并冻结日志。若仍遇到连接错误，保持车辆供电并保留失败记录，不要复位、halt、下载或切换到普通连接模式。

## 执行

默认命令会在 `logs\runs\<时间戳>-<进程号>\` 下创建一次导出目录，并在成功时只输出 CSV 的完整路径：

```powershell
powershell.exe -NoProfile -ExecutionPolicy Bypass -File .\tools\export_run_log.ps1 -Elf $elf
```

`ExecutionPolicy Bypass` 只用于这个 PowerShell 子进程，不会修改系统或用户的持久策略。导出成功时，脚本通过标准输出打印 CSV 的完整路径。

如需选择某支 ST-Link，可提供它的序列号：

```powershell
$csv = .\tools\export_run_log.ps1 -Elf $elf -ProbeSN '1234567890'
```

默认 SWD 频率为 4000 kHz，探针使用独占连接。只在手动排查时可选 1000 kHz，或明确启用 CubeProgrammer 的共享探针参数；脚本不会在连接失败后自动降频、改用共享或重试其他模式：

```powershell
$csv = .\tools\export_run_log.ps1 -Elf $elf -ProbeSN '1234567890' -ConnectionFrequencyKhz 1000
$csv = .\tools\export_run_log.ps1 -Elf $elf -ProbeSN '1234567890' -SharedConnection
```

频率参数只接受 `1000` 或 `4000`。共享选项只把 `shared` 显式加入 SWD Hot Plug 连接参数，不改变读取流程。

先查看命令计划时运行：

```powershell
.\tools\export_run_log.ps1 -Elf $elf -DryRun
```

`-DryRun` 会检查 ELF 并显示内存地址、大小、所选频率、共享选项、ProbeSN 是否提供，以及 Hot Plug 读取参数；不会启动 STM32CubeProgrammer 或连接 ST-Link。它会创建一次带有 ELF 检查日志的运行目录。

脚本只使用 `port=SWD mode=HOTPLUG freq=<1000|4000>`，可按需加 `shared` 和 `sn=<ProbeSN>`，并且只进行 `--upload` 内存读取。先读取 64 字节头，确认日志已冻结，再读取 `64 + count × 64` 字节的已用前缀，最后再次读取头并与开头及 payload 内的头逐字节比较。内存地址和总大小来自所选 ELF 的 `car_runtime_log_storage` 符号及 ROM 描述符，不从文档或历史构建中固定地址。

## 输出文件

成功目录内包含：

- `run.csv`：一行对应一条 64 字节记录。
- `runtime-log.bin`：原始已用日志前缀。
- `header-initial.bin`、`header-final.bin`：导出前后读取的 64 字节头。
- `run-metadata.json`：所选 ELF SHA-256、ROM build ID、RAM 地址/大小、记录数、停止原因、CRC、是否写满/丢记录、原始数据 SHA-256 和导出状态。成功时 `csv` 指向实际生成的 CSV；失败时 `csv` 为 `null`，`planned_csv_path` 仍保留原计划路径。
- `cubeprogrammer-*.log`、`python-*.log`：每步工具输出。

失败时会保留已经读取的原始二进制、命令日志和 `run-metadata.json`，并且不会留下 `run.csv`。此时 `csv` 为 `null`，不要把 `planned_csv_path` 当成已生成文件。先查看 `stage` 和 `error` 字段，再按日志指明的失败步骤处理。脚本不会尝试复位、停止内核、擦除、写入或改用普通连接模式。

如果 `error` 含 `DEV_CONNECT_ERR`，表示本次连接没有读到可用的 RAM header/payload；`initial_header` 和 `raw_sha256` 会为空，这不是试车数据。保持车辆供电，核对 ProbeSN 和 SWD 接线，并确认没有 IDE 调试会话占用探针。可由操作者明确选择 1000 kHz 或 `-SharedConnection` 作为排查选项；这些选项不保证能解决硬件或探针问题。不要自动终止 `ST-LINK_gdbserver` 等进程。错误提示会明确说明：断开调试器本身不会让车辆停车；应先使用 PE7 停止请求、让日志冻结到 state=2，并继续给主控供电。

离线验证已有原始文件时，不需要 ST-Link。示例使用本次审计生成的 DebugFollow 配对 ELF 和同 build ID 日志 fixture；重新构建后要重新生成配对 fixture。旧日志必须使用其原构建 ELF 快照，不能拿新 ELF 解码：

```powershell
$elf = (Resolve-Path '.\build\DebugFollow\CarTest.elf').Path
$raw = '.\test-data\log-audit-20261014\fixtures\runtime_log_debugfollow_current.bin'
python .\tools\runtime_log_export.py decode `
  --elf $elf `
  --raw $raw `
  --out-csv '.\logs\runs\offline.csv'
```

离线命令同样会检查 build ID、头和记录 CRC、完整长度、起止事件及最终停止记录；验证失败不会生成 CSV。

## CSV 字段单位

时间字段 `t_ms` 是原始 u32 毫秒计数，可能绕回；`t_elapsed_ms` 是相对 `start_ms` 的毫秒数，按单次 u32 回绕计算。日志必须短于一个完整 u32 毫秒计数周期（约 49.7 天）。编码器字段是有符号 64 位计数，不代表距离或速度。

舵机 `*_us` 是软件记录的微秒命令/比较值；电机 `*_permille` 是千分比软件命令；`tim3_ccr1_counts` 至 `tim3_ccr4_counts` 是原始 TIM3 比较计数。它们描述软件状态，不证明引脚、电机或舵机的实际物理输出。视觉误差为有符号归一化值的千分单位，置信度范围为 0–100。CSV 保留数值 `mode`、`fault`，并为它们增加 `mode_name`、`fault_name` 文字列；停止原因数值 `stop_reason` 及名称 `stop_reason_name` 位于解析结果的 header/metadata 中，不是 CSV 记录列。停止原因数值 1 的旧名称 `MANUAL_RELEASE` 保持兼容；显示名为 `manual_stop`。模式 12 的 `vision_hold_elapsed_ms` 以 20 ms 为步长量化并封顶 1000 ms；`vision_hold_valid_count` 是保持期内连续有效路线帧数。其他模式这两列为 0。`event_hold_enter`、`event_hold_exit`、`event_hold_deadline`、`event_button_start` 和 `event_button_stop` 标记对应状态变化；按钮事件只来自 PE7 实际 start/stop 计数，不会把故障取消运行请求误写成按键停车。`vision_hold_exit_cause` 为固件退出原因数值。标志位另提供 0/1 列，便于筛选。

头标志 `buffer_full` 表示常规记录空间已用尽（count 至少为 6143）；`records_truncated` 表示至少有一条记录被丢弃。缓冲区写满不代表数据已丢失，查看 `dropped_count` 可区分两种情况。

## 验证边界

主机生成的 C 日志夹具和 fake CLI 测试用于验证 ABI 解析、CRC、时间回绕、64 位编码器值、PowerShell 参数数组、标准错误捕获及失败保留逻辑。主循环在短临界区内复制共享状态和 PWM 计数，随后恢复调用者原有 PRIMASK，再执行 CRC、记录追加和缓存清理。它们不证明真实 H743、ST-Link、CubeProgrammer Hot Plug 或车辆导出行为；真实板级导出仍需在车辆停止、供电保持的条件下单独验证。
