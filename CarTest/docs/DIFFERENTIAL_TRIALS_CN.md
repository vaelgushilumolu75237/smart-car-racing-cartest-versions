# Follow 差速实车对照记录

本流程比较三份带 RAM 日志的固定 Follow ELF：`none` 为 d=0‰（左右 50/50），`medium` 为 d=50‰（转向时目标可达 55/45），`strong` 为 d=100‰（转向时目标可达 60/40）。直行请求为 500‰，软件 applied 常规上限为 600‰，硬上限为 700‰。这些是待实车验证的候选档。现有 VS Code Follow 启动项仍默认使用 `none`；`medium`、`strong` 不会自动切换，也不表示已经选出最佳档。

## 固件白名单与证据边界

构建完成后使用 [`control/ARTIFACTS.json`](../test-data/differential-20261013/control/ARTIFACTS.json)。其中必须有且只有 `none`、`medium`、`strong` 三项，记录 `variant`、`diff_permille`、相对该 JSON 的 `elf_path`、ELF SHA-256、ROM build ID 和 `servo_center_us`。分析器会检查每份 ELF 的真实 SHA/build ID。日志 metadata、白名单 ELF、RAM 原始字节或 CSV 不匹配时，该次数据不能通过身份校验。不要用 `NoLog` ELF、旧 ELF 或仅凭文件名认档。

本页的实车步骤由操作者执行。主机测试、虚拟 CLI、生成的合成 fixture，以及本次 `DEV_CONNECT_ERR` 记录都不是车辆试跑或成功 RAM 导出证据。差速门槛和安全操作仍按[Follow 上板操作说明](FOLLOW_BRINGUP_CN.md)执行。

## 每次试跑与导出

一次试跑对应一段独立物理 session。用 PE7 开始和结束同一 session；停止后确认 logger 为 FROZEN、`state=2`，并保持主控供电。按照本档白名单 ELF 执行 Hot Plug 导出；不要在冻结后复位、重新烧录，或把 IDE 断开当成停车。`DEV_CONNECT_ERR` 表示没有读到可用 RAM header/payload，保留失败 metadata 与 CLI 日志，但不要把它当成试跑样本，也不要从其他额外成功 run 中挑选替补。

默认导出为 4000 kHz、独占 ST-Link。只有操作者排查时才手动改用 `-ConnectionFrequencyKhz 1000` 或 `-SharedConnection`；导出脚本不自动换频、重试、kill 调试 server、halt、reset 或 flash。按现场情况核对 ProbeSN/SWD/探针占用。断开 IDE 不会停止车辆。

```powershell
$project = 'D:\Codex\smart-car-racing-codex-handoff\smart-car-racing\firmware\vehicle\CarTest'
Set-Location $project
$artifactsPath = Join-Path $project 'test-data\differential-20261013\control\ARTIFACTS.json'
$artifacts = Get-Content -LiteralPath $artifactsPath -Raw -Encoding UTF8 | ConvertFrom-Json
$profile = $artifacts.profiles.none   # 每组三次试跑分别选 none、medium、strong
$elf = [IO.Path]::GetFullPath((Join-Path (Split-Path -Parent $artifactsPath) $profile.elf_path))
.\tools\export_run_log.ps1 -Elf $elf -ProbeSN '<探针序列号>'
```

成功导出目录必须同时含 `run.csv`、`runtime-log.bin` 与**文件名为 `run-metadata.json`** 的当前 metadata。manifest 的 `path` 写导出目录（或直接写 `run.csv`）；相对路径以 `trial-manifest.json` 所在目录为基准。不要写成 `run.metadata.json` 或 `metadata.json`。

## manifest：一个 trial_set、九次固定尝试

每次正式对照只使用一个唯一 `trial_set_id`。顶层 `trial_set_id` 与九个 run 条目的 `trial_set_id` 必须逐字相同；`runs` 必须恰好包含每档三条不同物理 session：`none` 三条、`medium` 三条、`strong` 三条。条目至少包含下方模板中的字段；从生成的 [`manual-metadata-template.json`](../test-data/differential-20261013/log-analysis/manual-metadata-template.json) 复制条目，填完九条后保存为 `trial-manifest.json`。

```json
{
  "schema_version": 1,
  "dataset_type": "real",
  "trial_set_id": "<本组唯一编号>",
  "runs": [
    {
      "trial_set_id": "<与顶层相同>",
      "run_id": "<唯一物理 session ID>",
      "path": "<导出目录或 run.csv 的相对/绝对路径>",
      "variant": "none",
      "completed": null,
      "out_of_track": null,
      "layout_id": "<赛道布局编号>",
      "camera_build_id": "<摄像头固件/构建编号>",
      "battery_voltage_v": null,
      "battery_state": "<记录当时电池状态>",
      "lighting_id": "<光照条件编号>",
      "lighting_notes": "<当时光照备注>"
    }
  ]
}
```

示例只展示一条；实际 `runs` 必须按上面数量填写九条。将每一条复制三次并为同档的物理 session 赋不同 `run_id`。把 `completed`、`out_of_track` 填为现场观察到的 JSON 布尔值 `true`/`false`；把电压填为实测的正数，其他字段填非空值。不要保留 `null` 或尖括号占位内容。复制同一导出目录、raw 数据或 session 并改名不能算独立尝试。

三档须用相同的 `layout_id`、`camera_build_id`、`lighting_id`；九次实测电压的跨度不得超过 0.2 V。`battery_state` 和光照备注保留在结果中供复核。填写的人工观察不能由日志或测试脚本代填。

## 通过、淘汰与排名

- **整组 insufficient**：`trial_set_id` 缺失/不匹配、任一档尝试数不是恰好 3、任何一条日志或原始数据缺失/身份不符/时间不完整，或比较条件不可比。当前清单必须保留所有尝试，不得挑三条成功记录凑数。
- **档位 failed**：该档三次均有完整、唯一且可解码的真实数据，但任一次 `completed=false`、`out_of_track=true`、出现 hard fault、日志写满/丢记录、无效运行窗口或 applied 输出不符合当前算法门槛。已验证的失败也占用该档三次配额；不能用第四次替换。
- **档位 passed**：同档恰好三次均通过上述人工结果、日志完整性、ELF/RAM 身份、时间和运行窗口校验。完整试验组中可剔除 failed 档，并在其余通过档间排序；只有整组结构完整且可比时才允许给出排名。如果完整组中没有任何通过档，结果为 `no_eligible_profile`。缺数据的整组为 `insufficient`。两种状态都不会产生 `recommendation`。

真实日志必须由白名单对应 ELF 解析。分析器检查成功且 complete 的 metadata、FROZEN state=2 header、ROM build ID、原始 `runtime-log.bin` SHA 和实际 ABI/CRC 解码结果，再将解码 header 与 `run-metadata.json` 比较，并逐行比对 CSV 中全部必需字段。只检查 CSV 或 metadata 自报 SHA 不足以通过。五条历史 `DEV_CONNECT_ERR` 只有失败诊断，没有 RAM header/payload，不进入九次尝试。

## 分析命令

从工程根目录运行；填好 manifest 后再运行。命令会更新 `analysis.json`、`summary.csv`、`report.md` 和空白人工模板，并在指定输出目录保存结果。

```powershell
$project = 'D:\Codex\smart-car-racing-codex-handoff\smart-car-racing\firmware\vehicle\CarTest'
Set-Location $project
python .\tools\analyze_differential_runs.py `
  --manifest '.\test-data\differential-20261013\log-analysis\trial-manifest.json' `
  --artifacts '.\test-data\differential-20261013\control\ARTIFACTS.json' `
  --output-dir '.\test-data\differential-20261013\log-analysis' `
  --diagnostics-root '.\logs\runs'
```

当前 `trial-manifest.json` 仍为空；本次真实 CLI 分析因此记录为 `insufficient`、`recommendation: null`、0 次 manifest run 和 5 条失败导出诊断。最终 stdout、退出码、测试日志和输出状态见 [`log-analysis/VALIDATION.md`](../test-data/differential-20261013/log-analysis/VALIDATION.md)。

## 评分时间与指标口径

初始评分从 `event_start` 后**第一次道路 ready 记录**开始，不采用固定 150 ms 启动剔除。ready 条件是 `mode=10`、可靠路线、requested=500‰、`servo_applied_us` 在 1100–2500 µs、左右 applied 各在 400–600‰ 且和为 1000‰。`--startup-exclude-ms` 默认不额外剔除；指定时只在首个 ready 点之后追加。启动 ramp 尚未达到 ready 的记录不纳入跟随/误差评分。

视觉保持过期后，等待与零输出恢复 ramp 都从跟随、视觉误差、舵机及电机跟随评分中排除，直到再次满足同一 ready 条件；恢复等待单独记录。hold、hold-expired、timeout 和通信故障的时长/事件数覆盖整个物理 session（`event_start` 至 `event_stop`），包含首次 ready 前和恢复等待。`event_start`/`event_stop` 必须与 PE7 start/stop 事件相符。

持续量按相邻记录实际经过的毫秒积分，不按事件行数求平均。同一 HAL ms 的多条事件记录允许 `dt=0`：该间隔没有时长积分，但行内事件仍保留。HAL u32 回绕用 `t_ms` 模差与 `t_elapsed_ms` 差值交叉验证；倒退、回绕不一致或默认超过 `3 × period_ms` 的记录间隔会使数据不可用于评分。

可靠路线要求 `mode=10`、latest vision 有效且新鲜、无 timeout、无 active/expired hold；不额外设置 confidence 阈值。只有通过档才按每次 session 指标的中位数排序：hold 总时长、可靠路线视觉绝对误差、requested-vs-limited 平均/最大差、requested-vs-applied 平均/最大差，最后差值相同时优先较小 d。通信 soft fault 单独统计，不等同于 hard fault。舵机差值仅表示日志中的软件 requested/limited/applied 数值，不测量机械响应或真实舵机延迟。编码器结果只报告 signed counts/s，不换算轮径、距离或 m/s。
