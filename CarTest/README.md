# CarTest 历史归档：turn-response 完成后的 2026-10-14 上午版本

此目录保存 2026-10-14 上午完成的 turn-response 版本，使用 08:41（北京时间）生成的原始 H743 ELF 清单作为版本锚点。H743 源码来自 09:19（北京时间）在后续 log-audit 开始前捕获的 `before/source` 快照；它不是 08:41 同一时刻的文件副本。将一个独立验证副本按原工具链重新配置后，`DebugFollowTurn` 的 ROM Build ID 与 08:41 原始清单一致，因此源码/配置与原始 ELF 配对得到验证。归档保留原始 8 个 ELF，不以新构建产物替换。

OpenMV 部分来自 `CACHED_WIDTH_TURNRESPONSE_20261014`，17 个运行时 `.py` 文件逐个与 turn-response 结束时的 `current_sha256` 对照。归档不含后续 ring、log-audit 或 PI 行为代码。

## 版本变化

- OpenMV 表面 ROI 下界从 80×60 网格第 20 行（160×120 原图 y=40）调整为网格第 8 行（原图 y=16），观测滤波 alpha 从 0.45 调整为 0.70。
- OpenMV 转向权重按 160×120 原图高度分区：上方 30%（y<36）权重 0.2，下方 70%（y≥36）权重 0.8。
- 新增 H743 `DebugFollowTurn`：满转目标约为 70/30，正常舵机步进为 60 µs/20 ms，正常电机斜坡为 20 permille/10 ms。归档未加入 ring 或 PI 行为。
- 原始 8 个 H743 ELF 位于 `build/<preset>/CarTest.elf`；不要在归档目录内直接构建，以免覆盖这些历史二进制。请复制目录后在副本中运行：

```powershell
pwsh -NoProfile -ExecutionPolicy Bypass -File .\tools\build.ps1 -Preset DebugFollowTurn -Fresh
```

`stm32h743xi_flash.ld`、`project-description.json` 和通用 `.vscode` 设置从 04:22 前的已校验快照补充；`.vscode/launch.json` 与 `tasks.json` 使用 08:38 selector 工作区快照。它们的补充来源逐文件记录在 `origin-manifest.json`。源码原始 README 保存在 `docs/archive-origin/README-before-turn-response.md`，本文件是归档专用恢复说明。

## 证据边界

构建验证在隔离副本中进行，且只用于检查构建与 ROM Build ID 配对；归档内 ELF 仍是保存的原始 ELF。未在开发板或车辆上验证，不表示用户曾烧录这些文件，也不证明道路表现。`SHA256SUMS.txt` 校验除自身以外的全部包内文件。
