# CarTest 历史归档：2026-10-14 04:22（北京时区）之前

此目录用于恢复 2026-10-14 04:22 北京时间前的 CarTest 代码快照。H743 源码来自控制/工作区清单 `created_utc=2026-10-13T20:22:06.3934321Z`，即 2026-10-14 04:22:06 北京时间；配套保留清单中的 7 个原始 `CarTest.elf`。根目录 `README.md` 是本归档的中文恢复说明，原始源码 README 保存在 `docs/archive-origin/README-source-0422.md`。

OpenMV 部分使用对应的 pre-turn-response 包 `CARTEST_OPENMV_CACHED_WIDTH_DIFFTRIAL_20261013`，并保留其 17 个运行时 `.py` 文件及非日志测试源码。该 OpenMV 包的快照记录只有日期，没有精确到 04:22 的采集时刻，因此它表示对应的 turn-response 之前版本，不声称与 H743 在同一秒采集。

## 版本内容

- OpenMV 表面 ROI 下界为 80×60 网格第 20 行，对应 160×120 原图 y=40；观测滤波 alpha 为 0.45。
- H743 保留当时的 7 个原始预设 ELF，路径为 `build/<preset>/CarTest.elf`。这些是历史二进制，不由本归档重新构建生成。
- `Drivers/` 是从 `test-data/baseline-restore-20261015/validation/Drivers/` 补入的共享 STM32 vendor 依赖，共 176 个文件；它不是 04:22 快照的一部分，逐文件来源和 SHA-256 记录在 `origin-manifest.json`。

## 构建与恢复

请先复制解压目录作为独立构建副本，再在副本中运行构建，以免覆盖归档内的历史 ELF。示例（在副本根目录运行）：

```powershell
pwsh -NoProfile -ExecutionPolicy Bypass -File .\tools\build.ps1 -Preset DebugFollowDiff10 -Fresh
```

`SHA256SUMS.txt` 覆盖包内所有其他文件（校验文件本身不对自身做哈希）。`origin-manifest.json` 记录原始快照输入、补充依赖、排除的日志/缓存、ELF 哈希和 ROM Build ID。

## 证据边界

本归档只记录源码、原始 ELF 和独立副本上的编译/ROM Build ID 配对证据。未在车辆或开发板上验证，不表示用户曾烧录这些 ELF，也不证明道路表现。
