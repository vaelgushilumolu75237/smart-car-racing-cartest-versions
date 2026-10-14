# CarTest 2026-10-14 下午环岛修改前基线

这是一个独立的恢复归档，选取了 2026-10-14 下午环岛修改前的最后一组可重建快照：

- OpenMV 生产包：本地时间约 14:24，包标识 `CACHED_WIDTH_TURNRESPONSE_20261014`。
- H743 源码和配对 ELF：本地时间约 14:25，来自 `ring-slope-20261014/control/before`。
- 这组包对应用户所说的 14 号上午版本状态（下午环岛修改前）；独立的 17 个顶层运行时 `.py` 哈希与 `test-data/turn-response-20261014/vision/final/source-hashes.csv` 的 `current_sha256` 行逐项相符（17/17）。该比对是文件来源核对，不代表用户当时实际上传或车辆验收。
- 归档中的 `Drivers` 是当前共享的 STM32 HAL/CMSIS 供应商目录，因为历史源码快照本身没有保存 `Drivers`；它在来源清单中单独标记，不能当作历史源码哈希证据。
- `tools`、`docs`、部分测试文件优先来自 2026-10-14 09:19 的日志导出快照；两个为当前补入的必需工具也在来源清单中标记。

## 恢复 OpenMV

`openmv` 目录内的 17 个顶层 `.py` 文件必须作为一个完整、同版本的包上传到 OpenMV 文件系统根目录。不要把当前项目中的其他版本文件混入。`PACKAGE_MANIFEST.json` 和本目录的 README 只用于核对；上传时以清单列出的 17 个运行时 `.py` 文件为准。

归档没有包含 `openmv_clipped_midpoint`，也没有重新生成对照版。

## 恢复 H743

`build/<预设>/CarTest.elf` 保存了当时保留的 8 个 ELF：`Debug`、`DebugSteering`、`DebugBench`、`DebugFollow`、`DebugFollowNoLog`、`DebugFollowDiff5`、`DebugFollowDiff10` 和 `DebugFollowTurn`。这些文件没有在归档过程中重建或覆盖。

- 要复现旧版板端程序，优先使用对应的保留 ELF，并记录它的 SHA-256/Build ID。
- 在 VS Code 中按 F5 会按当前源码重新构建并产生新的 ELF；即使编译成功，也不能把它称为保留 ELF 的字节相同副本。
- 本归档不能证明车辆当时实际烧录的是哪一个 ELF，也没有在本次打包中烧录、复位、启动电机或做赛道验收。

## 核对文件来源

- `origin-manifest.json` 记录每个归档文件的来源、来源快照和打包后 SHA-256。
- `SHA256SUMS.txt` 是归档目录文件的校验清单（清单文件自身不做递归自校验）。
- 外部 `test-data/baseline-restore-20261015/archive-manifest.json` 记录最终 ZIP 的条目核对、校验和以及单次独立构建验证结果。

这是一份恢复基线，不是对当前实验版本、环岛策略、坡道助力或 PI 参数的确认；是否适合车辆运行必须重新完成硬件、台架和赛道验证。
