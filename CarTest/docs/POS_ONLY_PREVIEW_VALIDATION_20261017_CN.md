# 普通道路预瞄关闭版：软件验证与变更清单

日期：2026-10-17  
OpenMV Build ID：`AGGRESSIVE_TURN_POS_ONLY_20261017`  
Git 版本标签：`v20261017-position-only`  
发布状态：`experimental-software-tested`

## 变更清单

- `openmv/config.py` 把 Build ID 更新为本版，并设置
  `ORDINARY_PREVIEW_CONTROL_ENABLED = False`。
- `openmv/main.py` 的生产聚合路径显式传入该开关。普通 `road` 路线的滤波前
  控制量只保留现有 80/20 加权位置误差；预瞄候选仍用于诊断，实际应用的预瞄
  贡献为 0。旧 helper 的默认值仍为启用，避免改变其独立调用的既有行为。
- 启动信息显示 `ordinary_preview=off`；画面标记 `P_OFF`，并保留候选值。
  `CTRL=POS` 是本说明对控制方式的简称，不是新增的 UART 字段。
- UART V1 的 12 字节帧、发送路径和当前 `OBSERVATION_FILTER_ALPHA=0.70`
  观测滤波均保留。H743 源码、参数和构建产物没有改变。
- `openmv/tests/test_turn_response_weights.py` 检查开关必须是布尔值，并覆盖
  配置属性完全缺失的情况。`openmv/tests/test_steering_diag.py` 运行生产主循环，
  检查实际 UART 帧中的滤波后误差同时匹配位置控制值和 RAM 诊断记录。
- `docs/POS_ONLY_PREVIEW_20261017_CN.md` 说明控制和滤波边界、上传文件清单及
  OpenMV IDE 与 VS Code 的连接操作。VS Code 的 Run 执行当前编辑器内容，不能
  代替上传整包；从 OpenMV IDE 断开后保留 USB 连接。

## 软件验证

- 生产 OpenMV 主机回归运行 17 个套件、共 245 项：244 项通过，1 项板端专属
  Viper 等价性检查因没有板端 manifest 而跳过；所有 17 个命令均 exit 0。
  其中包含旧 V7 冻结回归（79 项运行，78 项通过，1 项跳过）和预瞄专门套件
  （22 项通过）。逐项完整输出见
  `test-data/position-only-preview-20261017/validation/openmv-host/complete-host-regression.log`。
- 20 个生产 `.py` 运行文件均由 OpenMV IDE 附带的 mpy-cross 成功编译，目标参数
  为 `-march=armv7emsp`。编译器为 MicroPython v1.25.0（输出 mpy v6.3）。
  完整输出和每个文件的 `.mpy` 哈希见
  `test-data/position-only-preview-20261017/validation/mpy-cross-armv7emsp/full-mpy-cross.log`。
  这些结果不代表摄像头固件上的运行验证。
- 基线清单内的 336 个源码文件和 16 个 ELF 均通过 SHA-256 复核；当前 H743
  `App/Inc/Src/Drivers/cmake` 源码树、CMake 文件与基线逐文件相同。当前 16 个
  H743 ELF 与保留基线的 SHA-256、文件大小全部相同，没有重建或烧录。核对结果见
  `test-data/position-only-preview-20261017/validation/baseline-integrity.json`。
- 完整相机上传目录为 `deliverables/OpenMV_PositionOnly_20261017/`，含 20 个
  顶层运行 `.py` 文件、manifest、校验和及简要说明。测试、`.mpy`、车辆日志和
  媒体不在上传目录内。
- 原始源码与 16 个 ELF 的可恢复 ZIP 为
  `deliverables/CarTest_PositionOnly_Baseline_Restore_20261017.zip`；旁边的
  `.sha256` 文件记录压缩包摘要。压缩包内保留原基线 `SHA256SUMS.txt` 和 manifest。
- GitHub 版本检查点按 `CMakePresets.json` 的 9 个非隐藏配置发布对应 ELF 与源码；
  本地恢复 ZIP 另保留全部 16 个原有 ELF，二者用途与文件范围不同。

## 验证边界

本轮没有向相机上传程序，没有烧录或复位 H743，也没有启动电机。主机测试和
交叉编译不能证明相机板端兼容性、H743 对 UART 帧的接收或实车转向效果；本版仍
需在硬件上验证。环岛分支未因本次普通道路预瞄开关而改变。
