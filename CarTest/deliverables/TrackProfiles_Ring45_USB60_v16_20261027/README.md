# Ring45 v16 交付说明

## 当前样本与验证边界

- Root 于 2026-10-27 确认：样本相机已部署 23 个 Python 文件及 v16 native APP；固件 SHA-256 为 **BAA2B1DE63A2DB71416DA23321C1A628CFD634E3AEC20F16C508E793A156326C**，APP 区整区回读通过。
- 当前 H743 仍是 C 档原有 ELF 状态，未重刷，本轮未启动电机。三档 ELF 仅供选择，不要同时刷入。
- 闪存空闲 **5,885,952 B** 是当时读数，空间会随诊断自动保存变化；标定文件未改。保留相机上的 ring_width_calibration.json。RAM 诊断和环形历史自动保存会占相机闪存；电脑归档目录留在 PC，不要复制或留在相机上。
- 台架数据来自静止直道和 MT9V034 320×120：IDE native zoom / Q10 JPEG 六个完整 10 秒窗为 **61.3 / 61.0 / 61.1 / 61.3 / 61.4 / 61.5 FPS**；无预览 headless 窗为 **84.0 / 82.7 / 82.7 FPS**。active C lease 使用合成输入，安全 wire 使用 invalid 帧；这些数据不代表移动赛道表现。
- 真实停车 C 冷启动通信快照单窗为 **98 FPS**；另一独立 scalar 窗为 **96 FPS**，采样窗口不同，不应合并成同一个 1 秒测量。该窗 missing/reject/invalid、输出、request、start 均为 0。
- 本轮尚无非空实车 CSV，也没有移动场景 FPS 或道路验收；环岛、弯道和完整赛道结果仍待用户架空检查及落地验证。

## H743 档位

目标为轮速设定，不是测得的车体速度。只按需要选择一个 ELF。

| 档位 | CarTest 相对路径 | 直线目标 | 外侧弯道目标 | 环控制 |
|---|---|---:|---:|---|
| A_FastNoRing | build/DebugFastNoRing/CarTest.elf | 5.6 rps | 4.4 rps | 关闭 |
| B_RingBase | build/DebugRingBase/CarTest.elf | 4.8 rps | 4.0 rps | 开启 |
| C_RingFast | build/DebugRingFast/CarTest.elf | 5.6 rps | 4.4 rps | 开启 |

## OpenMV 相机

- 本样本使用独立包 deliverables/OpenMV_Ring45_Native_v16_20261027。artifact-manifest.json SHA-256：d935ca1799b0a58b522298b9c6c0a14e4a3db8593eab59e2e4bdeed6fd69e1d4；firmware.bin SHA-256：baa2b1de63a2db71416da23321c1a628cfd634e3aec20f16c508e793a156326c。
- Native 固件目标为 OPENMV4P，APP-only，起始地址 0x08040000，不含 bootloader/ROMFS；不保证通用于其他 H750 相机。其他相机须先确认型号和镜像，再由用户手动刷 APP 并完成回读。
- 上传 Python 程序时，将生产 openmv/ 中 23 个 .py 文件一起上传；不要上传 validation/mpy/。保留 ring_width_calibration.json。
- 相机 /flash/restore_before_ring45_v16_20261027 的 21 文件备份，与 deliverables/PreRing45_USB60_20261026-141419.zip 匹配。这是相机内旧 Python 备份。
- deliverables/Before_v16_Promotion_20261027.zip 是电脑侧 promotion 前源码快照，含旧 23 文件、manifest、CMake presets 和三只 H743 ELF；它不是相机内备份，也不表示新路测验收。
- Native 包 rollback/ 是确认过的 J 版 APP 固件回退对（BIN SHA-256 1E471194F0C9B44239AADF70288103DABCCD840B6B884C65827B04A5B33087B7）。它不是 Python 备份或 H743 ELF；J 版仍有已知重复 IDE Run 超时边界，不能据此宣称问题已消失。

## 导出 H743 运行日志

先切换到 CarTest 根目录。运行后手动切 PE7，再按一次取消会话；保持供电，不复位，然后只运行当前档位对应的一条命令：

    Set-Location 'D:\Codex\smart-car-racing-codex-handoff\smart-car-racing\firmware\vehicle\CarTest'

    .\tools\export_run_log.ps1 -Elf build/DebugFastNoRing/CarTest.elf -SharedConnection
    .\tools\export_run_log.ps1 -Elf build/DebugRingBase/CarTest.elf -SharedConnection
    .\tools\export_run_log.ps1 -Elf build/DebugRingFast/CarTest.elf -SharedConnection

- -SharedConnection 是 PowerShell switch，不要写成 -SharedConnection true。
- 若状态为 IDLE，没有 CSV 是正常结果；非空实车 CSV 仍待真实会话记录与验证。