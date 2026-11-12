# SpeedPD 交付与主模型审查说明

本轮完成了软件修改、宿主验证和构建，尚未上传相机、烧录 H743、复位或启动电机。45 帧/秒、4.8 转/秒目标跟踪及实际过弯、上坡效果仍须板端和实车验证。

## 第一轮使用的配套文件

所有路径均相对于 CarTest 目录。

| 用途 | 文件或目录 |
| --- | --- |
| C 档：增量式 PI，直行目标 4.8 转/秒 | `deliverables/H743_SpeedPD_20261020/C_IncrementalPI/CarTest.elf` |
| B 档：65/35 开环对照 | `deliverables/H743_SpeedPD_20261020/B_OpenLoop/CarTest.elf` |
| 相机第一轮：新权重，PD 关闭 | `deliverables/OpenMV_Bottom80_Steering_20261021/no_pd` |
| 相机第二轮：位置 PD 开启 | `deliverables/OpenMV_Bottom80_Steering_20261021/pd` |
| 最后可选对照：PD 与自适应预瞄 | `deliverables/OpenMV_Bottom80_Steering_20261021/pd_preview` |

三套相机配置均可配合同一份新 C 档或 B 档固件。第一轮用 C 档配 `no_pd`，先架空验证执行和日志，再比较 PD。生产源码目录 `openmv` 默认是 `pd`，不要将它误当成第一轮 `no_pd`。

一次上传所选目录内全部 20 个 `.py` 文件；不要混用配置，不上传测试文件或验证产生的 `.mpy`。相机切换后启动行应分别显示 `BOTTOM80_NO_PD_20261020`、`BOTTOM80_POSPD_20261020` 或 `BOTTOM80_PD_PREVIEW_20261020`。相机上传与 H743 烧录是两个独立步骤。

VS Code 原有 C 项仍指向 `build/DebugFollowIncrementalPI/CarTest.elf`；交付时它与上述 C 档快照完全一致。以后重新构建可能改变 build 目录，日志导出必须选择实际烧录版本对应的 ELF。

## 本轮参数

- Follow 输出保护上限 100%，不是固定输出 100%；台架仍为 40%。开环各档原直行和差速目标保留。
- C 档左右目标单轮最高 4.8 转/秒；直行 4.8/4.8，最大转向按 65:35 比例约 4.8/2.584 转/秒，左右镜像。PI 增益保持原值。
- 舵机 1100～2500μs，正常变化限制 120μs/20ms；左右电机正常变化限制各 4 个百分点/10ms。启动和停止后恢复保留原慢爬升与完整 PWM 检查。
- 丢线保留 200ms 减速、2 秒截止与连续三帧恢复；通信超时 250ms 和 PE7 再按停车规则保留。
- 整幅图 y=24 分组：下 80% 占权重 0.8，上 20% 占 0.2；上部至少三行且跨度至少两格，否则降级只用下部有效路线。下部无合格支持则无效。
- PD 在相机侧基于滤波后位置偏差计算；D 时间系数 30ms、低通 60ms、贡献限制 ±100。预瞄独立开关默认关闭，确认入弯后才受限介入。12 字节 UART 协议不变。

## C 档日志导出

手动再次按 PE7 停车后保持核心板通电，不复位、不重新烧录。然后在 PowerShell 执行：

```powershell
cd 'D:\Codex\smart-car-racing-codex-handoff\smart-car-racing\firmware\vehicle\CarTest'
powershell -NoProfile -ExecutionPolicy Bypass -File .\tools\export_run_log.ps1 -Elf .\deliverables\H743_SpeedPD_20261020\C_IncrementalPI\CarTest.elf
```

B 档试跑时把 ELF 换成 `B_OpenLoop\CarTest.elf`。脚本使用 HOTPLUG，并保留所选探针与成功连接频率；不自动复位或烧录。

正常 C 档冻结会话导出 `run.csv`、`speed-pi-trace.csv` 和相应元数据。脚本核对固件身份、会话、冻结前后快照及 CRC。IDLE/ACTIVE 不作为成功的行驶 CSV；轮速诊断损坏时即使已有有效运行 CSV，也会以非零退出码报告诊断不完整。

实时监视使用 `car_rx_frame_rate_millifps`，除以 1000 得到接收帧/秒，约每秒更新；`car_rx_frame_rate_valid` 表示窗口有效。例如 45000 表示接收 45 帧/秒。该值包括无效视觉帧，不能替代相机采集处理帧率；无效占比另看 `car_rx_invalid_observation_permille`。

## 审查及验证结果

- 最终 11 个声明 H743 配置全部构建成功，构建前后源码输入哈希一致。
- C 宿主套件 70 案例、7169 断言；实际 main 输出路径 8 种配置、2499 检查；PI main 27 断言，均通过。最终输出 1000‰ 允许，1001‰ 仍故障停车，Bench 400‰ 允许、401‰ 拒绝。
- OpenMV 生产宿主测试 183 项通过；冻结 V7 回归共 79 项，78 通过、1 项板端测试跳过。
- 三套配置共 60 个模块完成 mpy-cross 1.25 ARMv7EM-SP 编译。本机没有与所报板端 MicroPython 1.28.0-49 匹配的编译器，故不能把此结果当作 1.28 实际运行验证。
- 导出模拟测试覆盖完整冻结导出、CRC 损坏、IDLE/ACTIVE 和会话不匹配；运行日志 64 字节布局及 V1/V2 兼容保持。真实硬件 HOTPLUG 连接是否成功仍取决于本次接线和供电。
- 同一组合成输入的主机测量中，行权重工作区由全清零改为仅清理已用标志后，中位耗时约下降 13%。这只是局部 CPython 结果，不表示图像流水线或板端达到 45 帧。

C 档 ELF SHA256：`ed89731d79262608e51f7f45db796af8531745245fe3cceab35620c2ce086aa7`；build ID：`7d85f31537056cd909b2dbb1e7790348`。

B 档 ELF SHA256：`da015c3940388f908a0e622bc9eae9354465c5c41380a1461aee6b2370b143a3`。

完整构建记录与文件哈希在 `deliverables/H743_SpeedPD_20261020`；相机各目录自带 `PACKAGE_MANIFEST.json`。私有测试证据在 `test-data/speedpd-20261020-validation`。

## 回退与实测顺序

修改前源码、配置与配套 ELF 恢复包为 `deliverables/CarTest_PreSpeedPD_20261020-174128_Restore.zip`，SHA256：`40518d4b929c9e407bf6f6bdea9db6c8fb748b9c939a46f9c0459600979c69fd`。原 `H743_Uphill_20261019` 文件未覆盖。

先架空验证方向、启动、停车及非空日志；再平地检查轮速和饱和；然后单弯、连续弯比较 `no_pd` 与 `pd`；确认后才试 `pd_preview`。清洁轮胎与坡面后重复上坡三次，并记录同次视频及日志。直道、弯道、单边恢复各采集至少 30 秒板端性能，以每个 10 秒窗口的新帧吞吐验证 45 帧目标。轮速不是车身速度，PWM 也不是机械转角；100% 软件上限不证明持续满负载能力。
