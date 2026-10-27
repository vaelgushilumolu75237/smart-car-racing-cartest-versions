# Ring45 离线帧率与环岛阈值实验 v17

本交付是**仅供离线验证的实验候选包**，状态为 `experimental-offline`。
`PACKAGE_MANIFEST.json` 中 `candidate_only=true`、`hardware_validation=false`、
`deployment_eligible=false`、`board_fps_verified=false`。本次没有相机上传、
H743 烧录、串口连接或电机启动；没有完成板上帧率测量，也没有完成赛道验收。

## 内容与配对

`OpenMV/` 包含本生产目录的 23 个 Python 运行模块。请始终将它们作为一组看待；
`validation/mpy/` 中的 `.mpy` 文件只用于 `armv7emsp` 交叉编译验收，不上传相机。
`SHA256SUMS.json` 覆盖本交付中的全部其它文件。没有加入本地赛道照片或私有采集帧。
相机上传时请将 `OpenMV/` 下 23 个 Python 文件与 `PACKAGE_MANIFEST.json` 作为一组复制；
不要格式化或擦除相机。原有私有 `ring_width_calibration.json` 只可用于同一相机、镜头和
安装位置；相机维修或更换、或光学/安装条件改变后，须重新检查并校准。本交付不包含该私有校准文件。

| 配置 | H743 预设 | ELF SHA-256 |
| --- | --- | --- |
| A_FastNoRing | `DebugFastNoRing` | `e94c3962e5987f5905d324aabe66ea850e69cf55de52edf2d330933cef14aee6` |
| B_RingBase | `DebugRingBase` | `3dea2331c3c019de33e62dd974265b451247b90a9ae20b36dfc47d171f569da1` |
| C_RingFast | `DebugRingFast` | `0656376f4165745cfb8d26526af3660d65a6247b5c3deec14e4d5b0a204d820d` |

A/B/C 三个 H743 ELF 都沿用 v16 配对文件，和冻结前备份及现有 v16 交付逐字节一致；
本次没有重建或改动 H743 固件。配置 A 关闭环岛路线，B/C 保持各自既有配置。
本实验只放宽环岛入口对**连续向外生长弧线**的识别证据；目标点有效性、当前边界
支持、宽度校准、故障失效关闭、普通弯道与环岛分流等保护条件仍必须成立。
入口确认状态不代表已经得到可行驶的有效中线；若当前目标构造被保护条件拒绝，车辆仍
不得据此转入环岛路线。`V7RING` 会在已有的 1 Hz 调试输出中区分弧线确认、目标有效性
和目标拒绝原因，供离线日志定位。

## 本次环岛入口候选参数

下表只调整环岛入口视觉候选识别，不放宽有效参考线或有效目标的保护条件。

| 参数 | v17 值 | 作用范围 |
| --- | ---: | --- |
| `TRACK_ELEMENTS_RING_ENTRY_MIN_NEAR_SUPPORT` | 4 | 环岛入口近端支持行数 |
| `TRACK_ELEMENTS_RING_ENTRY_MIN_NEAR_CONTIGUOUS` | 3 | 环岛入口近端连续支持行数 |
| `TRACK_ELEMENTS_RING_ENTRY_OPPOSITE_POSITION_TOLERANCE_X2` | 10 | 环岛入口对侧位置容差，单位为半格 |
| `TRACK_ELEMENTS_RING_ENTRY_REAPPEAR_SHIFT_X2` | 6 | 环岛入口边界重现位移门限，单位为半格 |
| `TRACK_ELEMENTS_RING_ARC_MIN_ROWS` | 4 | 入口弧线最少支持行数 |
| `TRACK_ELEMENTS_RING_ARC_MIN_CONTIGUOUS_ROWS` | 3 | 入口弧线最少连续支持行数 |
| `TRACK_ELEMENTS_RING_ARC_MIN_SPAN_ROWS` | 4 | 入口弧线最小纵向跨度 |
| `TRACK_ELEMENTS_RING_ARC_MIN_GROWTH_X2` | 2 | 入口弧线最小向外增长量，单位为半格 |
| `TRACK_ELEMENTS_RING_ARC_GROWING_SUPPORT_FRAMES` | 2 | 连续弧线增长确认帧数 |
| `TRACK_ELEMENTS_MAX_ENTRY_MS` | 4000 | OpenMV 视觉入口候选等待上限 |

`TRACK_ELEMENTS_MAX_ENTRY_MS=4000` 仅限制视觉入口候选等待，不改变 H743 现有的失帧
续行 2 秒、200 ms 减速或 250 ms 通信停止逻辑。环岛路径仍为实验状态。曾有单边缘
双侧歧义的真实赛道现象；离线复现尚未重现该情况，因此本版本不声称已修复它。`V7PAIR`
只为后续实机诊断补充配对拒绝原因，不是修复证据。旧 v16 板上性能结果不能
作为本版本的 FPS 或道路表现证据。

## 固件依赖和停止规则

继续保留并依赖原有 OpenMV4P native v16 APP：`deliverables/OpenMV_Ring45_Native_v16_20261027/firmware.bin`，SHA-256 `baa2b1de63a2db71416da23321c1a628cfd634e3aec20f16c508e793a156326c`；
其 artifact manifest SHA-256 为 `d935ca1799b0a58b522298b9c6c0a14e4a3db8593eab59e2e4bdeed6fd69e1d4`。
该 APP 未重建、未替换，仍保持独立原件。此处的依赖记录不扩大其适用范围；它是
特定样机的 APP-only 备份，不代表兼容 H7R2，也不能直接烧录到新板；该固件的道路测试
尚未完成。
该 APP 原 build-verification 记录的 Python 源码哈希与冻结的 v16 目录 deliverables/TrackProfiles_Ring45_USB60_v16_20261027/OpenMV 核对；
该目录清单 SHA-256 为 c1401effed0e07a499d30962c68fb1473a49b0303cdac7401c14007654e6bcad。该历史 APP 未按 v17 Python 源码重建，也不构成新板验收。

本轮没有上传候选包。相机修复并确认板型、电源正常后，可先在停车且电机禁止条件下加载
整套 Python 作板端验证；通过后再架空、落地分步实测。本版本尚不能作为已验收版本部署。
若出现持续异常转向、存在出界风险或触发保护停车，结束该轮测试并恢复冻结的 v16 配对包；
不得绕过无效观测、通信超时或本地启动确认等既有安全门。

当前证据仅覆盖主机回归和 `mpy-cross` 编译，不证明相机运行时帧率、H743 UART 接收、
转向效果、环岛通行或完整赛道表现。
