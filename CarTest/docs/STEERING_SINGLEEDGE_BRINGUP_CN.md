# 单边界循迹与舵机保护现场验收

本说明对应 2026-10-11 的 CarTest 单边界视觉与舵机保护版本。它给出软件接口、现场验收顺序和证据边界；软件通过不代表 H743 引脚、舵机机构、OpenMV 实板或赛道表现已经通过。

## 当前问题与结论边界

现场曾观察到转向机构的机械位置越过了原先记录的 1100 μs 标定位置；当时具体请求值和实际 PA8 脉宽均未知，也没有同步的示波器/逻辑分析仪记录。因此不能据此认定 MCU 实际输出了 1100 μs，也不能认定固件、舵机、连杆、供电或控制时序中的哪一项是原因。此次脉宽保护和单边界视觉改动没有证明机械问题已经解决。

当前软件把舵机请求约束在 1100–2500 μs，初始化目标中心值为 1530 μs；单边界 DebugSteering 在归一化误差 ±1000、现有 0.7 增益下自然请求范围约 1229–2209 μs。统一的输出变化率限制为每个 20 ms 舵机周期最多变化 10 μs。1530 μs 是软件命令，不是位置传感器读数；变化率限制约束连续数字命令，不能控制上电时未知的机械初始位置，也不等同于机械端点已经校准。舵机断电位置、堵转电流和机构行程仍需实测。

## 软件配置和启停

CarTest 保留四个互斥构建配置：

| Profile | 用途与输出边界 |
| --- | --- |
| `Debug` | 电机和舵机 PWM 均关闭，用于上电、通信和状态观察。 |
| `DebugSteering` | 仅此配置可输出舵机 PA8 诊断波形；电机输出关闭。逻辑请求受统一范围和变化率限制。测 PA8 时舵机 5 V 与电机 VM 必须物理断开。 |
| `DebugBench` | 架空电机诊断代码仍可构建，但电机验收许可关闭，电机输出禁止；舵机 PWM 也关闭。 |
| `DebugFollow` | PE7 仍沿用原按住/松开会话语义，松开即停止并取消本次运行资格；电机验收许可关闭，电机和舵机 PWM 均禁止。视觉恢复不能把 PE7 改成 toggle。 |

本版本的电机硬件验收许可默认关闭，电机保持禁止。没有完成并记录对应的物理验收前，不更改该许可，也不执行电机启动。本文不提供解除该锁的操作步骤。DebugSteering 中 PE7 不承担普通舵机启停，必须按当前 profile 的既有语义操作；不要把 Follow/Bench 的 PE7 行为套用到 DebugSteering。

视觉数据中的 `confidence` 只作诊断值。主控是否接受一帧仍取决于帧格式/CRC、序号推进、数据年龄、`valid` 和既有超时条件；较高置信度不能绕过这些条件。无效或含糊的单边界观测不得由缓存值伪装成当前有效图像。

## H743 与 OpenMV 的配套接口

本次必须把配套 H743 和 OpenMV 文件作为同一版本使用。相机需要接收 H743 的反向控制字节，单向 P4/TX 到 H743 的接线不足以启用该模式：

- OpenMV P4/TX → H743 PA10/RX。
- OpenMV P5/RX ← H743 PA9/TX。
- 两板共地；UART 为 115200、8N1、3.3 V 逻辑电平。

反向控制帧固定 8 字节：`A6 5A`、版本 `1`、操作码 `0=RESET` 或 `1=KEEP`、小端 `uint16` 序号、以及覆盖前 6 字节的小端 CRC-16/CCITT-FALSE。序号回绕按协议处理。控制帧按 50 ms 间隔发送，启动 RESET 阶段为 250 ms，KEEP 有效租约为 250 ms；缺失、过期、坏 CRC 或乱序控制不得被当成有效双向会话。没有有效反向链路时只允许原有双边界视觉路径，单边界回退必须保持关闭。接线和协议只能靠实板通信记录确认，静态代码或 PC 测试不能证明 PA9/P5 的电气连接。

视觉到主控的既有 v1 观测仍是独立的固定 12 字节协议。相机通过串口发送完整观测只证明它完成了本地写调用；H743 必须实际收到并接受新帧才会更新观测状态。`confidence` 不是有效性或安全许可。

## 构建和匹配 ELF

在当前工作区：

```powershell
Set-Location 'D:\Codex\smart-car-racing-codex-handoff\smart-car-racing\firmware\vehicle\CarTest'
powershell.exe -NoProfile -ExecutionPolicy Bypass -File .\tools\build.ps1 -Preset DebugSteering -Fresh
```

如果把交付 ZIP 解压到 `D:\`，工程目录为：

```powershell
Set-Location 'D:\smart-car-racing\firmware\vehicle\CarTest'
powershell.exe -NoProfile -ExecutionPolicy Bypass -File .\tools\build.ps1 -Preset DebugSteering -Fresh
```

对应目标必须是 `build\DebugSteering\CarTest.elf`。其他 profile 只能使用各自 `build\<Preset>\CarTest.elf`。本次 DebugSteering 的 VS Code 启动项精确名称为 `CarTest H743 - DebugSteering (motors locked; vision steering diagnostic)`，前置任务为 `CarTest: Build DebugSteering (motors locked; vision steering diagnostic)`；启动项必须指向对应 profile ELF，避免误用旁边配置。OpenMV MicroPython 文件另行整套上传到摄像头，不通过 STM32 ELF 烧录。此次文档/构建工作没有烧录 H743，也没有上传 OpenMV。

PowerShell 命令上的 `-ExecutionPolicy Bypass` 只作用于该构建子进程。构建成功和 ELF 文件存在不代表烧录成功；烧录后需记录实际选中的 ELF 绝对路径、修改时间、SHA-256、STM32CubeProgrammer/VS Code verify 结果和芯片启动状态。

## PA8 舵机波形和机械检查

此阶段只测数字波形：舵机 5 V 必须物理断开，电机 VM 也必须物理断开，只给 H743/OpenMV 供电。示波器或适合 3.3 V 信号的逻辑分析仪以 H743 板端 GND 为参考测 PA8/TIM1_CH1；不要把探头地接到电机输出端，也不要给舵机上电。记录探头倍率、采样率、周期和每个脉冲的高电平宽度。机械带电验收须等波形验收通过，并由现场负责人确认机构状况和原问题处置后另行安排；本说明不授权在波形验收前给舵机 5 V。

至少分开记录这些阶段的波形：

1. 复位、上电和初始化完成前后；
2. 1530 μs 中心请求及其稳定后的重复周期；
3. DebugSteering 当前映射可达到的左右请求（误差约 ±1000 时为 1229 μs 与 2209 μs），并核对实际输出的逐周期变化；
4. 软件硬限值 1100 μs 与 2500 μs 已有主机边界检查证据，但普通 DebugSteering 相机输入不能自然请求到这两个值。只有交付中存在且明确匹配当前锁定 ELF 的专用边界诊断程序时，才可按该程序的安全条件单独测量；没有该诊断程序就将 PA8 的 1100/2500 μs 实测记为“未测”，不要用 Live Watch 写变量或临时修改固件强行生成端点脉冲；
5. 视觉软故障/超时后的停止，以及重新满足新鲜帧条件后的恢复；
6. 首个完整 PWM 帧就绪边界。

检查周期是否为约 20 ms（50 Hz），并将 PA8 实测脉宽与当时的舵机请求、舵机输出状态、模式、故障和帧新鲜度记录对齐。当前源码符号包括 `car_servo_requested_us`、`car_servo_limited_us`、`car_servo_applied_pulse_us`、`car_servo_output_active`、`car_servo_frame_ready`、`car_servo_fault_latched`、`car_servo_sticky_fault_code` 及 `car_servo_fault_snapshot_*`。这些只是软件对照；`car_servo_frame_ready` 表示软件已观察到完整更新事件，仍需用 PA8 波形验证实际引脚。

1100 μs 和 2500 μs 是软件硬限值，不代表未知舵机/连杆可以安全到达这两个机械端点，也不是本说明要求用相机输入触发的脉冲。此前记录的标定范围在本次事件后尚未重新核实为安全机械行程。当前事件的请求值与 PA8 脉宽未知、原因未证实；在 PA8 波形验收通过且现场负责人复核机构状态和事件处置前，不做舵机带电机械测试，也不尝试软件端点。若后续经现场负责人另行安排的受控验收中出现碰限位、持续嗡鸣、卡滞或异常发热，应立即切断舵机 5 V 并记录；不得通过扩大软件端点来处理异常，也不得把原因预先归给本次固件。

首个完成的 20 ms PWM 帧才可作为舵机输出就绪证据；只看到 HAL 初始化成功、软件目标值或 `car_servo_output_active` 不够。测量结果应保留原始示波器/逻辑分析仪文件或带游标的截图。

## OpenMV 实板 45 次/秒验收

当前软件/主机回归不构成 45 次/秒实板证据。使用实际安装的 OpenMV，确认相机接线和 PA9 反向链路后，连续做 3 轮、每轮 60 秒的测量；每 10 秒独立统计至少 450 次新鲜观测写入，并记录 H743 收到的递增序号/帧年龄。统计应覆盖正常垃圾回收、串口日志和叠加显示负载。记录摄像头型号、固件、曝光/分辨率、BUILD、源包哈希、串口计数器、丢帧/无效帧、每轮开始结束时间和原始日志。

结果需分开报告 OpenMV 本地完整写入数、H743 接收/接受数和观测新鲜度；OpenMV 的 `uart.write()` 返回值、`clock.fps()` 或 PC 仿真不能单独证明 H743 收帧，更不能证明赛道识别准确率。没完成这组实板记录前，不报告“已达到 45 FPS”。本版本沿用冻结 V7 的主体检测与拒绝基线，不据此声称所有边界/环岛歧义已经解决；裁剪/局部环岛触及 ROI 时，画面可能不足以证明其属于外边界，真实环岛和单边界赛道画面仍须单独验收。

## 现场记录表

每次验收复制并填写。未知项写“未测”，不要推测。另有空白[现场验收记录模板](../test-data/steering-singleedge-20261011/delivery/physical-acceptance-record-template.md)，用于分开记录 PA8 波形、OpenMV 实板以及后续机构/赛道证据。

| 项目 | 记录 |
| --- | --- |
| 日期、地点、操作者、CarTest profile |  |
| H743 ELF 绝对路径、SHA-256、Verify 结果 |  |
| OpenMV 型号/固件、BUILD、整套包 SHA-256 |  |
| UART 正向与反向接线、共地和实测电平 |  |
| PE7 实际状态及当前 profile 预期语义 |  |
| PA8 数字波形阶段的电源状态：舵机 5 V 物理断开、VM 物理断开 |  |
| 示波器/逻辑分析仪型号、探头、地参考、采样设置 |  |
| PA8 周期及 1530 μs 中心测量值 |  |
| 启动、完整帧就绪、左右请求、故障停止/恢复测量值 |  |
| 1100/2500 μs 端点脉冲测量：匹配 ELF 的专用诊断；没有则记“未测” |  |
| 仅未来另行批准的舵机带电机构验收记录；本次保持“未测” |  |
| 该后续验收的舵机型号/5 V 实测值、舵角、回差、过冲/嗡鸣/卡滞、断电行为 |  |
| 三轮 OpenMV 实板数据；每 10 秒 fresh writes/接收/年龄 |  |
| 赛道、灯光、安装位置、连续运行和故障停止记录 |  |
| 原始波形、串口日志、视频和现场记录文件路径 |  |

PA8 波形、OpenMV 45 次/秒实板结果和场地循迹结果是三类独立证据；其中任一缺失都保持“未测”。软件 host tests、交叉编译、OpenMV 主机写入模拟和 zip 哈希均不能替代它们。
