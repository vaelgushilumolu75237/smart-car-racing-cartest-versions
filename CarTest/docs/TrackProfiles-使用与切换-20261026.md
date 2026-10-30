# 三档切换、日志导出与下一步实测

本轮软件和静止相机验证已完成；移动过弯、十字和左右环岛尚未验收。不要把“已有状态机”和合成烟测通过理解为实车已稳定通过。

## 当前配套版本

- 相机：`deliverables/TrackProfiles_H750_20261026/OpenMV`，共20个生产 `.py` 文件，整套上传。无需维护或上传 `openmv_clipped_midpoint`。
- 相机厂商固件保持5.0.1，MT9V034，图像320×120；没有刷旧H743相机固件。
- H743：本轮静止验证已烧录 **B档**，相机根目录20个文件也已更新并校验。未通过软件启动车辆。

| VS Code最前面的新入口 | ELF | 直道目标 | 弯道/环岛外轮目标 |
| --- | --- | --- | --- |
| 新A 提速无环岛 5.6/4.4 | `deliverables/TrackProfiles_H750_20261026/H743/A_FastNoRing/CarTest.elf` | 5.6转/秒 | 4.4转/秒，环岛关闭 |
| 新B 原速环岛 4.8/4.0 | `deliverables/TrackProfiles_H750_20261026/H743/B_RingBase/CarTest.elf` | 4.8转/秒 | 4.0转/秒 |
| 新C 提速环岛 5.6/4.4 | `deliverables/TrackProfiles_H750_20261026/H743/C_RingFast/CarTest.elf` | 5.6转/秒 | 4.4转/秒 |

这些是驱动轮目标，实际占空比由增量PI调整，上限100%。不是固定满占空比，也不是已测车身速度。舵机限制沿用稳定协调档实际使用的160μs/20ms，未加大控制增益。

## 怎么切档

1. 按PE7停车。若要保留这次RAM日志，保持核心板通电，先导出；烧录和复位会清除记录。
2. VS Code打开 **CarTest** 项目，在运行与调试下拉框选择最前面的新A、新B或新C，按F5构建并烧录对应档位。
3. 三档使用同一套相机代码，相机会从H743同步档位。不要手动修改相机算法开关，不要选历史旧A/B/C入口代替新入口。
4. 脱离调试后检查启动状态。当前档与相机确认一致且无故障后，再手动按PE7启动；再次按PE7停车。保留首次完整PWM与慢爬升保护。

可在实时监视查看：`car_camera_profile_required_ack` 与 `car_camera_profile_latest_ack` 应相同，A/B/C分别为1/2/3；`car_camera_profile_latest_version=3`，`car_camera_profile_ack_ready=1`。这只表示协议配对，不能证明机械转向或环岛路线正确。

若刚烧录后显示 `watchdog_reset`，先不要启动车辆。本轮读到过烧录停核留下的看门狗复位标志，一次正常软件复位后已清除；没有关闭看门狗或绕过故障保护。务必先导出要保留的记录，不能用反复烧录掩盖运行中的复位。

## PowerShell导出日志

以下是当前 **B档** 的命令。A/C试跑必须将ELF路径换成上表对应档位，不能混用。

```powershell
cd "D:\Codex\smart-car-racing-codex-handoff\smart-car-racing\firmware\vehicle\CarTest"
powershell.exe -NoProfile -ExecutionPolicy Bypass -File ".\tools\export_run_log.ps1" -Elf ".\deliverables\TrackProfiles_H750_20261026\H743\B_RingBase\CarTest.elf" -ConnectionFrequencyKhz 1000 -SharedConnection
```

跑完先按PE7停车，保持H743通电，不按复位、不按F5、不重新进入普通调试，然后导出。ST-Link可以停车后再接；连接必须使用Hot Plug，不能因此复位核心板。成功记录在 `logs/runs/时间目录/run.csv`。

从未启动时应报告IDLE、count=0，并且不生成空CSV。本轮没有手动启动车辆，所以“非空实车日志”仍需下一步架空启停验证。

## 已测性能及限制

H750静止场景、真实传感器、断开CDC、不预览：B和C各三个完整10秒窗口为62.5、62.4、62.4帧/秒。每帧计算路线有效，真实同步UART写入完整、无错误，报告CRC通过。

该性能探针为使电机保持禁止状态，使用模拟反向运行会话，并在线上强制发送无效V3/ACK0。它用于检查完整视觉计算开销，不能代替正常有效控制链路或实车环岛验收。正式程序的真实UART配对另行通过H743只读快照检查；不开启IDE时曾读到64、67帧/秒的1秒接收窗口。接收帧率与相机采集/处理帧率分别记录。

IDE或SDK打开CDC时会恢复实时绘图；本轮的60帧目标为**无IDE预览**，没有将旧H743相机的USB预览成绩套用到新H750。

## 下一步

先架空驱动轮，手动启动、停车，确认日志经历IDLE→ACTIVE→FROZEN并导出非空CSV。通过后落地依次测直道、单独左右弯、十字、左右环岛和整圈；各关键路线重复三次，保留同版本视频与H743日志。

环岛出口参数仍待移动画面校准。A档用于普通循迹对照，B档先验证环岛，最后再试C档提速。允许切内不代表允许选邻道或跳过规定环岛路线。

## 恢复稳定版

`deliverables/PreRing_H750_20261025-201503.zip` 保存本轮修改前源码、配置、19份现有构建ELF和两份冻结配套ELF；SHA-256为 `74ed64205c652e6f0457db7b15d76ce4d5c619ad8a49688c64c15e2504b25649`。恢复时相机与H743一起恢复对应版本，不能只换一侧。
