# A/B/C差速与上坡保速试验

本轮目标是先保速、再提速。当前版本约60%的连续弯道/十字通过率来自用户观察，尚未形成同条件统计验收。三档共用当前OpenMV程序，保持关闭预瞄及80/20位置权重。

| 试验档 | 控制 | 基础最大差速 | 丢线减速过程 | 最长续行 |
|---|---|---|---|---|
| A 当前基线 | 开环；冻结实测版本ELF | 70%/30% | 300ms | 3000ms |
| B 差速对照 | 开环 | 65%/35% | 200ms | 2000ms |
| C 上坡保速 | 增量式PI＋前馈 | 65%/35% | 200ms | 2000ms |

A的ELF来自本轮修改前实际构建文件，SHA256为`667c95a7aef79b2630486faf7421a87e067ee582439dbcfad166d2203222718d`。A启动项直接使用冻结文件，没有重新编译前置任务。B/C分别使用`DebugFollowTurn65`和`DebugFollowIncrementalPI`构建。

三份独立ELF在`deliverables/H743_Uphill_20261019/`的`A_Baseline`、`B_65_35`、`C_IncrementalPI`子目录；本轮配对信息以该目录的`artifact-manifest.json`为准。OpenMV沿用上一版，无需为A/B/C切换而重新上传；相机目录旧清单中的H743配对记录属于上一批交付。

## 输出行为

B直线左右轮基础占空比各50%。最大右转左轮65%、右轮35%，左转镜像；差速随已应用的受限舵机脉宽变化。C每轮独立增量式PI，直线初始目标2.4转/秒，最大差速时外轮3.12、内轮1.68转/秒；这是试验初值，未完成实车调参。

C根据轮速误差增加或减少占空比，实际输出不必保持65/35。每侧上限70%，达到上限后仍可能掉速。正常输出每10ms最多变化4个百分点；启动保留原慢爬升与完整舵机PWM保护。C没有新增坡道图像识别，也不会自动提高速度目标或学习参数。

C初始参数为Q16比例系数13107、积分系数26214、实际输出跟踪时间200ms。积分允许填满到70%上限，但不会继续在上限外累积。选择增量式便于从当前实际输出衔接；它本身不保证比位置式更快，效果仍取决于采样、参数、输出限制与车辆负载。

B/C丢线后200ms内从入口实际输出减速，例如65/35→40/30、50/50→40/40。舵机继续跟随最后可靠偏差。2000ms期限从最后可靠帧接收时间计算，零散有效帧不续期；连续3帧新鲜有效路线后恢复。C保持期间暂停PI，恢复时以当前实际输出重新衔接。两档保留通信中断250ms与PE7停车保护；C另外启用编码器反馈异常停车保护。

## 切换与导出

VS Code只显示A、B、C和关闭执行器的诊断入口。旧构建预设仍保留，方便后续排障。

先按PE7停车，保持核心板通电、不复位。ST-Link连接后，在PowerShell中进入项目：

```powershell
cd 'D:\Codex\smart-car-racing-codex-handoff\smart-car-racing\firmware\vehicle\CarTest'
```

按实际烧录版本选择一条导出命令：

```powershell
# A：冻结的原版ELF
powershell.exe -NoProfile -ExecutionPolicy Bypass -File .\tools\export_run_log.ps1 -Elf .\deliverables\H743_Uphill_20261019\A_Baseline\CarTest.elf

# B：65/35开环
powershell.exe -NoProfile -ExecutionPolicy Bypass -File .\tools\export_run_log.ps1 -Elf .\build\DebugFollowTurn65\CarTest.elf

# C：增量式PI
powershell.exe -NoProfile -ExecutionPolicy Bypass -File .\tools\export_run_log.ps1 -Elf .\build\DebugFollowIncrementalPI\CarTest.elf
```

日志保存在`logs/runs/`下。C另附会话PI诊断，记录目标/测量轮速、最后有效输出请求、控制状态与触及70%上限的累计时长。50ms行驶日志不能还原每个10ms控制周期，也不把PWM当作机械转角。

## 实车对照

先架空检查轮子方向、编码器反馈、PE7停车及非空CSV导出。随后A/B在相同连续弯道和十字路线各测5次，比较完成次数、出界与丢线保持；B/C在相近电池状态的同一平路接坡道各测5次，比较上坡完成、轮速下降及输出饱和。

C不能通过牺牲弯道完成率换取坡道动力。先确认平路无新增振荡、坡道掉速改善且保护有效，再安排另一次提速方案。没有实车数据时不宣称C已优于B。

本轮修改前恢复包：`deliverables/CarTest_PreUphill_20261019_Restore.zip`，含源码、配置及16份ELF，配套SHA256文件用于校验。工具不会自动上传相机、烧录、复位或启动电机。
