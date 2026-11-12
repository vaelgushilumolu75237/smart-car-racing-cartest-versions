# 位置控制对照版：AGGRESSIVE_TURN_POS_ONLY_20261017

本包只隔离普通道路控制链中的预瞄贡献。当前帧仍计算并显示
`preview_candidate`，但 `ORDINARY_PREVIEW_CONTROL_ENABLED = False` 时，
普通 `road` 路线在相机端生成的滤波前控制量只等于现有 80/20 加权位置误差；
实际应用的 `heading_contribution` 为 0，后续仍经过既有观测滤波和 H743 限幅。
双边识别、单边补线、滤波、UART V1 协议、H743
参数和环岛分支不在本轮修改范围内。这里的“位置”仍是现有下方 70% 权重
0.8、上方 30% 权重 0.2 的加权虚拟中心，不是改成某一条固定参考行。
这里用 `CTRL=POS` 简称滤波前控制来源；它不是新增的 UART 字段。启动 banner
实际显示 `ordinary_preview=off`，`V7DBG` 中显示 `preview_mode=OFF`。UART 发送值
仍沿用当前 `OBSERVATION_FILTER_ALPHA=0.70` 观测滤波和既有发送逻辑，因此此版本
没有改变 UART 字段、帧格式或 H743 参数。

## 上传

把 `deliverables/OpenMV_PositionOnly_20261017/` 中的 20 个顶层 `.py` 文件
（其中已经包含 `config.py`）一起上传到 OpenMV；同时上传本目录的
`PACKAGE_MANIFEST.json` 仅用于核对。不要把 `tests/`、`.mpy` 或本次目录外的
文件混入相机运行目录。上传后重新运行 `main.py`，开头应出现：

```
V7_BUILD=AGGRESSIVE_TURN_POS_ONLY_20261017 ... ordinary_preview=off ...
```

画面文字中的 `A+0 CAN+xxx` 表示实际应用预瞄为 0、检测到的候选仍为
`xxx`；`P_OFF` 表示画面上的黑色预瞄连线只是诊断几何，不参与发送控制。
`V7DBG` 中对应字段为 `preview_mode=OFF preview=+0
preview_candidate=+xxx`。这样可以把位置误差控制和候选预瞄影响分开观察。

## 在 VS Code 查看 OpenMV

本机使用的是 `singtown.openmv` 2.5.1。先在 OpenMV IDE 里停止脚本并点击
断开相机连接，USB 线保持连接；再在 VS Code 按 `Ctrl+Shift+P`，选择 `OpenMV: 连接`。左侧出现
OpenMV 面板后，可以使用“图像”查看相机画面；下方输出选择 OpenMV，可查看
脚本输出。这个 VS Code 插件入口用于查看/运行相机脚本，不能读取 H743 的
RAM 日志，也没有提供可输入 OpenMV REPL 的串行终端，不能用它替代
`export_run_log.ps1`。

插件的 Run 运行当前打开的 Python 编辑器内容，不会替你保存或上传整个本地工程；
依赖模块必须已经上传到相机，且与 `PACKAGE_MANIFEST.json` 的版本和哈希一致。
不要在这里按 H743 的 F5，
也不要让插件自动烧录 H743、复位车辆或启动电机。

插件说明页：
<https://marketplace.visualstudio.com/items?itemName=singtown.openmv>

## 边界

本包只提供软件对照和主机测试证据。尚未用相机实拍、H743 接收记录或实车
验证“首弯一定改善”；对照测试前应保持 H743 ELF 与原版本一致，并分别保存
OpenMV 启动 banner、V7DBG 和同次行驶的 H743 CSV。
