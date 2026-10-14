# CarTest OpenMV 软件验证记录

记录日期：2026-10-06。以下均为 PC Python 软件测试，不是 OpenMV MicroPython 实板、UART 电气、H743 接收或实车验证。

## 协议编码测试

从 `firmware/vehicle/CarTest/openmv` 执行：

```powershell
python -B .\tests\test_protocol_host.py
```

实际解释器为 Python 3.14.8，退出码 0；**6/6 个测试方法通过**，用时 0.005 s。覆盖本包 BUILD/API 一致性、9 条共同 v1 CSV 向量逐字节相等、CRC-16/CCITT-FALSE 检查值、帧长度与小端字段、invalid 零置信度/零误差、越界及畸形字段拒绝。

逐项输出保存在 [`tests/protocol-host-final.log`](tests/protocol-host-final.log)。向量文件 [`tests/vision-protocol-v1.csv`](tests/vision-protocol-v1.csv) 与 Sixth_Test 原文件字节相同，SHA-256 为 `707364E6EE162CB69075FB3694691863F9C26101ECFB392D6AAF88113E3DCE7F`。

## V7 算法与发送链路回归

从同一目录执行：

```powershell
python -B .\tests\run_v7_host_regression.py
```

实际解释器为 Python 3.14.8，退出码 0；**79 项中 78 项通过，1 项跳过**，用时 0.397 s。该 runner 把 Sixth_Test V7 测试副本放入临时目录，并把导入的 15 个 OpenMV 模块替换为 CarTest/openmv 当前文件；只在临时测试副本中把硬编码的 Sixth_Test BUILD 期望改为 CarTest BUILD。没有改写 Sixth_Test 的源文件、测试或冻结 ZIP。

通过项覆盖灰度掩码与蓝灰边界/窄黑边样本、拓扑、路径连续性、分支歧义拒绝、当前帧证据不足拒绝、边界门、滤波重置、invalid 发送、整帧 UART/CRC、序号唯一与 API/混装不匹配拒绝。跳过项是要求 OpenMV 板上 Viper 探针 manifest 的 `test_board_viper_outputs_match_host`；本次没有摄像头或板端运行，所以不把它报告为通过。完整逐项结果见 [`tests/v7-regression-final.log`](tests/v7-regression-final.log)。

## 文件来源核验

冻结源 ZIP SHA-256 为 `80E8F2C900B212DB1AD866CE6F823008658CAFFDF8F218F416B3819766CB960F`；冻结 manifest SHA-256 为 `EC60BFAF943F4D835774CBA7A26F88240536B1C2F7B785E67FBD4C73FCD6FF50`。已逐文件从该 ZIP 重新比对本包：除 `main.py` 和 `config.py` 外，其余 13 个运行文件字节完全相同；这两个文件的差异都由清单中声明的 BUILD 身份和诊断标签改动构成，算法/阈值/拒绝条件代码与冻结字节的预期变换完全一致。每个文件的新 SHA、冻结 SHA 和差异核验标志见 [`PACKAGE_MANIFEST.json`](PACKAGE_MANIFEST.json)。

## 尚无的实物证据

没有在 OpenMV H7Plus 上上传或运行，没有采集当前安装机位图像，没有测到 UART 写入电气链路或 H743 实际收帧，也没有测得蓝边/正式黑边赛道准确率、弯道/环岛/坡道表现或整车帧率。Sixth_Test 已记录的真实直道两窗口是 42.6/42.4 次完整发送每秒，未达 45 FPS；这只是上游 V7 旧记录，本包未重测。不要将其合成性能结果或 PC 回归当作现场通过。
