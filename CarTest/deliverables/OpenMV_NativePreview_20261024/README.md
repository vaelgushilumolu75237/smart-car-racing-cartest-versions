# OpenMV USB预览底层候选固件

这是给 **OPENMV4P / OpenMV H7 Plus** 摄像头的系统固件候选，不是H743主控的CarTest ELF，也不是正式验收版本。

目标仍是：320×120识线处理与电脑收到的不同新画面同时达到60帧以上。本候选已编译，**尚未刷入摄像头，尚未证明达到该目标**。不以缩图、重复画面或减少预览刷新率代替。

## 已保留的稳定版本

此前完整源码、两套相机程序及配套H743 ELF已另存：

`../CarTest_320x120_60FPS_USBPreview35_PreOptimization_20261024.zip`

SHA-256：`e355db834c12c6202f3d8299e164e6488d4071c97b8248315d45fd8f5aba4cba`。

该ZIP按原GitHub标签`v20261024-ackermann320-static60`生成，571项清单逐项校验。此ZIP保留的是脚本、源码与主控固件；本轮没有读取、改写摄像头当前系统Flash。

## 本候选做了什么

基于官方OpenMV v5.0.0、固定commit `7b7b4e73cb6a0fd09d793201a5c3c588ca0ffa79`，只增加可选的`snapshot(preview=False)`参数和`sensor.PREVIEW_CONTROL_API=1`、`csi.PREVIEW_CONTROL_API=1`能力标记。默认调用仍保留原有预览行为。

显式关闭自动预览后，脚本可在完成识线、发送控制包和绘制中线后，用完整320×120图生成指定JPEG质量的预览并发布一次。候选探针默认为**每帧、原尺寸、JPEG质量30**；JPEG质量影响电脑看到的图，不改变检测使用的灰度原图。此机制是否能达到60帧，需要实际板测。

`firmware.bin`和`firmware.elf`已由Linux CI构建并核对哈希。具体结果见[EVIDENCE.md](EVIDENCE.md)和`artifact-manifest.json`。没有自动烧录、清除相机文件或启动电机。

## 后续手动安装与验证

这一步需要操作者手动安装系统固件后才能继续板测，先保持车辆静止，不进行落地跑车。

1. 在OpenMV IDE使用“工具 → 加载自定义固件”，选择本目录的`firmware.bin`。不选择擦除内部文件系统；相机脚本仍单独管理。
2. 重新连接后，确认`sensor.PREVIEW_CONTROL_API`和`csi.PREVIEW_CONTROL_API`都为1。探针会主动检查，不允许旧固件静默忽略参数。
3. 用`tools/build_openmv_ram_preview_probe.py`生成RAM诊断源，再临时运行；它把11个生产模块装入RAM，并用模拟RESET/KEEP与UART接收器隔离主控，不依赖相机磁盘里旧脚本，不给电机发送控制。
4. 比较关闭预览、完整尺寸每帧Q30预览的板端10秒新采集数，并用电脑端magic/序号/CRC标记核验不同新帧、重复帧和丢帧。不能仅以IDE状态栏或图像内容hash判断。
5. 未经这些测试，不把本候选替换为正式行驶包；H743控制参数本轮保持不变。

若需退回官方相机系统，可使用本机IDE附带的`D:\OpenMV IDE\share\qtcreator\firmware\OPENMV4P\firmware.bin`。它是IDE附带镜像，不是从当前设备Flash读出的逐字节备份；脚本与H743版本按前述ZIP恢复。

当前项目要求不自动上传相机程序或烧录设备；本轮止于构建、软件验证及旧固件的RAM测试，手动安装与新固件板测待操作者恢复后继续。

## 已准备的测试命令

从 CarTest 目录运行，先释放 IDE 的相机连接，保留 USB。生成源代码不会访问设备；`--run` 临时执行 RAM 探针，不写相机磁盘。真实识线探针需要先手动安装并核对候选系统固件：

```powershell
python .\tools\build_openmv_ram_preview_probe.py
uv run --with pyserial --with pillow --with numpy python .\tools\benchmark_openmv_preview.py --port COM5 --ram-source .\logs\usb-preview-20261024\fullres-q30-ram-source.py --run --mode jpeg --marker-sequences --duration 35 --io-timeout 2 --frame-timeout 5
```

COM5 是本轮实际端口，重连后以当前枚举为准。报告分别记录板端处理窗口与电脑收到的前进序号。提前结束会明确标记，不能当成跑满35秒。SDK接收测试也不能替代实际IDE画面测试。

无需安装新固件即可复验人工合成标记，但它不测摄像头帧率：

```powershell
python .\tools\build_openmv_synthetic_marker_probe.py
uv run --with pyserial --with pillow --with numpy python .\tools\benchmark_openmv_preview.py --port COM5 --ram-source .\logs\usb-preview-20261024\synthetic-marker-ram-source.py --run --mode jpeg --marker-sequences --synthetic-marker-probe --duration 4 --io-timeout 2 --frame-timeout 5
```

最后一条命令已板测通过，75张收到的JPEG标记全部校验正确。真实赛道的新固件吞吐与IDE画面刷新仍未验收。

