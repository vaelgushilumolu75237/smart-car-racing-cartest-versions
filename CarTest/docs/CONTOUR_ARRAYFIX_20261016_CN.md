# OpenMV 数组初始化兼容修复：2026-10-16

相机版本：`CONTOUR_ARRAYFIX_20261016`。本版修复上一版 `CONTOUR_REPAIR_20261016` 在 OpenMV 初始化时报 `TypeError: unsupported types for __mul__: 'array', 'int'` 的问题。

## 原因及范围

`contour_trace.py`、`contour_route.py` 用数组乘整数来创建固定缓冲区。桌面 CPython 支持该运算，而 OpenMV 所用 MicroPython 不支持。交叉编译只编译表达式，不执行构造函数，因此之前的宿主测试和编译未发现该运行时问题。这是新增代码的兼容性缺陷。

参考依据是 [MicroPython v1.28.0 的数组实现](https://raw.githubusercontent.com/micropython/micropython/v1.28.0/py/objarray.c)：数组二元运算没有乘法分支；构造函数支持通过已知长度的 bytes/bytearray 初始化数组。

本版只修改缓冲区初始化及相机版本标识。赛道检测、预瞄合成、权重、滤波、环岛门控、串口协议、舵机参数和电机参数均不调整。H743 源码及配套 ELF 与上一版保持一致。

## 上传方式

使用 `deliverables/OpenMV_ContourArrayFix_20261016` 的整套 20 个顶层 `.py` 文件。删除或覆盖旧循迹模块；同名旧 `.mpy` 不要保留。不要删除与循迹无关的文件，也不用格式化整个相机存储。不要上传宿主交叉编译的 `.mpy`、tests、README 或清单。

上传完成后重启解释器，检查启动行：

```text
V7_BUILD=CONTOUR_ARRAYFIX_20261016
```

若 H743 已烧录上一版配套 `DebugFollowTurn/CarTest.elf`，本次相机兼容修复不需要再次烧录 H743。若原先运行的是其他固件，不能仅凭相机版本推定其与当前 H743 配套。

## 验证边界

新增测试在电脑上禁用 array 乘法，复现旧构造错误，并检查修复后的工作区初始化及合成路线执行；这属于 MicroPython 运算限制的宿主模拟，不是实际 OpenMV 运行。

最终运行文件哈希、宿主结果及 ARMv7em 交叉编译结果见 `openmv/PACKAGE_MANIFEST.json`。详细日志保存在 `test-data/contour-arrayfix-20261016/vision-validation`。本轮未连接摄像头执行程序，未验证板端启动、内存或耗时，也不宣称已通过实车弯道。上一版的带叠加视频回放及实车验收限制仍然适用。

源码、配置及 ELF 已在修改前保存快照。交付提供独立 ZIP 和新的 GitHub 标签，上一版标签及文件包保留。
