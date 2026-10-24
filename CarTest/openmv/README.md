# OpenMV 静态视觉预览验证

## 运行包

`PACKAGE_MANIFEST.json` 的 `runtime_files` 列表定义了 19 个运行模块。上传时按清单选择这 19 个 `.py` 文件；`tests/`、文档和 `fast_ring_slope.py` 不属于运行清单。`fast_ring_slope.py` 保留给既有主机测试导入，不是当前版本的运行依赖。

入口为 `main.py`。程序使用 V2 UART 16 字节帧，并要求 OpenMV 固件提供预览控制 API 1（`sensor.PREVIEW_CONTROL_API == 1` 和 `csi.PREVIEW_CONTROL_API == 1`）。固件镜像不包含在本源码包内。清单保留了已验证的 API 1 固件参考哈希；不代表新 UART FIFO API 已构建或安装。

## 主机验证

- `python -m unittest discover -s openmv/tests -p 'test_*.py'`：242 项运行，239 项通过，3 项跳过，0 项失败。
- OpenMV USB 预览工具主机测试：44 项通过。
- 19 个运行模块均通过 `mpy-cross -march=armv7emsp` 编译。

## 静态预览与板端测量

- 320×120 分辨率已配置并由预览图像尺寸观察确认。19 个模块从隔离相机文件系统加载；小型性能 runner 由 RAM 注入。
- IDE 收到 3693 个完整 JPEG payload；3693 个 payload 大小均匹配，0 个读取错误。5 个完整读取窗口分别为 60.4380、60.3276、60.2762、60.3608、60.289 次/秒。这是完整 payload 读取速率。本次未独立记录 IDE 逐帧解码成功和尺寸。固件解锁会清空缓冲区，避免重复读取同一已发布缓冲；但本次 IDE 测量未逐帧解码序号 marker，因此没有独立逐帧新鲜性校验。
- 主机 SDK 独立解码 marker 序号，得到 60.4–60.5 个新序号/秒。此结果与 IDE 的 payload 读取测量分开记录。
- IDE GUI 先将平均帧间隔截断为整数毫秒，再以 1000 除以该值计算帧率，显示值可能偏高，不作为精确帧率证据。
- CDC 关闭的 60 秒 headless 测试包含 6 个 10 秒窗口，路由计数为 796、796、797、795、797、795，共 4777 条有效路由；USB 状态始终为 `NATIVE_HEADLESS`，没有 JPEG marker 写入。它不测 USB 预览吞吐量。
- 另有 20 秒 `pyb.udelay(1389)` 每帧延迟近似实验，得到 55.6–55.7 次/秒。它模拟 16 字节 115200 波特率写入时间，不是实际 UART I/O 测量。

## 配对与验收范围

当前 H743 `C Incremental PI 20261020` 核心保持不变，未烧录或复位 H743，也未启动电机。当前相机测试使用无操作 UART sink，不证明 H743 实际接收。真实 UART 写入和完整闭环仍需单独验证；`full_control_60fps_accepted` 保持 `false`，未声明全链路 60 FPS、道路或整车验收通过。
