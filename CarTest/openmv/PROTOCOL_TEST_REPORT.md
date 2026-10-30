# OpenMV V2 协议与静态预览验证摘要

此记录适用于当前 320×120 V2 运行包，不适用于旧 V1 包或其性能记录。详细的静态预览测量和边界见 [`docs/OpenMV-USB60-Static-Validation-20261024.md`](../docs/OpenMV-USB60-Static-Validation-20261024.md)。

## 软件验证

- `python -m unittest discover -s openmv/tests -p 'test_*.py'`：242 项运行，239 项通过，3 项跳过，0 项失败。
- OpenMV USB 预览工具主机测试：44 项通过。
- 清单中的 19 个运行模块均由指定 `mpy-cross` 编译，目标为 `armv7emsp`，退出码均为 0。

## 静态预览测量

19 个模块从隔离相机文件系统加载，只有小型性能 runner 从 RAM 注入。320×120 已配置并由预览尺寸观察确认。IDE 收到 3693 个完整 JPEG payload，大小全部匹配且无读取错误；五个完整读取窗口为 60.4380、60.3276、60.2762、60.3608、60.289 次/秒。本次未独立记录 IDE 逐帧解码成功和尺寸。固件解锁会清空缓冲区，避免重复读取同一已发布缓冲；但本次 IDE 测量未逐帧解码序号 marker，因此没有独立逐帧新鲜性校验。SDK marker 序号速率 60.4–60.5 次/秒为另一项独立测量。IDE GUI 先将平均帧间隔截断为整数毫秒，再以 1000 除以该值计算帧率，显示值可能偏高，不作为精确帧率证据。

CDC 关闭的 60 秒 headless 测试得到六个 10 秒窗口计数 796、796、797、795、797、795，共 4777 条有效路由；没有 JPEG marker 写入。该结果不代表 USB 预览速率。

## 配对边界

H743 `C Incremental PI 20261020` 核心未修改、未烧录、未复位；电机未启动。相机测试使用无操作 UART sink，不证明 H743 收到 V2 帧。20 秒、每帧注入 1389 微秒的 `pyb.udelay` 只是 UART wire-time 近似，不是实际 UART 测量。完整 UART 闭环和 `full_control_60fps_accepted` 均未通过验收。
