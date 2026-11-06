# CarTest 亚博 K230 r24 移植候选

本目录包含完整K230运行包。复制全部运行文件、PACKAGE_MANIFEST.json及not-ready标定占位JSON
到 `/sdcard/cartest-k230/`，保留设备现有GUI和 `/sdcard/main.py`。
先手动运行 `probe_k230.py` 核对采图，再使用 `calibrate_k230.py` 重新标定，
最后手动运行 `run_cartest.py`。安装及运行均未代用户执行。

UART1: GPIO9 TX / GPIO10 RX；ST7701: 640×480；检测320×120、网格80×60。
原H750标定不适用。环岛、实板运行和60FPS均待验收。
配置入口 `k230_settings.py`，完整说明见 `../docs/K230_PORT_20261106.md`。
