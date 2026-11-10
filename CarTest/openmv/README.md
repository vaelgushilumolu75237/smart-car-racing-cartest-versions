# CarTest 亚博 K230 Gyro350 候选

本目录包含完整K230运行包。复制全部运行文件、PACKAGE_MANIFEST.json及not-ready标定占位JSON
到 `/sdcard/cartest-k230/`，保留设备现有GUI和 `/sdcard/main.py`。
先手动运行 `probe_k230.py` 核对采图，再使用 `calibrate_k230.py` 重新标定，
最后手动运行 `run_cartest.py`。安装及运行均未代用户执行。

UART1: GPIO9 TX / GPIO10 RX；ST7701: 640×480；检测320×120、网格80×60。
原H750标定不适用。环岛、实板运行和60FPS均待验收。
必须配对 H743 `DebugGyro350Speed140`：相机默认 GYRO350_REQUIRED=True，
没有主控新鲜陀螺仪能力位时只发送无效V3、ACK0，旧固件不能配对运行。
识别入环后由H743陀螺仪控制；相机连续确认INSIDE才开始累计350°。
350°后H743通知相机释放旧环岛，恢复普通路线观察。
配置入口 `k230_settings.py`，完整说明见 `../docs/GYRO350_20261106.md`
及 `../docs/K230_PORT_20261106.md`（安装包的说明放在上级Install目录）。


## v21 native camera pipeline

Installed camera benchmark: 1306 fresh frames / 20 seconds = 65.3 FPS; 1306 complete UART writes; zero UART errors. Only 144 frames passed route validity in the current view, so valid track observation rate and vehicle behavior still need verification. H743 was not flashed/reset and no motor commands were sent. LCD updates at 10 Hz. The 1280x720@90 sensor mode changes calibration geometry; retain and reject the previous private calibration until fresh width calibration is collected. Native Otsu tie resolution and step-3, phase-1 path row sampling differ from v20. Full 80x60 topology and boundary gates remain.

Validation: project runner 527 tests, 5 existing skips; 32 adaptation tests; 3 boundary equivalence tests; 33 portable bytecode compilations; installed device topology self-test 500 randomized/nested cases, zero mismatches.


## v23 single-edge width-cache seed correction

On fresh RESET/KEEP with an empty cached-width tracker, obtain a dense bilateral path before enabling fast step-3 sampling. Existing successful full observation UART write, boundary evidence and topology gates still control cache commits. Failed writes and stale control never seed cache. New regression reproduces sparse cache rejecting fragmented left/right observed edges while dense measured seed succeeds. No interpolation, inferred cache seeding or relaxed support thresholds. Sensor remains 1280x720@90; auto calibration remains enabled. Host validation only: 527 algorithm tests (5 existing skips), 41 port tests, 33 portable modules compiled. Camera installation and vehicle validation pending. This correction does not resolve the separate 720-mode near-row calibration limitation or prove the observed protocol-stop cause.
