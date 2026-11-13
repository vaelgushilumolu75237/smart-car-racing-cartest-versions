# 编码器标定记录与历史诊断

## 当前接线和标定值

| 车轮 | A 相 | B 相 | 前进方向 raw 累计值 | 每转平均值 | sign |
| --- | --- | --- | --- | ---: | ---: |
| 左 | PA1 / TIM2_CH2 | PA0 / TIM2_CH1 | 59345, 119331, 179984 | 59995 counts/rev | +1 |
| 右 | PC7 / TIM8_CH2 | PC6 / TIM8_CH1 | -59678, -119694, -180717 | 60239 counts/rev | -1 |

累计值来自同次采集，计数器未在每圈之间清零；左轮逐圈增量为 +59345、+59986、+60653，右轮为 -59678、-60016、-61023。每转值是三次绝对增量的平均值；正负号记录车体前进方向下 raw 计数的增减方向。参数已写入 `Inc/car_calibration.h`。

用户后来报告此前编码器供电地反接已纠正，纠正后两轮计数正常；本条是当时的诊断记录，不表示本轮重新测量了供电或输入脚电压。当前生产参数仍为左 59995 / +1、右 60239 / -1，Follow 标定已启用；实际基本 Follow 架空/短程检查按[当前操作说明](FOLLOW_BRINGUP_CN.md)执行。本段历史诊断不再作为当前锁定状态说明。

## 历史只读诊断

普通编码器采样仍由 `App/encoder_diag.c` 提供。2026-10-08 的右编码器弱上拉和 GPIO 输入对照 profile 已从当前 CMake preset、构建任务及 VS Code 启动菜单移除；不要使用旧指令或旧 ELF 重做该组测试。

当时的构建和测量证据保留在[编码器标定报告](../test-data/encoder-calibration-20261008/BUILD_CHECK_REPORT.md)、[AB 线序报告](../test-data/encoder-ab-wiring-20261008/BUILD_CHECK_REPORT.md)、[GPIO 输入报告](../test-data/encoder-gpio-input-diag-20261008/BUILD_CHECK_REPORT.md)和[内部弱上拉报告](../test-data/encoder-pullup-diag-20261008/BUILD_TEST_REPORT.md)。修改前的完整诊断说明保存在 `../test-data/steering-cleanup-20261009/before/files/docs/ENCODER_INPUT_DIAG_CN.md`。
