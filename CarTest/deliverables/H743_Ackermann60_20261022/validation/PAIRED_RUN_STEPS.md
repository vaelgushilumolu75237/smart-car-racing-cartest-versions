# Paired profile validation procedure

This is a future vehicle-test procedure. It was not performed for this
software-only package. Match each camera package with its H743 image:

| Pair | OpenMV package and tuning | H743 image and tuning |
| --- | --- | --- |
| Geometry/performance comparison | `geometry_perf_only`; 320 x 120 center crop and center weights; PD preview `.35/100`; speed stage off, so V2 phase and slowdown request stay zero | `C_Baseline/CarTest.elf`; coordination off; legacy 65:35 steering; 120 us / 20 ms servo slew |
| Coordinated | `coordinated`; same 320 x 120 center crop and center weights; PD preview `.45/120`; speed stage on | `C_Coordinated/CarTest.elf`; applied-servo-PWM Ackermann geometry and V2 slowdown request; 160 us / 20 ms servo slew |

The shared scene and base camera geometry remain fixed; the preview tuning,
speed-stage state, and H743 control profile are intentional differences between
the two pairs. Use the same board, vehicle, tires, steering linkage, supply,
track, start point, camera mount, lighting, route, and UART settings. If running
more than one pair, reverse the order on the next pair to reduce battery and
temperature order bias.

For each run, record the package names, firmware and camera IDs, H743 ELF SHA-256
and preset, battery voltage, track and lighting, start time, and any manual
intervention. Start a new run only after the previous run has been stopped and
its captured data has been exported and checked.

At the end of each run, stop with PE7 and keep the H743 powered. Do not reset or
power-cycle it before exporting. Use the bounded HotPlug reader/exporter to
capture the frozen RLOG and ACK1 data, then verify the ELF/build identity,
RLOG header and record CRCs, and ACK1 generation guards and CRC. `run.csv` is
created only for a valid frozen RLOG with captured records. The ACK1 block is a
separate one-row coordination snapshot, never a run timeline. IDLE or ACTIVE
state must not be reported as a successful run export. When camera trace data
is available, retain the raw `curve_trace_v2_hex` frames and verify their frame
CRC plus capture/sequence binding; report the associated phase and slowdown
request. Preserve the export and its metadata before manually reflashing or
resetting for the next profile.

Compare completion/lap time, path deviation or missed-boundary events, steering
PWM/model angle, wheel target ratio, left/right measured speed, motor PWM,
hold/fault events, and recovery behavior. Treat the 160 us / 20 ms coordinated
servo slew versus 120 us / 20 ms baseline slew as an intentional difference.
Keep host/build evidence separate from vehicle acceptance: compilation does
not prove that the board ran the image, that servo center and Ackermann geometry
match the chassis, or that the camera/motor loop is stable on track.

## 快速步骤

1. 按上表选择成对配置：`geometry_perf_only + C_Baseline`，或
   `coordinated + C_Coordinated`。先核对两侧 manifest、preset 和 ELF
   SHA-256，再由操作者分别上传 OpenMV 程序、手动烧录对应 H743 ELF。
2. 按相同场地、相机位置、路线和供电条件启动本档测试；记录相机/固件
   ID、电池电压和人工干预。预瞄参数和 speed-stage 开关按配对表保留。
3. 结束时用 PE7 停车。**保持 H743 供电，不复位、不掉电。** 等待 RLOG
   与 ACK1 状态冻结后，执行 HotPlug 读取与导出。
4. 核对 Flash/ELF 身份、RLOG header 和记录 CRC、ACK1 前后 generation
   guard 与 snapshot CRC。`run.csv` 只对应有效冻结 RLOG 的多条行驶记录；
   `ackermann-snapshot.csv` 只是一行协调状态快照。IDLE/ACTIVE、无记录或
   校验错误不能记作成功导出。若相机 trace 可用，保留原始
   `curve_trace_v2_hex` 帧并核验帧 CRC、capture/sequence 绑定及 phase/request。
5. 保存 CSV、JSON 元数据和本档的 ELF SHA-256。**确认导出文件完整且校验
   通过后，才手动切换下一档并烧录/复位。**重复配对时下一轮反转两档顺序。
6. 只按实际采集到的板端数据报告道路结果；编译与 host 测试不等于实车
   验收，也不构成帧率达标证据。
