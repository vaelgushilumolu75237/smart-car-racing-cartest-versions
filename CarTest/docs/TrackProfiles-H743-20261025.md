# H743 配套赛道档位：实现与验证记录

日期：2026-10-25  
范围：CarTest H743 固件实现、宿主测试和静态构建证据。这里没有实车运行或烧录结果。

## 档位

| 档位 | Preset | `CARTEST_TRACK_PROFILE` | 直道外轮目标 | 最大视觉减速后的外轮目标 | 说明 |
| --- | --- | ---: | ---: | ---: | --- |
| A | `DebugFastNoRing` | 1 | 5600 mRPS | 4400 mRPS | 无环岛 takeover |
| B | `DebugRingBase` | 2 | 4800 mRPS | 4000 mRPS | 环岛外轮与同档弯道目标相同 |
| C | `DebugRingFast` | 3 | 5600 mRPS | 4400 mRPS | 环岛外轮与同档弯道目标相同 |

三个新 preset 继承 `DebugFollowIncrementalPI` 的实际协调设置：编码器增量 PI、Ackermann 目标和视觉减速、65/35转向、硬件输出编译许可、RAM日志、2000 ms视觉丢失截止及200 ms减速。舵机限制保持稳定协调档的160 us/20 ms；电机启动和常规PWM渐变、1000‰输出上限、增量PI的 Q16 增益 `Kp=13107` / `Ki=26214` 与200 ms tracking 均未因新档改变。环岛外轮没有额外提速。

`CARTEST_TRACK_PROFILE` 的默认 cache 值仍为0，保留旧配置语义；配套三档中建议先选 B (`DebugRingBase`)，它保留旧4800/4000 mRPS目标，且具备新配对握手。`DebugFollowIncrementalPIBaseline` 仍是协调关闭的历史对照，舵机限制120 us/20 ms。已有 preset 没有删除或重命名。

VS Code `launch.json` 前三项现为“新A 提速无环岛 5.6/4.4”“新B 原速环岛 4.8/4.0”“新C 提速环岛 5.6/4.4”，各自指向对应新 ELF 并有匹配的 build task。历史配置保留原名并排在新三项后面。

## 配对协议与安全门

### 相机到 H743

- V2仍是16字节并继续解析；V3也是16字节，字段偏移不变，byte 11 是 profile ACK：0未知、1 A、2 B、3 C。
- `vp_pack()` 继续产生兼容V2；新增 `vp_pack_v3()` 产生V3。`vp_unpack_ex()` 可解析V2/V3；V2不读取新增的 `profile_ack` 字段。
- 新三档只在收到当前档匹配的V3 ACK、且确认是在最近一次 reverse RESET 之后收到并且小于250 ms时，才通过配对会话门。旧缓存 ACK、V2、未知ACK和错档 ACK 均不能启动新档；运行时确认过期或错档会触发软协议故障并停车。V3中CRC有效但ACK=4的共享向量判定为 `BAD_HEADER`。

### H743 到相机

- 原V1 RESET/KEEP仍为8字节；其50 ms节拍和序号独立保留。
- 新V2 profile-state为20字节：`A6 5A 02 02`、独立状态序号u16、profile u8、flags u8（bit 0 是真实 Follow session active）、revision=1 u16、session token u32、双前轮正向累计转数均值（mR，u32）、CRC16。多字节字段为小端，CRC覆盖byte 0..17。
- 状态帧目标周期100 ms，与V1串行发送，不交错帧内字节；状态序号独立于V1序号。
- session token 上电为0；仅被接受的实体PE7 Follow启动会递增，回绕时跳过0；停止或故障期间保留当前值。里程只用Follow会话内有效的正向编码器样本更新；反转、无效采样和溢出不会推进，不从请求目标积分。它只供视觉辅助状态，不单独触发环岛、转向或退出。

共享字节向量：`tests/data/vision-protocol-v3.csv`、`tests/data/vision-protocol-v3-invalid-ack.csv`、`tests/data/reverse-profile-state-v2.csv`。这些CSV同时供C宿主测试与相机侧回归读取。

## 诊断兼容

- 运行日志记录继续为64字节；ACK1快照继续为64字节。
- legacy profile 0的快照仍为V1、上限4800 mRPS。配对profile 1/2/3的快照使用V2、布局不变、上限5600 mRPS。其他旧版本继续由原解析路径支持。
- 新配对监视值使用各自4字节零初始化的D1 `.bss`标量，包括 `car_camera_profile_required_ack`、`car_camera_profile_latest_ack`、`car_camera_profile_latest_version`、`car_camera_profile_ack_ready`、`car_camera_profile_ack_seen_after_reset`、`car_camera_profile_ack_age_ms`、`car_camera_profile_last_ack_ms`、`car_camera_profile_mismatch_count`、`car_profile_state_sequence`、`car_profile_state_last_tx_ms`、`car_profile_state_tx_count`、`car_profile_state_id`、`car_profile_state_session_active`、`car_session_token`、`car_session_mean_forward_wheel_millirev`、`car_session_odom_valid_samples`、`car_session_odom_invalid_samples`。

根代理用 `run_status_reader.build_plan` 对新三份ELF的符号可用性和RAM跨度作了额外核对：36个required/optional符号可读，合计跨度4392字节。

## 宿主验证

从 `firmware/vehicle/CarTest` 运行：

```powershell
powershell.exe -NoProfile -ExecutionPolicy Bypass -File tests\host\run_tests.ps1
```

结果：`HOST_TEST_SUMMARY cases=73 assertions=7720 failures=0 vectors=13`，runner退出码0。覆盖共享V2/V3向量、CRC/格式错误、ACK错档和旧确认门、profile-state编解码、状态/KEEP调度无交错、session/reset/fault/启动保护、目标镜像和快照版本。相同runner还完成7项Python速度基线工具测试及生产源码提取验证。

本轮日志：

- 完整host runner：`tests/host/logs/msvc-host-20261025-205330-588.log`
- Ackermann与profile目标矩阵：`tests/host/logs/ackermann-control-20261025-205338-750.log`。四种profile（legacy、A、B、C）每项25个target/PI/servo/snapshot检查均通过；单独Ackermann测试10067项检查通过。

## 构建证据

15个可构建的声明preset均通过 `tools/build.ps1 -Preset <name>` 实际配置及交叉编译，退出码均为0。完整逐preset命令输出、退出码、ELF路径及SHA-256记录在：

- `logs/h750-ring-20261025/build-matrix/summary.csv`
- `logs/h750-ring-20261025/build-matrix/summary.json`
- 同目录下每个preset各一份 `.log`

三个新增ELF：

| Preset | 退出码 | ELF SHA-256 |
| --- | ---: | --- |
| `DebugFastNoRing` | 0 | `81d70e47382c21d7bd675f978630bb21f2ac4b070d37026307cce0b68867f2d2` |
| `DebugRingBase` | 0 | `a0b081432360fec2970a2691a2029e20a914c55e6782487cd04366e800459026` |
| `DebugRingFast` | 0 | `9cc991fce62459915ea89de4192f60a53f1fcf58bc982adfe27486c4e392bf17` |

构建日志含一条现存非阻断G++警告：C++编译命令收到 `-std=gnu11`。三个新preset的16个main.c有效编译定义均由build runner校验。上述证据仅表示静态配置及编译通过；没有烧录或启动。

## 尚需人工/实车验证

尚未验证摄像头与H743的真实UART配对握手、PE7手动启动/停止、旧ACK或错档时实车停车、通讯超时停车、编码器方向及每转标定、架空轮测试、地面转向响应、环岛进入/弧段/出口和完整赛道。尤其session里程正确性依赖实际编码器方向与每转计数配置。宿主测试、静态ELF和CSV向量都不能替代这些验收，也不能据此宣称赛道通过。
