# 轮廓跟随移植与双边候选恢复（2026-10-16）

## 范围与结论

本次新增 `openmv/contour_trace.py` 与 `openmv/contour_route.py`，分别提供有界四方向左右墙跟随器、独立几何后处理工具，以及一个受拓扑和边界证据约束的双边行中点候选恢复器。它们不改写现有 `main.py`、配置、H743 固件或传输协议；主循环接入仍由集成任务完成。

逐步移植依据是用户给出的左右两段 C 条件说明。测试目录中的 C oracle 是据这些条件另写的宿主参考程序，不是原始 C 源码。原项目完整的 `get_image_sum()` 实现未提供，因此 oracle 明确采用以下约定：已知白为 `known=1, road=255`，已知暗为 `known=1, road=0`，未知或越界读取返回 `125`。在墙跟随分支中，`125 != 255`，所以未知区域会按非白输入影响走向；但它永远不会记为真实暗边，也不能为候选恢复提供实测边界证据。

## API

两个模块都导出 `RUNTIME_API_VERSION = 1` 和 `COMPILER_BACKEND = "python-bounded"`。后者表示新增热循环为有界纯 Python，不应沿用项目其他灰度/路径代码的 MicroPython Viper 标签来描述本模块。

`ContourTraceWorkspace(width=80, height=60, max_points=900, iteration_budget=None, state_table_size=None, record_events=False)` 负责原始轮廓。`trace(mask_info, left_seed=None, right_seed=None)` 接收含 `width`、`height`、`road_mask`、`known_region` 的字典或同名属性对象；两条种子都省略时会自动寻找种子。坐标顺序固定为 `(y, x)`，方向 `0/1/2/3` 分别表示上/右/下/左，初始向上。存储坐标为可复用的平铺数组 `[y0, x0, y1, x1, ...]`，具体使用 `left_count` / `right_count` 截断读取。

工作区暴露左右点数、`left_dark_support` / `right_dark_support`、每点方向/累计转向误差/丢失计数、`left_row_support` / `right_row_support`、终止原因、最终误差、丢失数和迭代数。真实暗边支持标志仅在相邻采样为已知暗像素时为 1；未知、白色和越界都为 0。`record_events=True` 会额外启用预分配事件数组，供离线 C/Python 对照；默认关闭，生产工作区不保留完整逐步事件历史。

`lost` 与点的真实暗边支持是不同统计量：只有在一步成功移动后，才检查移动前当前位置按移动前方向得到的侧邻采样；若该采样为 `125`（unknown 或越界），对应侧的 `lost` 加一。初始种子和新点的侧邻都不单独触发本步计数。这一时序会影响左右优先调度，不能用新点支持标志替代。

`find_seeds(mask_info, center_x=39.5, row=None, search_rows=8, max_center_distance=None)` 从指定行向上找一个连续已知白区，并要求其左右都由已知暗像素分隔。存在多个合格区时返回 `False` 且 `seed_status` 为歧义；未知和裁剪边界不能构造虚假的双边种子。独立工具的默认邻中心距离按宽度给出，调用者可显式设为 0，或为离线场景传入合适的中心坐标。

终止原因包括 `EDGE`、`DIRECTION_LIMIT`、`CAPACITY`、`ITERATION_BUDGET`、`REPEATED_STATE`、`COLLISION`、`BAD_SEED` 和 `STATE_TABLE_FULL`。初始种子可位于最底行并先向上扩展；之后再次到达图像边框会有界终止。两侧丢失计数相同时遵照给定 C 规则优先扩展左侧。

`ContourPostWorkspace(width=80, height=60, max_samples=900)` 暴露与原墙跟随逻辑分开的工具：`filter_path` 做分段有界平滑并保留端点；`resample_path` 做等弧长采样并在未知或裁剪点处分段；`offset_inward(..., side, width)` 按调用者明确给出的宽度计算局部切线法线偏移并保留点索引；`extract_corners` 用三点夹角提取候选转角。它们是根据需求描述补写的辅助 API，不是冠军程序原实现。偏移 API 不会推断或校准赛道宽度，也没有通过真实赛道标定；检查已知采样仅用于拒绝无效路径，不能据此声称偏移点适合驱动。

## 路由候选恢复

`ContourRouteRecovery(settings)` 固定使用当前项目的 80×60 网格，内部墙跟随工作区为 `max_points=256`、`iteration_budget=1024`、关闭事件历史。配置要求有限的非负坡度、连接容差和参考行间隙；参考行必须位于图像范围。最少观测行数和纵向跨度都不低于 12，并保留配置中更高的限制。

`detect(mask_info, prior_result=None)` 仅在原检测结果明确报告 `INSUFFICIENT_FRESH_ROWS`、`NO_PAIRED_EDGES`、`NO_VALID_PATH` 或 `NO_CONNECTED_PATH` 时尝试恢复。已有有效结果直接原对象返回。失败、无资格或歧义时也原对象返回，不修改它的 `reason`、`scene`、拓扑或其他字段；失败诊断写入 `recovery.reason` 和 `recovery.trace_info["reason"]`。这使拒绝结果继续沿用既有单边/拓扑门控。

候选恢复要求 `prior_result.topology_evidence` 为当前帧拓扑字典，且 `reason` 为 `OK` 或 `PARTIAL_KNOWN_ROI`，`island_evidence`、`branch_evidence`、`ambiguous` 三个字段都明确为布尔 `False`。缺失或未知拓扑、歧义、分支、环岛证据均拒绝。mask 的宽高必须为 80×60，`floor` 必须与当前配置完全相同；若提供 `valid`，必须是布尔 `True`。

路由只在 `height-2` 行找种子，中心为 `(width-1)/2`，`max_center_distance=0`：唯一白区必须包含图像中心，避免底部中心失效后跳到相邻车道。它只接受 trace 点本身为已知白、同一行左侧 `x-1` / 右侧 `x+1` 为已知暗的实测水平跃迁。单行如出现多个不同边界会被标记歧义并丢弃；左右必须组成宽度合法的白色已知区，中间所有像素均须为已知白。行链从种子附近向上延伸，按配置检查间隙、斜率、宽度变化、最少行数/跨度和 `ROAD_REFERENCE_Y` 附近参考行。

成功时返回可复用结果字典，`valid=True` 仅表示“双边轮廓产生了候选”，仍要经过现有边界验证与拓扑门控。`selected_row_ints` 是预分配的 `array('i')`，每行 4 个整数 `[y, left_x, right_x, 7]`，行顺序从图像下方向上；`selected_rows` 与 `boundary_evidence` 有意为 `None`，不得直接当成已经验证的路径。参考点只使用当前每行 `(left_x + right_x) / 2`，来源标记为 `current_contour_bilateral`；不会使用未标定的法线偏移生成行中点。`quality` 只用于诊断，不代表授权。

`recovery.trace_info` 持有工作区引用、`used` / `attempted` / `reason`、`Lcount` / `Rcount`、原始坐标与真实暗边支持、逐行支持数、终止原因、丢失计数、种子、候选与歧义行数。数组属于工作区并会在下一次 `detect` 时复用；调用者应在下一帧覆盖它们前完成消费。

## 验证证据与边界

- 宿主 Python 测试：`test_contour_trace.py` 12 项、`test_contour_route.py` 9 项通过；覆盖直线、左右弯、S 弯、镜像、环形碰头、底行种子、unknown/裁剪、坏尺寸/种子、分支歧义、邻车道拒绝、有限工作区和几何工具。另有 320×240 直线与弯道尺寸测试。
- 宿主 C 对照：`test-data/contour-repair-20261016/port-validation/contour_trace_oracle.c` 由 Visual Studio 2026 x64 `cl /O2` 编译成功。`run_port_parity.py` 在 80×60 直线/弯道/S 弯/长未知左边/单点旧侧未知/单点新侧未知，以及 320×240 直线/弯道共 8 个合成 mask 上，逐事件比较坐标、方向、误差、丢失数和动作，并逐点比较两条 raw path、支持标志、丢失历史与方向/误差；全部完全一致。该结果证明的是这份明确约定下的 Python 与宿主参考 C 等价，不是与未提供的原始 C 二进制或完整 `get_image_sum` 做了对照。
- MicroPython 交叉编译：随附 `mpy_cross_compile.log`，使用 OpenMV IDE 内的 mpy-cross v1.25.0（输出 mpy v6.3）分别以 `-march=armv7em` 编译两个新增模块，退出码均为 0。`.mpy`、源码与 SHA256 在 `port-validation` 日志/报告中登记。
- 上述均为软件证据。未在 OpenMV 摄像头、H743、车辆或真实赛道上运行；没有板端执行时延、250 ms 接收周期余量、摄像头阈值效果、驱动闭环、真实赛道宽度标定或转向安全证据。纯 Python trace 的板端耗时尚待测量，不能由交叉编译成功推导。

本模块没有实现环岛消歧/控制状态机、坡道策略、停车策略或 IMU 推理；给定描述未包含这些逻辑，本次也没有对应传感器证据。路由集成后仍需复核原有拓扑/边界验证顺序与回放结果；当前候选不授权驱动。
