# 阿克曼无人车项目问题与整改记录

> 更新：2026-10-20。审查对象：提交 `704e6fc` 加当前工作区未提交的日志修改。
> 本文件只维护问题、修改进度和验证证据；架构与接口说明见 [PROJECT_ANALYSIS_CN.md](PROJECT_ANALYSIS_CN.md)。
> 本次已修复第3节L1、L2并做离线回归；没有重启实车节点、连接真实串口或发送控制指令。

## 1. 现在还有什么需要做

**日志主要缺口及L1/L2边界问题已修复，离线回归通过，待下一趟实车录制验收。** 本次仅完善异常关闭清单与统一合法JSON，不改变控制算法。

其余工作主要是验证新增记录在实车上实际落盘，以及补充测量和部署资料。03仍然先采集数据，不实施过期结果拦截。

| 编号 | 事项 | 当前状态 | 下一步 |
| --- | --- | --- | --- |
| L1 | 录包异常关闭跳过结束清单 | **已修复，离线验证通过** | 实车确认退出原因与结束清单落盘 |
| L2 | 自动指令异常值产生非标准JSON | **已修复，离线验证通过** | 实车日志保持严格JSON可读 |
| 02 | 非法/越界输入保护及发送审计 | 已实现，离线通过，待实车验证 | 核对审计协议、实际限速和低速行为 |
| 03 | 求解结果时效 | 待测量，暂不修改 | 取得正常行驶runtime并分析耗时 |
| 05 | 非Fixed定位精度 | 待实测 | 建立独立位置/航向基准 |
| 06 | GPS参考点、外参与时间 | 待测量评估 | 测安装偏移和采集/接收时间差 |
| 07 | 路网与通行能力 | 待场景验证 | 核对起步接入、急弯、道路宽度和净距 |
| 08 | 相机路径及回退 | 启用功能后验证 | 使用视觉录包配置，检查回退轨迹 |
| 09 | 接管与自动恢复 | 待操作验证 | 核对请求、实际模式、审计动作 |
| 10 | 云端任务状态 | 待业务验证 | 验证顺序目标与重连，不先重构 |
| 11 | 部署和验收证据 | 部分已有，仍需完善 | 归档参数、地图、外参、版本及测量记录 |

原问题编号不重排；已明确不整改的条目不再列入。L1、L2仅用于本次日志审查，分别关联11录包完整性和02异常输入记录。

## 2. 本次日志修改：已完成的部分

以下基于当前源码和重新运行的测试，不再作为缺失功能重复开发。

| 已完成项 | 实现及记录位置 | 仍需注意 |
| --- | --- | --- |
| 驱动实际发送审计 | `/vehicle/command_audit`，JSONL `command_audit_event`，三种bag均配置 | `written`只表示串口API返回完整字节数，不是MCU执行确认 |
| 审计内容 | 原始输入、限幅/死区/量化原因、最终协议、轮速、转角、限值、字节数和错误 | 无真实轮速和前轮角反馈，审计为尽力发布 |
| 遥控与模式原始事件 | JSONL `remote_command_event`、`mode_request_event`；三种bag均配置原始topic | Twist/UInt8无Header，记录的是NUC接收时间 |
| 目标与到达链 | 三种bag均有gps_goal、map_goal、goal_arrived和经纬度路径；JSONL原有事件保留 | 不等于包含云端发送/确认/显示记录 |
| GNSS解状态 | 三种bag及JSONL均配置 `/gnss/solution_status` | 仍要确认本趟实际收到消息 |
| 录包结束核验 | 同目录原清单JSON新增 `recording_result`，累计分包索引消息数、缺失必录项及读取错误 | L1异常关闭已补测；仍不能保证强杀/断电时落盘 |

驱动审计的 `write_status` 为 `written`、`short_write`、`write_error` 或 `port_unavailable`；始终 `mcu_execution_confirmed=false`。审计在串口尝试后发布，发布异常不触发额外控制或重试。

综合JSONL仍是一套日志，不需要另开“审计日志”进程。完整launch默认启用日志，但**rosbag仍需另开录制**。此前运行中的旧节点和旧bag不会自动补出新增字段，后续需在停车后使用更新后的节点验证。

依据：[base_driver_node.py](src/robot_base_driver/scripts/base_driver_node.py)、[system_run_logger.py](src/robot_base_driver/scripts/system_run_logger.py)、[bag_recorder.py](src/vehicle_bag_recorder/scripts/bag_recorder.py)。

## 3. 已修复的日志边界问题

### L1. 异常关闭时未能保存录包结束核验

- **状态（2026-10-20）**：已修复，离线验证通过，待实车核对。
- **位置**：[bag_recorder.py](src/vehicle_bag_recorder/scripts/bag_recorder.py) 的 `stop`。
- **触发**：SIGINT后等待20秒超时，发送SIGTERM后再次等待5秒仍超时。进程在取进程组或发信号时恰好退出，也可能抛出相关异常。
- **修复前已复现**：用mock模拟两次 `TimeoutExpired`，第二次异常向外抛出，`_record_bag_inventory` 没被调用。由于 `stopping` 已为true，再次调用 `stop` 也不会补写。
- **影响**：最需要核对的不完整录制只留下起始清单，缺少结束原因和核验状态。正常关闭的现有测试通过，并未覆盖这个分支。这是原关闭流程在新增核验职责下暴露的缺口，不是证明正常bag会丢失。
- **实现**：`stop`以finally兜底核验，保持原SIGINT等待20秒→SIGTERM等待5秒，不新增SIGKILL。`recording_result.shutdown`保存请求原因、关闭结果和分阶段异常；同时记录 `return_code`、`process_running`。仍运行或进程状态无法查询时不标核验完成。清单更新失败保留原文件，重复stop可重试清单，但不重发退出信号。
- **离线验收**：新增5项覆盖双重超时、SIGTERM成功、getpgid/killpg退出竞态、清单写失败后重试、poll异常；原active遗留、正常退出和损坏索引测试继续通过。测试使用mock进程及临时bag，核对原bag内容未变，没有操作真实进程。
- **限制**：强杀记录器或断电无法保证执行finally；磁盘不可写时只能告警，不能保证结束清单落盘。进程仍存活时保存的是当时的不完整快照，不会偷偷强杀或自动修复bag。

### L2. 新事件合法，但综合JSONL仍可能出现NaN/Infinity

- **状态（2026-10-20）**：已修复，离线验证通过，待实车日志验收。
- **位置**：[system_run_logger.py](src/robot_base_driver/scripts/system_run_logger.py) 的 `auto_command_cb`、`command_cb`、`write_record_locked`。
- **已实现部分**：新遥控回调和驱动审计已把非有限数保留为文本。
- **修复前遗漏**：自动指令、selected采样等旧入口仍保留浮点NaN/Inf；统一写入层使用默认 `json.dumps`，会输出标准JSON不允许的NaN/Infinity。
- **修复前已复现**：向模拟logger传入 `linear.x=NaN` 的自动和selected指令，自动事件及后续sample两行均被严格JSON解析拒绝。测试仅写临时文件，未发ROS消息。
- **影响**：02能正确停车并不代表异常输入的日志可被其他分析器读取；Python默认宽松 `json.loads` 会接受NaN，因此现有普通解析测试可能漏检。
- **实现**：统一写入边界递归转换字典、列表和元组中的非有限浮点数为 `"nan"`、`"inf"`、`"-inf"`，正常数值/布尔/空值保持原类型；随后 `allow_nan=False` 严格序列化。不改原内存状态，不将异常值伪装为0。其他不可序列化对象告警并返回失败，不使异常逃逸，后续记录仍可写入。
- **离线验收**：新增3项覆盖自动、selected、遥控、审计和嵌套状态的三种非有限数；测试解析器遇NaN/Infinity常量直接报错，全部记录可读且证据保留。另核对正常类型及原始数据未改变、非法对象后仍能继续记录。

本次只修复L1、L2；未修改NeuPAN、定位策略或电机行为，也未重启已有系统。安全停车后重新启动日志/录包节点应用代码，旧文件不会被改写为新的记录格式。

## 4. 已实现与待验证事项

### 02. 输入保护及发送结果

**状态：代码已实现、离线回归通过，待实车验证。**

已实现非有限值停车、正反向独立限幅（默认各1.5 m/s）、±25°转角限制、0.05 m/s死区、停车回正、参数合法性检查及串口最终边界复查。新增审计已能记录限幅和协议量化后的发送内容。

下次验证：

1. 检查实际部署的轮半径和正反向限值；不把软件默认值当作硬件安全极速。
2. 同时记录selected与command_audit，核对限幅、死区及量化结果，检查写入状态。
3. 通过实际车辆运动或已有独立反馈确认低速、倒车及转角行为。`command_speed_mps`只是指令换算值；`/odom.twist`当前为零，不能当真实速度反馈。
4. 不要求在车辆运动中注入NaN或超限指令；异常输入已有模拟串口测试，实车先验证正常边界和真实执行。

依据：[base_driver_node.py](src/robot_base_driver/scripts/base_driver_node.py)、[test_base_driver_command_limits.py](src/robot_base_driver/tests/test_base_driver_command_limits.py)。

### 03. 求解性能与结果时效

**状态：待测量评估，暂不修改。** 用户当前执行完路线后再发下一目标，不把途中抢占任务作为必改理由。

源码仍是求解前检查扫描、求解后发布动作。现有 `/neupan/runtime`、JSONL `neupan_runtime_event`、`auto_command_event` 和资源记录已经能用于第一轮分析，不重复开发性能日志。

- **历史证据**：此前检查的10份 `system_run_full_20261019_*.jsonl` 无可用runtime，不能据此证明性能不足或性能足够；本次未重新分析一趟新实车数据。
- **待采集**：直线、弯道、障碍较多以及视觉接收开启时的 `solver_ms`、`command_interval_ms`、mux原因、扫描/定位状态和资源占用。
- **统计方式**：样本数、p50/p95/p99、最大值及超过0.5秒次数；区分等待路径、到达和传感器故障。没有返回的求解不产生完成事件，要结合进程及自动超时判断。
- **修改条件**：如果观察到慢求解后迟到动作重新放行，再设计结果期限或发布前检查；稳定正常则暂缓。名义50 Hz不等于应采用20毫秒硬拒绝门限。
- **剩余记录边界**：每次动作所用扫描/位姿的完整时间绑定尚未记录；仅当需要追查输入年龄时再决定补充。

### 05. 非Fixed定位精度

- **已实现**：全局桥允许合格Float/差分位置缓慢校正平移，稳定条件满足后可更新续推预算；不使用未固定GNSS航向改变地图航向对齐。
- **尚未证明**：一致性好不等于绝对位置准确。100米/600秒不是无Fixed总行程上限，不能用持续运行代替精度验证。
- **排查方法**：对比独立位置基准，按Fixed、Float、无解、恢复分段统计误差，并记录 `runtime_position.budget_renewals`。
- **代码**：[global_odom_bridge.py](src/robot_base_driver/scripts/global_odom_bridge.py)。

### 06. GPS参考点、外参与时间对齐

- **已确认**：GPS投影直接使用天线经纬度，默认输出为 `base_link`，该处未做水平杆臂补偿；是否造成明显误差取决于真实安装位置。
- **本次补充**：普通扫描转换使用 `lookupTransform(..., Time(0))`，掉头扫描另有采集时刻TF处理。车辆运动时应检查扫描延迟与TF时间差，不能认为两个分支的时序策略相同。
- **排查方法**：先测安装偏移和采集/接收时差；若延迟为Δt，运动引入的平移量级约为vΔt，转弯还叠加旋转影响。存在偏差再设计补偿。
- **代码**：[gps_fix_to_map_odom.py](src/robot_base_driver/scripts/gps_fix_to_map_odom.py)、[neupan_core.py](src/neupan_ros/src/neupan_core.py) 的 `scan_callback`、`refresh_connection_scan`。

### 07. 路网与实车通行能力

- **已确认**：默认起终点吸附上限50米，道路宽度可估算；OSM最大右偏量0，即便enable为true也不代表参考线已靠右。
- **已有机制**：新路线方向与车头夹角至少135°时进入局部前进掉头连接；普通跟踪没有这一分支同等的车体扫掠准入。
- **排查方法**：核对实际道路宽度、离路起步、路口急弯和目标吸附位置。区分“道路图可达”“局部控制可跟踪”和“车体实际可通行”。
- **代码**：[global_astar_node.cpp](src/osm_map_handler/src/global_astar_node.cpp)、[osm_outdoor_start_connection.py](src/osm_map_handler/src/osm_outdoor_start_connection.py)。

### 08. 相机路径与回退

- **适用范围**：完整launch默认关闭相机控制；未启用时，不应将相机修正风险当成当前行驶故障原因。
- **已实现**：消费端核对心跳、路线时间戳和路径摘要，不匹配回退OSM，无原路线停车。默认目标距离是相机中心到右边界2米，不是右轮净距2米。
- **排查方法**：启用后检查GPU/NUC时间、采集时刻TF、相机外参、地面投影，以及回退中心线时的横向轨迹和转角变化。
- **代码**：[osm_camera_path_guard.py](src/osm_map_handler/src/osm_camera_path_guard.py)、[osm_camera_path_refinement.py](src/osm_map_handler/src/osm_camera_path_refinement.py)。

### 09. 接管与恢复

- **当前行为**：切回自动可立即使用未超时的自动指令，模式本身无心跳超时；手动模式不受自动定位门控限制。
- **本次判断**：这是需与实际操作方式核对的行为，不直接认定为缺陷。
- **排查方法**：人工移动车辆后恢复自动、遥控断线、定位恢复分别测试；先明确操作员期望，再决定是否修改。
- **代码**：[control_mode_mux.py](src/robot_base_driver/scripts/control_mode_mux.py)。

### 10. 云端任务状态

- **当前接口**：到达状态为0/1，经纬度路径JSON没有任务ID；云端完整实现本次未审查。
- **本次判断**：顺序单任务场景下，不应仅因为没有任务ID就要求重做任务系统。
- **排查方法**：先验证到达后发下一目标、旧锁存状态和重连时的展示是否正确；若实际出现错归属、取消或并发需求，再扩展契约。
- **代码**：[gps_goal_bridge.py](src/osm_map_handler/scripts/gps_goal_bridge.py)、[neupan_core.py](src/neupan_ros/src/neupan_core.py)。

### 11. 部署与验收

- **环境依据**：宿主机 `/opt/ros` 只有humble，但已有运行中的 `Neupan` Noetic容器。不能再用上一轮宿主机缺少Noetic来断言本轮无法测试。
- **依赖边界**：本仓库主要提供ROS包装，外部 `neupan` 算法包、模型、地图和外参同样决定运行结果；当前未完整审计外部算法源码和GPU/MCU实现。
- **排查方法**：每次试验保留commit、容器/依赖版本、launch参数、模型/地图/外参版本、运行数据和人工接管记录。
- **已有TF行为**：`map_odom_align_bridge` 输入超时会继续广播最近变换，因此“能查询TF”不能独立证明定位健康；要结合安全状态和消息年龄。

## 5. 当前日志覆盖与剩余证据

### 5.1 记录方式

完整launch默认综合JSONL按10 Hz采样部分状态，同时逐回调记录部分控制、任务和诊断事件。异步队列、磁盘写入和节点退出都可能影响记录，不能把“逐回调写事件”理解为绝不丢消息。

rosbag需单独启动。`record.launch` 默认 `vehicle_data.yaml`；现场常用命令选择 `hybridnets_data.yaml`，靠右测试选择 `camera_right_test.yaml`。视觉测试优先用对应专题配置，普通vehicle配置没有完整GPU图像/推理JSON及camera_right输入输出。

### 5.2 仍缺什么，分别怎么补

| 类别 | 关联问题 | 剩余内容 | 处理方式 |
| --- | --- | --- | --- |
| 软件修复待实车验收 | L1、L2 | 异常结束清单、统一合法JSON已修复并补回归 | 下趟运行核对新记录，见第3节限制 |
| 部署复现资料 | 11 | Git/依赖/容器版本、全量参数、地图与外参内容/哈希 | 建立每趟归档；可先操作留存，不必马上新建节点 |
| 真实物理反馈 | 02 | 实际轮速、前轮角、MCU执行确认 | 核实硬件已有能力或实测，不能由发送日志推断 |
| 独立定位与几何基准 | 05～07 | 位置/航向真值、天线安装偏移、道路宽度和净距 | 测量/标定；不能靠增加自身状态字段替代 |
| 时序与上游证据 | 06、08～10 | 两机时钟偏差、云端发送/确认、具体求解输入年龄 | 有对应现象时采集，两端时序不能只用NUC接收时间替代 |
| 场景验收 | 全部 | 真实运行数据是否齐全，指标是否符合需求 | 下次录制核验，不重复补已存在的话题 |

`/neupan/config_snapshot` 已包含规划节点参数、规划YAML和模型SHA256；它不是全系统参数和依赖归档。bag清单已含录制配置与命令，也不是完整部署快照。

消息计数只能证明有消息，不证明全程连续或无掉帧。`required_topics`不应无条件加入仅操作时才出现的目标、模式请求或遥控事件；按本趟实际操作核对计数。runtime在实际规划执行时才应出现。

## 6. 下次实车记录验收

| 阶段 | 检查内容 | 合格证据 |
| --- | --- | --- |
| 启动前 | 停车后使用更新的驱动、logger和录包程序；选正确配置 | 新订阅/发布接口已加载，配置和版本有记录 |
| 正常行驶 | 自动指令、selected、command_audit、runtime及定位/扫描状态 | JSONL字段非空；审计协议与实际限值一致，写入错误能识别 |
| 实际执行接管时 | remote、mode_request、control_mode、command_audit | 请求与生效顺序可解释；未做遥控操作则不要求有遥控消息 |
| 顺序目标任务 | gps_goal、map_goal、path、goal_arrived | 能关联本趟开始与结束；不把旧锁存数据当新任务证据 |
| 使用视觉时 | 图像、推理JSON、camera_right路径/状态、TF | 视觉专题bag中实际有消息；关闭功能时不要求对应消息 |
| 停止录制后 | 原清单中的 `recording_result` | 退出码、索引读取、active遗留和缺失必录项均核对；不能只看一个true标志 |

结束后重点检查 `topic_message_counts`：`/vehicle/command_audit`、`/neupan/runtime`、`/cmd_vel/selected`、`/gps_guard/status`、`/neupan/scan_health`；以及本趟实际操作应产生的目标、模式和遥控消息。

`index_verification_complete`要求记录子进程已退出、已有索引全部可读且无active遗留，不代表全程录制完整；`all_required_topics_recorded`只要求配置中的每项有消息，不保证频率。还要看 `shutdown` 及退出码。强杀或断电可能没有结束结果，应明确标记未验证，不能当作通过。

## 7. 本次审查与测试记录

### 7.1 审查范围

审查当前未提交修改中的驱动审计、综合logger新增回调、三份录包配置、录包索引清单、依赖声明和测试。本次仅在现有修改上修复L1/L2及补测，保留其他用户改动。

### 7.2 重新运行的回归

| 测试组 | 环境 | 本次结果 |
| --- | --- | --- |
| `robot_base_driver/tests/test_*.py` | Neupan容器，Noetic/devel | 122项：120通过、2跳过 |
| `src/Test/tests/test_*.py` | 宿主机系统Python | 101项：95通过、6跳过 |
| 合计 | 离线回归 | **215通过、8跳过，无失败** |

此前日志补齐新增14项，本次L1/L2再新增8项（L1为5、L2为3）；其中 `test_logging_records.py` 当前17项全部通过，包含真实临时bag索引与严格JSON解析。模拟串口、进程和ROS生命周期，不操作车辆。测试中的不可序列化对象告警属于预期注入，不是实车报错。`git diff --check`通过；本次没有完整catkin构建、Gazebo或实车验收。

本次普通回归仍默认跳过8项可选导航集成测试；它们此前已补跑，历史结果保留在7.3，本次没有重新运行这些专项测试。

```bash
docker exec -w /home/nuc/lxw/neupan_ws Neupan bash -c \
  'source /opt/ros/noetic/setup.bash; source devel/setup.bash; PYTHONDONTWRITEBYTECODE=1 /usr/bin/python3 -m unittest discover -s src/robot_base_driver/tests -p "test_*.py" -q'
/usr/bin/python3 -m unittest discover -s src/Test/tests -p 'test_*.py' -q
```

### 7.3 已补跑此前跳过的8项集成测试（2026-10-20）

在现有Neupan容器的Noetic/devel环境中，使用 `/root/miniconda3/envs/lxw/bin/python3`，分别在独立解释器中显式开启测试开关。三项ROS测试使用临时端口的独立Master和模拟传感器/消息；规划器控制输出重映射到测试话题，不启动底盘驱动。五项C++测试调用现有已编译worker，不依赖实车Master。

| 开关 / 测试类 | 验证内容 | 本次结果 |
| --- | --- | --- |
| `MUX_SAFETY_ROS_SMOKE=1` / `SafetyHeartbeatRosTest` | 安全状态缺失、心跳中断、手动接管、恢复 | 1项通过；本次心跳断流约1.037秒后测试输出归零 |
| `GPS_RUNTIME_ROS_SMOKE=1` / `RuntimeRosProcessTest` | Fixed初始化、源时间失配、Float恢复、旧Fixed数据超限停车 | 1项通过 |
| `NEUPAN_CPP_SMOKE=1` / `SharedCppTests` | 异步通信、迭代上限、非原点精度、前进掉头、未知区域拒绝 | 5项全部通过 |
| `NEUPAN_ROS_SMOKE=1` / `RosProcessTest` | 真实规划进程、正常路线、相机影子入口及反向路线阻塞停车 | 1项通过 |

**此前8项跳过均已实际执行：8通过、0跳过。结合当时修复前普通回归207通过，两批覆盖原215项测试，合计215通过。** 这是历史补跑记录，不是本次L1/L2修复后223项的一次全量输出；ROS测试分别运行以隔离各自的初始化与关闭生命周期。

启动初期曾出现Master尚未就绪的重试提示，随后连接成功；部分rospy调用有socket ResourceWarning，但测试断言通过并正常退出。未进行本次完整重编译、Gazebo或实车验收。1.037秒仅为该测试中的ROS输出响应，不是实车物理制动时间。现有C++二进制被实际测试，不代表本次已验证干净环境构建。

复现方法：在容器中source Noetic及devel，将lxw环境bin加入PATH，再到对应tests目录分别运行：

```bash
# src/robot_base_driver/tests（各命令为独立进程）
MUX_SAFETY_ROS_SMOKE=1 /root/miniconda3/envs/lxw/bin/python3 -m unittest test_control_mode_mux.SafetyHeartbeatRosTest -v
GPS_RUNTIME_ROS_SMOKE=1 /root/miniconda3/envs/lxw/bin/python3 -m unittest test_runtime_position.RuntimeRosProcessTest -v
# src/Test/tests
NEUPAN_CPP_SMOKE=1 /root/miniconda3/envs/lxw/bin/python3 -m unittest test_outdoor_start_connection.SharedCppTests -v
NEUPAN_ROS_SMOKE=1 /root/miniconda3/envs/lxw/bin/python3 -m unittest test_outdoor_start_connection.RosProcessTest -v
```

这8项验证导航集成功能，不覆盖L1/L2；两个日志边界问题的本次修复与新增测试见第3节及7.2。

## 8. 维护与执行规则

- bug修改、状态、测试和实车验收仅更新本文。架构/接口契约改变时才更新项目分析，不复制第二份问题进度。
- 状态区分：待排查、待修改、已修改待验证、已验证。已验证需注明是离线还是实车、适用场景和限制。
- 技术负责人确认问题依据、修改范围及验收标准；执行模型不得把待测量事项自动改成软件重构。
- 每项后续补充：日期、根因、修改文件/提交、测试命令与结果、实车结果、剩余限制。尽量更新原条目，避免在末尾持续追加重复“审查/修复说明”。
- 软件记录完整性与物理执行准确性分开验收；不将串口写入成功当作MCU执行确认。

源码与配置：[system_run_logger.py](src/robot_base_driver/scripts/system_run_logger.py)、[bag_recorder.py](src/vehicle_bag_recorder/scripts/bag_recorder.py)、[test_logging_records.py](src/robot_base_driver/tests/test_logging_records.py)、[vehicle_data.yaml](src/vehicle_bag_recorder/config/vehicle_data.yaml)、[hybridnets_data.yaml](src/vehicle_bag_recorder/config/hybridnets_data.yaml)、[camera_right_test.yaml](src/vehicle_bag_recorder/config/camera_right_test.yaml)。
