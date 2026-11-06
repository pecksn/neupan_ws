# test_motion_tools

2026-10-31对齐参考保持：`test_camera_advisory_followup.py`含真实定位桥0.8°合法校正、连续累计校正、真实重新锚定、桥重启/GNSS会话区分、状态过期及事件去重。全回归706项：698通过、8可选跳过；相机文件25项全通过。见[最新完整报告](../../videos/camera_alignment_hold_20261031/unit_regression.json)、[专项报告](../../videos/camera_alignment_hold_20261031/focused/unit_regression.json)与[旧包生产/消费重算](../../videos/camera_alignment_hold_20261031/recorded/recorded_input_checks.json)。不启动实车控制；正常对齐不清参考，真实故障不抹掉独立危险。

2026-10-30末端修正：`test_camera_advisory_followup.py`增加原摘要/移动末点、12 m短路线、不同道路段、异步替换、零时间戳身份及迟到参考测试；真实库核对修正末点与原生0.5 m到达判断。完整700项：692通过、8可选跳过。见[当前报告](../../videos/camera_terminal_reference_20261030/unit_regression.json)及[旧包生产/消费重算](../../videos/camera_terminal_reference_20261030/recorded/recorded_input_checks.json)。未启动实车控制，原输入重算不是新车闭环。

2026-10-30最新靠右保持：普通视觉走廊不再否决建议参考，遮挡/候选失败/普通弯道不默认回原OSM；实际任务/定位重置仍显式失效。`test_camera_advisory_followup.py`覆盖左右曲线法向偏移、左轮廓/旧缓存保持、消息配对、状态同步、实际位姿连接、真实库装载及异常诊断；接收端补充无多边形但右边界有效的协议测试。完整回归及源码指纹见[最新报告](../../videos/camera_reference_hold_20261030/unit_regression.json)，三组旧包重算见[原输入结果](../../videos/camera_reference_hold_20261030/recorded/recorded_input_checks.json)。旧位姿重算不是新车闭环，健康回放时钟也不代表CPU耗时。

2026-10-29当前主线：NeuPAN原生首动作直发，撤出独立派发、持续限速、整段改写、外加全预测硬否决及几何选点hook。旧`test_checked_execution.py`及对应实现已删除；`test_followup_safety.py`包含原生输出/任务/人工切换回归，`test_stable_reference_policy.py`覆盖带内区域不足不搜索。当前完整回归见[报告](../../videos/neupan_native_restore_20261029/unit_regression.json)，未完成实车闭环验收。

下方按日期保留的是历史报告说明，旧控制层的用例数量、参数与输出含义不代表当前实现。统一入口仍为：

```bash
python3 src/Test/scripts/verify_neupan_changes.py --unit-tests \
  --output-dir videos/neupan_native_restore_20261029
```

最新文档方案跟进：复查过期最多完整重绑复查一次，方向性停车占用覆盖正倒车/转角误差与变化率，靠右回放关联生产、消费与已有跟踪记录。当前专项51项，完整Noetic回归及实测缺口统一见[问题文档](../../PROJECT_ISSUES_CN.md#本节软件验证与证据)；不另建日期版整改说明。`verify_stable_navigation.py`支持重复`--camera-bag`和`--json-only`，按录制开始时间排序分片，输出JSON/NPZ而非控制话题。

2026-10-28控制衔接专项：`tests/test_checked_execution.py`覆盖结果过期不插零速、真实断流、有限解进度/到期、新障碍、手动切换、原生停车、版本并发、新解接管、最终年龄复查、测量约束及真实部署模型接口；均为离线假发布器。原生求解器保持唯一，制动参数夹具不是车辆标定。完整结果见[本轮报告](../../videos/neupan_checked_execution_20261028/report.md)。

```bash
python3 src/Test/scripts/verify_neupan_changes.py --unit-tests \
  --output-dir videos/neupan_checked_execution_20261028/noetic
```

2026-10-28三项复核整改：`tests/test_issue_review_followups.py`新增16项（1Hz平均/瞬时分离、局部前方证据、两轮拒绝上下文及缺失数据），完整Noetic回归671项：663通过、8可选跳过；两个功能包构建通过。原322条轮里程计296条可生成区间均速，瞬时反馈仍为0，不能当高频硬件测量。相机原包154轮无新增危险锁停，但没有新横向修正。见[本轮报告](../../videos/neupan_review_three_fixes_20261028/report.md)。

```bash
python3 src/Test/scripts/verify_neupan_changes.py --unit-tests \
  --output-dir videos/neupan_review_three_fixes_20261028/noetic
python3 src/Test/scripts/verify_stable_navigation.py --camera --feedback \
  --output-dir videos/neupan_review_three_fixes_20261028/recorded
```

2026-10-28前轮续改：`tests/test_navigation_completion.py`覆盖几何选点/特征配对、真实库模型接口、轮里程计运动估计、反馈发布前复查、减速目标与硬合同区分、历史危险范围及实际run最终确认。真实转角和制动标定仍缺。该轮主机报告保留[当时限制](../../videos/neupan_navigation_completion_host_20261028/report.md)；最终Noetic构建/全回归已在上方本轮补齐。

```bash
python3 src/Test/scripts/verify_neupan_changes.py --unit-tests \
  --test-pattern test_navigation_completion.py \
  --output-dir videos/neupan_navigation_completion专项
```

省略`--test-pattern`才是完整回归。JSON记录具体模式、Python解释器/环境及源码指纹。

2026-10-28稳定参考/连续限速：新增`tests/test_stable_reference_policy.py`28项，完整625项：617通过、8项可选跳过。覆盖1–3 m保持、空间过渡冻结/路宽变化、缓存过期不生成假危险、真实危险保护、对齐TF采样、建议/合同消费者分离、连续命令/限速保持、真实重求解、计算过期分类及异步有界证据。今天8个原始分片重算输出到`videos/neupan_stable_reference_20261028`；相机为实际生产/消费原位姿回放，遇障碍是静态真实优化器重算，不是缺失原解/热启动的精确重演或车辆闭环。

```bash
python3 src/Test/scripts/verify_neupan_changes.py --unit-tests --output-dir videos/neupan_stable_reference_20261028
python3 src/Test/scripts/verify_stable_navigation.py --camera --obstacles --output-dir videos/neupan_stable_reference_20261028
```

2026-10-27检查顺序补充：扫描绑定专项增加4项550 ms准备、健康前刷新及输入恢复分支测试，共14项；完整597项：589通过、8项可选跳过。最新报告独立放在`videos/neupan_precheck_order_20261027/`，不覆盖前轮报告；以下旧条目为各自版本结果。

```bash
python3 src/Test/scripts/verify_neupan_changes.py --unit-tests --output-dir videos/neupan_precheck_order_20261027
python3 src/Test/tests/test_scan_binding_followup.py --report-dir videos/neupan_precheck_order_20261027
```

2026-10-27空扫描与准备后绑定补充：`tests/test_scan_binding_followup.py`新增10项，真实库验证非空→空→空→非空及旧最小距离缓存；组合生产run/扫描队列复现准备350 ms＋求解200 ms。当前全量593项：585通过、8项可选跳过，独立报告在`videos/neupan_scan_binding_20261027/`，不覆盖此前结果。

```bash
python3 src/Test/scripts/verify_neupan_changes.py --unit-tests --output-dir videos/neupan_scan_binding_20261027
python3 src/Test/tests/test_scan_binding_followup.py --report-dir videos/neupan_scan_binding_20261027
```

2026-10-27实车起步问题整改：新增`tests/test_startup_recovery.py`18项，覆盖正常扫描同刻TF队列、原接收时间、完整帧提交、真实超时/乱序/恢复、起步诊断与低速、候选碰撞重算和内部异常保持。完整583项：575通过、8项可选跳过。下述脚本只读两组故障bag全部分片；可选`--solve`在每组一个静态场景调用3次真实优化器，不是闭环驾驶。报告统一放在工作区`videos/neupan_startup_recovery_20261027`，无ROS节点/车辆控制。

```bash
python3 src/Test/scripts/verify_neupan_changes.py --unit-tests --output-dir videos/neupan_startup_recovery_20261027
python3 src/Test/scripts/verify_startup_recovery.py --bags --solve --output-dir videos/neupan_startup_recovery_20261027
```

2026-10-27发布/恢复协议整改：新增`tests/test_camera_publication_protocol.py`共16项，完整565项：557通过、8项可选跳过。覆盖同路线降速/区域收缩、兼容更新、心跳、检查中撤销、最终时效/锁等待、缓存TTL/TF清理与两阶段恢复。报告在`videos/neupan_publication_fix_20261027/`，真实生产方法配假时钟，不是实车闭环。以下较早条目的数量是各自当时结果。

```bash
python3 src/Test/scripts/verify_neupan_changes.py --unit-tests --output-dir videos/neupan_publication_fix_20261027
python3 src/Test/scripts/verify_neupan_changes.py --publication-regression --output-dir videos/neupan_publication_fix_20261027
```

2026-10-27视频接收状态：新增`tests/test_video_reception_health.py`的9项测试，覆盖有效帧计数、实际FPS、首次等待、3秒断流、恢复和无效图像拒绝；服务另增2项重启锁交接测试。全量549项：541通过、8项可选跳过，报告在`videos/remote_video_health_20261027/unit_regression.json`。实机已看到等待与成功/FPS日志，断流恢复使用假时钟验证，未断开GPU网络进行实测。

2026-10-27原视频独立服务：新增`tests/test_remote_video_service.py`的8项回归，覆盖仅视频启动、JSON保留在NeuPAN、外部Master/端口不抢占、Master变化重连及停止进程身份检查。当时全量538项：530通过、8项可选跳过，报告在`videos/remote_video_service_20261027/unit_regression.json`；真实开机重启尚未测试，不能把enabled/active当作已重启验收。

2026-10-27：新增`tests/test_camera_right_policy.py`，并扩展发布边界回归；当前完整530项，522通过、8项可选跳过。覆盖同历元TF、原危险区域复查/确认、刹停近处重观测、1–2 m及滞回、单右边界真实区域、短/不交叠左边界及单双侧切换、长段/单向过渡、多周期收敛、平顺退出、执行区域隔离、容量锁停、实际硬限速和真实NeuPAN保存输入。新结果统一在`videos/neupan_policy_20261027/`，不替代实车验收。实际靠右控制使用segment_cache；旧realtime仅作影子对照，不具备本轮完整授权链。

可重复生成带跳过原因和源码指纹的完整回归报告（ROS1 lxw）：

```bash
python3 src/Test/scripts/verify_neupan_changes.py --unit-tests \
  --output-dir videos/neupan_policy_20261027
```

2026-10-26跟进：`tests/test_followup_safety.py`新增17项，覆盖空异常/写盘失败故障保持、同一扫描发布时效、odom原点不变性、形状/转向变化、实际走廊、停车释放及拒绝参考基准。完整回归493项：485通过、8项可选跳过。两份快照及三组完整bag对照见`videos/neupan_followup_20261026/`，与之前结果分开保留。回放基线为本轮修改前已部署模块，5 Hz对5 Hz，源码指纹随报告保存。

2026-10-26：参考生成与预测轨迹安全回归加入`tests/test_reference_and_trajectory_safety.py`，缓存重投影及观测隔离加入既有几何测试；完整回归476项，468通过、8项可选跳过。失败快照/录包只读对照统一使用`scripts/verify_neupan_changes.py`，报告目录`videos/neupan_update_20261026/`。脚本不发布车辆话题，录包使用原位姿/TF，不代表修改后的真实行驶结果。

2026-10-23 NUC观测/控制分离与路径历史整改：会话专项40项通过；Neupan/lxw完整回归422项，414通过、8项可选跳过。覆盖无道路连续观测及图像匹配、候选不进控制、满历史合法租约切换与接收实例恢复；未进行物理相机、网络恢复或实车驾驶验收。

`scripts/` 放人工运动测试和采集工具，`tests/` 放自动回归；两者不要混用。目录约定见 [功能包分类](../README.md)。

## 统一回归入口

自编的底盘限幅、控制切换、定位保护、驱动与定位组合、GNSS串口诊断、日志录包、相机路径及仿真几何测试均集中在 `tests/`。厂家/第三方自带测试和配套数据保持原位，不重复复制。这里只调整测试位置与导入路径，不修改运行节点或车辆参数。

进入 `Neupan`、按 `e`，在 ROS1 Noetic 的 `lxw` 环境执行：

```bash
cd /home/nuc/lxw/neupan_ws
conda activate lxw
source /opt/ros/noetic/setup.bash
source devel/setup.bash
python3 -m unittest discover -s src/Test/tests -p 'test_*.py' -v
```

默认不启动底盘；需要独立 ROS 进程、C++ 搜索或规划器依赖的可选测试按各自条件跳过。单项可将 `-p` 换成对应文件名，例如 `test_serial_diagnostics.py`。无 ROS 环境不能直接执行全部回归。

10月22日接收会话整改后：Neupan容器lxw环境389项，381通过、8项可选测试跳过。覆盖跨epoch/生产进程切换、迟到旧包、重复乱序、心跳断流与路径逐帧确认；实包解析使用假发布器，不发送车辆话题。不包含实车驾驶或GPU主动跳时验收。

2026-11-06按用户要求移除了Gazebo/TEB/RVO对照、其专属测试与测试视频。保留定位、相机、路径、控制和录包的主线回归；旧bag相关用例在无录包时按原有条件跳过。OSM隔离测试的小地图保留为 `fixtures/outdoor_test.osm`，不再依赖仿真目录。

2026-10-21 迁移验证：隔离ROS1环境234项（230通过、4项可选测试跳过）；室外路径测试在具备曲线库/Tk依赖的主机验证73项（67通过、6项可选测试跳过）。合计307项、297通过、10跳过。临时ROS1基础镜像缺少曲线库的Tk导入依赖，因此采用分环境验证；不代表完整Gazebo或实车验收。

## GNSS测量时间回归与静止采集

室外日志复核工具 `scripts/verify_outdoor_reference.py` 可在lxw环境读取综合JSONL、按记录位姿重放稀疏/10 cm参考采样及偏离保护，并可通过`--bag`核对实际收到的话题；用`--output`指定新的JSON报告。它不会发布ROS消息，也不会预测修改后真实轨迹，不能替代带完整扫描的避障复现。

可选`--no-obstacle-window 15:57:44 15:57:50`对首个日志做去障碍物求解对照：校验模型哈希，使用近似求解开始位姿、记录参考索引及两种假定初始控制序列。它不是原始求解复现，不会运行车辆控制，不能证明当时具体障碍物或唯一根因。

实物静止采集（不启动底盘或规划）：先在ROS1 lxw环境运行 `roslaunch test_motion_tools sensor_time_check.launch`，另一个相同环境终端执行下列被动订阅命令。默认保存到 `tracker_logs/sensor_time_日期_时间/`，一分钟后采集器自动退出，随后Ctrl+C关闭传感器launch。无需开启车辆全系统。

```bash
python3 src/Test/scripts/check_sensor_time.py --seconds 60
```

此入口只运行GNSS、雷达及FAST-LIO，缺少LIO保护/EKF属于预期，不能当作完整定位链路验收。

`test_gnss_observation_time.py` 覆盖整数纳秒、GPS周/周内毫秒、接收机闰秒、GGA接收机日期、跨日/月/年、0/50/120/200 ms延迟、重复/乱序、时钟阶跃、CRC及未知时间系统；ROS1下还验证shadow不改变fix/GGA/heading/solution原时间契约。只验证旁路实现，不代表已将定位切到测量时标。

```bash
python3 -m unittest discover -s src/Test/tests -p 'test_gnss_observation_time.py' -v
```

2026-10-21 旁路验收：新增23项时间回归全部通过；隔离ROS1扩展回归共258项，254通过、4项可选测试跳过。室外launch仅做配置解析（shadow/未核时时基参数贯通），未启动实车；测量时标控制切换和T3动态验收仍未完成。

measurement软件补测：`test_gnss_measurement_contract.py` 检查真实驱动→桥的不同位置/航向历元、0/50/120/200 ms延迟下转弯插值、四种位置/定向质量组合、投影同历元去重与元数据晚到、无效时间/CRC拒绝、时钟阶跃禁止AUTO及保留续推预算。测试为假发布器，不允许据此声称实车切换已验收；Float测试使用合格的模拟协方差，未修改实车协方差阈值。

```bash
python3 -m unittest discover -s src/Test/tests -p 'test_gnss_measurement_contract.py' -v
```

2026-10-21最终软件验证：真实ROS1/lxw环境统一回归358项，350通过、8项可选测试跳过；另一次隔离ROS master进程测试通过（局部位姿延迟120 ms）。底盘包编译、devel共享模块及室外原命令三端measurement/auto参数解析通过，禁止的旧全局EKF组合在启动前拒绝。未启动实车控制，未取得室外有效时间或动态验收。

`test_gnss_auto_start.py`新增启动UNKNOWN→有效GNSS时间无需重启/改参即可放行、两秒连续窗口、NUC未同步、重复帧、启动钟跳重新等待、运行钟跳/失同步锁存保护等测试。真实NUC只读内核查询返回已同步；测试不修改系统时间，不把自动一致性门控称为硬同步证明。

```bash
python3 -m unittest discover -s src/Test/tests -p 'test_gnss_auto_start.py' -v
```

本机lxw没有`defusedxml`，启动配置测试需要它；ROS系统环境已安装。本次完整回归仅给当前测试进程追加系统库搜索路径，没有安装库或改conda配置，可复用：

```bash
python3 -c 'import sys,unittest; sys.path.append("/usr/lib/python3/dist-packages"); suite=unittest.defaultTestLoader.discover("src/Test/tests", pattern="test_*.py"); result=unittest.TextTestRunner(verbosity=1).run(suite); sys.exit(not result.wasSuccessful())'
```

## 室外返程路径衔接回归（不驱动车辆）

在 ROS1 的 `lxw` 环境运行；覆盖反向目标、前进连接、实车 0.4 m 扫描过滤、近场缓冲区未验证标记、车身进入盲区拒绝、缓冲区障碍、采集 TF 延迟重试、空间/观测不足停车、靠右路径切换及室内配置隔离。规划与路径策略归属 `osm_map_handler`，另检查迁移后的引用及独立 3 s 心跳回退；NeuPAN 仅保留控制接入。

另覆盖稀疏路线段距离、日志中的 1.46 m 同向起步、仅 ≥135° 反向路线触发掉头，以及普通转弯/大偏移/目标在身后但路线未反向时保留原跟踪、不新增道路或扫描准入门槛。隔离 ROS1 测试检查偏移起步确实生成非零测试指令，3 m 偏移和 90° 转弯不进入搜索，反向且观测不足时持续零速度。

```bash
cd /home/nuc/lxw/neupan_ws
python src/Test/tests/test_outdoor_start_connection.py -v
```

编译后加两个开关可同时运行真实 C++ 搜索与隔离 ROS1 入口测试：独立临时 master、原室内 Hybrid A*、假传感器和测试速度话题，不启动底盘驱动。包含异步取消、超时停车、迭代耗尽不返回半条路径、最新扫描复查及车辆移动拒绝。

```bash
NEUPAN_CPP_SMOKE=1 NEUPAN_ROS_SMOKE=1 python src/Test/tests/test_outdoor_start_connection.py -v
```

## 相机路径几何回归（不驱动车辆）

从旧目录迁入的测试使用当前 `osm_map_handler` 模块，不再引用已删除的底盘包旧脚本。需要 `lxw` 环境中的 OpenCV / NumPy；默认仅运行几何、缓存、投影和回退测试，不启动 ROS。

```bash
python src/Test/tests/test_camera_right_refinement.py
```

`--ros` 是历史影子联调入口，要求预先启动独立 `127.0.0.1:11319` master 和影子节点，并保留原 20261008 测试 bag；不会自动启动这些依赖，禁止指向实车 master。

## NeuPAN录包障碍复核（不驱动车辆）

`verify_neupan_obstacles.py` 在Neupan容器的ROS1、lxw环境中，读取bag和对应综合日志，校验模型哈希，对比完整/降采样障碍点、记录轨迹与当前停车判据，并对少量样本做条件重算。它不启动节点、发布速度或打开串口。原回调时序和求解器warm-start未全部记录，因此不是逐步精确重放，也不等于实车碰撞验收。

```bash
cd /home/nuc/lxw/neupan_ws
OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1 MKL_NUM_THREADS=1 python3 \
  src/Test/scripts/verify_neupan_obstacles.py \
  --bag rosbags/rosbag20261020_160851/hybridnets_vehicle_data_20261020_160851_0.bag \
  --log tracker_logs/system_run_full_20261020_160844.jsonl \
  --output videos/neupan_obstacle_check_20261020_160851
```

输出JSON及扫描/预测车体叠加图，不更改原bag、模型或业务配置。

## 左右电机映射测试

MCU 命令 `CMD,<throttle>,<steering>` 只有统一油门，不能独立驱动左右电机。
在 Neupan 容器中，可以通过转动架空车轮并测量 `LT/RT` 扭矩响应来判断映射：

```bash
cd /home/nuc/lxw/neupan_ws
python3 src/Test/scripts/motor_side_mapping_test.py
```

运行前停止 `robot_base_driver`，并将两个驱动轮可靠架空。程序会保持零油门，依次
提示转动物理左轮和物理右轮，最后输出左右映射是否颠倒。

Open-loop motion tests for recording straight-line and S-curve vehicle data.

These tests do not use global planning, local planning, or obstacle avoidance. They only publish `geometry_msgs/Twist` to `/neupan_cmd_vel`:

- `linear.x`: forward speed in m/s
- `angular.z`: Ackermann steering angle in rad

The test launch starts `robot_base_driver` and the G90 GPS driver by default, so running it alone can move the car and record GPS/heading. It does not start global planning, local planning, NeuPAN, or TEB.

If another launch has already started `robot_base_driver` or the G90 GPS driver, set `start_base_driver:=false` or `start_gps_driver:=false` to avoid two nodes opening the same serial port.

## Straight Line

```bash
roslaunch test_motion_tools straight_record.launch speed:=0.4 duration:=20.0
```

## S Curve

```bash
roslaunch test_motion_tools s_curve_record.launch \
  speed:=0.4 \
  duration:=45.0 \
  steer_amplitude_deg:=18.0 \
  steer_period:=7.0 \
  rate:=1.0
```

Before sending non-zero speed, the motion node waits for valid `/gps/fix`, `/heading`, and at least one `/neupan_cmd_vel` subscriber by default. It then waits `start_delay:=2.0` seconds before movement. `ramp_time` defaults to `0.0`, so the recorded `speed` is the constant upper-computer commanded speed.

## Rosbag Topic

The launch files start `rosbag record` automatically. Bag files are grouped by
Beijing time under `/home/nuc/lxw/neupan_ws/rosbags/rosbagYYYYMMDD_HHMMSS/`; timestamped
filenames sort in recording order.

The rosbag records only one compact topic:

```text
/open_loop_motion/packet  test_motion_tools/MotionPacket
```

`MotionPacket` contains only these fields:

- `int32 lat`: latitude x 1e7
- `int32 lon`: longitude x 1e7
- `int16 alt`: altitude x 10, in decimeters
- `int16 yaw`: ENU yaw x 100, in centi-degrees
- `int8 speed`: upper-computer commanded speed x 100, in cm/s
- `int8 steer`: commanded steering angle, in degrees

By default, `/open_loop_motion/packet` is not published until `/gps/fix`, heading (`/heading` or `/heading_deg`), and `/neupan_cmd_vel` have all been received. If the compact bag has no packet messages, check the GPS serial port, or set `require_packet_sources:=false` for command-only tests.

Packets with `speed == 0` are skipped by default, so stop commands before/after the motion are not written to rosbag or JSONL.

## Synchronized JSONL Log

The launch also starts `open_loop_data_logger.py` by default. It samples the latest values at 20 Hz and writes one synchronized JSONL log:

```text
/home/nuc/lxw/neupan_ws/tracker_logs/open_loop_<mode>_sync_YYYYmmdd_HHMMSS.jsonl
```

Each JSONL line contains only the same six fields as the compact rosbag packet:

```json
{"lat":230398123,"lon":1133900456,"alt":12,"yaw":8560,"speed":30,"steer":0}
```

## Synchronized JSONL Fields

- `lat`: latitude x 1e7
- `lon`: longitude x 1e7
- `alt`: altitude x 10
- `yaw`: ENU yaw x 100
- `speed`: upper-computer commanded speed x 100
- `steer`: commanded steering angle in degrees
