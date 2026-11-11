# Comparison / ablation experiments

仅用于封闭场地低速实车对照，独立于主系统默认启动。中文操作命令见工作区根目录 `实车实验启动指令.md`。

| 入口 | 唯一主要差异 |
| --- | --- |
| `roslaunch neupan_experiments neupan_reference.launch` | 当前NeuPAN、受保护定位，作为同限速对照 |
| `roslaunch neupan_experiments gps_always.launch` | GPS质量不筛选，持续参与全局EKF，与本地增量融合；不启用GPS质量/失锁预算停车 |
| `roslaunch neupan_experiments teb_real.launch` | 当前受保护定位，TEB替换NeuPAN，二者不并行控制 |

## 隔离与数据含义

- 启动器只在内存/临时目录组合主系统launch，复用当前传感器、TF、驱动、OSM、录日志和指令mux；不写回主源码/配置。存在同名运行节点时拒绝启动，不能与主系统同时运行。
- GPS实验保持原GPS观测戳，不将历史位置刷新为新测量；EKF打开8秒历史平滑处理延迟观测，融合本地差分XY/航向增量、GPS绝对XY和原PVTSLNA航向。不按解类型、HDOP、卫星数选择GPS，位置保留测量协方差；航向使用明确的实验假设标准差5度，Fixed/Float相同。位置和航向分别使用各自真实历元，不借用相邻帧。
- 起步前需要至少一条有效数值位置和一条有效数值航向（不要求Fixed）；没有建立全球位置/朝向时不会盲目起步。初始化后不因GPS数据缺失或质量变差而主动停车。
- 完全无新GPS报文时，只能继续本地增量；不重发旧GPS冒充新观测。不融合NaN、非法经纬度、负协方差或重复戳。
- GPS实验没有GPS失锁时间/距离预算、Fixed等待或GPS质量停车；**不是关闭所有停车机制**。保留LIO健康、实时有限里程计、扫描/TF、碰撞、指令超时、硬件急停及电机保护，故仍可能因这些原因停车。不能承诺“跑到结束绝不停”。
- 所有这三项实验入口均限制最终前进0.8m/s、倒车0.45m/s；未验证为硬件安全上限。TEB使用当前车体宽0.85m、长0.95m、轴距0.67m的后轴参考包络；输出前轮转角rad，与现底盘驱动一致。
- TEB通过独立扫描/命令看门狗进入现有 `/cmd_vel/auto -> control_mode_mux -> /cmd_vel/selected`，不绕过底盘保护。启用标准TEB可行性/碰撞检查，不移除其算法自身停车。

## 历史来源

`teb_test/` 的11个实车文件来自 `c12fffa^`（在20261106清理提交前），未恢复Gazebo/仿真。历史独立启动文件改名为 `historical_outdoor_teb_test.launch`，仅留档，**不要直接用于当前车**；新入口动态复用当前主系统配置，并覆盖历史车体尺寸、指令接入和日志标签。

## 仅离线检查

```bash
cd /home/nuc/lxw/neupan_ws
conda activate lxw
source /opt/ros/noetic/setup.bash
source devel/setup.bash
python -m unittest discover -s src/comparison_ablation_experiments/neupan_experiments/tests
rosrun neupan_experiments run_experiment.py gps_always --validate-only
rosrun neupan_experiments run_experiment.py teb --validate-only
rosrun neupan_experiments run_experiment.py neupan --validate-only
```

这些命令不启动传感器/底盘。离线测试及编译不等于实车验收；先架空车轮核对TEB转角方向、幅值、遥控接管及急停，再在封闭场地测试。
