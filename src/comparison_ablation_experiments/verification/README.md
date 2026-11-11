# 离线验证记录（2026-11-11）

- `build_and_unit.txt`：catkin编译成功；9项实验测试中8项通过，独立ROS master集成测试默认跳过。
- `ekf_private_master.txt`：显式运行上述集成测试通过。仅启动独立master上的实验投影节点、真实EKF和假数据输入，验证3秒迟到、低质量GPS及非Fixed航向仍参与融合；停止GPS输入后不触发GPS质量停车，也不重发旧测量。
- `main_regression.txt`：主工程727项离线测试通过，3项跳过。
- `teb_plugin_load.txt`：恢复的OSM全局规划插件、已安装TEB局部规划插件均能加载；未调用initialize或连接底盘。
- 另外完成3个实验组合launch、3个外层入口、录包YAML和MD全部10个bash代码块的解析检查。

以上不是实车验收；未启动完整车辆系统、打开底盘/RTK串口、下发导航目标或真实运动指令。现有主源码及默认启动文件没有修改。
