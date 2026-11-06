# neupan_ws

自维护功能包的职责与目录约定见 [功能包分类](src/README.md)。

当前保留NeuPAN实车主线：雷达/LIO、GNSS、底盘、OSM、GPU视频与JSON接收、相机标定和录包。

- [核心启动指令](核心启动指令.md)
- [设备与网络配置](nuc15设备接入.md)
- [当前问题与验收](PROJECT_ISSUES_CN.md)
- [主线回归测试](src/Test/README.md)

2026-11-06已移除旧实车录包、历史测试视频、Gazebo/TEB/RVO对照及其专属测试。主车模型、地图、标定和未解决问题证据保留；`rosbags/`、`videos/`仍是后续新录制的输出位置。
