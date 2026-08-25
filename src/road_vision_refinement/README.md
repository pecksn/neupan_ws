# road_vision_refinement

这个包用于把 Astra Pro Plus RGB 相机看到的校园道路区域变成调试量，后续再选择性修正 OSM 全局路径。

当前版本只做两件事：

- `road_vision_node.py` 订阅 `/camera/rgb/image_raw`。`auto` 模式优先检测白色/黄色道路中心线，在中心线不可用时尝试道路右边缘；两者都不可用时进入无视觉参考状态。
- `path_refine_node.py` 订阅 `/osm_global_path` 与视觉偏差，发布 `/osm_global_path_refined`，不会自动接管 NeuPAN/TEB。

## 启动

先启动相机：

```bash
roslaunch astra_camera astra_pro_plus.launch
```

再启动视觉调试：

```bash
roslaunch road_vision_refinement road_vision_debug.launch
```

查看结果：

```bash
rqt_image_view /road_vision/debug_image
rqt_image_view /road_vision/mask
rqt_image_view /road_vision/center_line_mask
rqt_image_view /road_vision/road_mask
rostopic echo /road_vision/status
rostopic echo /road_vision/lateral_error
```

## 重要约定

`target_mode` 支持 `auto`、`center_line`、`road_edge` 和 `road_region`。`auto` 的优先级是 `center_line` -> `road_edge` -> 无视觉参考。`/road_vision/status` 的 `source` 表示当前实际采用的视觉来源。

中心线和道路边缘都不可见时，节点先保持上一有效视觉目标 `invalid_hold_sec` 秒；持续失效后，`path_refine_node.py` 按 `invalid_decay_alpha` 逐帧将视觉偏移平滑衰减回原始路径，不会突然跳变。默认保持 1 秒、之后以 0.08 的比例衰减。

`center_line_right_sign=1` 假设相机朝向车辆前方且图像右侧就是车辆右侧；倒车或方向相反时临时设为 `-1`。后续应改为根据 OSM/局部路径切线自动决定符号。

`/road_vision/lateral_error` 仍为启发式偏差，不是标定后的真实横向距离，状态中的 `metric_calibrated` 因此为 `false`。确认中心线检测、切换逻辑和误差方向稳定之前，不得用于实车闭环控制。

确认 debug 图稳定之前，不建议把 NeuPAN 的 `outdoor_initial_path_topic` 直接切到 `/osm_global_path_refined`。

完整的中心线/道路边缘联合测试、室外导航接入现状和已知问题见 [TESTING_GUIDE_CN.md](TESTING_GUIDE_CN.md)。
