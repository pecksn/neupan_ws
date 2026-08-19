# road_vision_refinement

这个包用于把 Astra Pro Plus RGB 相机看到的校园道路区域变成调试量，后续再选择性修正 OSM 全局路径。

当前版本只做两件事：

- `road_vision_node.py` 订阅 `/camera/rgb/image_raw`，输出道路区域 mask、debug 图和横向偏差。
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
rostopic echo /road_vision/status
rostopic echo /road_vision/lateral_error
```

## 重要约定

`/road_vision/lateral_error` 为启发式米制偏差，不是严格标定后的真实横向距离。正值表示图像里观测到的道路中心在右侧，因此建议把参考路径向车辆右侧修正。如果 RViz 中 `/osm_global_path_refined` 方向反了，把 `path_refine_node.py` 的 `invert_error` 参数改成 `true`。

确认 debug 图稳定之前，不建议把 NeuPAN 的 `outdoor_initial_path_topic` 直接切到 `/osm_global_path_refined`。
