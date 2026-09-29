# GPU HybridNets 双边界输出规范

## 1. 目标

GPU 板从可行驶区域中提取完整道路区域、左边界和右边界，通过 UDP JSON 发给 NUC。NUC 后续使用右边界实现靠右行驶，并用左边界检查道路宽度和车辆是否仍在道路内。

```text
相机 → HybridNets 可行驶区域 → 左右道路边界 → JSON → NUC
```

不要用车道线代替道路边界。园区没有车道线时也必须能够输出道路两侧边界。

## 2. 图像约定

- 分辨率：`1920×1080`
- 图像方向：旋转 `180°` 后的画面
- 坐标原点：左上角
- `u/x` 向右，`v/y` 向下
- 坐标名称：`rotated_image_pixels`

视频发送到 `192.168.10.21:5000`，JSON 发送到 `192.168.10.21:6000`。

## 3. 同一帧上下文

相机帧进入推理队列时生成帧号和采集时间，推理结果、JSON 和 GPU 本地调试图都必须使用同一份数据：

```python
frame_seq += 1
capture_timestamp_ns = (
    image_msg.header.stamp.sec * 1_000_000_000
    + image_msg.header.stamp.nanosec
)

frame_context = {
    "frame_seq": frame_seq,
    "capture_timestamp_ns": capture_timestamp_ns,
    "image": image.copy(),
}
```

不能在推理结束时重新生成 `capture_timestamp_ns`。

## 4. 左右边界提取

建议从可行驶区域 mask 中保留与图像底部中央连通的区域，然后从下向上逐行扫描：

```python
rows = np.linspace(image_height - 1, horizon_y, 45).astype(np.int32)
left_boundary = []
right_boundary = []

for v in rows:
    xs = np.flatnonzero(drivable_component[v] > 0)
    if xs.size:
        left_boundary.append([int(xs[0]), int(v)])
        right_boundary.append([int(xs[-1]), int(v)])

left_boundary = reject_outliers_and_smooth(left_boundary)
right_boundary = reject_outliers_and_smooth(right_boundary)
```

要求：

- 输出后处理后的完整左右边界点集，不按固定数量截断或只取少数点；
- 点按由近到远排列；
- 删除横向突变、孤立点和重复点；
- 左边界必须位于右边界左侧；
- 两条边界应来自同一个连通可行驶区域。
- 边界触及图像左/右边缘表示道路延伸到视野外，不等于检测失败；不得因此清空整条边界或把总体 `valid` 设为 `false`；
- 右边界部分贴近 `u=1919` 时，保留全部可见点，并使用中远处未贴边的连续点判断 `right_boundary_valid`；
- 单侧边界不可见时，只把该侧 `*_boundary_valid` 设为 `false`，可行驶区域仍可靠时总体 `valid` 保持 `true`。

## 5. 生效条件

`valid` 表示可行驶区域是否有效，不要求左右边界同时存在。左右边界分别使用 `left_boundary_valid` 和 `right_boundary_valid`。边界有效时满足：

| 条件 | 阈值 |
|---|---:|
| 左边界点 | 至少 `3` 个，实际发送完整点集 |
| 右边界点 | 至少 `3` 个，实际发送完整点集 |
| 道路多边形 | 至少 `3` 个点 |
| 总体置信度 | 至少 `0.65` |
| 坐标范围 | `0≤u<1920`、`0≤v<1080` |

NUC 严格模式要求可行驶区域连续 `3` 帧有效后，才发布 `/hybridnets/valid=true`。任意一帧失败、超时或乱序会立即失效并重新累计。靠右控制还必须额外满足 `right_boundary_valid=true`。

最低点数只用于判断消息结构是否能构成折线，不代表只发送这些点。GPU 必须发送完整的可行驶区域、左边界和右边界。计算 `confidence` 时还应考虑：

- 可行驶区域是否连接图像底部中央；
- 有效扫描行比例；
- 左右边界连续性；
- 道路宽度是否合理；
- 与前一帧的变化是否合理；
- 图像是否过暗或过曝。

## 6. 十字路口和开放区域

十字路口或普通道路遮挡时可能暂时没有左右边界。此时不要延长旧边界或制造虚假边界；只要可行驶区域可靠，仍保持 `valid=true`：

```json
{
  "scene_type": "intersection",
  "valid": true,
  "road_polygon_valid": true,
  "left_boundary_valid": false,
  "right_boundary_valid": false,
  "road_polygon_px": [[100, 1070], [1820, 1070], [1700, 450], [220, 450]],
  "left_boundary_px": [],
  "right_boundary_px": []
}
```

NUC 收到该状态后立即暂停靠右修正，按 OSM/NeuPAN 全局路径穿过路口。驶出路口并重新连续收到稳定边界后，才恢复靠右修正。

如果左右边界都没有、可行驶区域也不可靠，发送 `scene_type="unknown"`、`valid=false`。没有全局路径时不能仅凭开放的可行驶区域选择十字路口方向。

## 7. JSON 格式

```json
{
  "version": 3,
  "frame_seq": 12345,
  "capture_timestamp_ns": 1790670000000000000,
  "publish_timestamp_ns": 1790670000120000000,
  "frame_id": "camera_optical_frame",
  "image_width": 1920,
  "image_height": 1080,
  "coordinate_space": "rotated_image_pixels",
  "scene_type": "normal",
  "valid": true,
  "road_polygon_valid": true,
  "left_boundary_valid": true,
  "right_boundary_valid": true,
  "confidence": 0.82,
  "source_age_ms": 120.0,
  "inference_ms": 95.0,
  "inference_hz": 10.5,
  "road_polygon_px": [[80, 1070], [1840, 1070], [1320, 620], [650, 620]],
  "left_boundary_px": [[80, 1070], [110, 1030], [145, 990], [185, 950], [230, 910], [280, 870], [335, 830], [395, 790], [460, 750], [525, 710], [590, 670], [650, 620]],
  "right_boundary_px": [[1840, 1070], [1800, 1030], [1755, 990], [1710, 950], [1660, 910], [1610, 870], [1555, 830], [1500, 790], [1450, 750], [1400, 710], [1360, 670], [1320, 620]],
  "lane_lines_px": []
}
```

必须保留以下字段：

```text
version
frame_seq
capture_timestamp_ns
publish_timestamp_ns
image_width
image_height
coordinate_space
scene_type
valid
confidence
source_age_ms
road_polygon_valid
left_boundary_valid
right_boundary_valid
road_polygon_px
left_boundary_px
right_boundary_px
```

检测失败时仍发送 JSON：

```json
{
  "version": 3,
  "frame_seq": 12346,
  "capture_timestamp_ns": 1790670000200000000,
  "publish_timestamp_ns": 1790670000320000000,
  "image_width": 1920,
  "image_height": 1080,
  "coordinate_space": "rotated_image_pixels",
  "scene_type": "unknown",
  "valid": false,
  "road_polygon_valid": false,
  "left_boundary_valid": false,
  "right_boundary_valid": false,
  "confidence": 0.20,
  "source_age_ms": 120.0,
  "road_polygon_px": [],
  "left_boundary_px": [],
  "right_boundary_px": []
}
```

禁止在检测失败时重复发送上一帧的边界。

## 8. 实时性

- 推理最低 `8～10 Hz`，建议 `15 Hz`；
- 推理队列长度设为 `1`，始终处理最新帧；
- 视频编码、推理、JSON 发送和调试绘图分线程；
- 建议使用 TensorRT/ONNX FP16；
- GPU 与 NUC 使用 `chrony` 或 PTP 对时；
- 保留完整后处理边界；仅删除重复点和明确的异常点。

## 9. 调试图

GPU 应在对应的推理输入帧上绘制左右边界：

```python
overlay = frame_context["image"].copy()
cv2.polylines(overlay, [np.asarray(left_boundary)], False, (255, 0, 255), 4)
cv2.polylines(overlay, [np.asarray(right_boundary)], False, (180, 180, 255), 4)
```

NUC 本地 `/hybridnets/debug_image` 只用于接收验证，车辆控制直接读取 JSON。

## 10. NUC 联调

GPU 修改完成后，NUC 使用严格模式：

```bash
roslaunch hybridnets_receiver receiver.launch \
  start_overlay:=true \
  show_overlay:=false \
  require_control_fields:=true
```

检查：

```bash
rostopic hz /hybridnets/result_json
rostopic echo /hybridnets/valid
rostopic echo /hybridnets/confidence
rostopic echo /hybridnets/frame_seq
rostopic echo /hybridnets/left_boundary_valid
rostopic echo /hybridnets/right_boundary_valid
rostopic echo /hybridnets/intersection
```

只有 `/hybridnets/valid=true` 的结果才能进入后续纯视觉靠右控制节点。
