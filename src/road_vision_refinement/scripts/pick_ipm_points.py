#!/usr/bin/env python3
"""Pick image points and compute image-pixel -> base_link ground homography."""

"""
python3 src/road_vision_refinement/scripts/pick_ipm_points.py \
  /home/nuc/lxw/neupan_ws/ipm_ground_0000.png \
  --ground '[[1.0,-0.5],[1.0,0.5],[2.0,-1.0],[2.0,1.0],[3.0,-1.5],[3.0,1.5],[4.0,-2.0],[4.0,2.0]]' \
  --output /home/nuc/lxw/neupan_ws/ipm_points.json
"""
import argparse
import json

import cv2
import numpy as np


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("image", help="rectified RGB image")
    parser.add_argument("--ground", required=True,
                        help="JSON list of [x_forward, y_left] ground points")
    parser.add_argument("--output", default="ipm_points.json")
    args = parser.parse_args()

    image = cv2.imread(args.image)
    if image is None:
        raise SystemExit("cannot read image: {}".format(args.image))
    ground = np.asarray(json.loads(args.ground), dtype=np.float32)
    if ground.ndim != 2 or ground.shape[1] != 2 or len(ground) < 4:
        raise SystemExit("--ground must contain at least four [x,y] points")

    points = []
    display = image.copy()
    window = "Click ground points in the same order as --ground; q=quit"

    def on_mouse(event, x, y, _flags, _param):
        if event != cv2.EVENT_LBUTTONDOWN or len(points) >= len(ground):
            return
        points.append([float(x), float(y)])
        cv2.circle(display, (x, y), 5, (0, 0, 255), -1)
        cv2.putText(display, str(len(points)), (x + 6, y - 6),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)

    cv2.namedWindow(window, cv2.WINDOW_NORMAL)
    cv2.setMouseCallback(window, on_mouse)
    while len(points) < len(ground):
        cv2.imshow(window, display)
        key = cv2.waitKey(20) & 0xFF
        if key == ord("q") or key == 27:
            cv2.destroyAllWindows()
            return
    cv2.imshow(window, display)
    cv2.waitKey(500)
    cv2.destroyAllWindows()

    pixels = np.asarray(points, dtype=np.float32)
    matrix, inlier_mask = cv2.findHomography(pixels, ground, 0)
    if matrix is None:
        raise SystemExit("failed to compute homography")

    result = {
        "image": args.image,
        "image_width": int(image.shape[1]),
        "image_height": int(image.shape[0]),
        "pixel_uv": points,
        "ground_xy_base_link": ground.tolist(),
        "image_to_base_link_ground": matrix.tolist(),
        "inliers": None if inlier_mask is None else inlier_mask.ravel().tolist(),
        "convention": "base_link x forward, y left; H maps [u,v,1] to [x,y,1]",
    }
    with open(args.output, "w", encoding="utf-8") as handle:
        json.dump(result, handle, indent=2)
    print(json.dumps(result, indent=2))


if __name__ == "__main__":
    main()
