#!/usr/bin/env python3
"""Click validation pixels and compare IPM predictions with known ground points."""

import argparse
import json
import math

import cv2
import numpy as np


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("image")
    parser.add_argument("--ground", required=True, help="JSON list of known [x,y] points")
    parser.add_argument("--matrix", required=True, help="ipm_points.json")
    args = parser.parse_args()

    image = cv2.imread(args.image)
    if image is None:
        raise SystemExit("cannot read image: {}".format(args.image))
    ground = np.asarray(json.loads(args.ground), dtype=np.float64)
    if ground.ndim != 2 or ground.shape[1] != 2 or len(ground) < 1:
        raise SystemExit("--ground must contain [x,y] points")

    with open(args.matrix, "r", encoding="utf-8") as handle:
        matrix_data = json.load(handle)
    H = np.asarray(matrix_data["image_to_base_link_ground"], dtype=np.float64)

    points = []
    display = image.copy()
    window = "Click validation points in --ground order; q=quit"

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

    pixels = np.asarray(points, dtype=np.float64)
    homogeneous = np.column_stack((pixels, np.ones(len(pixels))))
    projected = (H @ homogeneous.T).T
    predicted = projected[:, :2] / projected[:, 2:3]
    errors = np.linalg.norm(predicted - ground, axis=1)

    print("index  pixel(u,v)       known(x,y)       predicted(x,y)    error(m)")
    for index, (pixel, known, estimate, error) in enumerate(zip(pixels, ground, predicted, errors), 1):
        print("{:>5}  ({:7.1f},{:7.1f})  ({:6.3f},{:6.3f})  ({:6.3f},{:6.3f})    {:.3f}".format(
            index, pixel[0], pixel[1], known[0], known[1], estimate[0], estimate[1], error))
    print("max_error_m={:.3f}".format(float(np.max(errors))))
    print("mean_error_m={:.3f}".format(float(np.mean(errors))))


if __name__ == "__main__":
    main()
