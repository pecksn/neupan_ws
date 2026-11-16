#!/usr/bin/env python3
# Module contract: Offline LiDAR/camera target fitting and rigid extrinsic estimation.
# Coordinate direction and image orientation must match the datasets used for
# calibration.

"""Offline MID360/camera extrinsic calibration from FAST-Calib target scenes."""

import argparse
import glob
import itertools
import json
import os

import cv2
import numpy as np
import rosbag


K = np.array(
    [
        [1056.5434295756897, 0.0, 887.6099684823234],
        [0.0, 1057.3350109603232, 687.9125738476482],
        [0.0, 0.0, 1.0],
    ],
    dtype=np.float64,
)
D = np.array(
    [
        -0.40130317211542343,
        0.1845567579908426,
        0.00034687862877420003,
        0.0014981691723600034,
        -0.04118561811519803,
    ]
)

MARKER_SIZE = 0.20
MARKER_HALF_WIDTH = 0.55
MARKER_HALF_HEIGHT = 0.35
CIRCLE_WIDTH = 0.50
CIRCLE_HEIGHT = 0.40
CIRCLE_RADIUS = 0.12


# Extract finite XYZ samples from a PointCloud2 message using the requested stride.
# Inputs: msg, stride.
def pointcloud_xyz(msg, stride=3):
    dtype = {
        "names": ["x", "y", "z"],
        "formats": ["<f4", "<f4", "<f4"],
        "offsets": [0, 4, 8],
        "itemsize": msg.point_step,
    }
    data = np.ndarray((msg.width * msg.height,), dtype=dtype, buffer=msg.data)
    return np.column_stack((data["x"], data["y"], data["z"]))[::stride]


# Load the supported LiDAR point representation for offline target fitting.
# Inputs: path.
def load_lidar_points(path):
    clouds = []
    with rosbag.Bag(path) as bag:
        for _, msg, _ in bag.read_messages(topics=["/livox/lidar"]):
            clouds.append(pointcloud_xyz(msg))
    points = np.vstack(clouds)
    valid = np.isfinite(points).all(axis=1)
    points = points[valid]
    return points[
        (points[:, 0] > 1.7)
        & (points[:, 0] < 2.7)
        & (points[:, 1] > -1.5)
        & (points[:, 1] < 1.5)
        & (points[:, 2] > 0.03)
        & (points[:, 2] < 1.8)
    ]


# Fit the calibration target plane with the supplied deterministic seed and return the
# supported plane evidence.
# Inputs: points, seed.
# Explicit rejection paths: RuntimeError.
def fit_target_plane(points, seed=7):
    rng = np.random.default_rng(seed)
    best = None
    for _ in range(1600):
        tri = points[rng.choice(len(points), 3, replace=False)]
        normal = np.cross(tri[1] - tri[0], tri[2] - tri[0])
        length = np.linalg.norm(normal)
        if length < 1e-8:
            continue
        normal /= length
        if abs(normal[2]) > 0.45:  # reject the floor and horizontal furniture
            continue
        distance = np.abs(points @ normal - tri[0] @ normal)
        mask = distance < 0.012
        score = int(mask.sum())
        if best is None or score > best[0]:
            best = (score, mask)
    if best is None or best[0] < 500:
        raise RuntimeError("could not find the calibration-board plane")

    plane = points[best[1]]
    center = plane.mean(axis=0)
    _, _, axes = np.linalg.svd(plane - center, full_matrices=False)
    uv = (plane - center) @ axes[:2].T
    spans = np.percentile(uv, 99.5, axis=0) - np.percentile(uv, 0.5, axis=0)
    if spans[0] < spans[1]:
        axes[[0, 1]] = axes[[1, 0]]
        uv = uv[:, [1, 0]]
    return plane, center, axes, uv


# Fit the target circle in planar coordinates for correspondence extraction.
# Inputs: points_2d.
def fit_circle(points_2d):
    a = np.column_stack(
        (2.0 * points_2d[:, 0], 2.0 * points_2d[:, 1], np.ones(len(points_2d)))
    )
    b = np.sum(points_2d**2, axis=1)
    cx, cy, c = np.linalg.lstsq(a, b, rcond=None)[0]
    radius = np.sqrt(max(0.0, c + cx * cx + cy * cy))
    return np.array([cx, cy]), radius


# Extract target-circle centers from the LiDAR bag evidence.
# Inputs: bag_path.
# Explicit rejection paths: RuntimeError.
def lidar_circle_centers(bag_path):
    points = load_lidar_points(bag_path)
    plane, origin, axes, uv = fit_target_plane(points)
    lo = np.percentile(uv, 0.5, axis=0)
    hi = np.percentile(uv, 99.5, axis=0)
    board_center = (lo + hi) / 2.0

    fitted = []
    radii = []
    for du, dv in [
        (-CIRCLE_WIDTH / 2, CIRCLE_HEIGHT / 2),
        (CIRCLE_WIDTH / 2, CIRCLE_HEIGHT / 2),
        (CIRCLE_WIDTH / 2, -CIRCLE_HEIGHT / 2),
        (-CIRCLE_WIDTH / 2, -CIRCLE_HEIGHT / 2),
    ]:
        guess = board_center + np.array([du, dv])
        radial = np.linalg.norm(uv - guess, axis=1)
        ring = uv[(radial > 0.085) & (radial < 0.165)]
        if len(ring) < 20:
            raise RuntimeError("not enough LiDAR samples around a target hole")
        circle, radius = fit_circle(ring)
        # Refit only points close to the first fitted circumference.
        radial = np.linalg.norm(uv - circle, axis=1)
        ring = uv[np.abs(radial - CIRCLE_RADIUS) < 0.018]
        circle, radius = fit_circle(ring)
        fitted.append(circle)
        radii.append(radius)

    centers = np.array([origin + p[0] * axes[0] + p[1] * axes[1] for p in fitted])
    return centers, np.asarray(radii), len(plane)


# Extract target-circle centers from the supplied calibration image.
# Inputs: image_path.
# Explicit rejection paths: RuntimeError.
def camera_circle_centers(image_path):
    image = cv2.imread(image_path)
    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_250)
    detector = cv2.aruco.ArucoDetector(dictionary, cv2.aruco.DetectorParameters())
    corners, ids, _ = detector.detectMarkers(image)
    if ids is None or set(ids.ravel()) != {1, 2, 3, 4}:
        raise RuntimeError("expected ArUco IDs 1,2,3,4")

    marker_positions = {
        1: (-MARKER_HALF_WIDTH, MARKER_HALF_HEIGHT),
        2: (MARKER_HALF_WIDTH, MARKER_HALF_HEIGHT),
        4: (MARKER_HALF_WIDTH, -MARKER_HALF_HEIGHT),
        3: (-MARKER_HALF_WIDTH, -MARKER_HALF_HEIGHT),
    }
    board_points = []
    marker_image_centers = []
    for detected, marker_id in zip(corners, ids.ravel()):
        board_points.append(marker_positions[int(marker_id)])
        marker_image_centers.append(detected.reshape(4, 2).mean(axis=0))

    # Work in undistorted normalized coordinates. A plane-to-plane homography
    # transfers the four robust ArUco centers to the physical hole centers, so
    # metric scale comes from the 0.50 x 0.40 m hole pattern rather than from
    # the printed ArUco side length.
    normalized_markers = cv2.undistortPoints(
        np.asarray(marker_image_centers, np.float64).reshape(-1, 1, 2), K, D
    ).reshape(-1, 2)
    homography, _ = cv2.findHomography(
        np.asarray(board_points, np.float64), normalized_markers
    )
    target = np.array(
        [
            [-CIRCLE_WIDTH / 2, CIRCLE_HEIGHT / 2, 0],
            [CIRCLE_WIDTH / 2, CIRCLE_HEIGHT / 2, 0],
            [CIRCLE_WIDTH / 2, -CIRCLE_HEIGHT / 2, 0],
            [-CIRCLE_WIDTH / 2, -CIRCLE_HEIGHT / 2, 0],
        ],
        dtype=np.float64,
    )
    normalized_holes = cv2.perspectiveTransform(
        target[:, :2].reshape(-1, 1, 2), homography
    ).reshape(-1, 2)
    ok, rvec, tvec = cv2.solvePnP(
        target, normalized_holes, np.eye(3), np.zeros(5), flags=cv2.SOLVEPNP_IPPE
    )
    if not ok:
        raise RuntimeError("camera target pose solve failed")
    rotation = cv2.Rodrigues(rvec)[0]
    return (rotation @ target.T).T + tvec.reshape(1, 3)


# Estimate the rigid transformation mapping the source point set to the target point
# set.
# Inputs: source, target.
def rigid_transform(source, target):
    source_mean = source.mean(axis=0)
    target_mean = target.mean(axis=0)
    u, _, vt = np.linalg.svd((source - source_mean).T @ (target - target_mean))
    rotation = vt.T @ u.T
    if np.linalg.det(rotation) < 0:
        vt[-1] *= -1
        rotation = vt.T @ u.T
    translation = target_mean - rotation @ source_mean
    residual = (rotation @ source.T).T + translation - target
    return rotation, translation, np.sqrt(np.mean(np.sum(residual**2, axis=1)))


# Run this module's command-line or ROS entry point. Keep startup, resource ownership,
# and shutdown within this process.
# Explicit rejection paths: RuntimeError.
def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("scene_dir")
    parser.add_argument("--output", required=True)
    args = parser.parse_args()

    bag_paths = sorted(glob.glob(os.path.join(args.scene_dir, "scene_*.bag")))
    scene_names = [
        os.path.splitext(os.path.basename(path))[0]
        for path in bag_paths
        if os.path.isfile(
            os.path.join(
                args.scene_dir, os.path.splitext(os.path.basename(path))[0] + ".jpg"
            )
        )
    ]
    if len(scene_names) < 3:
        raise RuntimeError("at least three scene_*.bag/.jpg pairs are required")
    lidar_sets = []
    camera_sets = []
    diagnostics = []
    for name in scene_names:
        lidar, radii, plane_count = lidar_circle_centers(
            os.path.join(args.scene_dir, name + ".bag")
        )
        camera = camera_circle_centers(os.path.join(args.scene_dir, name + ".jpg"))
        lidar_sets.append(lidar)
        camera_sets.append(camera)
        diagnostics.append(
            {
                "scene": name,
                "plane_points": plane_count,
                "fitted_radii_m": radii.tolist(),
            }
        )

    # Select the rectangle symmetry using the measured mechanical initial guess.
    # Use the previously verified calibration as a correspondence prior. The
    # four-hole rectangle has a 180-degree symmetry, so a coarse axis-only
    # prior can choose the wrong labeling for strongly angled scenes.
    initial_rotation = np.array(
        [
            [-0.031382438, -0.999488148, -0.006211597],
            [-0.013786343, 0.006646921, -0.999882871],
            [0.999412367, -0.031293127, -0.013987883],
        ]
    )
    initial_translation = np.array([0.001691342, -0.080829955, -0.071189642])
    ordered_lidar = []
    used_camera = []
    for index, (lidar, camera) in enumerate(zip(lidar_sets, camera_sets)):
        predicted = (initial_rotation @ lidar.T).T + initial_translation
        permutation = min(
            itertools.permutations(range(4)),
            key=lambda p: np.sum((predicted[list(p)] - camera) ** 2),
        )
        candidate = lidar[list(permutation)]
        _, _, geometry_rmse = rigid_transform(candidate, camera)
        diagnostics[index]["single_scene_geometry_rmse_m"] = float(geometry_rmse)
        diagnostics[index]["used"] = bool(geometry_rmse <= 0.02)
        if geometry_rmse <= 0.02:
            ordered_lidar.append(candidate)
            used_camera.append(camera)

    if len(ordered_lidar) < 3:
        raise RuntimeError("fewer than three geometrically valid scenes")

    all_lidar = np.vstack(ordered_lidar)
    all_camera = np.vstack(used_camera)
    rotation, translation, rmse = rigid_transform(all_lidar, all_camera)
    result = {
        "convention": "p_camera = R_camera_lidar * p_lidar + t_camera_lidar",
        "R_camera_lidar": rotation.tolist(),
        "t_camera_lidar": translation.tolist(),
        "rmse_m": float(rmse),
        "scenes": diagnostics,
    }
    os.makedirs(os.path.dirname(args.output), exist_ok=True)
    with open(args.output, "w", encoding="utf-8") as stream:
        json.dump(result, stream, indent=2)
    print(json.dumps(result, indent=2))


if __name__ == "__main__":
    main()
