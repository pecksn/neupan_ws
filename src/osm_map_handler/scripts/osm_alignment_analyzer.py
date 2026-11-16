#!/usr/bin/env python3
# Module contract: Offline geometry comparison between geographic measurements and OSM
# road segments. Distances are diagnostic evidence rather than online localization
# corrections.

import argparse
import csv
import json
import math
import xml.etree.ElementTree as ET
from collections import defaultdict
from pathlib import Path


EARTH_RADIUS = 6378137.0


# Project latitude/longitude into planar metres about the selected map origin.
# Inputs: lat, lon, origin_lat, origin_lon.
def latlon_to_xy(lat, lon, origin_lat, origin_lon):
    x = (
        (lon - origin_lon)
        * math.pi
        / 180.0
        * EARTH_RADIUS
        * math.cos(origin_lat * math.pi / 180.0)
    )
    y = (lat - origin_lat) * math.pi / 180.0 * EARTH_RADIUS
    return x, y


# Parse offline analysis input paths and coordinate options.
def parse_args():
    parser = argparse.ArgumentParser(
        description="Measure GPS-to-OSM road alignment from JSONL logs."
    )
    parser.add_argument(
        "logs", nargs="+", help="system_run_logger or compact open_loop JSONL logs"
    )
    parser.add_argument("--osm", default="/home/nuc/lxw/ros_maps/map.osm")
    parser.add_argument("--origin-lat", type=float, default=23.0398)
    parser.add_argument("--origin-lon", type=float, default=113.39002)
    parser.add_argument("--sample-step", type=int, default=10)
    parser.add_argument("--max-samples", type=int, default=2000)
    parser.add_argument("--max-dist", type=float, default=30.0)
    parser.add_argument("--csv-out", default="")
    return parser.parse_args()


# Read OSM road geometry and convert the selected segments into the local planar map.
# Inputs: osm_path, origin_lat, origin_lon.
def load_osm_segments(osm_path, origin_lat, origin_lon):
    root = ET.parse(osm_path).getroot()
    nodes = {}
    for node in root.findall("node"):
        node_id = node.attrib.get("id")
        lat = float(node.attrib.get("lat", "nan"))
        lon = float(node.attrib.get("lon", "nan"))
        if node_id and math.isfinite(lat) and math.isfinite(lon):
            nodes[node_id] = latlon_to_xy(lat, lon, origin_lat, origin_lon)

    segments = []
    for way in root.findall("way"):
        tags = {
            tag.attrib.get("k", ""): tag.attrib.get("v", "")
            for tag in way.findall("tag")
        }
        highway = tags.get("highway", "")
        if not highway:
            continue
        refs = [nd.attrib.get("ref", "") for nd in way.findall("nd")]
        refs = [ref for ref in refs if ref in nodes]
        for start, end in zip(refs, refs[1:]):
            x1, y1 = nodes[start]
            x2, y2 = nodes[end]
            length = math.hypot(x2 - x1, y2 - y1)
            if length < 1e-6:
                continue
            segments.append(
                {
                    "way_id": way.attrib.get("id", ""),
                    "name": tags.get("name", ""),
                    "highway": highway,
                    "x1": x1,
                    "y1": y1,
                    "x2": x2,
                    "y2": y2,
                    "length": length,
                }
            )
    return segments


# Extract a usable geographic coordinate pair from an input record.
# Inputs: record.
def extract_lat_lon(record):
    data = record.get("data", record)
    gps_fix = data.get("gps_fix")
    if isinstance(gps_fix, dict):
        lat = gps_fix.get("latitude")
        lon = gps_fix.get("longitude")
        if isinstance(lat, (int, float)) and isinstance(lon, (int, float)):
            return lat, lon

    lat = data.get("lat")
    lon = data.get("lon")
    if isinstance(lat, int) and isinstance(lon, int):
        return lat / 1e7, lon / 1e7
    return None


# Read supported measurement records into the point set used by the alignment report.
# Inputs: log_paths, origin_lat, origin_lon, sample_step, max_samples.
def load_points(log_paths, origin_lat, origin_lon, sample_step, max_samples):
    points = []
    seen = 0
    for log_path in log_paths:
        with open(log_path, "r") as handle:
            for line in handle:
                if not line.strip():
                    continue
                try:
                    record = json.loads(line)
                except ValueError:
                    continue
                lat_lon = extract_lat_lon(record)
                if lat_lon is None:
                    continue
                lat, lon = lat_lon
                if not (math.isfinite(lat) and math.isfinite(lon)):
                    continue
                if abs(lat) < 1e-7 and abs(lon) < 1e-7:
                    continue
                seen += 1
                if seen % max(1, sample_step) != 0:
                    continue
                x, y = latlon_to_xy(lat, lon, origin_lat, origin_lon)
                points.append(
                    {"lat": lat, "lon": lon, "x": x, "y": y, "log": str(log_path)}
                )
                if len(points) >= max_samples:
                    return points
    return points


# Find the nearest supported OSM segment and return the corresponding geometric
# evidence.
# Inputs: point, segments.
def nearest_segment(point, segments):
    best = None
    px, py = point["x"], point["y"]
    for segment in segments:
        x1, y1 = segment["x1"], segment["y1"]
        x2, y2 = segment["x2"], segment["y2"]
        dx, dy = x2 - x1, y2 - y1
        length = segment["length"]
        t = max(0.0, min(1.0, ((px - x1) * dx + (py - y1) * dy) / (length * length)))
        sx, sy = x1 + t * dx, y1 + t * dy
        east_error = sx - px
        north_error = sy - py
        dist = math.hypot(east_error, north_error)
        heading = math.degrees(math.atan2(dy, dx))
        signed_left = (px - x1) * (-dy / length) + (py - y1) * (dx / length)
        candidate = {
            **segment,
            "snap_x": sx,
            "snap_y": sy,
            "dist": dist,
            "east_error": east_error,
            "north_error": north_error,
            "signed_left": signed_left,
            "heading_deg": heading,
        }
        if best is None or candidate["dist"] < best["dist"]:
            best = candidate
    return best


# Compute the arithmetic mean of the supplied values.
# Inputs: values.
def mean(values):
    return sum(values) / len(values) if values else float("nan")


# Run the offline alignment analysis and report distances; this tool does not publish
# vehicle commands.
def main():
    args = parse_args()
    log_paths = [Path(path) for path in args.logs]
    segments = load_osm_segments(args.osm, args.origin_lat, args.origin_lon)
    points = load_points(
        log_paths, args.origin_lat, args.origin_lon, args.sample_step, args.max_samples
    )
    rows = []
    for point in points:
        nearest = nearest_segment(point, segments)
        if nearest is None or nearest["dist"] > args.max_dist:
            continue
        rows.append({**point, **nearest})

    if not rows:
        print("No usable GPS-to-OSM samples.")
        return 1

    print("samples:", len(rows))
    print("mean_dist_m: %.3f" % mean([row["dist"] for row in rows]))
    print(
        "mean_east_error_m: %.3f  (OSM snap x - GPS x)"
        % mean([row["east_error"] for row in rows])
    )
    print(
        "mean_north_error_m: %.3f (OSM snap y - GPS y)"
        % mean([row["north_error"] for row in rows])
    )

    by_way = defaultdict(list)
    for row in rows:
        by_way[row["way_id"]].append(row)
    print("\nTop ways by sample count:")
    for way_id, group in sorted(
        by_way.items(), key=lambda item: len(item[1]), reverse=True
    )[:10]:
        first = group[0]
        print(
            "way=%s count=%d name=%r highway=%s mean_dist=%.2f mean_east=%.2f mean_north=%.2f heading=%.1f"
            % (
                way_id,
                len(group),
                first["name"],
                first["highway"],
                mean([row["dist"] for row in group]),
                mean([row["east_error"] for row in group]),
                mean([row["north_error"] for row in group]),
                mean([row["heading_deg"] for row in group]),
            )
        )

    if args.csv_out:
        fieldnames = [
            "log",
            "lat",
            "lon",
            "x",
            "y",
            "way_id",
            "name",
            "highway",
            "snap_x",
            "snap_y",
            "dist",
            "east_error",
            "north_error",
            "signed_left",
            "heading_deg",
        ]
        with open(args.csv_out, "w", newline="") as handle:
            writer = csv.DictWriter(handle, fieldnames=fieldnames)
            writer.writeheader()
            for row in rows:
                writer.writerow({key: row.get(key, "") for key in fieldnames})
        print("\nwrote:", args.csv_out)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
