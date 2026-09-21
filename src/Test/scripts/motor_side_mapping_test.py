#!/usr/bin/env python3
"""Identify whether physical left/right motors match the MCU LT/RT fields."""

import argparse
import re
import statistics
import sys
import time

import serial


MOTOR_RE = re.compile(
    r"^MOTOR,LT:(?P<lt>[-+0-9.eE]+),RT:(?P<rt>[-+0-9.eE]+),"
    r"LMOS:(?P<lmos>[-+0-9.eE]+),LCOIL:(?P<lcoil>[-+0-9.eE]+),"
    r"RMOS:(?P<rmos>[-+0-9.eE]+),RCOIL:(?P<rcoil>[-+0-9.eE]+)$"
)


def read_samples(port, seconds):
    samples = []
    deadline = time.monotonic() + seconds
    while time.monotonic() < deadline:
        line = port.readline().decode("utf-8", errors="replace").strip()
        match = MOTOR_RE.match(line)
        if not match:
            continue
        sample = {key: float(value) for key, value in match.groupdict().items()}
        samples.append(sample)
        print(
            "\rLT {lt:+6.3f} RT {rt:+6.3f} Nm | "
            "LMOS {lmos:5.1f} LCOIL {lcoil:5.1f} | "
            "RMOS {rmos:5.1f} RCOIL {rcoil:5.1f} C".format(**sample),
            end="", flush=True,
        )
    print()
    if not samples:
        raise RuntimeError("未收到带标签的 MOTOR 遥测帧")
    return samples


def median(samples, key):
    return statistics.median(sample[key] for sample in samples)


def response(samples, baseline, key):
    return max(abs(sample[key] - baseline[key]) for sample in samples)


def main():
    parser = argparse.ArgumentParser(description="左右电机遥测映射安全测试")
    parser.add_argument("--port", default="/dev/ttyUSB0")
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--seconds", type=float, default=6.0,
                        help="每侧手动转轮采样时间")
    parser.add_argument("--min-response", type=float, default=0.03,
                        help="有效扭矩变化下限 (Nm)")
    args = parser.parse_args()

    print("安全要求：左右驱动轮必须可靠架空，人员远离轮胎。")
    print("当前协议无法独立驱动左右轮；本测试保持零油门并测量手动转轮响应。")
    if input("确认已架空且 robot_base_driver 已停止？输入 YES：").strip() != "YES":
        print("已取消")
        return 2

    with serial.Serial(args.port, args.baud, timeout=0.2) as port:
        port.write(b"CMD,0.00,0.00\n")
        port.flush()
        print("采集静止基线 3 秒……")
        base_samples = read_samples(port, 3.0)
        baseline = {key: median(base_samples, key) for key in ("lt", "rt")}

        input("只手动转动物理【左轮】，按回车后持续来回转动……")
        left_samples = read_samples(port, args.seconds)
        input("停止左轮。只手动转动物理【右轮】，按回车后持续来回转动……")
        right_samples = read_samples(port, args.seconds)
        port.write(b"CMD,0.00,0.00\n")
        port.flush()

    matrix = {
        "physical_left": {
            "LT": response(left_samples, baseline, "lt"),
            "RT": response(left_samples, baseline, "rt"),
        },
        "physical_right": {
            "LT": response(right_samples, baseline, "lt"),
            "RT": response(right_samples, baseline, "rt"),
        },
    }
    print("\n最大扭矩变化 (Nm)：")
    print("  物理左轮 -> LT {:.3f}, RT {:.3f}".format(
        matrix["physical_left"]["LT"], matrix["physical_left"]["RT"]))
    print("  物理右轮 -> LT {:.3f}, RT {:.3f}".format(
        matrix["physical_right"]["LT"], matrix["physical_right"]["RT"]))

    left_field = max(matrix["physical_left"], key=matrix["physical_left"].get)
    right_field = max(matrix["physical_right"], key=matrix["physical_right"].get)
    strongest = min(matrix["physical_left"][left_field],
                    matrix["physical_right"][right_field])
    if strongest < args.min_response or left_field == right_field:
        print("结论：响应不足或两侧串扰，无法可靠判断；请更明显地转轮后重测。")
        return 1
    if left_field == "LT" and right_field == "RT":
        print("结论：左右映射正常（物理左=LT，物理右=RT）。")
    else:
        print("结论：左右映射反了（物理左=RT，物理右=LT）。")
    return 0


if __name__ == "__main__":
    try:
        sys.exit(main())
    except KeyboardInterrupt:
        print("\n测试已中止；油门保持为零。")
        sys.exit(130)
    except (OSError, serial.SerialException, RuntimeError) as exc:
        print("错误：{}".format(exc), file=sys.stderr)
        sys.exit(1)
