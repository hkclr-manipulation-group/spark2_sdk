"""Monitor joint torque changes while an operator gently applies external force.

This is a sensing/response test, not force control: the current SDK exposes
joint torque feedback but no public force-control command.
"""

from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from application_utils import connect_robot


def main() -> None:
    parser = argparse.ArgumentParser(description="Spark2 关节力矩外力响应测试")
    parser.add_argument("--duration", type=float, default=15.0, help="采样时长，单位 s")
    parser.add_argument("--rate", type=float, default=10.0, help="采样频率，单位 Hz")
    parser.add_argument("--threshold", type=float, default=0.5, help="力矩变化提示阈值")
    args = parser.parse_args()
    if args.duration <= 0 or args.rate <= 0 or args.threshold <= 0:
        parser.error("所有参数必须大于 0")

    robot = connect_robot()
    try:
        print("保持机械臂静止，正在采集 1 秒基线……")
        baseline_samples = []
        for _ in range(max(1, round(args.rate))):
            baseline_samples.append(robot.get_tor().arm.to_list())
            time.sleep(1.0 / args.rate)
        baseline = [sum(row[j] for row in baseline_samples) / len(baseline_samples) for j in range(6)]

        print("请轻柔地对末端施加外力；超过阈值的关节会标记为 *")
        deadline = time.monotonic() + args.duration
        while time.monotonic() < deadline:
            torque = robot.get_tor().arm.to_list()
            delta = [value - zero for value, zero in zip(torque, baseline)]
            text = "  ".join(
                f"J{i + 1}:{value:+7.3f}{'*' if abs(value) >= args.threshold else ' '}"
                for i, value in enumerate(delta)
            )
            print(text)
            time.sleep(1.0 / args.rate)
    finally:
        robot.stop()


if __name__ == "__main__":
    main()
