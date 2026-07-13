"""Estimate bidirectional Cartesian repeatability at one target point."""

import argparse
import math
import statistics
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from spark2_sdk import Pose, Position
from application_utils import connect_robot, move_and_wait


def distance(a: Pose, b: Pose) -> float:
    return math.sqrt(
        (a.position.x - b.position.x) ** 2
        + (a.position.y - b.position.y) ** 2
        + (a.position.z - b.position.z) ** 2
    )


def main() -> None:
    parser = argparse.ArgumentParser(description="Spark2 重复定位测试")
    parser.add_argument("--cycles", type=int, default=10, help="往返次数")
    parser.add_argument("--distance", type=float, default=0.03, help="目标 X 偏移，单位 m")
    parser.add_argument("--speed", type=int, default=15, help="速度百分比")
    args = parser.parse_args()
    if not 2 <= args.cycles <= 100 or not 0 < args.distance <= 0.05:
        parser.error("cycles 应在 2~100，distance 应在 0~0.05 m")

    robot = connect_robot()
    try:
        origin = robot.get_tool_pose()
        target = Pose(
            Position(origin.position.x + args.distance, origin.position.y, origin.position.z),
            origin.orientation,
        )
        samples = []
        for cycle in range(args.cycles):
            move_and_wait(robot, lambda: robot.move_tool_line(target, args.speed))
            measured = robot.get_tool_pose()
            error_mm = distance(measured, target) * 1000.0
            samples.append(error_mm)
            print(f"Cycle {cycle + 1:02d}: target position error = {error_mm:.3f} mm")
            move_and_wait(robot, lambda: robot.move_tool_line(origin, args.speed))

        print(f"Mean error: {statistics.mean(samples):.3f} mm")
        print(f"Max error:  {max(samples):.3f} mm")
        print(f"Std dev:    {statistics.stdev(samples):.3f} mm")
    finally:
        robot.stop()


if __name__ == "__main__":
    main()
