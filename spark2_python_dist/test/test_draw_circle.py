"""Draw a circle in the tool XY plane, relative to the current tool pose."""

from __future__ import annotations

import argparse
import math
import sys
from pathlib import Path
from typing import List

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from spark2_sdk import Pose, Position, Spark2

from application_utils import run_motion


def make_circle(start: Pose, radius: float, points: int) -> List[Pose]:
    # Put the first point at the current pose to avoid an initial position jump.
    center_x = start.position.x - radius
    center_y = start.position.y
    path = []
    for index in range(1, points + 1):
        angle = 2.0 * math.pi * index / points
        path.append(
            Pose(
                position=Position(
                    x=center_x + radius * math.cos(angle),
                    y=center_y + radius * math.sin(angle),
                    z=start.position.z,
                ),
                orientation=start.orientation,
            )
        )
    return path


def main() -> None:
    parser = argparse.ArgumentParser(description="Spark2 末端 XY 平面画圆测试")
    parser.add_argument("--radius", type=float, default=0.03, help="圆半径，单位 m")
    parser.add_argument("--points", type=int, default=48, help="圆周离散点数")
    parser.add_argument("--speed", type=int, default=15, help="轨迹速度百分比")
    args = parser.parse_args()
    if args.radius <= 0 or args.points < 8 or not 1 <= args.speed <= 100:
        parser.error("radius > 0、points >= 8，并且 speed 在 1~100 之间")

    def command(robot: Spark2) -> None:
        start = robot.get_tool_pose()
        path = make_circle(start, args.radius, args.points)
        print(f"执行圆形轨迹：半径 {args.radius:.3f} m，{args.points} 个点")
        robot.move_tool_line_path(path, [args.speed] * len(path), [])

    run_motion(command, timeout=60.0)


if __name__ == "__main__":
    main()
