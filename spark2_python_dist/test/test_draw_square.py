"""Draw a closed square in the tool XY plane from the current tool pose."""

from __future__ import annotations

import argparse
import sys
from pathlib import Path
from typing import List

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from spark2_sdk import Pose, Position, Spark2

from application_utils import run_motion


def make_square(start: Pose, side: float) -> List[Pose]:
    x, y, z = start.position.x, start.position.y, start.position.z
    corners = [(x + side, y), (x + side, y + side), (x, y + side), (x, y)]
    return [
        Pose(Position(px, py, z), start.orientation)
        for px, py in corners
    ]


def main() -> None:
    parser = argparse.ArgumentParser(description="Spark2 末端 XY 平面正方形轨迹测试")
    parser.add_argument("--side", type=float, default=0.05, help="边长，单位 m")
    parser.add_argument("--speed", type=int, default=15, help="轨迹速度百分比")
    args = parser.parse_args()
    if args.side <= 0 or not 1 <= args.speed <= 100:
        parser.error("side > 0，并且 speed 在 1~100 之间")

    def command(robot: Spark2) -> None:
        path = make_square(robot.get_tool_pose(), args.side)
        print(f"执行正方形轨迹：边长 {args.side:.3f} m")
        robot.move_tool_line_path(path, [args.speed] * len(path), [])

    run_motion(command, timeout=60.0)


if __name__ == "__main__":
    main()
