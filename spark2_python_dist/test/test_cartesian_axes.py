"""Exercise positive and negative X/Y/Z tool translations."""

import argparse
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from spark2_sdk import Pose, Position
from application_utils import connect_robot, move_and_wait


def shifted(pose, axis: int, distance: float) -> Pose:
    xyz = [pose.position.x, pose.position.y, pose.position.z]
    xyz[axis] += distance
    return Pose(Position(*xyz), pose.orientation)


def main() -> None:
    parser = argparse.ArgumentParser(description="Spark2 笛卡尔 XYZ 方向运动测试")
    parser.add_argument("--distance", type=float, default=0.02, help="移动距离，单位 m")
    parser.add_argument("--speed", type=int, default=10, help="速度百分比")
    args = parser.parse_args()
    if not 0 < args.distance <= 0.05 or not 1 <= args.speed <= 30:
        parser.error("distance 应在 0~0.05 m，speed 应在 1~30")

    robot = connect_robot()
    try:
        origin = robot.get_tool_pose()
        for axis, name in enumerate("XYZ"):
            for sign in (1.0, -1.0):
                print(f"{name} 方向: {sign * args.distance:+.3f} m")
                target = shifted(origin, axis, sign * args.distance)
                move_and_wait(robot, lambda p=target: robot.move_tool_line(p, args.speed))
                move_and_wait(robot, lambda: robot.move_tool_line(origin, args.speed))
        print("XYZ 正负方向测试完成")
    finally:
        robot.stop()


if __name__ == "__main__":
    main()
