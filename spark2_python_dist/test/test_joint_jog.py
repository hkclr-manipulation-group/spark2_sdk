"""Move each arm joint independently and return to the starting posture."""

import argparse
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from spark2_sdk import JointState6f
from application_utils import connect_robot, move_and_wait


def main() -> None:
    parser = argparse.ArgumentParser(description="Spark2 单关节点动测试")
    parser.add_argument("--delta", type=float, default=5.0, help="关节增量，单位 degree")
    parser.add_argument("--speed", type=int, default=10, help="速度百分比")
    args = parser.parse_args()
    if not 0 < args.delta <= 10 or not 1 <= args.speed <= 30:
        parser.error("delta 应在 0~10 degree，speed 应在 1~30")

    robot = connect_robot()
    try:
        origin = robot.get_pos().arm.to_list()
        for joint in range(6):
            target = origin.copy()
            target[joint] += args.delta
            print(f"J{joint + 1}: +{args.delta:.1f} degree")
            move_and_wait(robot, lambda p=target: robot.move_pos(JointState6f(p), args.speed))
            move_and_wait(robot, lambda: robot.move_pos(JointState6f(origin), args.speed))
        print("六个关节均已测试并返回起始位置")
    finally:
        robot.stop()


if __name__ == "__main__":
    main()
