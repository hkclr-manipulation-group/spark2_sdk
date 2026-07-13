"""Record joint position, velocity and torque during a short square path."""

import argparse
import csv
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from spark2_sdk import Pose, Position
from application_utils import connect_robot, wait_until_idle


def main() -> None:
    parser = argparse.ArgumentParser(description="Spark2 运动反馈 CSV 记录测试")
    parser.add_argument("--side", type=float, default=0.03, help="方形边长，单位 m")
    parser.add_argument("--speed", type=int, default=10, help="速度百分比")
    parser.add_argument("--output", default="motion_feedback.csv", help="输出 CSV 文件")
    args = parser.parse_args()
    if not 0 < args.side <= 0.05 or not 1 <= args.speed <= 30:
        parser.error("side 应在 0~0.05 m，speed 应在 1~30")

    stream = open(args.output, "w", newline="", encoding="utf-8")
    robot = connect_robot()
    try:
        start = robot.get_tool_pose()
        x, y, z = start.position.x, start.position.y, start.position.z
        path = [
            Pose(Position(px, py, z), start.orientation)
            for px, py in ((x + args.side, y), (x + args.side, y + args.side),
                           (x, y + args.side), (x, y))
        ]
        writer = csv.writer(stream)
        writer.writerow(["time_s"] + [f"pos_j{i}" for i in range(1, 7)]
                        + [f"vel_j{i}" for i in range(1, 7)]
                        + [f"tor_j{i}" for i in range(1, 7)])

        def sample(elapsed: float) -> None:
            writer.writerow([f"{elapsed:.4f}"] + robot.get_pos().arm.to_list()
                            + robot.get_vel().arm.to_list() + robot.get_tor().arm.to_list())

        robot.move_tool_line_path(path, [args.speed] * len(path), [])
        wait_until_idle(robot, 60.0, sample)
        print(f"反馈数据已保存到: {Path(args.output).resolve()}")
    finally:
        stream.close()
        robot.stop()


if __name__ == "__main__":
    main()
