"""Shared helpers for Spark2 on-robot application tests."""

from __future__ import annotations

import time
import sys
from pathlib import Path
from typing import Callable, Optional

PYTHON_DIST_DIR = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PYTHON_DIST_DIR))

from spark2_sdk import JointState6b, PlanResult, RobotState, Spark2


CONFIG_DIR = Path(__file__).resolve().parents[2] / "configuration"


def connect_robot() -> Spark2:
    if input("确认工作空间安全并已准备急停？输入 YES 继续: ").strip() != "YES":
        raise SystemExit("测试已取消")

    robot = Spark2(str(CONFIG_DIR))
    robot.start()
    robot.enable_arm_joint(JointState6b([True] * 6))
    return robot


def wait_until_idle(
    robot: Spark2,
    timeout: float,
    sample: Optional[Callable[[float], None]] = None,
) -> None:
    started = time.monotonic()
    # Give the controller time to consume the command before accepting IDLE.
    time.sleep(0.1)
    while True:
        elapsed = time.monotonic() - started
        status = robot.get_status()
        if sample is not None:
            sample(elapsed)
        if status.plan_result != PlanResult.SUCCESS:
            raise RuntimeError(f"轨迹执行失败: {status.plan_result.name}")
        if status.robot_state == RobotState.IDLE:
            return
        if elapsed >= timeout:
            raise TimeoutError(f"等待运动完成超时（{timeout:.1f} 秒）")
        time.sleep(0.1)


def move_and_wait(robot: Spark2, command: Callable[[], None], timeout: float = 30.0) -> None:
    command()
    wait_until_idle(robot, timeout)


def run_motion(command: Callable[[Spark2], None], timeout: float) -> None:
    robot = connect_robot()
    try:
        command(robot)
        wait_until_idle(robot, timeout)
        print("测试轨迹执行完成")
    finally:
        robot.stop()
