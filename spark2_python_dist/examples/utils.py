import os
import sys
import time
from typing import Any
from spark2_sdk.types import RobotState, PlanResult

def get_config_prefix_path() -> str:
    example_dir = os.path.dirname(os.path.abspath(__file__))
    python_dist_dir = os.path.dirname(example_dir)
    sdk_dir = os.path.dirname(python_dist_dir)
    return os.path.join(sdk_dir, "configuration")

def print_feedback(
    arm: Any,
    dt: float,
    timeout: float,
    prefix_text: str = "",
    print_joint: bool = False,
    print_tool: bool = False,
) -> None:
    elapsed_time = 0.0
    interval = dt
    is_idle = False
    is_interrupted = False

    while elapsed_time <= timeout:
        print(prefix_text, end="")
        print(f"Elapsed time: {elapsed_time:.1f}s")

        if print_joint:
            joint_pos = arm.get_pos()
            print("Current joint position")
            print("\tArm:", list(joint_pos.arm))
            print("\tGripper:", list(joint_pos.gripper))

        if print_tool:
            tool_pose = arm.get_tool_pose()
            print("Current tool pose")
            print(f"\tposition(x, y, z): {tool_pose.position.x}, {tool_pose.position.y}, {tool_pose.position.z}")
            print(
                f"\torientation(w, x, y, z): {tool_pose.orientation.w}, "
                f"{tool_pose.orientation.x}, {tool_pose.orientation.y}, {tool_pose.orientation.z}"
            )

        sys_status = arm.get_status()
        arm.print_status(sys_status)
        is_idle = sys_status.robot_state == RobotState.IDLE
        is_interrupted = sys_status.plan_result != PlanResult.SUCCESS
        print("---------------------------------------------------------------")

        if elapsed_time >= timeout or is_idle:
            break

        time.sleep(interval)
        elapsed_time += interval

    print("is_idle:", is_idle)
    if is_interrupted:
        print("IS INTERRUPTED!!!")
    print("===============================================================")
    time.sleep(1)
