import os
import sys
import time
from typing import Any
from .types import RobotState, PlanResult

def get_config_prefix_path() -> str:
    """
    Search upward through parent directories to find 'cuarm_configuration'.
    Sets OS environment variable automatically if found.
    """
    # If already explicitly set by user, honor their terminal decision
    if "CONFIG_PREFIX_PATH" in os.environ and os.environ["CONFIG_PREFIX_PATH"]:
        return os.environ["CONFIG_PREFIX_PATH"] + "/arm_v1"

    # Start looking upward from this active script file's directory layout
    current_dir = os.path.dirname(os.path.abspath(__file__))
    
    # Iterate through all parental directories up to filesystem root
    while True:
            potential_config_dir = os.path.join(current_dir, "cuarm_configuration")
            
            if os.path.isdir(potential_config_dir):
                # Found it! Set the environment variable to this absolute string path
                os.environ["CONFIG_PREFIX_PATH"] = potential_config_dir
                print(f"[SDK Init] Auto-detected config path: {os.environ['CONFIG_PREFIX_PATH']}")
                return os.environ["CONFIG_PREFIX_PATH"] + "/arm_v1"
            
            # Move up to the next parent directory
            parent_dir = os.path.dirname(current_dir)
            
            # Break the loop if we hit the filesystem root (dirname ceases to change)
            if parent_dir == current_dir:
                break
            current_dir = parent_dir

    # Sane fallback warning if repository layout is completely broken/missing the folder
    print("[SDK Warning] Could not automatically locate 'cuarm_configuration' in parent tree.")
    return ""

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
