from utils import get_config_prefix_path, print_feedback
from spark2_sdk import Spark2, Pose, Position, JointState6b, JointState6f

def main() -> None:
    config_prefix_path = get_config_prefix_path()
    robot = Spark2(config_prefix_path)
    robot.start()
    robot.enable_arm_joint(JointState6b([True, True, True, True, True, True]))
    print("Arm started")

    v = 30
    t = 5.0
    print_dt = 0.3
    timeout = 50.0

    # --------------------move joint from point to point---------------------
    prefix_text = "==> move_pos (10.0, 20.0, 30.0, 40.0, 50.0, 60.0)\n"
    robot.move_pos(JointState6f([10.0, 20.0, 30.0, 40.0, 50.0, 60.0]), v, 0.0)
    print_feedback(robot, print_dt, timeout, prefix_text, print_joint=True, print_tool=True)
    current_pose = robot.get_tool_pose()

    # --------------------move tool from point to point----------------------
    target_pose = Pose(
        position=Position(
            x=current_pose.position.x,
            y=current_pose.position.y - 0.1,
            z=current_pose.position.z - 0.05,
        ),
        orientation=current_pose.orientation,
    )
    prefix_text = (
        f"==> move_tool_point (position: ({target_pose.position.x}, "
        f"{target_pose.position.y}, {target_pose.position.z}), "
        f"orientation: ({target_pose.orientation.w}, {target_pose.orientation.x}, "
        f"{target_pose.orientation.y}, {target_pose.orientation.z}))\n"
    )
    robot.move_tool_point(target_pose, v, 0.0)
    print_feedback(robot, print_dt, timeout, prefix_text, print_joint=True, print_tool=True)

    # --------------------move tool in a straight line-----------------------
    prefix_text = "==> move_pos (10.0, 20.0, 30.0, 40.0, 50.0, 60.0)\n"
    robot.move_pos(JointState6f([10.0, 20.0, 30.0, 40.0, 50.0, 60.0]), v, 0.0)
    print_feedback(robot, print_dt, timeout, prefix_text, print_joint=True, print_tool=True)
    current_pose = robot.get_tool_pose()

    target_pose = Pose(
        position=Position(
            x=current_pose.position.x,
            y=current_pose.position.y + 0.1,
            z=current_pose.position.z - 0.05,
        ),
        orientation=current_pose.orientation,
    )
    prefix_text = (
        f"==> move_tool_line (position: ({target_pose.position.x}, "
        f"{target_pose.position.y}, {target_pose.position.z}), "
        f"orientation: ({target_pose.orientation.w}, {target_pose.orientation.x}, "
        f"{target_pose.orientation.y}, {target_pose.orientation.z}))\n"
    )
    robot.move_tool_line(target_pose, 0, t)
    print_feedback(robot, print_dt, timeout, prefix_text, print_joint=True, print_tool=True)

    # --------------------------------goHome--------------------------------
    prefix_text = "==> Go to home position (0.0, 0.0, 0.0, 0.0, 0.0, 0.0)\n"
    robot.go_home(20, t)  # t would extend if leading joint exceeded v
    print_feedback(robot, print_dt, timeout, prefix_text, print_joint=True, print_tool=True)

    print("Motion completed")
    robot.stop()


if __name__ == "__main__":
    main()
