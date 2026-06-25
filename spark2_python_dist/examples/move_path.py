from utils import get_config_prefix_path, print_feedback
from spark2_sdk import Spark2, Pose, Position, JointState6b, JointState6f

def main() -> None:
    config_prefix_path = get_config_prefix_path()
    robot = Spark2(config_prefix_path)
    robot.start()
    robot.enable_arm_joint(JointState6b([True, True, True, True, True, True]))
    print("Arm started")

    print_dt = 0.3
    timeout = 50.0

    prefix_text = "==> move_pos (10.0, 20.0, 30.0, 40.0, 50.0, 60.0)\n"
    robot.move_pos(JointState6f([10.0, 20.0, 30.0, 40.0, 50.0, 60.0]), 0, 5.0)
    print_feedback(robot, print_dt, timeout, prefix_text, print_joint=True, print_tool=True)
    current_pose = robot.get_tool_pose()

    # ----------------------------move tool in a straight line---------------------------------
    prefix_text = "==> Move Tool Line\n"
    target_pose = Pose(
        position=Position(
            x=current_pose.position.x,
            y=current_pose.position.y - 0.1,
            z=current_pose.position.z - 0.05,
        ),
        orientation=current_pose.orientation,
    )
    path1 = [target_pose, current_pose]
    robot.move_tool_line_path(path1, [30, 30], [])
    print_feedback(robot, print_dt, timeout, prefix_text, print_joint=True, print_tool=True)

    prefix_text = "==> move_pos (10.0, 20.0, 30.0, 40.0, 50.0, 60.0)\n"
    robot.move_pos(JointState6f([10.0, 20.0, 30.0, 40.0, 50.0, 60.0]), 0, 5.0)
    print_feedback(robot, print_dt, timeout, prefix_text, print_joint=True, print_tool=True)
    current_pose = robot.get_tool_pose()

    # --------------------------------move tool from point to point---------------------------
    prefix_text = "==> Move Tool Point\n"
    target_pose = Pose(
        position=Position(
            x=current_pose.position.x - 0.2,
            y=current_pose.position.y + 0.1,
            z=current_pose.position.z - 0.05,
        ),
        orientation=current_pose.orientation,
    )
    path2 = [target_pose, current_pose]
    robot.move_tool_point_path(path2, [], [5.0, 5.0])
    print_feedback(robot, print_dt, timeout, prefix_text, print_joint=True, print_tool=True)

    # --------------------------------move joint from point to point--------------------------------
    prefix_text = "==> Move Joint Pos\n"
    path3 = [
        JointState6f([-10.0, -20.0, -30.0, -10.0, -10.0, -10.0]),
        JointState6f([0.0, 0.0, 0.0, 0.0, 0.0, 0.0]),
    ]
    robot.move_pos_path(path3, [], [10.0, 10.0])
    print_feedback(robot, print_dt, timeout, prefix_text, print_joint=True, print_tool=True)

    print("Motion completed")
    robot.stop()

if __name__ == "__main__":
    main()
