from spark2_sdk.utils import get_config_prefix_path, print_feedback
from spark2_sdk import Spark2, Pose, Position, Quaternion, JointState6b, JointState6f


def main() -> None:
    config_prefix_path = get_config_prefix_path()
    arm = Spark2(config_prefix_path)
    arm.start()
    arm.enable_arm_joint(JointState6b([True, True, True, True, True, True]))
    print("Arm started")

    print_dt = 0.3
    timeout = 50.0

    prefix_text = "==> move_pos (10.0, 20.0, 30.0, 40.0, 50.0, 60.0)\n"
    arm.move_pos(JointState6f([10.0, 20.0, 30.0, 40.0, 50.0, 60.0]), 0, 5.0)
    print_feedback(arm, print_dt, timeout, prefix_text, print_joint=True, print_tool=True)

    prefix_text = "==> Move Tool Line\n"
    path1 = [
        Pose(
            position=Position(x=0.351085, y=-0.12439, z=0.410693),
            orientation=Quaternion(w=0.40558, x=0.704416, y=-0.061629, z=0.579228),
        ),
        Pose(
            position=Position(x=0.301085, y=-0.16439, z=0.340693),
            orientation=Quaternion(w=0.40558, x=0.704416, y=-0.061629, z=0.579228),
        ),
    ]
    arm.move_tool_line_path(path1, [30, 30], [])
    print_feedback(arm, print_dt, timeout, prefix_text, print_joint=True, print_tool=True)

    prefix_text = "==> Go Home\n"
    arm.go_home(50, 5.0)
    print_feedback(arm, print_dt, timeout, prefix_text, print_joint=True, print_tool=True)

    prefix_text = "==> Move Tool Point\n"
    path2 = [
        Pose(
            position=Position(x=0.431085, y=-0.09439, z=0.410693),
            orientation=Quaternion(w=0.40558, x=0.704416, y=-0.061629, z=0.579228),
        ),
        Pose(
            position=Position(x=0.193, y=0.045, z=0.777),
            orientation=Quaternion(w=0.707107, x=0.0, y=0.0, z=0.707107),
        ),
    ]
    arm.move_tool_point_path(path2, [], [5.0, 5.0])
    print_feedback(arm, print_dt, timeout, prefix_text, print_joint=True, print_tool=True)

    prefix_text = "==> Move Joint Pos\n"
    path3 = [
        JointState6f([-10.0, -20.0, -30.0, -10.0, -10.0, -10.0]),
        JointState6f([0.0, 0.0, 0.0, 0.0, 0.0, 0.0]),
    ]
    arm.move_pos_path(path3, [], [10.0, 10.0])
    print_feedback(arm, print_dt, timeout, prefix_text, print_joint=True, print_tool=True)

    print("Motion completed")
    arm.stop()


if __name__ == "__main__":
    main()
