from spark2_sdk.utils import get_config_prefix_path, print_feedback
from spark2_sdk import Spark2, Pose, Position, Quaternion, JointState6b, JointState6f

def main() -> None:
    config_prefix_path = get_config_prefix_path()
    arm = Spark2(config_prefix_path)
    arm.start()
    arm.enable_arm_joint(JointState6b([True, True, True, True, True, True]))
    print("Arm started")

    v = 30
    t = 5.0
    print_dt = 0.3
    timeout = 50.0

    prefix_text = "==> move_pos (10.0, 20.0, 30.0, 40.0, 50.0, 60.0)\n"
    arm.move_pos(JointState6f([10.0, 20.0, 30.0, 40.0, 50.0, 60.0]), v, 0.0)
    print_feedback(arm, print_dt, timeout, prefix_text, print_joint=True, print_tool=True)

    prefix_text = (
        "==> move_tool_point (position: (0.431085, -0.09439, 0.410693), "
        "orientation: (0.40558, 0.704416, -0.061629, 0.579228))\n"
    )
    arm.move_tool_point(
        Pose(
            position=Position(x=0.431085, y=-0.09439, z=0.410693),
            orientation=Quaternion(w=0.40558, x=0.704416, y=-0.061629, z=0.579228),
        ),
        v,
        0.0,
    )
    print_feedback(arm, print_dt, timeout, prefix_text, print_joint=True, print_tool=True)

    prefix_text = "==> move_pos (10.0, 20.0, 30.0, 40.0, 50.0, 60.0)\n"
    arm.move_pos(JointState6f([10.0, 20.0, 30.0, 40.0, 50.0, 60.0]), v, 0.0)
    print_feedback(arm, print_dt, timeout, prefix_text, print_joint=True, print_tool=True)

    prefix_text = (
        "==> move_tool_line (position: (0.431085, -0.09439, 0.410693), "
        "orientation: (0.40558, 0.704416, -0.061629, 0.579228))\n"
    )
    arm.move_tool_line(
        Pose(
            position=Position(x=0.431085, y=-0.09439, z=0.410693),
            orientation=Quaternion(w=0.40558, x=0.704416, y=-0.061629, z=0.579228),
        ),
        0,
        t,
    )
    print_feedback(arm, print_dt, timeout, prefix_text, print_joint=True, print_tool=True)

    print("==> Go to home position (0.0, 0.0, 0.0, 0.0, 0.0, 0.0)")
    arm.go_home(20, t)
    print_feedback(arm, print_dt, timeout, "==> Go home\n", print_joint=True, print_tool=True)

    print("Motion completed")
    arm.stop()


if __name__ == "__main__":
    main()
