from spark2_sdk.utils import get_config_prefix_path, print_feedback
from spark2_sdk import Spark2, JointState6b

def main() -> None:
    config_prefix_path = get_config_prefix_path()
    robot = Spark2(config_prefix_path)
    robot.start()
    robot.enable_robot_joint(JointState6b([True, True, True, True, True, True]))
    print("Arm started")

    print("==> You can freely move the robot robot to any pose.")
    robot.start_teach()
    input("Press Enter to stop...")
    print("Key pressed. Hand off the robot now.")

    robot.stop_teach()
    print("Stop teach mode and go home.")
    robot.go_home()
    print_feedback(robot, 0.3, 15.0, "==> Go home.\n", print_joint=True)

    print("Motion completed")
    robot.stop()


if __name__ == "__main__":
    main()
