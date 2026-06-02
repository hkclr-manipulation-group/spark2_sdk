from spark2_sdk.utils import get_config_prefix_path, print_feedback
from spark2_sdk import Spark2, JointState6b

def main() -> None:
    config_prefix_path = get_config_prefix_path()
    arm = Spark2(config_prefix_path)
    arm.start()
    arm.enable_arm_joint(JointState6b([True, True, True, True, True, True]))
    print("Arm started")

    print("==> You can freely move the robot arm to any pose.")
    arm.start_teach()
    input("Press Enter to stop...")
    print("Key pressed. Hand off the robot now.")

    arm.stop_teach()
    print("Stop teach mode and go home.")
    arm.go_home()
    print_feedback(arm, 0.3, 15.0, "==> Go home.\n", print_joint=True)

    print("Motion completed")
    arm.stop()


if __name__ == "__main__":
    main()
