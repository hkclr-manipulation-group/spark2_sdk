from utils import get_config_prefix_path, print_feedback
from spark2_sdk import Spark2, JointState6b

def main() -> None:
    config_prefix_path = get_config_prefix_path()
    robot = Spark2(config_prefix_path)
    robot.start()
    robot.enable_arm_joint(JointState6b([True, True, True, True, True, True]))
    print("Arm started")

    print_dt = 0.3
    timeout = 50.0

    #-------------------------Start Teach---------------------------------
    input("Press any key to START TEACH...\n")
    robot.start_teach()

    #-------------------------Stop Teach----------------------------------
    input("Press any key to STOP TEACH...\n")
    robot.stop_teach()

    #----------------------------Reset Playback---------------------------
    input("Press any key to MOVE TO INITIAL POSITION of playback...\n")
    robot.reset_playback()
    print_feedback(robot, print_dt, timeout, "==> Go to initial position of playback.\n", print_joint=True)

    #----------------------------Start Playback---------------------------
    input("Press any key to START REPLAY...\n")
    robot.start_playback()

    #----------------------------Stop Playback---------------------------
    input("Press any key to STOP REPLAY...\n")
    robot.stop_playback()

    #----------------------------Go Home-------------------------------
    input("Press any key to GO HOME...\n")
    robot.go_home(30, 0)
    print_feedback(robot, print_dt, timeout, "==> Go home.\n", print_joint=True)

    print("Motion completed")
    robot.stop()
if __name__ == "__main__":
    main()
