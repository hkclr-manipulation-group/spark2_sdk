from utils import get_config_prefix_path
from spark2_sdk import Spark2, Pose, Position, Quaternion, JointState6f

def main() -> None:
    config_prefix_path = get_config_prefix_path()
    robot = Spark2(config_prefix_path)
    kinematics = robot.get_kinematics()

    joint_pos = JointState6f([10.0, 10.0, 10.0, 0.0, 0.0, 0.0])
    ee_pose = Pose(
        position=Position(x=0.431085, y=-0.09439, z=0.410693),
        orientation=Quaternion(w=0.40558, x=0.704416, y=-0.061629, z=0.579228),
    )

    success, joint_pos = kinematics.inverse_kinematics(ee_pose, joint_pos)
    pose1 = kinematics.forward_kinematics(joint_pos)

    print("IK Target")
    print(f"\tposition(x, y, z): {ee_pose.position.x}, {ee_pose.position.y}, {ee_pose.position.z}")
    print(
        f"\torientation(w, x, y, z): {ee_pose.orientation.w}, "
        f"{ee_pose.orientation.x}, {ee_pose.orientation.y}, {ee_pose.orientation.z}"
    )

    if success:
        print("IK result:", ", ".join(str(x) for x in joint_pos.to_list()))

    print("--------------------------------------------------------")
    print("FK result")
    print(f"\tposition(x, y, z): {pose1.position.x}, {pose1.position.y}, {pose1.position.z}")
    print(
        f"\torientation(w, x, y, z): {pose1.orientation.w}, "
        f"{pose1.orientation.x}, {pose1.orientation.y}, {pose1.orientation.z}"
    )


if __name__ == "__main__":
    main()
