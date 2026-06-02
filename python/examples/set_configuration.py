from spark2_sdk.utils import get_config_prefix_path
from spark2_sdk import Spark2, Position, Pose


def main() -> None:
    config_prefix_path = get_config_prefix_path()
    arm = Spark2(config_prefix_path)
    arm.start()
    print("Arm started")

    configurator = arm.get_configurator()
    kinematics = arm.get_kinematics()

    tool_offset = configurator.get_tool_offset()
    tool_pose = arm.get_tool_pose()
    print("Before set_tool_offset(x, y, z):", tool_offset.x, tool_offset.y, tool_offset.z)
    print("Before Tool pose")
    print("\tposition(x, y, z):", tool_pose.position.x, tool_pose.position.y, tool_pose.position.z)
    print(
        "\torientation(w, x, y, z):",
        tool_pose.orientation.w,
        tool_pose.orientation.x,
        tool_pose.orientation.y,
        tool_pose.orientation.z,
    )
    print("--------------------------------------------------------")

    configurator.set_tool_offset(Position(x=0.01, y=0.02, z=0.03))
    tool_offset1 = configurator.get_tool_offset()
    tool_pose1 = arm.get_tool_pose()

    print("After set_tool_offset(x, y, z):", tool_offset1.x, tool_offset1.y, tool_offset1.z)
    print("After Tool pose")
    print("\tposition(x, y, z):", tool_pose1.position.x, tool_pose1.position.y, tool_pose1.position.z)
    print(
        "\torientation(w, x, y, z):",
        tool_pose1.orientation.w,
        tool_pose1.orientation.x,
        tool_pose1.orientation.y,
        tool_pose1.orientation.z,
    )
    print("--------------------------------------------------------")

    tool_pose2 = kinematics.ee_to_tool_pose(tool_pose, tool_offset1)
    print("Verification")
    print("\tposition(x, y, z):", tool_pose2.position.x, tool_pose2.position.y, tool_pose2.position.z)
    print(
        "\torientation(w, x, y, z):",
        tool_pose2.orientation.w,
        tool_pose2.orientation.x,
        tool_pose2.orientation.y,
        tool_pose2.orientation.z,
    )


if __name__ == "__main__":
    main()
