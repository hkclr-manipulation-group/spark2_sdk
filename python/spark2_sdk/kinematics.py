from _spark2_sdk_cpp import KinematicsInternal, PositionInternal, QuaternionInternal
from .types import EulerAngle, JointState6f, Pose, Position, Quaternion
from .convertor import position_to_internal, internal_to_position, quaternion_to_internal, \
internal_to_quaternion, euler_to_internal, internal_to_euler, pose_to_internal, internal_to_pose

class Kinematics:
    def __init__(self, internal: KinematicsInternal) -> None:
        if internal is None:
            raise RuntimeError(
                "Kinematics cannot be created directly. "
                "Please access it through your robot instance, for example: robot.kinematics"
            )
        self._internal = internal

    # --- Forward / inverse kinematics ---
    def forward_kinematics(self, arm_pos: JointState6f) -> Pose:
        pose = self._internal.forward_kinematics(arm_pos.to_list())
        return internal_to_pose(pose)

    def inverse_kinematics(self, ee_pose: Pose, arm_pos: JointState6f) -> tuple[bool, JointState6f]:
        success, arm_pos_solution = self._internal.inverse_kinematics(ee_pose.to_list(), arm_pos.to_list())
        return success, JointState6f.from_sequence(arm_pos_solution)

    # --- Orientation conversion ---
    def euler_to_quaternion(self, euler: EulerAngle) -> Quaternion:
        euler_internal = euler_to_internal(euler)
        quat = self._internal.euler_to_quaternion(euler_internal)
        return internal_to_quaternion(quat)

    def quaternion_to_euler(self, quat: Quaternion) -> EulerAngle:
        quat_internal = QuaternionInternal(w=quat.w, x=quat.x, y=quat.y, z=quat.z)
        euler = self._internal.quaternion_to_euler(quat_internal)
        return internal_to_euler(euler)

    # --- Frame transforms ---
    def ee_to_tool_pose(self, ee_pose: Pose, tool_offset: Position) -> Pose:
        ee_pose_internal = pose_to_internal(ee_pose)
        tool_offset_internal = position_to_internal(tool_offset)
        tool_pose = self._internal.ee_to_tool_pose(ee_pose_internal, tool_offset_internal)
        return internal_to_pose(tool_pose)

    def tool_to_ee_pose(self, tool_pose: Pose, tool_offset: Position) -> Pose:
        tool_pose_internal = pose_to_internal(tool_pose)
        tool_offset_internal = position_to_internal(tool_offset)
        ee_pose = self._internal.tool_to_ee_pose(tool_pose_internal, tool_offset_internal)
        return internal_to_pose(ee_pose)
