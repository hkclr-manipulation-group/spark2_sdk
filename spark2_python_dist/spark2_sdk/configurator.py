from .types import (
    JointLimits1f,
    JointLimits6f,
    JointState6f,
    Position,
)
from .convertor import position_to_internal, internal_to_position, joint_limits_6f_to_internal, \
internal_to_joint_limits_6f, joint_limits_1f_to_internal, internal_to_joint_limits_1f
from _spark2_sdk_cpp import ConfiguratorInternal, PositionInternal, JointLimitsFloatInternal

class Configurator:
    def __init__(self, internal: ConfiguratorInternal) -> None:
        if internal is None:
            raise RuntimeError(
                "Configurator cannot be created directly. "
                "Please access it through your robot instance, for example: robot.configurator"
            )
        self._internal = internal

    # --- Tool offset ---
    def set_tool_offset(self, offset: Position) -> None:
        self._internal.set_tool_offset(PositionInternal(x=offset.x, y=offset.y, z=offset.z))

    def get_tool_offset(self) -> Position:
        offset = self._internal.get_tool_offset()
        return Position(x=offset.x, y=offset.y, z=offset.z)

    # --- Arm safety limits ---
    def set_arm_pos_limits(self, limit: JointLimits6f) -> None:
        limit_internal = joint_limits_6f_to_internal(limit)
        self._internal.set_arm_pos_limits(limit_internal)

    def set_arm_vel_limits(self, limit: JointLimits6f) -> None:
        limit_internal = joint_limits_6f_to_internal(limit)
        self._internal.set_arm_vel_limits(limit_internal)

    def get_arm_pos_limits(self) -> JointLimits6f:
        limits = self._internal.get_arm_pos_limits()
        return internal_to_joint_limits_6f(limits)

    def get_arm_vel_limits(self) -> JointLimits6f:
        limits = self._internal.get_arm_vel_limits()
        return internal_to_joint_limits_6f(limits)

    # --- Arm hardware & protection limits ---
    def get_arm_max_pos(self) -> JointLimits6f:
        limits = self._internal.get_arm_max_pos()
        return internal_to_joint_limits_6f(limits)

    def get_arm_max_vel(self) -> JointLimits6f:
        limits = self._internal.get_arm_max_vel()
        return internal_to_joint_limits_6f(limits)

    def get_arm_max_pos_jump(self) -> JointState6f:
        state = self._internal.get_arm_max_pos_jump()
        return JointState6f.from_sequence([state[i] for i in range(6)])

    def get_arm_max_vel_jump(self) -> JointState6f:
        state = self._internal.get_arm_max_vel_jump()
        return JointState6f.from_sequence([state[i] for i in range(6)])

    def get_arm_max_pos_follow_error(self) -> JointState6f:
        state = self._internal.get_arm_max_pos_follow_error()
        return JointState6f.from_sequence([state[i] for i in range(6)])

    def get_arm_max_vel_follow_error(self) -> JointState6f:
        state = self._internal.get_arm_max_vel_follow_error()
        return JointState6f.from_sequence([state[i] for i in range(6)])

    def get_arm_max_tor_follow_error(self) -> JointState6f:
        state = self._internal.get_arm_max_tor_follow_error()
        return JointState6f.from_sequence([state[i] for i in range(6)])

    # --- Gripper safety limits ---
    def set_gripper_pos_limits(self, limit: JointLimits1f) -> None:
        limit_internal = joint_limits_1f_to_internal(limit)
        self._internal.set_gripper_pos_limits(limit_internal)

    def get_gripper_pos_limits(self) -> JointLimits1f:
        limits = self._internal.get_gripper_pos_limits()
        return internal_to_joint_limits_1f(limits)

    def get_gripper_max_pos(self) -> JointLimits1f:
        limits = self._internal.get_gripper_max_pos()
        return internal_to_joint_limits_1f(limits)
