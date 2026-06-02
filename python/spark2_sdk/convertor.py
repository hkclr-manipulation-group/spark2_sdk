from _spark2_sdk_cpp import PoseInternal, PositionInternal, QuaternionInternal, EulerAngleInternal, \
RobotJointStateFloatInternal, RobotJointStateBoolInternal, SystemStatusInternal, JointLimitsFloatInternal, \
RobotStateInternal, PlanResultInternal
from .types import *
from typing import List

def pose_to_internal(pose: Pose) -> PoseInternal:
    return PoseInternal(
        position=PositionInternal(x=pose.position.x, y=pose.position.y, z=pose.position.z),
        orientation=QuaternionInternal(w=pose.orientation.w, x=pose.orientation.x, y=pose.orientation.y, z=pose.orientation.z),
    )

def internal_to_pose(pose_internal: PoseInternal) -> Pose:
    return Pose(
        position=Position(x=pose_internal.position.x, y=pose_internal.position.y, z=pose_internal.position.z),
        orientation=Quaternion(w=pose_internal.orientation.w, x=pose_internal.orientation.x, y=pose_internal.orientation.y, z=pose_internal.orientation.z),
    )

def position_to_internal(position: Position) -> PositionInternal:
    return PositionInternal(x=position.x, y=position.y, z=position.z)

def internal_to_position(position_internal: PositionInternal) -> Position:
    return Position(x=position_internal.x, y=position_internal.y, z=position_internal.z)

def quaternion_to_internal(quaternion: Quaternion) -> QuaternionInternal:
    return QuaternionInternal(w=quaternion.w, x=quaternion.x, y=quaternion.y, z=quaternion.z)

def internal_to_quaternion(quaternion_internal: QuaternionInternal) -> Quaternion:
    return Quaternion(w=quaternion_internal.w, x=quaternion_internal.x, y=quaternion_internal.y, z=quaternion_internal.z)

def euler_to_internal(euler_angle: EulerAngle) -> EulerAngleInternal:
    return EulerAngleInternal(roll=euler_angle.roll, pitch=euler_angle.pitch, yaw=euler_angle.yaw)

def internal_to_euler(euler_angle_internal: EulerAngleInternal) -> EulerAngle:
    return EulerAngle(roll=euler_angle_internal.roll, pitch=euler_angle_internal.pitch, yaw=euler_angle_internal.yaw)

def internal_to_robot_joint_state_f(robot_joint_state_internal: RobotJointStateFloatInternal) -> RobotJointStatef:
    return RobotJointStatef(
        arm=JointState6f(robot_joint_state_internal.arm),
        gripper=JointState1f(robot_joint_state_internal.gripper),
    )

def internal_to_robot_joint_state_b(robot_joint_state_internal: RobotJointStateBoolInternal) -> RobotJointStateb:
    return RobotJointStateb(
        arm=JointState6b(robot_joint_state_internal.arm),
        gripper=JointState1b(robot_joint_state_internal.gripper),
    )

def internal_to_system_status(system_status_internal: SystemStatusInternal) -> SystemStatus:
    return SystemStatus(
        robot_state=RobotState(value=int(system_status_internal.robot_state.value)),
        plan_result=PlanResult(value=int(system_status_internal.plan_result.value)),
        robot_diagnostic_flags=system_status_internal.robot_diagnostic_flags,
        arm_joint_diagnostic_flags=JointState6u(system_status_internal.arm_joint_diagnostic_flags),
        gripper_joint_diagnostic_flags=JointState1u(system_status_internal.gripper_joint_diagnostic_flags),
    )

def system_status_to_internal(system_status: SystemStatus) -> SystemStatusInternal:
    return SystemStatusInternal(
        robot_state=RobotStateInternal(value=int(system_status.robot_state.value)),
        plan_result=PlanResultInternal(value=int(system_status.plan_result.value)),
        robot_diagnostic_flags=system_status.robot_diagnostic_flags,
        arm_joint_diagnostic_flags=system_status.arm_joint_diagnostic_flags.to_list(),
        gripper_joint_diagnostic_flags=system_status.gripper_joint_diagnostic_flags.to_list(),
    )

def joint_limits_6f_to_internal(joint_limits: JointLimits6f) -> List[JointLimitsFloatInternal]:
    return [JointLimitsFloatInternal(min=joint_limits[i].min, max=joint_limits[i].max) for i in range(len(joint_limits))]

def internal_to_joint_limits_6f(joint_limits_internal: List[JointLimitsFloatInternal]) -> JointLimits6f:
    return JointLimits6f(data=[[joint_limits_internal[i].min, joint_limits_internal[i].max] for i in range(len(joint_limits_internal))])

def joint_limits_1f_to_internal(joint_limits: JointLimits1f) -> List[JointLimitsFloatInternal]:
    return [JointLimitsFloatInternal(min=joint_limits[i].min, max=joint_limits[i].max) for i in range(len(joint_limits))]

def internal_to_joint_limits_1f(joint_limits_internal: List[JointLimitsFloatInternal]) -> JointLimits1f:
    return JointLimits1f(data=[[joint_limits_internal[i].min, joint_limits_internal[i].max] for i in range(len(joint_limits_internal))])