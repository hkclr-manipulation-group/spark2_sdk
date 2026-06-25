from _spark2_sdk_cpp import Spark2Internal, ConfiguratorInternal, KinematicsInternal, \
SystemStatusInternal, RobotStateInternal, PlanResultInternal, PoseInternal, PositionInternal, QuaternionInternal
from typing import List

from .types import *
from .kinematics import Kinematics
from .configurator import Configurator
from .convertor import pose_to_internal, internal_to_pose, internal_to_robot_joint_state_f, \
internal_to_robot_joint_state_b, internal_to_system_status, system_status_to_internal

class Spark2:
    def __init__(self, config_prefix_path: str = "") -> None:
        self._internal = Spark2Internal(config_prefix_path)
        self._configurator = None
        self._kinematics = None

    # --- Connection ---
    def start(self) -> None:
        self._internal.start()

    def stop(self) -> None:
        self._internal.stop()
        
    def enable_arm_joint(self, arm_state: JointState6b) -> None:
        self._internal.enable_arm_joint(arm_state.to_list())

    # --- Motion mode ---
    def set_arm_smoothing_method(self, method: SmoothingMethod) -> None:
        self._internal.set_arm_smoothing_method(method.value)

    # --- Manual mode: joint space ---
    def move_pos(self, arm_pos: JointState6f, v: int = 50, t: float = 0.0) -> None:
        self._internal.move_pos(arm_pos.to_list(), v, t)

    def move_pos_path(self, arm_pos: List[JointState6f], v: List[int], t: List[float]) -> None:
        arm_pos_internal = [pos.to_list() for pos in arm_pos]
        self._internal.move_pos_path(arm_pos_internal, v, t)

    def move_vel(self, arm_vel: JointState6f, a: int = 50, t: float = 0.0) -> None:
        self._internal.move_vel(arm_vel.to_list(), a, t)

    # --- Manual mode: end-effector ---
    def move_ee_point(self, arm_ee: Pose, v: int = 50, t: float = 0.0) -> None:
        pose_internal = pose_to_internal(arm_ee)
        self._internal.move_ee_point(pose_internal, v, t)

    def move_ee_point_path(self, arm_ee: List[Pose], v: List[int], t: List[float]) -> None:
        pose_internal = [pose_to_internal(ee) for ee in arm_ee]
        self._internal.move_ee_point_path(pose_internal, v, t)

    def move_ee_line(self, arm_ee: Pose, v: int = 50, t: float = 0.0) -> None:
        pose_internal = pose_to_internal(arm_ee)
        self._internal.move_ee_line(pose_internal, v, t)

    def move_ee_line_path(self, arm_ee: List[Pose], v: List[int], t: List[float]) -> None:
        pose_internal = [pose_to_internal(ee) for ee in arm_ee]
        self._internal.move_ee_line_path(pose_internal, v, t)

    # --- Manual mode: tool frame ---
    def move_tool_point(self, arm_tool: Pose, v: int = 50, t: float = 0.0) -> None:
        pose_internal = pose_to_internal(arm_tool)
        self._internal.move_tool_point(pose_internal, v, t)

    def move_tool_point_path(self, arm_tool: List[Pose], v: List[int], t: List[float]) -> None:
        pose_internal = [pose_to_internal(tool) for tool in arm_tool]
        self._internal.move_tool_point_path(pose_internal, v, t)

    def move_tool_line(self, arm_tool: Pose, v: int = 50, t: float = 0.0) -> None:
        pose_internal = pose_to_internal(arm_tool)
        self._internal.move_tool_line(pose_internal, v, t)

    def move_tool_line_path(self, arm_tool: List[Pose], v: List[int], t: List[float]) -> None:
        pose_internal = [pose_to_internal(tool) for tool in arm_tool]
        self._internal.move_tool_line_path(pose_internal, v, t)

    # --- Manual mode: home & gripper ---
    def go_home(self, v: int = 50, t: float = 0.0) -> None:
        self._internal.go_home(v, t)
    def move_gripper_pos(self, pos: JointState1f, v: int = 50, t: float = 0.0) -> None:
        self._internal.move_gripper_pos(pos.to_list(), v, t)

    # --- Teach mode ---
    def start_teach(self) -> None:
        self._internal.start_teach()
        
    def stop_teach(self) -> None:
        self._internal.stop_teach()

    # --- Playback mode ---
    def start_playback(self) -> None:
        self._internal.start_playback()

    def stop_playback(self) -> None:
        self._internal.stop_playback()

    def reset_playback(self) -> None:
        self._internal.reset_playback()

    # --- Feedback ---
    def get_pos(self) -> RobotJointStatef:
        pos = self._internal.get_pos()
        return internal_to_robot_joint_state_f(pos)

    def get_vel(self) -> RobotJointStatef:
        vel = self._internal.get_vel()
        return internal_to_robot_joint_state_f(vel)

    def get_tor(self) -> RobotJointStatef:
        tor = self._internal.get_tor()
        return internal_to_robot_joint_state_f(tor)

    def get_ee_pose(self) -> Pose:
        pose = self._internal.get_ee_pose()
        return internal_to_pose(pose)

    def get_tool_pose(self) -> Pose:
        pose = self._internal.get_tool_pose()
        return internal_to_pose(pose)

    # --- Status ---
    def is_arm_joint_enabled(self) -> JointState6b:
        arm_state = self._internal.is_arm_joint_enabled()
        return JointState6b.from_sequence(arm_state)

    def get_status(self) -> SystemStatus:
        status = self._internal.get_status()
        return internal_to_system_status(status)
        
    def print_status(self, status: SystemStatus) -> None:
        status_internal = system_status_to_internal(status)
        self._internal.print_status(status_internal)

    # --- Subsystems ---
    def get_configurator(self) -> Configurator:
        if self._configurator is None:
            raw_config = self._internal.get_configurator()
            self._configurator = Configurator(raw_config)
        return self._configurator

    def get_kinematics(self) -> Kinematics:
        if self._kinematics is None:
            raw_kin = self._internal.get_kinematics()
            self._kinematics = Kinematics(raw_kin)
        return self._kinematics
