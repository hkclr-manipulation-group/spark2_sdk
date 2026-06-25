# Spark2 SDK: Repository Structure and API

- Language: Python 3.8+
- Backend: C++ SDK exposed via `_spark2_sdk_cpp` (pybind11)

## Repository Structure

The installable Python package lives under `spark2_python_dist/` in the Spark2 SDK repository. Shared robot configuration is bundled at the repository root in `configuration/`.

```text
spark2_sdk/
|-- configuration/                (robot configuration)
|-- spark2_python_dist/
|   |-- pyproject.toml
|   |-- spark2_sdk_python.md
|   |-- spark2_sdk/
|   |   |-- __init__.py           
|   |   |-- spark2.py             (main SDK class API)
|   |   |-- configurator.py       (configuration API)
|   |   |-- kinematics.py         (kinematics API)
|   |   |-- types.py              (shared data types/enums)
|   |   |-- convertor.py          (internal C++/Python type conversion)
|   |   `-- lib/                  (pre-built shared library)
|   `-- examples/
|       |-- utils.py              (example helpers)
|       |-- move_point.py
|       |-- move_path.py
|       |-- teach_and_playback.py
|       |-- set_configuration.py
|       `-- calculate_kinematics.py
```

Platform and toolchain subdirectories under `lib/` are selected automatically at import time based on OS, CPU architecture, and (on Windows) the MSVC runtime version.

## Core SDK API (`Spark2`)

| API | Description | Group |
|---|---|---|
| `Spark2(config_prefix_path="")` | Create SDK client; optional path prefix for configuration files. | constructor |
| `start(), stop()` | Start or stop arm runtime session. | connection |
| `enable_arm_joint(arm_state)` | Enable arm joints by 6-element boolean state (`JointState6b`). | connection |
| `set_arm_smoothing_method(method)` | Select smoothing strategy for single-target arm motion. | motion mode |
| `move_pos(arm_pos, v=50, t=0.0)` | Command joint-space target position. | manual motion |
| `move_pos_path(path, v_path, t_path)` | Command sequence of joint-space waypoints. | manual motion |
| `move_vel(arm_vel, a=50, t=0.0)` | Command joint-space velocity. | manual motion |
| `move_ee_point(pose, v=50, t=0.0)` | Move end-effector to cartesian pose (point mode). | manual motion |
| `move_ee_point_path(path, v_path, t_path)` | Move through end-effector pose waypoints (point mode). | manual motion |
| `move_ee_line(pose, v=50, t=0.0)` | Move end-effector to cartesian pose (line mode). | manual motion |
| `move_ee_line_path(path, v_path, t_path)` | Move through end-effector pose waypoints (line mode). | manual motion |
| `move_tool_point(pose, v=50, t=0.0)` | Move tool frame to cartesian pose (point mode). | manual motion |
| `move_tool_point_path(path, v_path, t_path)` | Move tool frame through cartesian waypoints (point mode). | manual motion |
| `move_tool_line(pose, v=50, t=0.0)` | Move tool frame to cartesian pose (line mode). | manual motion |
| `move_tool_line_path(path, v_path, t_path)` | Move tool frame through cartesian waypoints (line mode). | manual motion |
| `go_home(v=50, t=0.0)` | Move robot arm to predefined home pose. | manual motion |
| `move_gripper_pos(pos, v=50, t=0.0)` | Command gripper joint position. | manual motion |
| `start_teach(), stop_teach()` | Enter or exit teach mode. | teach mode |
| `start_playback(), stop_playback(), reset_playback()` | Control playback mode lifecycle. | playback mode |
| `get_pos(), get_vel(), get_tor()` | Read current joint position, velocity, and torque. | feedback |
| `get_ee_pose(), get_tool_pose()` | Read current end-effector/tool cartesian pose. | feedback |
| `is_arm_joint_enabled()` | Read per-joint arm enable state. | status |
| `get_status(), print_status(status)` | Read or print aggregated runtime status (`SystemStatus`). | status |
| `get_configurator(), get_kinematics()` | Access specialized subsystems for config/kinematics. | subsystems |

## Configuration API (`Configurator`)

| API | Description |
|---|---|
| `set_tool_offset(offset), get_tool_offset()` | Set or get tool frame offset from end-effector frame. |
| `set_arm_pos_limits(limit), set_arm_vel_limits(limit)` | Set joint position/velocity safety limits. |
| `get_arm_pos_limits(), get_arm_vel_limits()` | Get active joint safety limits. |
| `get_arm_max_pos(), get_arm_max_vel()` | Get hardware max position/velocity constraints. |
| `get_arm_max_pos_jump(), get_arm_max_vel_jump()` | Get anti-jump protection limits. |
| `get_arm_max_pos_follow_error(), get_arm_max_vel_follow_error(), get_arm_max_tor_follow_error()` | Get following-error thresholds. |
| `set_gripper_pos_limits(limit), get_gripper_pos_limits()` | Set or get gripper position limits. |
| `get_gripper_max_pos()` | Get gripper hardware max range. |

## Kinematics API (`Kinematics`)

| API | Description |
|---|---|
| `forward_kinematics(arm_pos)` | Compute cartesian pose from 6-joint arm state. |
| `inverse_kinematics(ee_pose, arm_pos)` | Solve 6-joint arm state for a target cartesian pose; returns `(bool, JointState6f)`. |
| `euler_to_quaternion(euler)` | Convert intrinsic XYZ Euler angles to quaternion. |
| `quaternion_to_euler(quat)` | Convert quaternion to intrinsic XYZ Euler angles. |
| `ee_to_tool_pose(ee_pose, tool_offset)` | Transform end-effector pose into tool pose. |
| `tool_to_ee_pose(tool_pose, tool_offset)` | Transform tool pose into end-effector pose. |

## Types (`types.py`)

| Type / Enum | Description |
|---|---|
| `Position`, `Quaternion`, `EulerAngle` | Cartesian position, orientation, and intrinsic XYZ Euler angles. |
| `Pose` | Cartesian pose = `Position` + `Quaternion` orientation. |
| `JointState6f` / `JointState1f` | 6-DOF arm joint values / 1-DOF gripper value. |
| `JointState6b` / `JointState1b` | 6-DOF arm enable flags / gripper enable flag. |
| `JointState6u` / `JointState1u` | Per-joint diagnostic bitmasks for arm and gripper. |
| `List[T]` | Waypoint sequence (e.g. `List[Pose]`, `List[JointState6f]`). |
| `RobotJointStatef` / `RobotJointStateb` | Combined arm+gripper state in float or bool forms. |
| `JointLimits6f` / `JointLimits1f` | Per-joint min/max limits for arm and gripper (`LimitPair` per joint). |
| `SmoothingMethod` | `LINEAR`, `COS`, `CUBIC`, `QUINTIC`, `NONE`. |
| `RobotState` | `STARTUP`, `IDLE`, `MOVING`, `SETTLING`, `ERROR`, `RECOVERY`, `SHUTDOWN`. |
| `PlanResult` | `SUCCESS`, `POSE_NOT_REACHABLE`, `LINEAR_PATH_FAILED`. |
| `SystemStatus` | Aggregated status: `robot_state`, `plan_result`, `robot_diagnostic_flags`, per-joint diagnostic flags. |
| `DiagnosticFlags` | Bitmask flags for target/actuator saturation, jump limits, boundary safety, planner events, and hardware/tracking faults. |
