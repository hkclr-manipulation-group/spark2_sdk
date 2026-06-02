# Spark2 SDK: Repository Structure and API

- Language: C++20

## Repository Structure

```text
spark2_sdk/
|-- CMakeLists.txt
|-- include/
|   |-- spark2.h          (main SDK class API)
|   |-- configurator.h   (configuration API)
|   |-- kinematics.h     (kinematics API)
|   `-- types.h          (shared data types/enums)
|-- examples/
|   |-- move_single_point.cpp
|   |-- move_path.cpp
|   |-- set_configuration.cpp
|   |-- teach_mode.cpp
|   |-- playback_mode.cpp
|   `-- calculate_kinematics.cpp
```

## Core SDK API (`Spark2`)

| API | Description | Group |
|---|---|---|
| `Spark2(config_prefix_path="")` | Create SDK client; optional path prefix for configuration files. | constructor |
| `start(), stop()` | Start or stop arm runtime session. | connection |
| `enableArmJoint(arm_state)` | Enable arm joints by 6-element boolean state array. | connection |
| `setArmSmoothingMethod(method)` | Select smoothing strategy for single-target arm motion. | motion mode |
| `movePos(arm_pos, v=50, t=0)` | Command joint-space target position. | manual motion |
| `movePosPath(path, vPath, tPath)` | Command sequence of joint-space waypoints. | manual motion |
| `moveVel(arm_vel, a=10, t=0)` | Command joint-space velocity. | manual motion |
| `moveEEPoint(pose, v=50, t=0)` | Move end-effector to cartesian pose (point mode). | manual motion |
| `moveEEPointPath(path, vPath, tPath)` | Move through end-effector pose waypoints (point mode). | manual motion |
| `moveEELine(pose, v=50, t=0)` | Move end-effector to cartesian pose (line mode). | manual motion |
| `moveEELinePath(path, vPath, tPath)` | Move through end-effector pose waypoints (line mode). | manual motion |
| `moveToolPoint(pose, v=50, t=0)` | Move tool frame to cartesian pose (point mode). | manual motion |
| `moveToolPointPath(path, vPath, tPath)` | Move tool frame through cartesian waypoints (point mode). | manual motion |
| `moveToolLine(pose, v=50, t=0)` | Move tool frame to cartesian pose (line mode). | manual motion |
| `moveToolLinePath(path, vPath, tPath)` | Move tool frame through cartesian waypoints (line mode). | manual motion |
| `goHome(v=50, t=0)` | Move robot arm to predefined home pose. | manual motion |
| `moveGripperPos(pos, v=50, t=0)` | Command gripper joint position. | manual motion |
| `startTeach(), stopTeach()` | Enter or exit teach mode. | teach mode |
| `startPlayback(), stopPlayback(), resetPlayback()` | Control playback mode lifecycle. | playback mode |
| `loadPlayback(filename)` | Load playback trajectory or script file. | playback mode |
| `getPos(), getVel(), getTor()` | Read current joint position, velocity, and torque. | feedback |
| `getEEPose(), getToolPose()` | Read current end-effector/tool cartesian pose. | feedback |
| `isArmJointEnabled()` | Read per-joint arm enable state. | status |
| `getStatus(), printStatus(status)` | Read or print aggregated runtime status (`SystemStatus`). | status |
| `getConfigurator(), getKinematics()` | Access specialized subsystems for config/kinematics. | subsystems |

## Configuration API (`Configurator`)

| API | Description |
|---|---|
| `setToolOffset(offset), getToolOffset()` | Set or get tool frame offset from end-effector frame. |
| `setArmPosLimits(limit), setArmVelLimits(limit)` | Set joint position/velocity safety limits. |
| `getArmPosLimits(), getArmVelLimits()` | Get active joint safety limits. |
| `getArmMaxPos(), getArmMaxVel()` | Get hardware max position/velocity constraints. |
| `getArmMaxPosJump(), getArmMaxVelJump()` | Get anti-jump protection limits. |
| `getArmMaxPosFollowError(), getArmMaxVelFollowError(), getArmMaxTorFollowError()` | Get following-error thresholds. |
| `setGripperPosLimits(limit), getGripperPosLimits()` | Set or get gripper position limits. |
| `getGripperMaxPos()` | Get gripper hardware max range. |

## Kinematics API (`Kinematics`)

| API | Description |
|---|---|
| `forwardKinematics(arm_pos)` | Compute cartesian pose from 6-joint arm state. |
| `inverseKinematics(ee_pose, arm_pos)` | Solve 6-joint arm state for a target cartesian pose; returns `bool`. |
| `eulerToQuaternion(euler)` | Convert intrinsic XYZ Euler angles to quaternion. |
| `quaternionToEuler(quat)` | Convert quaternion to intrinsic XYZ Euler angles. |
| `eeToToolPose(ee_pose, tool_offset)` | Transform end-effector pose into tool pose. |
| `toolToEEPose(tool_pose, tool_offset)` | Transform tool pose into end-effector pose. |

## Types (`types.h`)

| Type / Enum | Description |
|---|---|
| `Position`, `Quaternion`, `EulerAngle` | Cartesian position, orientation, and intrinsic XYZ Euler angles. |
| `Pose` | Cartesian pose = `Position` + `Quaternion` orientation. |
| `JointState6f / JointState1f` | 6-DOF arm joint values / 1-DOF gripper value. |
| `JointState6b / JointState1b` | 6-DOF arm enable flags / gripper enable flag. |
| `JointState6u / JointState1u` | Per-joint diagnostic bitmasks for arm and gripper. |
| `Path<T>` | Waypoint sequence (`std::vector<T>`). |
| `RobotJointStatef / RobotJointStateb` | Combined arm+gripper state in float or bool forms. |
| `JointLimits<T>`, `JointLimits6f / JointLimits1f` | Per-joint min/max limits for arm and gripper. |
| `SmoothingMethod` | `kLinear`, `kCos`, `kCubic`, `kQuintic`, `kNone`. |
| `RobotState` | `kStartup`, `kIdle`, `kMoving`, `kSettling`, `kError`, `kRecovery`, `kShutdown`. |
| `PlanResult` | `kSuccess`, `kPoseNotReachable`, `kLinearPathFailed`. |
| `SystemStatus` | Aggregated status: `robot_state`, `plan_result`, `robot_diagnostic_flags`, per-joint diagnostic flags. |
| `DiagnosticFlags` | Bitmask flags for target/actuator saturation, jump limits, boundary safety, planner events, and hardware/tracking faults. |
