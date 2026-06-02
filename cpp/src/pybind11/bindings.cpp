#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include "spark2.h"
#include "configurator.h"
#include "kinematics.h"
#include "types.h"

namespace py = pybind11;

PYBIND11_MODULE(_spark2_sdk_cpp, m) {
    m.doc() = "Python bindings for the Spark2 robot SDK";

    py::enum_<spark2::SmoothingMethod>(m, "SmoothingMethodInternal")
        .value("LINEAR", spark2::SmoothingMethod::kLinear)
        .value("COS", spark2::SmoothingMethod::kCos)
        .value("CUBIC", spark2::SmoothingMethod::kCubic)
        .value("QUINTIC", spark2::SmoothingMethod::kQuintic)
        .value("NONE", spark2::SmoothingMethod::kNone)
        .export_values();

    py::enum_<spark2::RobotState>(m, "RobotStateInternal")
        .value("STARTUP", spark2::RobotState::kStartup)
        .value("IDLE", spark2::RobotState::kIdle)
        .value("MOVING", spark2::RobotState::kMoving)
        .value("SETTLING", spark2::RobotState::kSettling)
        .value("ERROR", spark2::RobotState::kError)
        .value("RECOVERY", spark2::RobotState::kRecovery)
        .value("SHUTDOWN", spark2::RobotState::kShutdown)
        .export_values();

    py::enum_<spark2::PlanResult>(m, "PlanResultInternal")
        .value("SUCCESS", spark2::PlanResult::kSuccess)
        .value("POSE_NOT_REACHABLE", spark2::PlanResult::kPoseNotReachable)
        .value("LINEAR_PATH_FAILED", spark2::PlanResult::kLinearPathFailed)
        .export_values();

    py::module_ diag = m.def_submodule("DiagnosticFlagsInternal", "Diagnostic flags for Spark2 system status");
    diag.attr("NONE") = spark2::DiagnosticFlags::kNone;
    diag.attr("TARGET_POS_SATURATION") = spark2::DiagnosticFlags::kTargetPosSaturation;
    diag.attr("TARGET_VEL_SATURATION") = spark2::DiagnosticFlags::kTargetVelSaturation;
    diag.attr("TARGET_TOR_SATURATION") = spark2::DiagnosticFlags::kTargetTorSaturation;
    diag.attr("ACTUATOR_POS_SATURATION") = spark2::DiagnosticFlags::kActuatorPosSaturation;
    diag.attr("ACTUATOR_VEL_SATURATION") = spark2::DiagnosticFlags::kActuatorVelSaturation;
    diag.attr("ACTUATOR_TOR_SATURATION") = spark2::DiagnosticFlags::kActuatorTorSaturation;
    diag.attr("ACTUATOR_POS_JUMP_SATURATION") = spark2::DiagnosticFlags::kActuatorPosJumpSaturation;
    diag.attr("ACTUATOR_VEL_JUMP_SATURATION") = spark2::DiagnosticFlags::kActuatorVelJumpSaturation;
    diag.attr("ACTUATOR_TOR_JUMP_SATURATION") = spark2::DiagnosticFlags::kActuatorTorJumpSaturation;
    diag.attr("BOUNDARY_VEL_CLAMP") = spark2::DiagnosticFlags::kBoundaryVelClamp;
    diag.attr("BOUNDARY_JOINT_IMPEDANCE") = spark2::DiagnosticFlags::kBoundaryJointImpedance;
    diag.attr("PLAN_TIMELINE_EXTENDED") = spark2::DiagnosticFlags::kPlanTimelineExtended;
    diag.attr("PLAN_VEL_LIMIT_INVALID") = spark2::DiagnosticFlags::kPlanVelLimitInvalid;
    diag.attr("PLAN_DELTA_TOO_LARGE") = spark2::DiagnosticFlags::kPlanDeltaTooLarge;
    diag.attr("PLAN_VELOCITY_SNAP") = spark2::DiagnosticFlags::kPlanVelocitySnap;
    diag.attr("PLAN_POINT_SKIPPED") = spark2::DiagnosticFlags::kPlanPointSkipped;
    diag.attr("FAULT_POS_HARD_LIMIT_REACHED") = spark2::DiagnosticFlags::kFaultPosHardLimitReached;
    diag.attr("FAULT_VEL_HARD_LIMIT_REACHED") = spark2::DiagnosticFlags::kFaultVelHardLimitReached;
    diag.attr("FAULT_TOR_HARD_LIMIT_REACHED") = spark2::DiagnosticFlags::kFaultTorHardLimitReached;
    diag.attr("FAULT_POS_TRACKING_FAILED") = spark2::DiagnosticFlags::kFaultPosTrackingFailed;
    diag.attr("FAULT_VEL_TRACKING_FAILED") = spark2::DiagnosticFlags::kFaultVelTrackingFailed;
    diag.attr("FAULT_TOR_TRACKING_FAILED") = spark2::DiagnosticFlags::kFaultTorTrackingFailed;
    diag.attr("FAULT_ARM_NOT_FOUND") = spark2::DiagnosticFlags::kFaultArmNotFound;
    diag.attr("FAULT_GRIPPER_NOT_FOUND") = spark2::DiagnosticFlags::kFaultGripperNotFound;
    diag.attr("FAULT_HARDWARE_INIT_FAILED") = spark2::DiagnosticFlags::kFaultHardwareInitFailed;
    diag.attr("FAULT_UNKNOWN") = spark2::DiagnosticFlags::kFaultUnknown;

    py::class_<spark2::Quaternion>(m, "QuaternionInternal")
        .def(py::init([](float w, float x, float y, float z) {
            return spark2::Quaternion{w, x, y, z};
        }), py::arg("w") = 1.0f, py::arg("x") = 0.0f, py::arg("y") = 0.0f, py::arg("z") = 0.0f)
        .def_readwrite("w", &spark2::Quaternion::w)
        .def_readwrite("x", &spark2::Quaternion::x)
        .def_readwrite("y", &spark2::Quaternion::y)
        .def_readwrite("z", &spark2::Quaternion::z)
        .def("__repr__", [](const spark2::Quaternion &q) {
            return py::str("Quaternion(w={0}, x={1}, y={2}, z={3})").format(q.w, q.x, q.y, q.z);
        });

    py::class_<spark2::EulerAngle>(m, "EulerAngleInternal")
        .def(py::init([](float roll, float pitch, float yaw) {
            return spark2::EulerAngle{roll, pitch, yaw};
        }), py::arg("roll") = 0.0f, py::arg("pitch") = 0.0f, py::arg("yaw") = 0.0f)
        .def_readwrite("roll", &spark2::EulerAngle::roll)
        .def_readwrite("pitch", &spark2::EulerAngle::pitch)
        .def_readwrite("yaw", &spark2::EulerAngle::yaw)
        .def("__repr__", [](const spark2::EulerAngle &e) {
            return py::str("EulerAngle(roll={0}, pitch={1}, yaw={2})").format(e.roll, e.pitch, e.yaw);
        });

    py::class_<spark2::Position>(m, "PositionInternal")
        .def(py::init([](float x, float y, float z) {
            return spark2::Position{x, y, z};
        }), py::arg("x") = 0.0f, py::arg("y") = 0.0f, py::arg("z") = 0.0f)
        .def_readwrite("x", &spark2::Position::x)
        .def_readwrite("y", &spark2::Position::y)
        .def_readwrite("z", &spark2::Position::z)
        .def("__repr__", [](const spark2::Position &p) {
            return py::str("Position(x={0}, y={1}, z={2})").format(p.x, p.y, p.z);
        });

    py::class_<spark2::Pose>(m, "PoseInternal")
        .def(py::init([](const spark2::Position &position, const spark2::Quaternion &orientation) {
            return spark2::Pose{position, orientation};
        }),
            py::arg("position") = spark2::Position{},
            py::arg("orientation") = spark2::Quaternion{1.0f, 0.0f, 0.0f, 0.0f})
        .def_readwrite("position", &spark2::Pose::position)
        .def_readwrite("orientation", &spark2::Pose::orientation)
        .def("__repr__", [](const spark2::Pose &p) {
            return py::str("Pose(position={0}, orientation={1})").format(
                py::repr(py::cast(p.position)),
                py::repr(py::cast(p.orientation))
            );
        });
        
    using JointLimitsf = spark2::JointLimits<float>;
    py::class_<JointLimitsf>(m, "JointLimitsFloatInternal")
        .def(py::init([](float min, float max) {
            return JointLimitsf{min, max};
        }), py::arg("min") = 0.0f, py::arg("max") = 0.0f)
        .def_readwrite("min", &JointLimitsf::min)
        .def_readwrite("max", &JointLimitsf::max)
        .def("__repr__", [](const JointLimitsf &l) {
            return py::str("JointLimitsFloat(min={0}, max={1})").format(
                py::repr(py::cast(l.min)),
                py::repr(py::cast(l.max))
            );
        });

    py::class_<spark2::RobotJointStatef>(m, "RobotJointStateFloatInternal")
        .def(py::init([](const spark2::JointState6f &arm, const spark2::JointState1f &gripper) {
            return spark2::RobotJointStatef{arm, gripper};
        }),
            py::arg("arm") = spark2::JointState6f{},
            py::arg("gripper") = spark2::JointState1f{})
        .def_readwrite("arm", &spark2::RobotJointStatef::arm)
        .def_readwrite("gripper", &spark2::RobotJointStatef::gripper)
        .def("__repr__", [](const spark2::RobotJointStatef &s) {
            return py::str("RobotJointStateFloat(arm={0}, gripper={1})").format(
                py::repr(py::cast(s.arm)),
                py::repr(py::cast(s.gripper))
            );
        });

    py::class_<spark2::RobotJointStateb>(m, "RobotJointStateBoolInternal")
        .def(py::init([](const spark2::JointState6b &arm, const spark2::JointState1b &gripper) {
            return spark2::RobotJointStateb{arm, gripper};
        }),
            py::arg("arm") = spark2::JointState6b{},
            py::arg("gripper") = spark2::JointState1b{})
        .def_readwrite("arm", &spark2::RobotJointStateb::arm)
        .def_readwrite("gripper", &spark2::RobotJointStateb::gripper)
        .def("__repr__", [](const spark2::RobotJointStateb &s) {
            return py::str("RobotJointStateBool(arm={0}, gripper={1})").format(
                py::repr(py::cast(s.arm)),
                py::repr(py::cast(s.gripper))
            );
        });

    py::class_<spark2::SystemStatus>(m, "SystemStatusInternal")
        .def(py::init([](
            spark2::RobotState robot_state,
            spark2::PlanResult plan_result,
            uint32_t robot_diagnostic_flags,
            const spark2::JointState6u &arm_joint_diagnostic_flags,
            const spark2::JointState1u &gripper_joint_diagnostic_flags
        ) {
            return spark2::SystemStatus{
                robot_state,
                plan_result,
                robot_diagnostic_flags,
                arm_joint_diagnostic_flags,
                gripper_joint_diagnostic_flags,
            };
        }),
            py::arg("robot_state") = spark2::RobotState::kStartup,
            py::arg("plan_result") = spark2::PlanResult::kSuccess,
            py::arg("robot_diagnostic_flags") = 0u,
            py::arg("arm_joint_diagnostic_flags") = spark2::JointState6u{},
            py::arg("gripper_joint_diagnostic_flags") = spark2::JointState1u{})
        .def_readwrite("robot_state", &spark2::SystemStatus::robot_state)
        .def_readwrite("plan_result", &spark2::SystemStatus::plan_result)
        .def_readwrite("robot_diagnostic_flags", &spark2::SystemStatus::robot_diagnostic_flags)
        .def_readwrite("arm_joint_diagnostic_flags", &spark2::SystemStatus::arm_joint_diagnostic_flags)
        .def_readwrite("gripper_joint_diagnostic_flags", &spark2::SystemStatus::gripper_joint_diagnostic_flags)
        .def("__repr__", [](const spark2::SystemStatus &s) {
            return py::str("SystemStatus(robot_state={0}, plan_result={1}, robot_diagnostic_flags={2}, arm_joint_diagnostic_flags={3}, gripper_joint_diagnostic_flags={4})").format(
                py::repr(py::cast(s.robot_state)),
                py::repr(py::cast(s.plan_result)),
                s.robot_diagnostic_flags,
                py::repr(py::cast(s.arm_joint_diagnostic_flags)),
                py::repr(py::cast(s.gripper_joint_diagnostic_flags))
            );
        });

    py::class_<spark2::Configurator, std::shared_ptr<spark2::Configurator>>(m, "ConfiguratorInternal")
        // Tool Offset
        .def("set_tool_offset", &spark2::Configurator::setToolOffset)
        .def("get_tool_offset", &spark2::Configurator::getToolOffset)

        // Arm Safety Limits
        .def("set_arm_pos_limits", &spark2::Configurator::setArmPosLimits)
        .def("set_arm_vel_limits", &spark2::Configurator::setArmVelLimits)
        .def("get_arm_pos_limits", &spark2::Configurator::getArmPosLimits)
        .def("get_arm_vel_limits", &spark2::Configurator::getArmVelLimits)
        .def("get_arm_max_pos", &spark2::Configurator::getArmMaxPos)
        .def("get_arm_max_vel", &spark2::Configurator::getArmMaxVel)
        .def("get_arm_max_pos_jump", &spark2::Configurator::getArmMaxPosJump)
        .def("get_arm_max_vel_jump", &spark2::Configurator::getArmMaxVelJump)
        .def("get_arm_max_pos_follow_error", &spark2::Configurator::getArmMaxPosFollowError)
        .def("get_arm_max_vel_follow_error", &spark2::Configurator::getArmMaxVelFollowError)
        .def("get_arm_max_tor_follow_error", &spark2::Configurator::getArmMaxTorFollowError)

        // Gripper Safety Limits
        .def("set_gripper_pos_limits", &spark2::Configurator::setGripperPosLimits)
        .def("get_gripper_pos_limits", &spark2::Configurator::getGripperPosLimits)
        .def("get_gripper_max_pos", &spark2::Configurator::getGripperMaxPos);

    py::class_<spark2::Kinematics, std::shared_ptr<spark2::Kinematics>>(m, "KinematicsInternal")
        // No public constructor bound due to private C++ constructor.
        // We rely on Spark2 or a factory to return a reference/pointer.
        .def("forward_kinematics", &spark2::Kinematics::forwardKinematics)
        .def("inverse_kinematics", [](const std::shared_ptr<spark2::Kinematics>& self, 
            const spark2::Pose& ee_pose, 
            spark2::JointState6f arm_pos) {
        bool success = self->inverseKinematics(ee_pose, arm_pos);
        return py::make_tuple(success, arm_pos);
        })
        .def("euler_to_quaternion", &spark2::Kinematics::eulerToQuaternion)
        .def("quaternion_to_euler", &spark2::Kinematics::quaternionToEuler)
        .def("ee_to_tool_pose", &spark2::Kinematics::eeToToolPose)
        .def("tool_to_ee_pose", &spark2::Kinematics::toolToEEPose);

    py::class_<spark2::Spark2>(m, "Spark2Internal")
        .def(py::init<const std::string &>(), py::arg("config_prefix_path") = "")
        .def("start", &spark2::Spark2::start)
        .def("stop", &spark2::Spark2::stop)
        .def("enable_arm_joint", &spark2::Spark2::enableArmJoint)
        .def("set_arm_smoothing_method", &spark2::Spark2::setArmSmoothingMethod)
        .def("move_pos", &spark2::Spark2::movePos, py::arg("arm_pos"), py::arg("v") = 50, py::arg("t") = 0)
        .def("move_pos_path", &spark2::Spark2::movePosPath)
        .def("move_vel", &spark2::Spark2::moveVel, py::arg("arm_vel"), py::arg("a") = 10, py::arg("t") = 0)
        .def("move_ee_point", &spark2::Spark2::moveEEPoint, py::arg("arm_ee"), py::arg("v") = 50, py::arg("t") = 0)
        .def("move_ee_point_path", &spark2::Spark2::moveEEPointPath)
        .def("move_ee_line", &spark2::Spark2::moveEELine, py::arg("arm_ee"), py::arg("v") = 50, py::arg("t") = 0)
        .def("move_ee_line_path", &spark2::Spark2::moveEELinePath)
        .def("move_tool_point", &spark2::Spark2::moveToolPoint, py::arg("arm_tool"), py::arg("v") = 50, py::arg("t") = 0)
        .def("move_tool_point_path", &spark2::Spark2::moveToolPointPath)
        .def("move_tool_line", &spark2::Spark2::moveToolLine, py::arg("arm_tool"), py::arg("v") = 50, py::arg("t") = 0)
        .def("move_tool_line_path", &spark2::Spark2::moveToolLinePath)
        .def("go_home", &spark2::Spark2::goHome, py::arg("v") = 50, py::arg("t") = 0)
        .def("move_gripper_pos", &spark2::Spark2::moveGripperPos, py::arg("pos"), py::arg("v") = 50, py::arg("t") = 0)
        .def("start_teach", &spark2::Spark2::startTeach)
        .def("stop_teach", &spark2::Spark2::stopTeach)
        .def("start_playback", &spark2::Spark2::startPlayback)
        .def("stop_playback", &spark2::Spark2::stopPlayback)
        .def("reset_playback", &spark2::Spark2::resetPlayback)
        .def("load_playback", &spark2::Spark2::loadPlayback)
        .def("get_pos", &spark2::Spark2::getPos)
        .def("get_vel", &spark2::Spark2::getVel)
        .def("get_tor", &spark2::Spark2::getTor)
        .def("get_ee_pose", &spark2::Spark2::getEEPose)
        .def("get_tool_pose", &spark2::Spark2::getToolPose)
        .def("is_arm_joint_enabled", &spark2::Spark2::isArmJointEnabled)
        .def("get_status", &spark2::Spark2::getStatus)
        .def("print_status", &spark2::Spark2::printStatus)
        .def("get_configurator", [](py::object& self) {
            auto& cpp_spark = self.cast<spark2::Spark2&>();
            return std::shared_ptr<spark2::Configurator>(
                &cpp_spark.getConfigurator(), 
                [keep_alive = self](spark2::Configurator*) {}
            );
        })
        .def("get_kinematics", [](py::object& self) {
            auto& cpp_spark = self.cast<spark2::Spark2&>();
            return std::shared_ptr<spark2::Kinematics>(
                &cpp_spark.getKinematics(), 
                [keep_alive = self](spark2::Kinematics*) {}
            );
        });
}
