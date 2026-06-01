#ifndef TYPES_H
#define TYPES_H

#include <array>
#include <vector>
#include <numbers>
#include <cstdint>

namespace spark{
    constexpr float kPi = std::numbers::pi_v<float>;
    constexpr float kDegToRad = kPi / 180.0f;
    constexpr float kRadToDeg = 180.0f / kPi;
    
    enum class SmoothingMethod{kLinear, kCos, kCubic, kQuintic, kNone};

    struct Quaternion{ float w, x, y, z; };
    struct EulerAngle{ float roll, pitch, yaw; }; //Intrinsic XYZ rotation: rotate first around x by roll, then around new y by pitch, then around new z by yaw
    struct Position{ float x, y, z; };
    struct Pose{ Position position; Quaternion orientation; };

    using JointState6f = std::array<float, 6>;
    using JointState1f = std::array<float, 1>;
    using JointState6b = std::array<bool, 6>;
    using JointState1b = std::array<bool, 1>;
    using JointState6u = std::array<uint32_t, 6>;
    using JointState1u = std::array<uint32_t, 1>;

    template <typename T> using Path = std::vector<T>;

    template<typename T> struct JointLimits{ T min; T max; };
    using JointLimits6f = std::array<JointLimits<float>, 6>;
    using JointLimits1f = std::array<JointLimits<float>, 1>;

    template <typename T1, typename T2> struct RobotJointState { T1 arm; T2 gripper; };
    using RobotJointStatef = RobotJointState<JointState6f, JointState1f>;
    using RobotJointStateb = RobotJointState<JointState6b, JointState1b>;

    //operational state
    enum class RobotState{
        kStartup    = 0,
        kIdle       = 1, // Completely stationary; safe to accept new paths
        kMoving     = 2, // Actively running an interpolator, velocity command, or jog stream
        kSettling   = 3, // After planning finished / Changing Control Mode / Stopping
        kError      = 4, // Safeguard stop triggered; hardware limits breached or E-stop
        kRecovery   = 5, // Resetting safety loops and clearing faults
        kShutdown   = 6, // Disabling amplifiers and powering down safely
    };

    enum class PlanResult {
        kSuccess                    = 0,  // IK passed, time allocation valid, safe to execute
        kPoseNotReachable           = 1,  // Target 6D pose is physically outside the workspace
        kLinearPathFailed           = 2,  // Reachable target, but continuous linear path is blocked (Singularity / Joint Limit)
    };

    namespace DiagnosticFlags {    
        constexpr uint32_t kNone                         = 0; // No warning
        
        // --- Target Profile Saturation (Pre-Interpolation) ---
        constexpr uint32_t kTargetPosSaturation          = 1 << 0; // User target clamped by soft position limits
        constexpr uint32_t kTargetVelSaturation          = 1 << 1; // User target clamped by soft velocity limits
        constexpr uint32_t kTargetTorSaturation          = 1 << 2; // User target clamped by soft torque limits
        
        // --- Real-Time Command Saturation (Post-PID Loop) ---
        constexpr uint32_t kActuatorPosSaturation        = 1 << 3; // Actuator command clamped by soft position limits
        constexpr uint32_t kActuatorVelSaturation        = 1 << 4; // Actuator command clamped by soft velocity limits
        constexpr uint32_t kActuatorTorSaturation        = 1 << 5; // Actuator command clamped by soft torque limits
        constexpr uint32_t kActuatorPosJumpSaturation    = 1 << 6; // Actuator command clamped by position jump limit
        constexpr uint32_t kActuatorVelJumpSaturation    = 1 << 7; // Actuator command clamped by velocity jump limit
        constexpr uint32_t kActuatorTorJumpSaturation    = 1 << 8; // Actuator command clamped by torque jump limit
        
        // --- Soft Workspace Boundary Safety Interceptions ---
        constexpr uint32_t kBoundaryVelClamp             = 1 << 9;  // Velocity zeroed in limit direction due to position boundary reached
        constexpr uint32_t kBoundaryJointImpedance       = 1 << 10; // Joint impedance applied due to position boundary reached
        
        // --- Algorithmic & Kinematic Planner Modifications ---
        constexpr uint32_t kPlanTimelineExtended         = 1 << 11; // Trajectory segment duration (dt) stretched for velocity limits
        constexpr uint32_t kPlanVelLimitInvalid          = 1 << 12; // Specified max velocity profile is smaller than the minimum allowed velocity
        constexpr uint32_t kPlanDeltaTooLarge            = 1 << 13; // Distance between points is too large for a dynamic timeline
        constexpr uint32_t kPlanVelocitySnap             = 1 << 14; // Large velocity shift over zero distance
        constexpr uint32_t kPlanPointSkipped             = 1 << 15; // Duplicated points skipped
    
        // --- Fault Flags ---
        constexpr uint32_t kFaultPosHardLimitReached     = 1 << 16;
        constexpr uint32_t kFaultVelHardLimitReached     = 1 << 17;
        constexpr uint32_t kFaultTorHardLimitReached     = 1 << 18;
        constexpr uint32_t kFaultPosTrackingFailed       = 1 << 19;
        constexpr uint32_t kFaultVelTrackingFailed       = 1 << 20;
        constexpr uint32_t kFaultTorTrackingFailed       = 1 << 21;
        
        constexpr uint32_t kFaultArmNotFound             = 1 << 22;
        constexpr uint32_t kFaultGripperNotFound         = 1 << 23;
        constexpr uint32_t kFaultHardwareInitFailed      = 1 << 24;
    
        constexpr uint32_t kFaultUnknown                 = 1 << 31;
    };

    struct SystemStatus{
        RobotState   robot_state;
        PlanResult   plan_result;
        uint32_t     robot_diagnostic_flags;
        JointState6u arm_joint_diagnostic_flags;
        JointState1u gripper_joint_diagnostic_flags;
    };
}
#endif