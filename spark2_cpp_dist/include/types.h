#ifndef TYPES_H
#define TYPES_H

#include <array>
#include <vector>
#include <numbers>
#include <cstdint>

namespace spark2{
    constexpr float kPi = std::numbers::pi_v<float>;
    constexpr float kDegToRad = kPi / 180.0f;
    constexpr float kRadToDeg = 180.0f / kPi;
    
    // Values are SDK-local; platform mapping is by name in toPlatformSmoothing().
    // kQuinticPath is required for multi-waypoint Path APIs.
    enum class SmoothingMethod{kLinear, kCos, kCubic, kQuintic, kNone, kQuinticPath};

    struct Quaternion{ float w, x, y, z; };
    struct EulerAngle{ float roll, pitch, yaw; }; //Intrinsic XYZ rotation: rotate first around x by roll, then around new y by pitch, then around new z by yaw
    struct Position{ float x, y, z; };
    struct Pose{ Position position; Quaternion orientation; };

    using JointState6f = std::array<float, 6>;
    using JointState1f = std::array<float, 1>;
    using JointState6b = std::array<bool, 6>;
    using JointState1b = std::array<bool, 1>;
    using JointState6u = std::array<uint64_t, 6>;
    using JointState1u = std::array<uint64_t, 1>;

    template <typename T> using Path = std::vector<T>;

    template<typename T> struct JointLimits{ T min; T max; };
    using JointLimits6f = std::array<JointLimits<float>, 6>;
    using JointLimits1f = std::array<JointLimits<float>, 1>;

    template <typename T1, typename T2> struct RobotJointState { T1 arm; T2 gripper; };
    using RobotJointStatef = RobotJointState<JointState6f, JointState1f>;
    using RobotJointStateb = RobotJointState<JointState6b, JointState1b>;

    // Operational state (SDK numbering; platform SystemState has kUnknown=0 then these shifted by +1)
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
        constexpr uint64_t kNone                         = 0; // No warning
        
        // --- Target Profile Saturation (Pre-Interpolation) ---
        constexpr uint64_t kTargetPosSaturation          = 1ULL << 0; // User target clamped by soft position limits
        constexpr uint64_t kTargetVelSaturation          = 1ULL << 1; // User target clamped by soft velocity limits
        constexpr uint64_t kTargetTorSaturation          = 1ULL << 2; // User target clamped by soft torque limits
        
        // --- Real-Time Command Saturation (Post-PID Loop) ---
        constexpr uint64_t kActuatorPosSaturation        = 1ULL << 3; // Actuator command clamped by soft position limits
        constexpr uint64_t kActuatorVelSaturation        = 1ULL << 4; // Actuator command clamped by soft velocity limits
        constexpr uint64_t kActuatorTorSaturation        = 1ULL << 5; // Actuator command clamped by soft torque limits
        constexpr uint64_t kActuatorPosJumpSaturation    = 1ULL << 6; // Actuator command clamped by position jump limit
        constexpr uint64_t kActuatorVelJumpSaturation    = 1ULL << 7; // Actuator command clamped by velocity jump limit
        constexpr uint64_t kActuatorTorJumpSaturation    = 1ULL << 8; // Actuator command clamped by torque jump limit
        
        // --- Soft Workspace Boundary Safety Interceptions ---
        constexpr uint64_t kBoundaryVelClamp             = 1ULL << 9;  // Velocity zeroed in limit direction due to position boundary reached
        constexpr uint64_t kBoundaryJointImpedance       = 1ULL << 10; // Joint impedance applied due to position boundary reached
        
        // --- Algorithmic & Kinematic Planner Modifications ---
        constexpr uint64_t kPlanTimelineExtended         = 1ULL << 11; // Trajectory segment duration (dt) stretched for velocity limits
        constexpr uint64_t kPlanVelLimitInvalid          = 1ULL << 12; // Specified max velocity profile is smaller than the minimum allowed velocity
        constexpr uint64_t kPlanDeltaTooLarge            = 1ULL << 13; // Distance between points is too large for a dynamic timeline
        constexpr uint64_t kPlanVelocitySnap             = 1ULL << 14; // Large velocity shift over zero distance
        constexpr uint64_t kPlanPointSkipped             = 1ULL << 15; // Duplicated points skipped
    
        // --- Fault Flags ---
        constexpr uint64_t kFaultPosHardLimitReached     = 1ULL << 16;
        constexpr uint64_t kFaultVelHardLimitReached     = 1ULL << 17;
        constexpr uint64_t kFaultTorHardLimitReached     = 1ULL << 18;
        constexpr uint64_t kFaultPosTrackingFailed       = 1ULL << 19;
        constexpr uint64_t kFaultVelTrackingFailed       = 1ULL << 20;
        constexpr uint64_t kFaultTorTrackingFailed       = 1ULL << 21;
        
        constexpr uint64_t kFaultArmNotFound             = 1ULL << 22;
        constexpr uint64_t kFaultGripperNotFound         = 1ULL << 23;
        constexpr uint64_t kFaultHardwareInitFailed      = 1ULL << 24;

        constexpr uint64_t kFaultArmSizeMismatch         = 1ULL << 25;
        constexpr uint64_t kFaultGripperSizeMismatch     = 1ULL << 26;
        constexpr uint64_t kFaultArmJointSizeMismatch    = 1ULL << 27;
        constexpr uint64_t kFaultGripperJointSizeMismatch = 1ULL << 28;
        constexpr uint64_t kFaultRobotNameMismatch       = 1ULL << 29;

        constexpr uint64_t kFaultCollisionDetected       = 1ULL << 30;
        constexpr uint64_t kFaultRecoveryRequired        = 1ULL << 31;
        constexpr uint64_t kFaultUnknown                 = 1ULL << 31; // legacy alias

        constexpr uint64_t kInvalidControlTypeCombination         = 1ULL << 32;
        constexpr uint64_t kCartesianControlRequirePositionTarget = 1ULL << 33;
        constexpr uint64_t kControlStrategyNotAvailable           = 1ULL << 34;
        constexpr uint64_t kWaypointControlStrategyNotAllowed     = 1ULL << 35;
        constexpr uint64_t kWaypointTargetTypeNotAllowed          = 1ULL << 36;
        constexpr uint64_t kWaypointSmoothingMethodNotAllowed     = 1ULL << 37;
        constexpr uint64_t kPlaybackControlRequirePositionTarget  = 1ULL << 38;
        constexpr uint64_t kPlaybackControlStartPoseNotReachable  = 1ULL << 39;
        constexpr uint64_t kUnknown                               = 1ULL << 63;
    };

    struct SystemStatus{
        RobotState   robot_state;
        PlanResult   plan_result;
        uint64_t     robot_diagnostic_flags;
        JointState6u arm_joint_diagnostic_flags;
        JointState1u gripper_joint_diagnostic_flags;
    };
}
#endif
