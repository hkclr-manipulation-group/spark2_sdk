#include "configurator.h"
#include "configurator_impl.h"

#include <cmath>
#include <stdexcept>

#include "types.h"

namespace spark2{
//--------------------------------- Configurator::Impl ---------------------------------
    void Configurator::Impl::registerCallback(
        std::function<robot::platform::SdkConfigReq&()> getConfigReq,
        std::function<const robot::platform::SdkConfigRes&()> getConfigRes,
        std::function<void()> sendConfig,
        std::function<bool()> isRobotStarted){
        get_config_req_callback_ = std::move(getConfigReq);
        get_config_res_callback_ = std::move(getConfigRes);
        send_config_callback_ = std::move(sendConfig);
        is_robot_started_callback_ = std::move(isRobotStarted);
    }

    void Configurator::Impl::ensureRobotStarted(){
        if (!get_config_req_callback_ || !get_config_res_callback_ || !send_config_callback_ || !is_robot_started_callback_){
            throw std::runtime_error("Configurator::ensureRobotStarted failed: registerCallback hasn't been called");
        }

        if (!is_robot_started_callback_()){
            throw std::runtime_error("Configurator::ensureRobotStarted failed: robot hasn't been started");
        }
    }

//--------------------------------- Configurator ---------------------------------------
    Configurator::Configurator() : pimpl_(std::make_unique<Impl>()){}
    Configurator::~Configurator() = default;

    void Configurator::setToolOffset(const Position& offset){
        pimpl_->ensureRobotStarted();
        auto& config_req = pimpl_->get_config_req_callback_();
        config_req.payload.arm[pimpl_->arm_i_].tool_offset = robot::platform::PositionTarget{offset.x, offset.y, offset.z};
        config_req.payload.read_only = 0;
        pimpl_->send_config_callback_();
    }

    void Configurator::setArmPosLimits(const JointLimits6f& limit){
        pimpl_->ensureRobotStarted();
        auto& config_req = pimpl_->get_config_req_callback_();
        auto& arm_cfg = config_req.payload.arm[pimpl_->arm_i_];
        for (int j = 0; j < pimpl_->arm_joint_size_; j++){
            arm_cfg.soft_limit_position[j][0] = limit[j].min * kDegToRad;
            arm_cfg.soft_limit_position[j][1] = limit[j].max * kDegToRad;
        }
        config_req.payload.read_only = 0;
        pimpl_->send_config_callback_();
    }

    void Configurator::setArmVelLimits(const JointLimits6f& limit){
        pimpl_->ensureRobotStarted();
        auto& config_req = pimpl_->get_config_req_callback_();
        auto& arm_cfg = config_req.payload.arm[pimpl_->arm_i_];
        for (int j = 0; j < pimpl_->arm_joint_size_; j++){
            const float abs_min = std::abs(limit[j].min);
            const float abs_max = std::abs(limit[j].max);
            arm_cfg.soft_limit_velocity[j] = std::max(abs_min, abs_max) * kDegToRad;
        }
        config_req.payload.read_only = 0;
        pimpl_->send_config_callback_();
    }

    void Configurator::setGripperPosLimits(const JointLimits1f& limit){
        pimpl_->ensureRobotStarted();
        auto& config_req = pimpl_->get_config_req_callback_();
        for (int j = 0; j < pimpl_->gripper_joint_size_; j++){
            config_req.payload.gripper[pimpl_->gripper_i_].soft_limit_position[j][0] = limit[j].min * kDegToRad;
            config_req.payload.gripper[pimpl_->gripper_i_].soft_limit_position[j][1] = limit[j].max * kDegToRad;
        }
        config_req.payload.read_only = 0;
        pimpl_->send_config_callback_();
    }

    Position Configurator::getToolOffset() const{
        pimpl_->ensureRobotStarted();
        const auto& config_res = pimpl_->get_config_res_callback_();
        const auto& offset = config_res.payload.arm[pimpl_->arm_i_].tool_offset;
        return Position{offset.x, offset.y, offset.z};
    }

    JointLimits6f Configurator::getArmPosLimits() const{
        pimpl_->ensureRobotStarted();
        const auto& config_res = pimpl_->get_config_res_callback_();
        JointLimits6f limits{};
        for (int j = 0; j < pimpl_->arm_joint_size_; j++){
            limits[j].min = config_res.payload.arm[pimpl_->arm_i_].soft_limit_position[j][0] * kRadToDeg;
            limits[j].max = config_res.payload.arm[pimpl_->arm_i_].soft_limit_position[j][1] * kRadToDeg;
        }
        return limits;
    }

    JointLimits6f Configurator::getArmVelLimits() const{
        pimpl_->ensureRobotStarted();
        const auto& config_res = pimpl_->get_config_res_callback_();
        JointLimits6f limits{};
        for (int j = 0; j < pimpl_->arm_joint_size_; j++){
            const float v = config_res.payload.arm[pimpl_->arm_i_].soft_limit_velocity[j] * kRadToDeg;
            limits[j].min = -v;
            limits[j].max = v;
        }
        return limits;
    }

    JointLimits6f Configurator::getArmMaxPos() const{
        pimpl_->ensureRobotStarted();
        const auto& config_res = pimpl_->get_config_res_callback_();
        JointLimits6f limits{};
        for (int j = 0; j < pimpl_->arm_joint_size_; j++){
            limits[j].min = config_res.payload.arm[pimpl_->arm_i_].hard_limit_position[j][0] * kRadToDeg;
            limits[j].max = config_res.payload.arm[pimpl_->arm_i_].hard_limit_position[j][1] * kRadToDeg;
        }
        return limits;
    }

    JointLimits6f Configurator::getArmMaxVel() const{
        pimpl_->ensureRobotStarted();
        const auto& config_res = pimpl_->get_config_res_callback_();
        JointLimits6f limits{};
        for (int j = 0; j < pimpl_->arm_joint_size_; j++){
            const float v = config_res.payload.arm[pimpl_->arm_i_].hard_limit_velocity[j] * kRadToDeg;
            limits[j].min = -v;
            limits[j].max = v;
        }
        return limits;
    }

    JointState6f Configurator::getArmMaxPosJump() const{
        pimpl_->ensureRobotStarted();
        const auto& config_res = pimpl_->get_config_res_callback_();
        JointState6f state{};
        for (int j = 0; j < pimpl_->arm_joint_size_; j++){
            state[j] = config_res.payload.arm[pimpl_->arm_i_].jump_limit_position[j] * kRadToDeg;
        }
        return state;
    }

    JointState6f Configurator::getArmMaxVelJump() const{
        pimpl_->ensureRobotStarted();
        const auto& config_res = pimpl_->get_config_res_callback_();
        JointState6f state{};
        for (int j = 0; j < pimpl_->arm_joint_size_; j++){
            state[j] = config_res.payload.arm[pimpl_->arm_i_].jump_limit_velocity[j] * kRadToDeg;
        }
        return state;
    }

    JointState6f Configurator::getArmMaxPosFollowError() const{
        pimpl_->ensureRobotStarted();
        const auto& config_res = pimpl_->get_config_res_callback_();
        JointState6f state{};
        for (int j = 0; j < pimpl_->arm_joint_size_; j++){
            state[j] = config_res.payload.arm[pimpl_->arm_i_].follow_limit_position[j] * kRadToDeg;
        }
        return state;
    }

    JointState6f Configurator::getArmMaxVelFollowError() const{
        pimpl_->ensureRobotStarted();
        const auto& config_res = pimpl_->get_config_res_callback_();
        JointState6f state{};
        for (int j = 0; j < pimpl_->arm_joint_size_; j++){
            state[j] = config_res.payload.arm[pimpl_->arm_i_].follow_limit_velocity[j] * kRadToDeg;
        }
        return state;
    }

    JointState6f Configurator::getArmMaxTorFollowError() const{
        pimpl_->ensureRobotStarted();
        const auto& config_res = pimpl_->get_config_res_callback_();
        JointState6f state{};
        for (int j = 0; j < pimpl_->arm_joint_size_; j++){
            state[j] = config_res.payload.arm[pimpl_->arm_i_].follow_limit_torque[j];
        }
        return state;
    }

    JointLimits1f Configurator::getGripperPosLimits() const{
        pimpl_->ensureRobotStarted();
        const auto& config_res = pimpl_->get_config_res_callback_();
        JointLimits1f limits{};
        for (int j = 0; j < pimpl_->gripper_joint_size_; j++){
            limits[j].min = config_res.payload.gripper[pimpl_->gripper_i_].soft_limit_position[j][0] * kRadToDeg;
            limits[j].max = config_res.payload.gripper[pimpl_->gripper_i_].soft_limit_position[j][1] * kRadToDeg;
        }
        return limits;
    }

    JointLimits1f Configurator::getGripperMaxPos() const{
        pimpl_->ensureRobotStarted();
        const auto& config_res = pimpl_->get_config_res_callback_();
        JointLimits1f limits{};
        for (int j = 0; j < pimpl_->gripper_joint_size_; j++){
            limits[j].min = config_res.payload.gripper[pimpl_->gripper_i_].hard_limit_position[j][0] * kRadToDeg;
            limits[j].max = config_res.payload.gripper[pimpl_->gripper_i_].hard_limit_position[j][1] * kRadToDeg;
        }
        return limits;
    }
}
