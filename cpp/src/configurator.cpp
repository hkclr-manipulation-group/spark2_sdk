#include "configurator.h"
#include "configurator_impl.h"

#include <iostream>
#include "robot_platform_utils/cpp/include/cuarm_message_handler.h"
#include "robot_platform_utils/cpp/include/cuarm_state.h"

namespace spark2{
//--------------------------------- Configurator::Impl ---------------------------------
    void Configurator::Impl::registerCallback(
        std::function<PanelCommand&()> getPanelCommand, 
        std::function<PlannerState&()> getPlannerState, 
        std::function<void()> sendPanelCommand,
        std::function<bool()> isRobotStarted){
        get_panel_command_callback_ = getPanelCommand;
        get_planner_state_callback_ = getPlannerState;
        send_panel_command_callback_ = sendPanelCommand;
        is_robot_started_callback_ = isRobotStarted;
    }

    void Configurator::Impl::ensureRobotStarted(){
        if (!get_panel_command_callback_ || !get_planner_state_callback_ || !send_panel_command_callback_ || !is_robot_started_callback_){
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
        PanelCommand& panel_command = pimpl_->get_panel_command_callback_();
        panel_command.tool_offset[pimpl_->arm_i_][0] = offset.x;
        panel_command.tool_offset[pimpl_->arm_i_][1] = offset.y;
        panel_command.tool_offset[pimpl_->arm_i_][2] = offset.z;
        pimpl_->send_panel_command_callback_();
    }

    void Configurator::setArmPosLimits(const JointLimits6f& limit){
        pimpl_->ensureRobotStarted();
        PanelCommand& panel_command = pimpl_->get_panel_command_callback_();
        for (int j=0; j<pimpl_->arm_joint_size_; j++){
            panel_command.arm_soft_limit_position_deg[pimpl_->arm_i_][j][0] = limit[j].min;
            panel_command.arm_soft_limit_position_deg[pimpl_->arm_i_][j][1] = limit[j].max;
        }
        pimpl_->send_panel_command_callback_();
    }

    void Configurator::setArmVelLimits(const JointLimits6f& limit){
        pimpl_->ensureRobotStarted();
        PanelCommand& panel_command = pimpl_->get_panel_command_callback_();
        for (int j=0; j<pimpl_->arm_joint_size_; j++){
            panel_command.arm_soft_limit_velocity_deg[pimpl_->arm_i_][j][0] = limit[j].min;
            panel_command.arm_soft_limit_velocity_deg[pimpl_->arm_i_][j][1] = limit[j].max;
        }
        pimpl_->send_panel_command_callback_();
    }

    void Configurator::setGripperPosLimits(const JointLimits1f& limit){
        pimpl_->ensureRobotStarted();
        PanelCommand& panel_command = pimpl_->get_panel_command_callback_();
        for (int j=0; j<pimpl_->gripper_joint_size_; j++){
            panel_command.gripper_soft_limit_position_deg[pimpl_->gripper_i_][j][0] = limit[j].min;
            panel_command.gripper_soft_limit_position_deg[pimpl_->gripper_i_][j][1] = limit[j].max;
        }
        pimpl_->send_panel_command_callback_();
    }

    Position Configurator::getToolOffset() const{
        pimpl_->ensureRobotStarted();
        PlannerState& planner_state = pimpl_->get_planner_state_callback_();
        Position pos;
        pos.x = planner_state.tool_offset[pimpl_->arm_i_][0];
        pos.y = planner_state.tool_offset[pimpl_->arm_i_][1];
        pos.z = planner_state.tool_offset[pimpl_->arm_i_][2];
        return pos;
    }

    JointLimits6f Configurator::getArmPosLimits() const{
        pimpl_->ensureRobotStarted();
        PlannerState& planner_state = pimpl_->get_planner_state_callback_();
        JointLimits6f limits;
        for (int j=0; j<pimpl_->arm_joint_size_; j++){
            limits[j].min = planner_state.arm_soft_limit_position[pimpl_->arm_i_][j][0];
            limits[j].max = planner_state.arm_soft_limit_position[pimpl_->arm_i_][j][1];
        }
        return limits;
    }

    JointLimits6f Configurator::getArmVelLimits() const{
        pimpl_->ensureRobotStarted();
        PlannerState& planner_state = pimpl_->get_planner_state_callback_();
        JointLimits6f limits;
        for (int j=0; j<pimpl_->arm_joint_size_; j++){
            limits[j].min = planner_state.arm_soft_limit_velocity[pimpl_->arm_i_][j][0];
            limits[j].max = planner_state.arm_soft_limit_velocity[pimpl_->arm_i_][j][1];
        }
        return limits;
    }

    JointLimits6f Configurator::getArmMaxPos() const{
        pimpl_->ensureRobotStarted();
        PlannerState& planner_state = pimpl_->get_planner_state_callback_();
        JointLimits6f limits;
        for (int j=0; j<pimpl_->arm_joint_size_; j++){
            limits[j].min = planner_state.arm_hard_limit_position[pimpl_->arm_i_][j][0];
            limits[j].max = planner_state.arm_hard_limit_position[pimpl_->arm_i_][j][1];
        }
        return limits;
    }

    JointLimits6f Configurator::getArmMaxVel() const{
        pimpl_->ensureRobotStarted();
        PlannerState& planner_state = pimpl_->get_planner_state_callback_();
        JointLimits6f limits;
        for (int j=0; j<pimpl_->arm_joint_size_; j++){
            limits[j].min = planner_state.arm_hard_limit_velocity[pimpl_->arm_i_][j][0];
            limits[j].max = planner_state.arm_hard_limit_velocity[pimpl_->arm_i_][j][1];
        }
        return limits;
    }

    JointState6f Configurator::getArmMaxPosJump() const{
        pimpl_->ensureRobotStarted();
        PlannerState& planner_state = pimpl_->get_planner_state_callback_();
        JointState6f state;
        for (int j=0; j<pimpl_->arm_joint_size_; j++){
            state[j] = planner_state.arm_jump_limit_position[pimpl_->arm_i_][j];
        }
        return state;
    }

    JointState6f Configurator::getArmMaxVelJump() const{
        pimpl_->ensureRobotStarted();
        PlannerState& planner_state = pimpl_->get_planner_state_callback_();
        JointState6f state;
        for (int j=0; j<pimpl_->arm_joint_size_; j++){
            state[j] = planner_state.arm_jump_limit_velocity[pimpl_->arm_i_][j];
        }
        return state;
    }

    JointState6f Configurator::getArmMaxPosFollowError() const{
        pimpl_->ensureRobotStarted();
        PlannerState& planner_state = pimpl_->get_planner_state_callback_();
        JointState6f state;
        for (int j=0; j<pimpl_->arm_joint_size_; j++){
            state[j] = planner_state.arm_follow_limit_position[pimpl_->arm_i_][j];
        }
        return state;
    }

    JointState6f Configurator::getArmMaxVelFollowError() const{
        pimpl_->ensureRobotStarted();
        PlannerState& planner_state = pimpl_->get_planner_state_callback_();
        JointState6f state;
        for (int j=0; j<pimpl_->arm_joint_size_; j++){
            state[j] = planner_state.arm_follow_limit_velocity[pimpl_->arm_i_][j];
        }
        return state;
    }

    JointState6f Configurator::getArmMaxTorFollowError() const{
        pimpl_->ensureRobotStarted();
        PlannerState& planner_state = pimpl_->get_planner_state_callback_();
        JointState6f state;
        for (int j=0; j<pimpl_->arm_joint_size_; j++){
            state[j] = planner_state.arm_follow_limit_torque[pimpl_->arm_i_][j];
        }
        return state;
    }

    JointLimits1f Configurator::getGripperPosLimits() const{
        pimpl_->ensureRobotStarted();
        PlannerState& planner_state = pimpl_->get_planner_state_callback_();
        JointLimits1f limits;
        for (int j=0; j<pimpl_->gripper_joint_size_; j++){
            limits[j].min = planner_state.gripper_soft_limit_position[pimpl_->gripper_i_][j][0];
            limits[j].max = planner_state.gripper_soft_limit_position[pimpl_->gripper_i_][j][1];
        }
        return limits;
    }

    JointLimits1f Configurator::getGripperMaxPos() const{
        pimpl_->ensureRobotStarted();
        PlannerState& planner_state = pimpl_->get_planner_state_callback_();
        JointLimits1f limits;
        for (int j=0; j<pimpl_->gripper_joint_size_; j++){
            limits[j].min = planner_state.gripper_hard_limit_position[pimpl_->gripper_i_][j][0];
            limits[j].max = planner_state.gripper_hard_limit_position[pimpl_->gripper_i_][j][1];
        }
        return limits;
    }
}