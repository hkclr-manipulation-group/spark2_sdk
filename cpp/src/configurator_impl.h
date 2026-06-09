#ifndef CONFIGURATOR_IMPL_H
#define CONFIGURATOR_IMPL_H

#include "robot_platform_utils/cpp/include/cuarm_state.h"

namespace spark2{
    struct Configurator::Impl{
        int arm_i_ = 0;
        int gripper_i_ = 0;
        int arm_joint_size_;
        int gripper_joint_size_;
        std::function<PanelCommand&()> get_panel_command_callback_ = nullptr;
        std::function<PlannerState&()> get_planner_state_callback_ = nullptr;
        std::function<void()> send_panel_command_callback_ = nullptr;
        std::function<bool()> is_robot_started_callback_ = nullptr;

        void registerCallback(std::function<PanelCommand&()> getPanelCommand, std::function<PlannerState&()> getPlannerState, 
            std::function<void()> sendPanelCommand, std::function<bool()> isRobotStarted);
        void ensureRobotStarted();
    };
}
#endif