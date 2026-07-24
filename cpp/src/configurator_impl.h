#ifndef CONFIGURATOR_IMPL_H
#define CONFIGURATOR_IMPL_H

#include <functional>

#include "robot_platform_utils/cpp/include/platform_state.h"

namespace spark2{
    struct Configurator::Impl{
        int arm_i_ = 0;
        int gripper_i_ = 0;
        int arm_joint_size_ = 6;
        int gripper_joint_size_ = 1;

        std::function<robot::platform::SdkConfigReq&()> get_config_req_callback_ = nullptr;
        std::function<const robot::platform::SdkConfigRes&()> get_config_res_callback_ = nullptr;
        std::function<void()> send_config_callback_ = nullptr;
        std::function<bool()> is_robot_started_callback_ = nullptr;

        void registerCallback(
            std::function<robot::platform::SdkConfigReq&()> getConfigReq,
            std::function<const robot::platform::SdkConfigRes&()> getConfigRes,
            std::function<void()> sendConfig,
            std::function<bool()> isRobotStarted);
        void ensureRobotStarted();
    };
}
#endif
