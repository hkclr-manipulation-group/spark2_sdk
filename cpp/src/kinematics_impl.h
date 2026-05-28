#ifndef KINEMATICS_IMPL_H
#define KINEMATICS_IMPL_H

#include "cuarm_robotics/src/robotics/robotics.h"

namespace spark{
    struct Kinematics::Impl{
        Robotics* robotics_;
        int arm_i_ = 0;

        void initialize(const std::string& config_prefix_path, const YAML::Node& config);
        void ensureRoboticsInitialized();
    };
}

#endif