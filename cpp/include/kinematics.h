#ifndef KINEMATICS_H
#define KINEMATICS_H

#include <memory>

#include "types.h"

namespace spark{
    class Kinematics{
        public:
            ~Kinematics();
            Pose forwardKinematics(const JointState6f& arm_pos) const;
            bool inverseKinematics(const Pose& ee_pose, JointState6f& arm_pos) const;

            Quaternion eulerToQuaternion(const EulerAngle& euler);
            EulerAngle quaternionToEuler(const Quaternion& quat);
            Pose eeToToolPose(const Pose& ee_pose, const Position& tool_offset);
            Pose toolToEEPose(const Pose& tool_pose, const Position& tool_offset);
        private:
            Kinematics();
            struct Impl;
            std::unique_ptr<Impl> pimpl_;
            friend class Spark;
    };
}
#endif 