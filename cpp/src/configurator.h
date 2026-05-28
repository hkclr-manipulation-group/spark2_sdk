#ifndef CONFIGURATOR_H
#define CONFIGURATOR_H

#include <memory>
#include <string>

#include "types.h"

namespace spark{
    class Configurator{
        public:
            ~Configurator();

            // Tool Offset
            void setToolOffset(const Position& offset);
            Position getToolOffset() const;

            // Arm Safety Limits
            void setArmPosLimits(const JointLimits6f& limit);
            void setArmVelLimits(const JointLimits6f& limit);
            JointLimits6f getArmPosLimits() const;
            JointLimits6f getArmVelLimits() const;

            JointLimits6f getArmMaxPos() const;
            JointLimits6f getArmMaxVel() const;

            JointState6f getArmMaxPosJump() const;
            JointState6f getArmMaxVelJump() const;

            JointState6f getArmMaxPosFollowError() const;
            JointState6f getArmMaxVelFollowError() const;
            JointState6f getArmMaxTorFollowError() const;

            //Gripper Safety Limits
            void setGripperPosLimits(const JointLimits1f& limit);
            JointLimits1f getGripperPosLimits() const;
            JointLimits1f getGripperMaxPos() const;
            
        private:
            Configurator();
            struct Impl;
            std::unique_ptr<Impl> pimpl_;
            friend class Spark;
    };
}

#endif