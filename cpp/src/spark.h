#ifndef LITEARM_H
#define LITEARM_H

#include <vector>
#include <string>

#include "types.h"
#include "kinematics.h"
#include "configurator.h"

namespace spark{
    class Spark{
        public:
            Spark(std::string config_prefix_path = "");
            ~Spark();

            // Connection
            void start();
            void stop();
            void enableArmJoint(JointState6b arm_state);

            // Motion Mode
            void setArmSmoothingMethod(SmoothingMethod method); //Smoothing method for single target movement

            // Manual Mode
            void movePos(const JointState6f& arm_pos, int v=50, float t=0);
            void movePosPath(const Path<JointState6f>& arm_pos, Path<int> v, Path<float> t);

            void moveVel(const JointState6f& arm_vel, int a=10, float t=0);

            void moveEEPoint(const Pose& arm_ee, int v=50, float t=0);
            void moveEEPointPath(const Path<Pose>& arm_ee, Path<int> v, Path<float> t);
            void moveEELine(const Pose& arm_ee, int v=50, float t=0);
            void moveEELinePath(const Path<Pose>& arm_ee, Path<int> v, Path<float> t);

            void moveToolPoint(const Pose& arm_tool, int v=50, float t=0);
            void moveToolPointPath(const Path<Pose>& arm_tool, Path<int> v, Path<float> t);
            void moveToolLine(const Pose& arm_tool, int v=50, float t=0);
            void moveToolLinePath(const Path<Pose>& arm_tool, Path<int> v, Path<float> t);
            
            void goHome(int v=50, float t=0);
            void moveGripperPos(const JointState1f& pos, int v=50, float t=0);

            // Teach Mode
            void startTeach();
            void stopTeach();

            // Playback Mode
            void startPlayback();
            void stopPlayback();
            void resetPlayback();
            void loadPlayback(const std::string& filename);

            // Feedback
            RobotJointStatef getPos() const;
            RobotJointStatef getVel() const;
            RobotJointStatef getTor() const;
            Pose getEEPose() const;
            Pose getToolPose() const;
            bool isLatestTargetReceived() const;

            // Status
            JointState6b isArmJointEnabled() const;
            SystemStatus getStatus() const;
            void printStatus() const;

            Configurator& getConfigurator();
            Kinematics& getKinematics();

        private:
            struct Impl;
            std::unique_ptr<Impl> pimpl_;
    };
}
#endif