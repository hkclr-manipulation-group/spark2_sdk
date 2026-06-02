#include "kinematics.h"
#include "kinematics_impl.h"

#include <iostream>
#include <filesystem>

namespace Eigen {
    using Vector6f = Matrix<float, 6, 1>;
    using Vector6d = Matrix<double, 6, 1>;
    using Vector7d = Matrix<double, 7, 1>;
}

namespace spark2{
    Eigen::Map<const Eigen::Vector3f> toEigen(const Position& p) {
        return Eigen::Map<const Eigen::Vector3f>(&p.x);
    }
    
    Eigen::Quaternionf toEigen(const Quaternion& q) {
        return Eigen::Quaternionf(q.w, q.x, q.y, q.z);
    }
    
    Eigen::Map<const Eigen::Vector6f> toEigen(const JointState6f& p) {
        return Eigen::Map<const Eigen::Vector6f>(p.data());
    }

//--------------------------------- Kinematics::Impl ---------------------------------
    void Kinematics::Impl::initialize(const std::string& config_prefix_path, const YAML::Node& config){
        YAML::Node arm_config = config["robot"]["arm"][arm_i_];
        int joint_size = arm_config["joint_size"].as<int>();
        Eigen::Matrix<float, 2, 6> joint_pos_limits;
        for (int i =0; i <joint_size; i++){
            joint_pos_limits(0,i) = arm_config["safety"]["joint_soft_limit"]["position"][0][i].as<float>() * kDegToRad;
            joint_pos_limits(1,i) = arm_config["safety"]["joint_soft_limit"]["position"][1][i].as<float>() * kDegToRad;
        }

        std::string robot_type = arm_config["type"].as<std::string>();
        std::vector<std::string> joint_names = arm_config["urdf_joints"].as<std::vector<std::string>>();
        std::vector<std::string> link_names  = arm_config["urdf_links"].as<std::vector<std::string>>();
        Vector3d tool_offset = Vector3d::Zero();
        if (config["robot"]["gripper"] && config["robot"]["gripper"].size() > arm_i_){
            YAML::Node offset_node = config["gripper"][arm_i_]["offset_from_ee"];
            tool_offset << offset_node[0].as<float>(), offset_node[1].as<float>(), offset_node[2].as<float>();
        }
        std::string urdf_filename = config["robot"]["urdf"].as<std::string>();
        std::string urdf_path = std::filesystem::canonical(std::filesystem::path(config_prefix_path + '/' + urdf_filename));
        std::string library_name = config["rt_control"]["robotics_library"].as<std::string>();
        std::string ee_link = link_names[link_names.size()-1];
        robotics_ = new Robotics(urdf_path, joint_names, ee_link, tool_offset, arm_i_, joint_size, 
            joint_pos_limits, library_name);
        VectorXf gear_ratio(joint_size);
        robotics_->setArm(robot_type, gear_ratio);
    }

    void Kinematics::Impl::ensureRoboticsInitialized(){
        if (!robotics_){
            throw std::runtime_error("Kinematics::ensureRoboticsInitialized failed: robotics is nullptr");
        }
    }

//--------------------------------- Kinematics ---------------------------------------
    Kinematics::Kinematics():pimpl_(std::make_unique<Impl>()){

    }

    Kinematics::~Kinematics() {
        if (pimpl_ && pimpl_->robotics_) {
            delete pimpl_->robotics_;
        }
    }

    Pose Kinematics::forwardKinematics(const JointState6f& arm_pos) const{
        pimpl_->ensureRoboticsInitialized();
        Eigen::Vector6f q = toEigen(arm_pos)*kDegToRad;
        Eigen::Vector7d ee_pose;
        pimpl_->robotics_->FK(q.cast<double>(), &ee_pose);
        
        Pose ee_pose_out;
        ee_pose_out.position = Position{
            static_cast<float>(ee_pose(0)), 
            static_cast<float>(ee_pose(1)), 
            static_cast<float>(ee_pose(2))
        };
        ee_pose_out.orientation = Quaternion{
            static_cast<float>(ee_pose(3)),
            static_cast<float>(ee_pose(4)),
            static_cast<float>(ee_pose(5)),
            static_cast<float>(ee_pose(6))
        };
        return ee_pose_out;
    }

    bool Kinematics::inverseKinematics(const Pose& ee_pose, JointState6f& arm_pos) const{
        pimpl_->ensureRoboticsInitialized();
        Eigen::Vector3f p = toEigen(ee_pose.position);
        Eigen::Quaternionf quat = toEigen(ee_pose.orientation);
        Eigen::Matrix3f R = quat.toRotationMatrix();
        Eigen::VectorXd q = toEigen(arm_pos).cast<double>() * kDegToRad;

        // Fixed: Cast R and p to double explicitly to match IK signature requirements
        bool success = pimpl_->robotics_->IK(R.cast<double>(), p.cast<double>(), q);
        if (!success){
            return false;
        }
        arm_pos = JointState6f{
            static_cast<float>(q(0) * kRadToDeg), 
            static_cast<float>(q(1) * kRadToDeg), 
            static_cast<float>(q(2) * kRadToDeg), 
            static_cast<float>(q(3) * kRadToDeg), 
            static_cast<float>(q(4) * kRadToDeg), 
            static_cast<float>(q(5) * kRadToDeg)
        };
        return true;
    }

    Quaternion Kinematics::eulerToQuaternion(const EulerAngle& euler) {
        pimpl_->ensureRoboticsInitialized();
        Eigen::Vector3d euler_vec = Eigen::Vector3d{euler.roll, euler.pitch, euler.yaw};
        Eigen::Quaterniond q = Robotics::toQuat(euler_vec);
        return Quaternion{static_cast<float>(q.w()), static_cast<float>(q.x()), static_cast<float>(q.y()), static_cast<float>(q.z())};
    }

    EulerAngle Kinematics::quaternionToEuler(const Quaternion& quat) {
        pimpl_->ensureRoboticsInitialized();
        Eigen::Quaternionf q = toEigen(quat);
        Eigen::Vector3d euler_vec = Robotics::toEulerAngle(q.cast<double>());
        return EulerAngle{static_cast<float>(euler_vec(0)), static_cast<float>(euler_vec(1)), static_cast<float>(euler_vec(2))};
    }
    
    Pose Kinematics::eeToToolPose(const Pose& ee_pose, const Position& tool_offset) {
        pimpl_->ensureRoboticsInitialized();
        Eigen::Vector7d ee_pose_vec;
        ee_pose_vec << ee_pose.position.x, ee_pose.position.y, ee_pose.position.z, 
                       ee_pose.orientation.w, ee_pose.orientation.x, ee_pose.orientation.y, ee_pose.orientation.z;
    
        Eigen::Vector3d offset = toEigen(tool_offset).cast<double>();
        Eigen::Vector7d tool_pose_vec = Robotics::eeToTool(ee_pose_vec, offset);
        
        return Pose{
            {static_cast<float>(tool_pose_vec(0)), static_cast<float>(tool_pose_vec(1)), static_cast<float>(tool_pose_vec(2))}, 
            {static_cast<float>(tool_pose_vec(3)), static_cast<float>(tool_pose_vec(4)), static_cast<float>(tool_pose_vec(5)), static_cast<float>(tool_pose_vec(6))}
        };
    }
    
    Pose Kinematics::toolToEEPose(const Pose& tool_pose, const Position& tool_offset) {
        pimpl_->ensureRoboticsInitialized();
        Eigen::Vector7d tool_pose_vec;
        tool_pose_vec << tool_pose.position.x, tool_pose.position.y, tool_pose.position.z, 
                         tool_pose.orientation.w, tool_pose.orientation.x, tool_pose.orientation.y, tool_pose.orientation.z;
    
        Eigen::Vector3d offset = toEigen(tool_offset).cast<double>();
        Eigen::Vector7d ee_pose_vec = Robotics::toolToEE(tool_pose_vec, offset);
        
        return Pose{
            {static_cast<float>(ee_pose_vec(0)), static_cast<float>(ee_pose_vec(1)), static_cast<float>(ee_pose_vec(2))}, 
            {static_cast<float>(ee_pose_vec(3)), static_cast<float>(ee_pose_vec(4)), static_cast<float>(ee_pose_vec(5)), static_cast<float>(ee_pose_vec(6))}
        };
    }
}
