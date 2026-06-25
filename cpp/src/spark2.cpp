#include "spark2.h"

#include <algorithm> // Required for std::clamp
#include <filesystem>
#include <memory>
#include <chrono>
#include <thread>
#include <mutex>
#include <functional>

#include "robot_platform_utils/cpp/include/config_loader.h"
#include "robot_platform_utils/cpp/include/cuarm_state.h"
#include "robot_platform_utils/cpp/include/cuarm_udp.h"
#include "robot_platform_utils/cpp/include/time_utils.h"
#include "configurator_impl.h"
#include "kinematics_impl.h"

namespace spark2{
    
    enum class MotionMode{
        kInitialized = 0,
        kManual = 1,
        kTeaching = 2,
        kPlayback = 3,
    };

    struct Spark2::Impl{
        std::unique_ptr<Configurator> configurator_;
        std::unique_ptr<Kinematics> kinematics_;

        int arm_size_;
        std::vector<int> arm_joint_size_;
        int gripper_size_;
        std::vector<int> gripper_joint_size_;
        std::unique_ptr<PanelCommand> panel_command_;
        std::unique_ptr<PlannerState> planner_state_;
        std::unique_ptr<CuarmUdp<PlannerState, PanelCommand>> udp_;
        std::atomic<bool> shutdown_{false};
        std::atomic<bool> first_state_received_{false};
        std::thread receive_thread_;
        YAML::Node config_;
        bool started_ = false; // True if the arm has been started

        float receive_dt_us_;
        float send_dt_us_;
        ControlType initial_target_type_;
        ControlType initial_actuator_mode_;
        ControlType last_target_type_;
        ControlType last_actuator_mode_;
        ControlType actuator_mode_under_position_control_;
        MotionControl last_motion_control_;
        std::vector<RobotJointStatef> playback_;
        std::mutex panel_command_mutex_;


        void initialize(const YAML::Node& config){
            //Initialize class variables
            panel_command_ = std::make_unique<PanelCommand>();
            planner_state_ = std::make_unique<PlannerState>();

            arm_size_ = config["robot"]["arm"].size();
            for (int arm_i=0; arm_i<arm_size_; arm_i++){
                arm_joint_size_.push_back(config["robot"]["arm"][arm_i]["joint_size"].as<int>());
            }
            
            if (config["robot"]["gripper"]){
                gripper_size_ = config["robot"]["gripper"].size();
                for (int gri_i=0; gri_i<gripper_size_; gri_i++){
                    gripper_joint_size_.push_back(config["robot"]["gripper"][gri_i]["joint_size"].as<int>());
                }
            }
            receive_dt_us_ = config["panel"]["dt_receive"].as<float>() * 1000000; // Convert to microseconds
            send_dt_us_ = config["panel"]["dt_send"].as<float>() * 1000000; // Convert to microseconds
            config_ = config;

            //Initialize panel_command_
            panel_command_->sequence_id = 0;
            panel_command_->need_setting_update = true;
            panel_command_->simulation = config["rt_control"]["hardware_simulation"].as<bool>();
            panel_command_->motion_type = MotionControl::kJoint;
            panel_command_->connection_state = ConnectionState::kRemote;
            panel_command_->reset_control_mem = false;
            panel_command_->reset_interpolation = false;
            panel_command_->control_algorithm = ControlAlgorithm::kNone;
            panel_command_->playback_cmd = PlaybackState::kStop;
            
            panel_command_->interpolation_type = stringToEnum<InterpolationMethod>(config["panel"]["general"]["interpolation"].as<std::string>());
            panel_command_->InterpolationAccTime = config["panel"]["general"]["acc_time"].as<float>();
            panel_command_->InterpolationConstVelTime = config["panel"]["general"]["vel_time"].as<float>();
            panel_command_->NoneInterpolationSaturationRatio = config["panel"]["general"]["none_interpolation_saturation_adjust"].as<float>();

            initial_target_type_ = stringToEnum<ControlType>(config["panel"]["general"]["target_type"].as<std::string>());
            initial_actuator_mode_ = stringToEnum<ControlType>(config["panel"]["general"]["actuator_mode"].as<std::string>());
            actuator_mode_under_position_control_ = (initial_target_type_ == ControlType::kPosition)? initial_actuator_mode_ : ControlType::kPosition;
            
            panel_command_->target_type = ControlType::kVelocity;
            panel_command_->actuator_mode = ControlType::kVelocity;
            last_motion_control_ = panel_command_->motion_type;
            last_target_type_ = panel_command_->target_type;
            last_actuator_mode_ = panel_command_->actuator_mode;

            panel_command_->arm_target_mode = PanelTargetMode::kSinglePoint;
            panel_command_->ArmWaypointSize = 0;
            panel_command_->ArmSize = arm_size_;
            for (int arm_i=0; arm_i<arm_size_; arm_i++){
                YAML::Node arm_node = config["robot"]["arm"][arm_i];
                std::string arm_name = arm_node["name"].as<std::string>();
                std::string arm_type = arm_node["type"].as<std::string>();
                std::string robot_name = arm_name + "-" + arm_type;
                strcpy(panel_command_->RobotName[arm_i], robot_name.c_str());
                panel_command_->JointSize[arm_i] = arm_joint_size_[arm_i];

                for (int i =0; i < arm_joint_size_[arm_i]; i++){
                    panel_command_->MotorCmdDeg[arm_i][i] = 0;
                    panel_command_->JointCmdDeg[arm_i][i] = 0;
                    if (i < 6)
                        panel_command_->TaskCmdDeg[arm_i][i] = 0;
                }

                //Init joint enable state
                for (int joint_i=0; joint_i<arm_joint_size_[arm_i]; joint_i++){
                    bool enable = config["robot"]["arm"][arm_i]["enable_joint"][joint_i].as<bool>();
                    panel_command_->enable_joint[arm_i][joint_i] = enable;
                }
            }

            panel_command_->GripperSize = gripper_size_;
            for (int gri_i=0; gri_i<gripper_size_; gri_i++){
                panel_command_->GripperJointSize[gri_i] = gripper_joint_size_[gri_i];
                for (int i=0; i < gripper_joint_size_[gri_i]; i++){
                    panel_command_->GripperJointCmd[gri_i][i] = 0;
                    planner_state_->GripperJointPos[gri_i][i] = 0;
                }
            }
            
            //Config setting
            for (int arm_i=0; arm_i<arm_size_; arm_i++){
                panel_command_->tool_offset[arm_i][0] = 0.0f; //Default tool offset is 0
                panel_command_->tool_offset[arm_i][1] = 0.0f; //Default tool offset is 0
                panel_command_->tool_offset[arm_i][2] = 0.0f; //Default tool offset is 0
                for (int joint_i=0; joint_i<arm_joint_size_[arm_i]; joint_i++){
                    panel_command_->arm_soft_limit_position_deg[arm_i][joint_i][0] = config["robot"]["arm"][arm_i]["safety"]["joint_soft_limit"]["position"][0][joint_i].as<float>();
                    panel_command_->arm_soft_limit_position_deg[arm_i][joint_i][1] = config["robot"]["arm"][arm_i]["safety"]["joint_soft_limit"]["position"][1][joint_i].as<float>();
                    panel_command_->arm_soft_limit_velocity_deg[arm_i][joint_i][0] = config["robot"]["arm"][arm_i]["safety"]["joint_soft_limit"]["velocity"][joint_i].as<float>() * -1.0f;
                    panel_command_->arm_soft_limit_velocity_deg[arm_i][joint_i][1] = config["robot"]["arm"][arm_i]["safety"]["joint_soft_limit"]["velocity"][joint_i].as<float>();
                }
            }
            
            for (int gri_i=0; gri_i<gripper_size_; gri_i++){
                panel_command_->tool_offset[gri_i][0] = config["robot"]["gripper"][gri_i]["tool_offset"][0].as<float>();
                panel_command_->tool_offset[gri_i][1] = config["robot"]["gripper"][gri_i]["tool_offset"][1].as<float>();
                panel_command_->tool_offset[gri_i][2] = config["robot"]["gripper"][gri_i]["tool_offset"][2].as<float>();
                for (int joint_i=0; joint_i<gripper_joint_size_[gri_i]; joint_i++){
                    panel_command_->gripper_soft_limit_position_deg[gri_i][joint_i][0] = config["robot"]["gripper"][gri_i]["safety"]["joint_soft_limit"]["position"][0][joint_i].as<float>();
                    panel_command_->gripper_soft_limit_position_deg[gri_i][joint_i][1] = config["robot"]["gripper"][gri_i]["safety"]["joint_soft_limit"]["position"][1][joint_i].as<float>();
                }
            }
        }
        
        void udpReceiveTask() {
            while (!shutdown_.load()){
                long t0 = get_time_now();
                std::unique_ptr<PlannerState> temp = std::make_unique<PlannerState>();
                if (udp_->receive(temp.get(), 50)){
                    //Convert from rad to deg for pos and vel.
                    for (int arm_i=0; arm_i<arm_size_; arm_i++){
                        for (int i=0; i < arm_joint_size_[arm_i]; i++){
                            temp->JointPos[arm_i][i] *= kRadToDeg;
                            temp->JointVel[arm_i][i] *= kRadToDeg;
                            temp->MotorPos[arm_i][i] *= kRadToDeg;
                            temp->MotorVel[arm_i][i] *= kRadToDeg;

                            temp->arm_soft_limit_position[arm_i][i][0] *= kRadToDeg;
                            temp->arm_soft_limit_position[arm_i][i][1] *= kRadToDeg;
                            temp->arm_soft_limit_velocity[arm_i][i][0] *= kRadToDeg;
                            temp->arm_soft_limit_velocity[arm_i][i][1] *= kRadToDeg;
                            temp->arm_soft_limit_torque[arm_i][i][0]   *= kRadToDeg;
                            temp->arm_soft_limit_torque[arm_i][i][1]   *= kRadToDeg;

                            temp->arm_hard_limit_position[arm_i][i][0] *= kRadToDeg;
                            temp->arm_hard_limit_position[arm_i][i][1] *= kRadToDeg;
                            temp->arm_hard_limit_velocity[arm_i][i][0] *= kRadToDeg;
                            temp->arm_hard_limit_velocity[arm_i][i][1] *= kRadToDeg;
                            temp->arm_hard_limit_torque[arm_i][i][0]   *= kRadToDeg;
                            temp->arm_hard_limit_torque[arm_i][i][1]   *= kRadToDeg;

                            temp->arm_follow_limit_position[arm_i][i] *= kRadToDeg;
                            temp->arm_follow_limit_velocity[arm_i][i] *= kRadToDeg;
                            temp->arm_follow_limit_torque[arm_i][i]   *= kRadToDeg;

                            temp->arm_jump_limit_position[arm_i][i] *= kRadToDeg;
                            temp->arm_jump_limit_velocity[arm_i][i] *= kRadToDeg;
                            temp->arm_jump_limit_torque[arm_i][i]   *= kRadToDeg;
                        }

                        for (int gri_i=0; gri_i<gripper_size_; gri_i++){
                            for (int joint_i=0; joint_i<gripper_joint_size_[gri_i]; joint_i++){
                                temp->gripper_soft_limit_position[gri_i][joint_i][0] *= kRadToDeg;
                                temp->gripper_soft_limit_position[gri_i][joint_i][1] *= kRadToDeg;
                                temp->gripper_hard_limit_position[gri_i][joint_i][0] *= kRadToDeg;
                                temp->gripper_hard_limit_position[gri_i][joint_i][1] *= kRadToDeg;
                            }
                        }
                    }
                    //Update planner state
                    planner_state_ = std::move(temp);
                    first_state_received_ = true;
                }
                
                long t1 = get_time_now();
                if (t1 - t0 < receive_dt_us_){
                    sleep_period(receive_dt_us_ - (t1 - t0));
                }
            }
        }

        void udpSendOnce(){
            panel_command_->send_timestamp = get_time_now();
            
            //Convert from deg to rad for pos and vel.
            float a = (panel_command_->target_type != ControlType::kTorque)? kDegToRad : 1;
            for (int arm_i=0; arm_i<arm_size_; arm_i++){
                for (int i=0; i < arm_joint_size_[arm_i]; i++)
                    panel_command_->JointCmd[arm_i][i] = panel_command_->JointCmdDeg[arm_i][i] * a;
                for (int i=0; i < 6; i++){
                    panel_command_->TaskCmd[arm_i][i]  = panel_command_->TaskCmdDeg[arm_i][i];
                    if (i > 2) panel_command_->TaskCmd[arm_i][i] *= kDegToRad;  //Last 3 variables are rotation
                }
            }

            for (int i = 0; i < panel_command_->ArmWaypointSize; i++){
                for (int arm_i=0; arm_i<arm_size_; arm_i++){
                    if (panel_command_->motion_type == MotionControl::kJoint){
                        for (int joint_i=0; joint_i < arm_joint_size_[arm_i]; joint_i++){
                            panel_command_->ArmWaypointCmd[i][arm_i][joint_i] = panel_command_->ArmWaypointCmdDeg[i][arm_i][joint_i] * a;
                        }
                    }else if (panel_command_->motion_type == MotionControl::kTask || panel_command_->motion_type == MotionControl::kTaskLine){
                        for (int j=0; j < 6; j++){
                            panel_command_->ArmWaypointCmd[i][arm_i][j] = panel_command_->ArmWaypointCmdDeg[i][arm_i][j];
                            if (j > 2) panel_command_->ArmWaypointCmd[i][arm_i][j] *= kDegToRad;  //Last 3 variables are rotation
                        }
                    }
                }
            }

            for (int arm_i=0; arm_i<arm_size_; arm_i++){
                for (int i=0; i<arm_joint_size_[arm_i]; i++){
                    panel_command_->arm_soft_limit_position[arm_i][i][0] = panel_command_->arm_soft_limit_position_deg[arm_i][i][0] * kDegToRad;
                    panel_command_->arm_soft_limit_position[arm_i][i][1] = panel_command_->arm_soft_limit_position_deg[arm_i][i][1] * kDegToRad;
                    panel_command_->arm_soft_limit_velocity[arm_i][i][0] = panel_command_->arm_soft_limit_velocity_deg[arm_i][i][0] * kDegToRad;
                    panel_command_->arm_soft_limit_velocity[arm_i][i][1] = panel_command_->arm_soft_limit_velocity_deg[arm_i][i][1] * kDegToRad;
                }
            }

            for (int gri_i=0; gri_i<gripper_size_; gri_i++){
                for (int joint_i=0; joint_i<gripper_joint_size_[gri_i]; joint_i++){
                    panel_command_->gripper_soft_limit_position[gri_i][joint_i][0] = panel_command_->gripper_soft_limit_position_deg[gri_i][joint_i][0] * kDegToRad;
                    panel_command_->gripper_soft_limit_position[gri_i][joint_i][1] = panel_command_->gripper_soft_limit_position_deg[gri_i][joint_i][1] * kDegToRad;
                }
            }
            udp_->send(panel_command_.get());
            panel_command_->sequence_id++;
            last_motion_control_ = (MotionControl)panel_command_->motion_type; 
            last_target_type_ = panel_command_->target_type;
            last_actuator_mode_ = panel_command_->actuator_mode;
        }

        void udpSendAndAckTask(float exit_timeout_s = 20.0f){
            //Time in us
            long t = 0;
            long dt = receive_dt_us_*2;
            long resend_timeout = 1000000; // 1sec
            long exit_timeout = exit_timeout_s*1e6; // 10 sec
            bool need_send = true;

            std::lock_guard<std::mutex> lock(panel_command_mutex_);
            if (last_actuator_mode_ != panel_command_->actuator_mode || panel_command_->reset_interpolation || panel_command_->reset_control_mem){
                panel_command_->need_setting_update = true;
            }

            while (t < exit_timeout && need_send){
                udpSendOnce();

                long t1 = 0;
                while (t1 < resend_timeout){
                    if (panel_command_->need_setting_update){
                        if (isLatestTargetReceived()){
                            panel_command_->need_setting_update = false;
                            panel_command_->reset_interpolation = false;
                            panel_command_->reset_control_mem = false; //Reset task/null saved pose for arm
                            break;
                        }
                    }else if (planner_state_->setting_update_finished && isLatestTargetReceived()){
                        need_send = false;
                        break;
                    }
                    sleep_period(dt);
                    t1 += dt;
                }
                t += t1;
            }

            if (t >= exit_timeout){
                throw std::runtime_error("Spark2::udpSendAndAckTask failed: No ACK received after " 
                    + std::to_string(exit_timeout/1000000.0) + " seconds\n");
            }
        }

        bool isLatestTargetReceived(){
            ensureRobotStarted();
            return planner_state_->received_sequence_id == panel_command_->sequence_id - 1;
        }

        void ensureRobotStarted(){
            if (!started_){
                throw std::runtime_error("Spark2::ensureRobotStarted failed: robot hasn't been started");
            }
        }

        template<typename T>
        void validatePathSize(const std::string& function_name, const Path<T>& arm_pos, const Path<int>& v, const Path<float>& t){
            if (v.size() != arm_pos.size() && t.size() != arm_pos.size()) {
                throw std::invalid_argument(
                    "Spark2::" + function_name + " failed: Size mismatch. "
                    "Either velocity path ('v') or time path ('t') must have the same size as position path ('arm_pos'), but got " + 
                    std::to_string(v.size()) + " and " + std::to_string(t.size()) + " for v and " + std::to_string(arm_pos.size()) + " for arm_pos"
                );
            }
        }

        bool isRobotStarted() {
            return started_;
        }

        PanelCommand& getPanelCommand() {
            if (!panel_command_) {
                throw std::runtime_error("Spark2::getPanelCommand failed: panel_command_ is nullptr");
            }
            return *panel_command_;
        }

        PlannerState& getPlannerState() {
            if (!planner_state_) {
                throw std::runtime_error("Spark2::getPlannerState failed: planner_state_ is nullptr");
            }
            return *planner_state_;
        }

        void sendPanelCommand(){
            udpSendAndAckTask();
        }

        void switchTargetType(ControlType target_type, ControlType panel_command){
            panel_command_->target_type = target_type;
            panel_command_->actuator_mode = panel_command;

            //Set initial value 
            int arm_i = 0;
            for (int j=0; j<arm_joint_size_[arm_i]; j++){
                panel_command_->JointCmdDeg[arm_i][j] = (target_type == ControlType::kPosition)? planner_state_->JointPos[arm_i][j] : 0;
            }
        }
    };

    Spark2::Spark2(std::string config_prefix_path) : pimpl_(std::make_unique<Impl>()){
        std::string config_path = config_prefix_path + "/config.yaml";
        YAML::Node config = loadYamlConfig(config_path);
        pimpl_->initialize(config);
        pimpl_->configurator_ = std::unique_ptr<Configurator>(new Configurator());
        pimpl_->kinematics_ = std::unique_ptr<Kinematics>(new Kinematics());
        pimpl_->kinematics_->pimpl_->initialize(config_prefix_path, config);
        
        //Register callback functions for the configurator
        pimpl_->configurator_->pimpl_->registerCallback(
            [this]() -> PanelCommand& { return pimpl_->getPanelCommand(); }, 
            [this]() -> PlannerState& { return pimpl_->getPlannerState(); }, 
            [this]() { pimpl_->sendPanelCommand(); },
            [this]() { return pimpl_->isRobotStarted(); }
        );

    }

    Spark2::~Spark2(){
        stop();

        pimpl_->shutdown_.store(true);
        if (pimpl_->receive_thread_.joinable()){
            pimpl_->receive_thread_.join();
        }

        if (pimpl_->udp_) {
            pimpl_->udp_->close();
            pimpl_->udp_.reset();
        }

        pimpl_->configurator_.reset();
        pimpl_->kinematics_.reset();
        pimpl_.reset();
    }

    // Connection
    void Spark2::start(){
        //Initialize UDP
        try {
            YAML::Node config = pimpl_->config_;
            int buffer_size = 8192;
            pimpl_->udp_ = std::make_unique<CuarmUdp<PlannerState, PanelCommand>>(
                config["panel"]["ip"].as<std::string>(),
                config["panel"]["port"].as<int>(),
                config["rt_control"]["ip"].as<std::string>(),
                config["rt_control"]["port"].as<int>(), 
                &CuarmMessageHandler::unpack_planner_state,
                &CuarmMessageHandler::pack_panel_command, buffer_size);

            //Start UDP receive thread
            pimpl_->receive_thread_ = std::thread(&Impl::udpReceiveTask, pimpl_.get());
            
            //Wait for the first state
            std::cout << "Connecting to Robot..." <<std::endl;
            while (!pimpl_->first_state_received_.load()){
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
            }
            pimpl_->started_ = true;

            //Send the first command of 0 velocity
            pimpl_->udpSendAndAckTask(20); //exit_time_s = 20

            //Switch back to config's initial mode
            pimpl_->switchTargetType(pimpl_->initial_target_type_, pimpl_->initial_actuator_mode_);
            pimpl_->udpSendAndAckTask();
        }catch (const std::exception& e) {
            throw std::runtime_error("Failed to initialize UDP communication: " + std::string(e.what()));
        }
        std::cout << "Successfully connected to Robot." <<std::endl;
    }

    void Spark2::stop(){
        if (!pimpl_->started_) return;
        pimpl_->panel_command_->connection_state = ConnectionState::kShutDown;
        pimpl_->udpSendAndAckTask(); 
        pimpl_->started_ = false;
    }
    
    void Spark2::enableArmJoint(JointState6b arm_state){
        pimpl_->ensureRobotStarted();
        int arm_i = 0;
        for (int joint_i=0; joint_i<pimpl_->arm_joint_size_[arm_i]; joint_i++){
            pimpl_->panel_command_->enable_joint[arm_i][joint_i] = arm_state[joint_i];
        }
        pimpl_->panel_command_->need_setting_update = true;
        pimpl_->udpSendAndAckTask(); 
        std::cout <<"FINISH ENABLE JOINT\n";
    }

    // Motion Mode
    void Spark2::setArmSmoothingMethod(SmoothingMethod method){
        pimpl_->ensureRobotStarted();
        switch (method){
            case SmoothingMethod::kLinear:
                pimpl_->panel_command_->interpolation_type = InterpolationMethod::kLinear;
                break;
            case SmoothingMethod::kCos:
                pimpl_->panel_command_->interpolation_type = InterpolationMethod::kCos;
                break;
            case SmoothingMethod::kCubic:
                pimpl_->panel_command_->interpolation_type = InterpolationMethod::kCubic;
                break;
            case SmoothingMethod::kQuintic:
                pimpl_->panel_command_->interpolation_type = InterpolationMethod::kQuintic;
                break;
            case SmoothingMethod::kNone:
                pimpl_->panel_command_->interpolation_type = InterpolationMethod::kNone;
                break;
        }
        pimpl_->udpSendAndAckTask(); 
    }

    // Manual Mode
    void Spark2::movePos(const JointState6f& arm_pos, int v, float t){
        pimpl_->ensureRobotStarted();
        pimpl_->panel_command_->motion_type = MotionControl::kJoint;
        pimpl_->panel_command_->arm_target_mode = PanelTargetMode::kSinglePoint;
        pimpl_->switchTargetType(ControlType::kPosition, pimpl_->actuator_mode_under_position_control_);
        pimpl_->panel_command_->InterpolationAccTime = t;
        pimpl_->panel_command_->interpolation_speed_ratio = std::clamp(v / 100.0f, 0.0f, 1.0f);
        
        int arm_i = 0;
        for (int j=0; j<pimpl_->arm_joint_size_[arm_i]; j++){
            pimpl_->panel_command_->JointCmdDeg[arm_i][j] = arm_pos[j];
        }

        pimpl_->udpSendAndAckTask(); 
    }

    void Spark2::movePosPath(const Path<JointState6f>& arm_pos, Path<int> v, Path<float> t){
        pimpl_->ensureRobotStarted();
        pimpl_->validatePathSize("movePosPath", arm_pos, v, t);

        pimpl_->panel_command_->motion_type = MotionControl::kJoint;
        pimpl_->panel_command_->arm_target_mode = PanelTargetMode::kWaypoint;
        pimpl_->switchTargetType(ControlType::kPosition, pimpl_->actuator_mode_under_position_control_);
        pimpl_->panel_command_->reset_interpolation = true;

        int arm_i = 0;
        pimpl_->panel_command_->ArmWaypointSize = arm_pos.size();
        for (int i=0; i<pimpl_->panel_command_->ArmWaypointSize; i++){
            for (int k=0; k<pimpl_->arm_joint_size_[arm_i]; k++){
                pimpl_->panel_command_->ArmWaypointCmdDeg[i][arm_i][k] = arm_pos[i][k];
            }
        }

        if (t.size() == arm_pos.size()){
            for (int i=0; i<pimpl_->panel_command_->ArmWaypointSize; i++){
                pimpl_->panel_command_->ArmWaypointInterpolationTime[i] = t[i];
                pimpl_->panel_command_->ArmWaypointInterpolationSpeedRatio[i] = 0;
            }
        }else{
            for (int i=0; i<pimpl_->panel_command_->ArmWaypointSize; i++){
                pimpl_->panel_command_->ArmWaypointInterpolationTime[i] = 0;
                pimpl_->panel_command_->ArmWaypointInterpolationSpeedRatio[i] = std::clamp(v[i] / 100.0f, 0.0f, 1.0f);
            }
        }

        pimpl_->udpSendAndAckTask(); 
    }

    void Spark2::moveVel(const JointState6f& arm_vel, int a, float t){
        pimpl_->ensureRobotStarted();
        pimpl_->panel_command_->motion_type = MotionControl::kJoint;
        pimpl_->panel_command_->arm_target_mode = PanelTargetMode::kSinglePoint;
        pimpl_->panel_command_->target_type = ControlType::kVelocity;
        pimpl_->panel_command_->actuator_mode = ControlType::kVelocity;
        pimpl_->panel_command_->InterpolationAccTime = t;
        pimpl_->panel_command_->interpolation_speed_ratio = std::clamp(a / 100.0f, 0.0f, 1.0f);

        int arm_i = 0;
        for (int j=0; j<pimpl_->arm_joint_size_[arm_i]; j++){
            pimpl_->panel_command_->JointCmdDeg[arm_i][j] = arm_vel[j];
        }

        pimpl_->udpSendAndAckTask(); 
    }

    void Spark2::moveEEPoint(const Pose& arm_ee, int v, float t){
        pimpl_->ensureRobotStarted();
        Position tool_offset = pimpl_->configurator_->getToolOffset();
        Pose tool_pose = pimpl_->kinematics_->eeToToolPose(arm_ee, tool_offset);
        moveToolPoint(tool_pose, v, t);
    }

    void Spark2::moveEEPointPath(const Path<Pose>& arm_ee, Path<int> v, Path<float> t){
        pimpl_->ensureRobotStarted();
        pimpl_->validatePathSize("moveEEPointPath", arm_ee, v, t);

        Path<Pose> tool_pose;
        Position tool_offset = pimpl_->configurator_->getToolOffset();
        for (int i=0; i<arm_ee.size(); i++){
            tool_pose[i] = pimpl_->kinematics_->eeToToolPose(arm_ee[i], tool_offset);
        }
        moveToolPointPath(tool_pose, v, t);
    }

    void Spark2::moveEELine(const Pose& arm_ee, int v, float t){
        pimpl_->ensureRobotStarted();
        Position tool_offset = pimpl_->configurator_->getToolOffset();
        Pose tool_pose = pimpl_->kinematics_->eeToToolPose(arm_ee, tool_offset);
        moveToolLine(tool_pose, v, t);
    }

    void Spark2::moveEELinePath(const Path<Pose>& arm_ee, Path<int> v, Path<float> t){
        pimpl_->ensureRobotStarted();
        pimpl_->validatePathSize("moveEELinePath", arm_ee, v, t);

        Path<Pose> tool_pose;
        Position tool_offset = pimpl_->configurator_->getToolOffset();
        for (int i=0; i<arm_ee.size(); i++){
            tool_pose[i] = pimpl_->kinematics_->eeToToolPose(arm_ee[i], tool_offset);
        }
        moveToolLinePath(tool_pose, v, t);
    }

    void Spark2::moveToolPoint(const Pose& arm_tool, int v, float t){
        pimpl_->ensureRobotStarted();
        pimpl_->panel_command_->motion_type = MotionControl::kTask;
        pimpl_->panel_command_->task_orien_type = OrientControl::kEnd;
        pimpl_->panel_command_->arm_target_mode = PanelTargetMode::kSinglePoint;
        pimpl_->switchTargetType(ControlType::kPosition, pimpl_->actuator_mode_under_position_control_);
        pimpl_->panel_command_->reset_control_mem = true;
        pimpl_->panel_command_->InterpolationAccTime = t;
        pimpl_->panel_command_->interpolation_speed_ratio = std::clamp(v / 100.0f, 0.0f, 1.0f);

        int arm_i = 0;
        Pose current_pose = getToolPose();
        EulerAngle current_euler = pimpl_->kinematics_->quaternionToEuler(current_pose.orientation);
        EulerAngle target_euler = pimpl_->kinematics_->quaternionToEuler(arm_tool.orientation);
        pimpl_->panel_command_->TaskCmdDeg[arm_i][0] = arm_tool.position.x - current_pose.position.x;
        pimpl_->panel_command_->TaskCmdDeg[arm_i][1] = arm_tool.position.y - current_pose.position.y;
        pimpl_->panel_command_->TaskCmdDeg[arm_i][2] = arm_tool.position.z - current_pose.position.z;
        pimpl_->panel_command_->TaskCmdDeg[arm_i][3] = target_euler.roll - current_euler.roll;
        pimpl_->panel_command_->TaskCmdDeg[arm_i][4] = target_euler.pitch - current_euler.pitch;
        pimpl_->panel_command_->TaskCmdDeg[arm_i][5] = target_euler.yaw - current_euler.yaw;
        
        pimpl_->udpSendAndAckTask(); 
    }

    void Spark2::moveToolPointPath(const Path<Pose>& arm_tool, Path<int> v, Path<float> t){
        pimpl_->ensureRobotStarted();
        pimpl_->validatePathSize("moveToolPointPath", arm_tool, v, t);

        pimpl_->panel_command_->motion_type = MotionControl::kTask;
        pimpl_->panel_command_->task_orien_type = OrientControl::kEnd;
        pimpl_->panel_command_->arm_target_mode = PanelTargetMode::kWaypoint;
        pimpl_->switchTargetType(ControlType::kPosition, pimpl_->actuator_mode_under_position_control_);
        pimpl_->panel_command_->reset_control_mem = true;
        pimpl_->panel_command_->reset_interpolation = true;

        int arm_i = 0;
        Pose current_pose = getToolPose();
        EulerAngle current_euler = pimpl_->kinematics_->quaternionToEuler(current_pose.orientation);
        pimpl_->panel_command_->ArmWaypointSize = arm_tool.size();
        for (int i=0; i<pimpl_->panel_command_->ArmWaypointSize; i++){
            EulerAngle target_euler = pimpl_->kinematics_->quaternionToEuler(arm_tool[i].orientation);
            pimpl_->panel_command_->ArmWaypointCmdDeg[i][arm_i][0] = arm_tool[i].position.x - current_pose.position.x;
            pimpl_->panel_command_->ArmWaypointCmdDeg[i][arm_i][1] = arm_tool[i].position.y - current_pose.position.y;
            pimpl_->panel_command_->ArmWaypointCmdDeg[i][arm_i][2] = arm_tool[i].position.z - current_pose.position.z;
            pimpl_->panel_command_->ArmWaypointCmdDeg[i][arm_i][3] = target_euler.roll - current_euler.roll;
            pimpl_->panel_command_->ArmWaypointCmdDeg[i][arm_i][4] = target_euler.pitch - current_euler.pitch;
            pimpl_->panel_command_->ArmWaypointCmdDeg[i][arm_i][5] = target_euler.yaw - current_euler.yaw;
        }
        
        if (t.size() == arm_tool.size()){
            for (int i=0; i<pimpl_->panel_command_->ArmWaypointSize; i++){
                pimpl_->panel_command_->ArmWaypointInterpolationTime[i] = t[i];
                pimpl_->panel_command_->ArmWaypointInterpolationSpeedRatio[i] = 0;
            }
        }else{
            for (int i=0; i<pimpl_->panel_command_->ArmWaypointSize; i++){
                pimpl_->panel_command_->ArmWaypointInterpolationTime[i] = 0;
                pimpl_->panel_command_->ArmWaypointInterpolationSpeedRatio[i] = std::clamp(v[i] / 100.0f, 0.0f, 1.0f);
            }
        }

        pimpl_->udpSendAndAckTask(); 
    }

    void Spark2::moveToolLine(const Pose& arm_tool, int v, float t){
        pimpl_->ensureRobotStarted();
        pimpl_->panel_command_->motion_type = MotionControl::kTaskLine;
        pimpl_->panel_command_->task_orien_type = OrientControl::kEnd;
        pimpl_->panel_command_->arm_target_mode = PanelTargetMode::kSinglePoint;
        pimpl_->switchTargetType(ControlType::kPosition, pimpl_->actuator_mode_under_position_control_);
        pimpl_->panel_command_->reset_control_mem = true;
        pimpl_->panel_command_->InterpolationAccTime = t;
        pimpl_->panel_command_->interpolation_speed_ratio = std::clamp(v / 100.0f, 0.0f, 1.0f);

        int arm_i = 0;
        Pose current_pose = getToolPose();
        EulerAngle current_euler = pimpl_->kinematics_->quaternionToEuler(current_pose.orientation);
        EulerAngle target_euler = pimpl_->kinematics_->quaternionToEuler(arm_tool.orientation);
        pimpl_->panel_command_->TaskCmdDeg[arm_i][0] = arm_tool.position.x - current_pose.position.x;
        pimpl_->panel_command_->TaskCmdDeg[arm_i][1] = arm_tool.position.y - current_pose.position.y;
        pimpl_->panel_command_->TaskCmdDeg[arm_i][2] = arm_tool.position.z - current_pose.position.z;
        pimpl_->panel_command_->TaskCmdDeg[arm_i][3] = target_euler.roll - current_euler.roll;
        pimpl_->panel_command_->TaskCmdDeg[arm_i][4] = target_euler.pitch - current_euler.pitch;
        pimpl_->panel_command_->TaskCmdDeg[arm_i][5] = target_euler.yaw - current_euler.yaw;
        pimpl_->udpSendAndAckTask(); 
    }

    void Spark2::moveToolLinePath(const Path<Pose>& arm_tool, Path<int> v, Path<float> t){
        pimpl_->ensureRobotStarted();
        pimpl_->validatePathSize("moveToolLinePath", arm_tool, v, t);

        pimpl_->panel_command_->motion_type = MotionControl::kTaskLine;
        pimpl_->panel_command_->task_orien_type = OrientControl::kEnd;
        pimpl_->panel_command_->arm_target_mode = PanelTargetMode::kWaypoint;
        pimpl_->switchTargetType(ControlType::kPosition, pimpl_->actuator_mode_under_position_control_);
        pimpl_->panel_command_->reset_control_mem = true;
        pimpl_->panel_command_->reset_interpolation = true;
        
        int arm_i = 0;
        Pose current_pose = getToolPose();
        EulerAngle current_euler = pimpl_->kinematics_->quaternionToEuler(current_pose.orientation);
        pimpl_->panel_command_->ArmWaypointSize = arm_tool.size();
        for (int i=0; i<pimpl_->panel_command_->ArmWaypointSize; i++){
            EulerAngle target_euler = pimpl_->kinematics_->quaternionToEuler(arm_tool[i].orientation);
            pimpl_->panel_command_->ArmWaypointCmdDeg[i][arm_i][0] = arm_tool[i].position.x - current_pose.position.x;
            pimpl_->panel_command_->ArmWaypointCmdDeg[i][arm_i][1] = arm_tool[i].position.y - current_pose.position.y;
            pimpl_->panel_command_->ArmWaypointCmdDeg[i][arm_i][2] = arm_tool[i].position.z - current_pose.position.z;
            pimpl_->panel_command_->ArmWaypointCmdDeg[i][arm_i][3] = target_euler.roll - current_euler.roll;
            pimpl_->panel_command_->ArmWaypointCmdDeg[i][arm_i][4] = target_euler.pitch - current_euler.pitch;
            pimpl_->panel_command_->ArmWaypointCmdDeg[i][arm_i][5] = target_euler.yaw - current_euler.yaw;
        }

        if (t.size() == arm_tool.size()){
            for (int i=0; i<pimpl_->panel_command_->ArmWaypointSize; i++){
                pimpl_->panel_command_->ArmWaypointInterpolationTime[i] = t[i];
                pimpl_->panel_command_->ArmWaypointInterpolationSpeedRatio[i] = 0;
            }
        }else{
            for (int i=0; i<pimpl_->panel_command_->ArmWaypointSize; i++){
                pimpl_->panel_command_->ArmWaypointInterpolationTime[i] = 0;
                pimpl_->panel_command_->ArmWaypointInterpolationSpeedRatio[i] = std::clamp(v[i] / 100.0f, 0.0f, 1.0f);
            }
        }

        pimpl_->udpSendAndAckTask(); 
    }

    void Spark2::goHome(int v, float t){
        pimpl_->ensureRobotStarted();
        JointState6f home_pos;
        
        int arm_i = 0;
        for (int j=0; j<pimpl_->arm_joint_size_[arm_i]; j++){
            home_pos[j] = 0;
        }
        movePos(home_pos, v, t);
    }

    void Spark2::moveGripperPos(const JointState1f& pos, int v, float t){
        pimpl_->ensureRobotStarted();
        for (int i=0; i<pimpl_->gripper_size_; i++){
            for (int j=0; j<pimpl_->gripper_joint_size_[i]; j++){
                pimpl_->panel_command_->GripperJointCmd[i][j] = pos[j];
            }
        }
        pimpl_->udpSendAndAckTask(); 
    }

    // Teach Mode
    void Spark2::startTeach(){
        pimpl_->ensureRobotStarted();
        pimpl_->panel_command_->motion_type = MotionControl::kControlAlgorithm;
        pimpl_->panel_command_->control_algorithm = ControlAlgorithm::kGravity;
        pimpl_->switchTargetType(ControlType::kTorque, ControlType::kTorque);
        pimpl_->panel_command_->enable_recording = true;
        pimpl_->udpSendAndAckTask(); 
    }

    void Spark2::stopTeach(){
        pimpl_->ensureRobotStarted();
        pimpl_->panel_command_->motion_type = MotionControl::kJoint;
        pimpl_->panel_command_->control_algorithm = ControlAlgorithm::kNone;
        pimpl_->switchTargetType(ControlType::kPosition, pimpl_->actuator_mode_under_position_control_);
        pimpl_->panel_command_->enable_recording = false;
        pimpl_->udpSendAndAckTask(); 
    }

    // Playback Mode
    void Spark2::startPlayback(){
        pimpl_->ensureRobotStarted();
        pimpl_->panel_command_->motion_type = MotionControl::kPlayback;
        pimpl_->switchTargetType(ControlType::kPosition, pimpl_->actuator_mode_under_position_control_);
        pimpl_->panel_command_->playback_cmd = PlaybackState::kStart;
        pimpl_->panel_command_->InterpolationAccTime = 0;
        pimpl_->panel_command_->interpolation_speed_ratio = 0.2f;
        pimpl_->udpSendAndAckTask(); 
    }

    void Spark2::stopPlayback(){
        pimpl_->ensureRobotStarted();
        pimpl_->panel_command_->motion_type = MotionControl::kPlayback;
        pimpl_->switchTargetType(ControlType::kPosition, pimpl_->actuator_mode_under_position_control_);
        pimpl_->panel_command_->playback_cmd = PlaybackState::kStop;
        pimpl_->panel_command_->InterpolationAccTime = 0;
        pimpl_->panel_command_->interpolation_speed_ratio = 0.2f;
        pimpl_->udpSendAndAckTask();
    }

    void Spark2::resetPlayback(){
        pimpl_->ensureRobotStarted();
        pimpl_->panel_command_->motion_type = MotionControl::kPlayback;
        pimpl_->switchTargetType(ControlType::kPosition, pimpl_->actuator_mode_under_position_control_);
        pimpl_->panel_command_->playback_cmd = PlaybackState::kReset;
        pimpl_->panel_command_->InterpolationAccTime = 0;
        pimpl_->panel_command_->interpolation_speed_ratio = 0.2f;
        pimpl_->udpSendAndAckTask();
    }

    // Feedback
    RobotJointStatef Spark2::getPos() const{
        pimpl_->ensureRobotStarted();
        int arm_i = 0;
        RobotJointStatef state;

        JointState6f joint_state;
        for (int i=0; i<pimpl_->arm_joint_size_[arm_i]; i++){
            joint_state[i] = pimpl_->planner_state_->JointPos[arm_i][i];
        }
        state.arm = joint_state;

        JointState1f gripper_state{0};
        if (pimpl_->gripper_size_ > 0){
            int gripper_i = 0;
            for (int i=0; i<pimpl_->gripper_joint_size_[gripper_i]; i++){
                gripper_state[i] = pimpl_->planner_state_->GripperJointPos[gripper_i][i];
            }
        }
        state.gripper = gripper_state;
        return state;
    }

    RobotJointStatef Spark2::getVel() const{
        pimpl_->ensureRobotStarted();
        int arm_i = 0;
        RobotJointStatef state;

        JointState6f joint_state;
        for (int i=0; i<pimpl_->arm_joint_size_[arm_i]; i++){
            joint_state[i] = pimpl_->planner_state_->JointVel[arm_i][i];
        }
        state.arm = joint_state;

        JointState1f gripper_state{0};
        if (pimpl_->gripper_size_ > 0){
            int gripper_i = 0;
            for (int i=0; i<pimpl_->gripper_joint_size_[gripper_i]; i++){
                gripper_state[i] = pimpl_->planner_state_->GripperJointPos[gripper_i][i];
            }
        }
        state.gripper = gripper_state;
        return state;
    }

    RobotJointStatef Spark2::getTor() const{
        pimpl_->ensureRobotStarted();
        int arm_i = 0;
        RobotJointStatef state;

        JointState6f joint_state;
        for (int i=0; i<pimpl_->arm_joint_size_[arm_i]; i++){
            joint_state[i] = pimpl_->planner_state_->JointTor[arm_i][i];
        }
        state.arm = joint_state;

        JointState1f gripper_state{0};
        if (pimpl_->gripper_size_ > 0){
            int gripper_i = 0;
            for (int i=0; i<pimpl_->gripper_joint_size_[gripper_i]; i++){
                gripper_state[i] = pimpl_->planner_state_->GripperJointPos[gripper_i][i];
            }
        }
        state.gripper = gripper_state;
        return state;
    }

    Pose Spark2::getEEPose() const{
        pimpl_->ensureRobotStarted();
        int arm_i = 0;
        Pose pose;
        pose.position.x = pimpl_->planner_state_->EEPose[arm_i][0];
        pose.position.y = pimpl_->planner_state_->EEPose[arm_i][1];
        pose.position.z = pimpl_->planner_state_->EEPose[arm_i][2];
        pose.orientation.w = pimpl_->planner_state_->EEPose[arm_i][3];
        pose.orientation.x = pimpl_->planner_state_->EEPose[arm_i][4];
        pose.orientation.y = pimpl_->planner_state_->EEPose[arm_i][5];
        pose.orientation.z = pimpl_->planner_state_->EEPose[arm_i][6];
        return pose;
    }

    Pose Spark2::getToolPose() const{
        pimpl_->ensureRobotStarted();
        int arm_i = 0;
        Pose pose;
        pose.position.x = pimpl_->planner_state_->ToolPose[arm_i][0];
        pose.position.y = pimpl_->planner_state_->ToolPose[arm_i][1];
        pose.position.z = pimpl_->planner_state_->ToolPose[arm_i][2];
        pose.orientation.w = pimpl_->planner_state_->ToolPose[arm_i][3];
        pose.orientation.x = pimpl_->planner_state_->ToolPose[arm_i][4];
        pose.orientation.y = pimpl_->planner_state_->ToolPose[arm_i][5];
        pose.orientation.z = pimpl_->planner_state_->ToolPose[arm_i][6];
        return pose;
    }

    // Status
    JointState6b Spark2::isArmJointEnabled() const{
        JointState6b enabled;
        int arm_i = 0;
        for (int j=0; j<pimpl_->arm_joint_size_[arm_i]; j++){
            enabled[j] = pimpl_->planner_state_->enabled_joint[arm_i][j];
        }
        return enabled;
    }
    
    SystemStatus Spark2::getStatus() const{
        SystemStatus status;
        int arm_i = 0;
        status.robot_state =  static_cast<spark2::RobotState>(pimpl_->planner_state_->system_state);
        status.plan_result = static_cast<spark2::PlanResult>(pimpl_->planner_state_->plan_result);
        status.robot_diagnostic_flags = pimpl_->planner_state_->system_diagnostic_flags;
        for (int j=0; j<pimpl_->arm_joint_size_[arm_i]; j++){
            status.arm_joint_diagnostic_flags[j] = pimpl_->planner_state_->arm_joint_diagnostic_flags[arm_i][j];
        }
        std::cout <<"\n";
        if (pimpl_->gripper_size_ > 0){
            int gripper_i = 0;
            for (int j=0; j<pimpl_->gripper_joint_size_[gripper_i]; j++){
                status.gripper_joint_diagnostic_flags[j] = spark2::DiagnosticFlags::kNone;
            }
        }
        return status;
    }

    void Spark2::printStatus(const SystemStatus& status) const{
        int arm_i = 0;
        std::cout << "Robot State: " << enumToString(status.robot_state) << std::endl;
        std::cout << "Plan Result: " << enumToString(status.plan_result) << std::endl;
        std::cout << "Robot Diagnostic Flags: ";
        print_diagnostic_flags(status.robot_diagnostic_flags);

        //Print
        std::cout << "Arm Joint Diagnostic Flags: ";
        bool has_diagnostic_flags = false;
        for (int j=0; j<pimpl_->arm_joint_size_[arm_i]; j++){
            if (status.arm_joint_diagnostic_flags[j] != DiagnosticFlags::kNone){
                std::cout << "Joint " << j <<": ";
                print_diagnostic_flags(status.arm_joint_diagnostic_flags[j]);
                has_diagnostic_flags = true;
            }
        }
        if (!has_diagnostic_flags){
            std::cout << "None\n";
        }

        //Print gripper joint diagnostic flags
        if (pimpl_->gripper_size_ > 0){
            std::cout << "Gripper Joint Diagnostic Flags: ";
            has_diagnostic_flags = false;
            int gripper_i = 0;
            for (int j=0; j<pimpl_->gripper_joint_size_[gripper_i]; j++){
                if (status.gripper_joint_diagnostic_flags[j] != DiagnosticFlags::kNone){
                    std::cout << "Joint " << j <<": ";
                    print_diagnostic_flags(status.gripper_joint_diagnostic_flags[j]);
                    has_diagnostic_flags = true;
                }
            }
            if (!has_diagnostic_flags){
                std::cout << "None\n";
            }
        }

    }

    Configurator& Spark2::getConfigurator(){
        return *pimpl_->configurator_;
    }

    Kinematics& Spark2::getKinematics(){
        return *pimpl_->kinematics_;
    }



}