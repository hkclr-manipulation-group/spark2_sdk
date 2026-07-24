#include "spark2.h"

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <filesystem>
#include <functional>
#include <iostream>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <thread>
#include <variant>
#include <vector>

#include "robot_platform_utils/cpp/include/client_keys.h"
#include "robot_platform_utils/cpp/include/config_loader.h"
#include "robot_platform_utils/cpp/include/cuarm_udp.h"
#include "robot_platform_utils/cpp/include/mcast_client.h"
#include "robot_platform_utils/cpp/include/mcast_server_discovery.h"
#include "robot_platform_utils/cpp/include/platform_serialization.h"
#include "robot_platform_utils/cpp/include/platform_state.h"
#include "robot_platform_utils/cpp/include/time_utils.h"

#include "configurator_impl.h"
#include "kinematics_impl.h"

namespace spark2{
namespace {

using robot::platform::CartesianTarget;
using robot::platform::CommandResponseStatus;
using robot::platform::ControlStrategy;
using robot::platform::ControlType;
using robot::platform::CoreRequestVariant;
using robot::platform::CoreRequestVariantPtr;
using robot::platform::CoreResponseVariant;
using robot::platform::CoreResponseVariantPtr;
using robot::platform::CuarmUdp;
using robot::platform::FrameReference;
using robot::platform::MonitoringRequestVariant;
using robot::platform::MonitoringRequestVariantPtr;
using robot::platform::PlaybackState;
using robot::platform::PositionTarget;
using robot::platform::SdkCommandReq;
using robot::platform::SdkCommandRes;
using robot::platform::SdkConfigReq;
using robot::platform::SdkConfigRes;
using robot::platform::SdkHandshakeReq;
using robot::platform::SdkHeartbeatReq;
using robot::platform::SrvState;
using robot::platform::SystemState;
using PlatformSmoothingMethod = robot::platform::SmoothingMethod;

constexpr uint16_t kDefaultSdkClientId = 10003;
constexpr float kAckPollMs = 10.0f;
constexpr float kCommandAckTimeoutMs = 5000.0f;
constexpr float kConfigAckTimeoutMs = 1000.0f;
constexpr float kConfigExitTimeoutMs = 50000.0f;

using SdkMcastClient = robot::platform::McastClient<SrvState, CoreRequestVariantPtr, CoreResponseVariantPtr>;
using MonitoringUdp = CuarmUdp<std::monostate, MonitoringRequestVariantPtr>;

bool packCoreRequest(
    const CoreRequestVariantPtr& request,
    std::uint8_t* buffer,
    std::size_t buffer_size,
    std::size_t& written_size){
    return robot::platform::serialization::packMessage(
        request, buffer, buffer_size, written_size, robot::platform::getClientHmacKey);
}

bool unpackSrvState(const std::uint8_t* buffer, std::size_t size, SrvState& out){
    out = SrvState{};
    out.magic_header = 0;
    return robot::platform::serialization::unpackMessage(buffer, size, out, robot::platform::getClientHmacKey);
}

bool unpackCoreResponse(const std::uint8_t* buffer, std::size_t size, CoreResponseVariantPtr& out){
    out = nullptr;
    return robot::platform::serialization::unpackMessage(buffer, size, out, robot::platform::getClientHmacKey);
}

bool packMonitoringRequest(
    const MonitoringRequestVariantPtr& request,
    std::uint8_t* buffer,
    std::size_t buffer_size,
    std::size_t& written_size){
    return robot::platform::serialization::packMessage(
        request, buffer, buffer_size, written_size, robot::platform::getClientHmacKey);
}

bool isCommandAckSuccess(const CoreResponseVariantPtr& ack){
    return ack && std::holds_alternative<SdkCommandRes>(*ack)
        && std::get<SdkCommandRes>(*ack).payload.status == CommandResponseStatus::kSuccess;
}

bool isConfigAckSuccess(const CoreResponseVariantPtr& ack){
    return ack && std::holds_alternative<SdkConfigRes>(*ack)
        && std::get<SdkConfigRes>(*ack).payload.status == CommandResponseStatus::kSuccess;
}

PlatformSmoothingMethod toPlatformSmoothing(SmoothingMethod method){
    switch (method){
        case SmoothingMethod::kLinear: return PlatformSmoothingMethod::kLinear;
        case SmoothingMethod::kCos: return PlatformSmoothingMethod::kCos;
        case SmoothingMethod::kCubic: return PlatformSmoothingMethod::kCubic;
        case SmoothingMethod::kQuintic: return PlatformSmoothingMethod::kQuintic;
        case SmoothingMethod::kNone: return PlatformSmoothingMethod::kNone;
        case SmoothingMethod::kQuinticPath: return PlatformSmoothingMethod::kQuinticPath;
    }
    return PlatformSmoothingMethod::kCos;
}

RobotState mapSystemState(SystemState state){
    switch (state){
        case SystemState::kUnknown:  return RobotState::kStartup;
        case SystemState::kStartup:  return RobotState::kStartup;
        case SystemState::kIdle:     return RobotState::kIdle;
        case SystemState::kMoving:   return RobotState::kMoving;
        case SystemState::kSettling: return RobotState::kSettling;
        case SystemState::kError:    return RobotState::kError;
        case SystemState::kRecovery: return RobotState::kRecovery;
        case SystemState::kShutdown: return RobotState::kShutdown;
    }
    return RobotState::kStartup;
}

Pose poseFromCartesian(const CartesianTarget& c){
    Pose pose;
    pose.position = Position{c.x, c.y, c.z};
    pose.orientation = Quaternion{c.qw, c.qx, c.qy, c.qz};
    return pose;
}

void fillCartesianFromPose(const Pose& pose, CartesianTarget& out){
    float pose7[7] = {
        pose.position.x, pose.position.y, pose.position.z,
        pose.orientation.w, pose.orientation.x, pose.orientation.y, pose.orientation.z
    };
    robot::platform::writeCartesian(pose7, out);
}

std::string robotStateToString(RobotState state){
    switch (state){
        case RobotState::kStartup: return "Startup";
        case RobotState::kIdle: return "Idle";
        case RobotState::kMoving: return "Moving";
        case RobotState::kSettling: return "Settling";
        case RobotState::kError: return "Error";
        case RobotState::kRecovery: return "Recovery";
        case RobotState::kShutdown: return "Shutdown";
    }
    return "Unknown";
}

std::string planResultToString(PlanResult result){
    switch (result){
        case PlanResult::kSuccess: return "Success";
        case PlanResult::kPoseNotReachable: return "PoseNotReachable";
        case PlanResult::kLinearPathFailed: return "LinearPathFailed";
    }
    return "Unknown";
}

}  // namespace

struct Spark2::Impl{
    std::unique_ptr<Configurator> configurator_;
    std::unique_ptr<Kinematics> kinematics_;

    std::unique_ptr<SdkMcastClient> mcast_;
    std::unique_ptr<MonitoringUdp> monitoring_;

    uint16_t client_id_ = kDefaultSdkClientId;
    uint32_t session_id_ = 0;
    uint32_t sequence_id_ = 0;

    SdkConfigReq config_req_{};
    SdkCommandReq cmd_req_{};
    SdkConfigRes last_config_res_{};
    bool have_config_res_ = false;

    std::shared_ptr<SrvState> srv_state_ = std::make_shared<SrvState>();
    std::shared_ptr<SrvState> transmit_state_;
    std::mutex state_mutex_;
    std::mutex send_mutex_;

    int arm_size_ = 0;
    int gripper_size_ = 0;
    std::vector<int> arm_joint_size_;
    std::vector<int> gripper_joint_size_;
    std::string robot_name_;
    std::string config_prefix_path_;

    ControlType initial_target_type_ = ControlType::kPosition;
    ControlType initial_actuator_mode_ = ControlType::kVelocity;
    ControlType actuator_mode_under_position_control_ = ControlType::kVelocity;
    PlatformSmoothingMethod default_smoothing_ = PlatformSmoothingMethod::kCos;
    float default_acc_t_ = 0.f;
    YAML::Node config_;

    std::atomic<bool> shutdown_{false};
    std::atomic<bool> first_state_received_{false};
    bool started_ = false;
    std::thread receive_thread_;
    std::thread heartbeat_thread_;
    float receive_dt_us_ = 10000.f;
    float heartbeat_dt_ms_ = 10.f;

    void ensureRobotStarted() const{
        if (!started_){
            throw std::runtime_error("Spark2::ensureRobotStarted failed: robot hasn't been started");
        }
    }

    bool isRobotStarted() const{
        return started_;
    }

    SdkConfigReq& getConfigReq(){ return config_req_; }
    const SdkConfigRes& getConfigRes() const{
        if (!have_config_res_){
            throw std::runtime_error("Spark2::getConfigRes failed: no SdkConfigRes received yet");
        }
        return last_config_res_;
    }

    template<typename T>
    void validatePathSize(const std::string& function_name, const Path<T>& path, const Path<int>& v, const Path<float>& t){
        if (v.size() != path.size() && t.size() != path.size()){
            throw std::invalid_argument(
                "Spark2::" + function_name + " failed: Size mismatch. "
                "Either velocity path ('v') or time path ('t') must have the same size as the path, but got "
                + std::to_string(v.size()) + " and " + std::to_string(t.size())
                + " for v/t and " + std::to_string(path.size()) + " for path");
        }
        if (path.size() > MAX_WAYPOINTS){
            throw std::invalid_argument(
                "Spark2::" + function_name + " failed: path size "
                + std::to_string(path.size()) + " exceeds MAX_WAYPOINTS="
                + std::to_string(MAX_WAYPOINTS));
        }
    }

    void initialize(const YAML::Node& config){
        config_ = config;
        arm_size_ = static_cast<int>(config["robot"]["arm"].size());
        arm_joint_size_.clear();
        for (int arm_i = 0; arm_i < arm_size_; arm_i++){
            arm_joint_size_.push_back(config["robot"]["arm"][arm_i]["joint_size"].as<int>());
        }

        gripper_size_ = 0;
        gripper_joint_size_.clear();
        if (config["robot"]["gripper"]){
            gripper_size_ = static_cast<int>(config["robot"]["gripper"].size());
            for (int gri_i = 0; gri_i < gripper_size_; gri_i++){
                gripper_joint_size_.push_back(config["robot"]["gripper"][gri_i]["joint_size"].as<int>());
            }
        }

        if (config["panel"]["dt_receive"]){
            receive_dt_us_ = config["panel"]["dt_receive"].as<float>() * 1e6f;
        } else if (config["panel"]["dt_receive_ms"]){
            receive_dt_us_ = config["panel"]["dt_receive_ms"].as<float>() * 1e3f;
        }
        heartbeat_dt_ms_ = config["panel"]["dt_heartbeat_ms"]
            ? config["panel"]["dt_heartbeat_ms"].as<float>() : 10.f;

        client_id_ = config["rt_control"]["client_id"]
            ? static_cast<uint16_t>(config["rt_control"]["client_id"].as<int>())
            : kDefaultSdkClientId;

        robot_name_ = config["robot"]["name"].as<std::string>();
        default_acc_t_ = config["panel"]["general"]["acc_time"]
            ? config["panel"]["general"]["acc_time"].as<float>() : 0.f;

        const YAML::Node general = config["panel"]["general"];
        std::string smoothing_str = "Cos";
        if (general["smoothing_method"]){
            smoothing_str = general["smoothing_method"].as<std::string>();
        } else if (general["interpolation"]){
            smoothing_str = general["interpolation"].as<std::string>();
        }
        initial_target_type_ = robot::platform::stringToEnum<ControlType>(general["target_type"].as<std::string>());
        initial_actuator_mode_ = robot::platform::stringToEnum<ControlType>(general["actuator_mode"].as<std::string>());
        actuator_mode_under_position_control_ =
            (initial_target_type_ == ControlType::kPosition) ? initial_actuator_mode_ : ControlType::kPosition;
        const auto smoothing = robot::platform::stringToEnum<PlatformSmoothingMethod>(smoothing_str);
        default_smoothing_ = smoothing;

        config_req_ = SdkConfigReq{};
        cmd_req_ = SdkCommandReq{};
        config_req_.payload.read_only = 1;
        config_req_.payload.simulation = config["rt_control"]["hardware_simulation"].as<bool>() ? 1 : 0;
        config_req_.payload.arm_size = static_cast<uint8_t>(arm_size_);
        config_req_.payload.gripper_size = static_cast<uint8_t>(gripper_size_);
        std::snprintf(config_req_.payload.robot_name, sizeof(config_req_.payload.robot_name), "%s", robot_name_.c_str());
        std::snprintf(cmd_req_.payload.robot_name, sizeof(cmd_req_.payload.robot_name), "%s", robot_name_.c_str());
        cmd_req_.payload.arm_size = static_cast<uint8_t>(arm_size_);
        cmd_req_.payload.gripper_size = static_cast<uint8_t>(gripper_size_);
        cmd_req_.payload.target_count = 1;
        cmd_req_.payload.enable_jog = 0;
        cmd_req_.payload.activated_control_strategy = ControlStrategy::kJoint;

        srv_state_->payload.arm_size = static_cast<uint8_t>(arm_size_);
        srv_state_->payload.gripper_size = static_cast<uint8_t>(gripper_size_);

        for (int arm_i = 0; arm_i < arm_size_; arm_i++){
            const int nj = arm_joint_size_[arm_i];
            config_req_.payload.arm_joint_size[arm_i] = static_cast<uint8_t>(nj);
            cmd_req_.payload.arm_joint_size[arm_i] = static_cast<uint8_t>(nj);
            srv_state_->payload.arm_joint_size[arm_i] = static_cast<uint8_t>(nj);

            auto& arm_cfg = config_req_.payload.arm[arm_i];
            arm_cfg.control_strategy = ControlStrategy::kJoint;
            arm_cfg.filter_type = smoothing;
            arm_cfg.target_type = initial_target_type_;
            arm_cfg.actuator_mode = initial_actuator_mode_;
            arm_cfg.playback_cmd = PlaybackState::kStop;
            arm_cfg.frame_reference = FrameReference::kTool;
            arm_cfg.reset_control_mem = 0;
            arm_cfg.reset_interpolation = 0;
            arm_cfg.tool_offset = PositionTarget{0.f, 0.f, 0.f};

            for (int joint_i = 0; joint_i < nj; joint_i++){
                arm_cfg.enable_joint[joint_i] =
                    config["robot"]["arm"][arm_i]["enable_joint"][joint_i].as<bool>() ? 1 : 0;
                arm_cfg.soft_limit_position[joint_i][0] =
                    config["robot"]["arm"][arm_i]["safety"]["joint_soft_limit"]["position"][0][joint_i].as<float>() * kDegToRad;
                arm_cfg.soft_limit_position[joint_i][1] =
                    config["robot"]["arm"][arm_i]["safety"]["joint_soft_limit"]["position"][1][joint_i].as<float>() * kDegToRad;
                arm_cfg.soft_limit_velocity[joint_i] = std::abs(
                    config["robot"]["arm"][arm_i]["safety"]["joint_soft_limit"]["velocity"][joint_i].as<float>()) * kDegToRad;
                if (config["robot"]["arm"][arm_i]["safety"]["joint_soft_limit"]["torque"]){
                    arm_cfg.soft_limit_torque[joint_i] = std::abs(
                        config["robot"]["arm"][arm_i]["safety"]["joint_soft_limit"]["torque"][joint_i].as<float>());
                } else {
                    arm_cfg.soft_limit_torque[joint_i] = 40.f;
                }
            }

            if (gripper_size_ > arm_i && config["robot"]["gripper"][arm_i]["offset_from_ee"]){
                const auto& off = config["robot"]["gripper"][arm_i]["offset_from_ee"];
                arm_cfg.tool_offset = PositionTarget{
                    off[0].as<float>(), off[1].as<float>(), off[2].as<float>()};
            }
        }

        for (int gri_i = 0; gri_i < gripper_size_; gri_i++){
            const int nj = gripper_joint_size_[gri_i];
            config_req_.payload.gripper_joint_size[gri_i] = static_cast<uint8_t>(nj);
            cmd_req_.payload.gripper_joint_size[gri_i] = static_cast<uint8_t>(nj);
            srv_state_->payload.gripper_joint_size[gri_i] = static_cast<uint8_t>(nj);
            config_req_.payload.gripper[gri_i].target_type = initial_target_type_;
            for (int joint_i = 0; joint_i < nj; joint_i++){
                if (config["robot"]["gripper"][gri_i]["safety"]
                    && config["robot"]["gripper"][gri_i]["safety"]["joint_soft_limit"]){
                    config_req_.payload.gripper[gri_i].soft_limit_position[joint_i][0] =
                        config["robot"]["gripper"][gri_i]["safety"]["joint_soft_limit"]["position"][0][joint_i].as<float>() * kDegToRad;
                    config_req_.payload.gripper[gri_i].soft_limit_position[joint_i][1] =
                        config["robot"]["gripper"][gri_i]["safety"]["joint_soft_limit"]["position"][1][joint_i].as<float>() * kDegToRad;
                }
            }
        }
    }

    std::shared_ptr<SrvState> snapshotState(){
        std::lock_guard<std::mutex> lock(state_mutex_);
        if (transmit_state_){
            srv_state_ = transmit_state_;
        }
        return srv_state_;
    }

    void receiveLoop(){
        while (!shutdown_.load()){
            const long t0 = robot::platform::get_time_now();
            SrvState temp{};
            if (mcast_ && mcast_->receive(temp, 50)){
                auto shared = std::make_shared<SrvState>(std::move(temp));
                {
                    std::lock_guard<std::mutex> lock(state_mutex_);
                    transmit_state_ = shared;
                    srv_state_ = shared;
                }
                first_state_received_ = true;
            }
            const long t1 = robot::platform::get_time_now();
            const long elapsed = t1 - t0;
            if (elapsed < static_cast<long>(receive_dt_us_)){
                robot::platform::sleep_period(static_cast<int>(receive_dt_us_ - static_cast<float>(elapsed)));
            }
        }
    }

    void heartbeatLoop(){
        while (!shutdown_.load()){
            if (monitoring_ && session_id_ != 0){
                SdkHeartbeatReq heartbeat{};
                heartbeat.client_id = client_id_;
                heartbeat.session_id = session_id_;
                heartbeat.timestamp_us = static_cast<uint64_t>(robot::platform::get_time_now());
                MonitoringRequestVariantPtr request =
                    std::make_unique<MonitoringRequestVariant>(heartbeat);
                try {
                    monitoring_->send(request);
                } catch (const std::exception& e){
                    std::cerr << "Spark2 heartbeat send failed: " << e.what() << std::endl;
                }
            }
            std::this_thread::sleep_for(
                std::chrono::duration<float, std::milli>(heartbeat_dt_ms_));
        }
    }

    void sendConfigAndWait(){
        std::lock_guard<std::mutex> lock(send_mutex_);
        if (!mcast_){
            throw std::runtime_error("Spark2::sendConfigAndWait failed: mcast client is null");
        }

        config_req_.client_id = client_id_;
        config_req_.session_id = session_id_;
        config_req_.payload.read_only = 0;

        const long t0 = robot::platform::get_time_now();
        long t1 = t0;
        bool success = false;
        while ((t1 - t0) < static_cast<long>(kConfigExitTimeoutMs * 1000.f)){
            config_req_.sequence_id = ++sequence_id_;
            config_req_.timestamp_us = static_cast<uint64_t>(robot::platform::get_time_now());
            auto request = std::make_unique<CoreRequestVariant>(config_req_);
            mcast_->send(std::move(request));

            CoreResponseVariantPtr ack = mcast_->waitAck(client_id_, sequence_id_, kConfigAckTimeoutMs);
            if (isConfigAckSuccess(ack)){
                last_config_res_ = std::get<SdkConfigRes>(*ack);
                have_config_res_ = true;
                robot::platform::updateConfigRequest(last_config_res_, config_req_);
                config_req_.payload.read_only = 1;
                success = true;
                break;
            }
            t1 = robot::platform::get_time_now();
        }
        if (!success){
            throw std::runtime_error("Spark2::sendConfigAndWait failed: no successful SdkConfigRes within timeout");
        }
    }

    void sendCommandAndWait(float ack_timeout_ms = kCommandAckTimeoutMs){
        std::lock_guard<std::mutex> lock(send_mutex_);
        if (!mcast_){
            throw std::runtime_error("Spark2::sendCommandAndWait failed: mcast client is null");
        }

        cmd_req_.client_id = client_id_;
        cmd_req_.session_id = session_id_;
        cmd_req_.payload.activated_control_strategy = config_req_.payload.arm[0].control_strategy;
        cmd_req_.sequence_id = ++sequence_id_;
        cmd_req_.timestamp_us = static_cast<uint64_t>(robot::platform::get_time_now());

        auto request = std::make_unique<CoreRequestVariant>(cmd_req_);
        mcast_->send(std::move(request));

        CoreResponseVariantPtr ack = mcast_->waitAck(client_id_, sequence_id_, ack_timeout_ms);
        if (!isCommandAckSuccess(ack)){
            std::string status = "missing or unexpected SdkCommandRes";
            if (ack && std::holds_alternative<SdkCommandRes>(*ack)){
                status = robot::platform::enumToString(std::get<SdkCommandRes>(*ack).payload.status);
            }
            throw std::runtime_error("Spark2::sendCommandAndWait failed: " + status);
        }
    }

    void applyArmMode(
        ControlStrategy strategy,
        ControlType target_type,
        ControlType actuator_mode,
        bool reset_interpolation = false,
        bool reset_control_mem = false,
        PlaybackState playback = PlaybackState::kStop,
        PlatformSmoothingMethod* smoothing = nullptr){
        bool need_config = false;
        for (int arm_i = 0; arm_i < arm_size_; arm_i++){
            auto& arm = config_req_.payload.arm[arm_i];
            if (arm.control_strategy != strategy
                || arm.target_type != target_type
                || arm.actuator_mode != actuator_mode
                || arm.playback_cmd != playback
                || (smoothing && arm.filter_type != *smoothing)
                || reset_interpolation
                || reset_control_mem){
                need_config = true;
            }
            arm.control_strategy = strategy;
            arm.target_type = target_type;
            arm.actuator_mode = actuator_mode;
            arm.playback_cmd = playback;
            if (smoothing){
                arm.filter_type = *smoothing;
            }
            arm.reset_interpolation = reset_interpolation ? 1 : 0;
            arm.reset_control_mem = reset_control_mem ? 1 : 0;
        }
        if (need_config){
            sendConfigAndWait();
            for (int arm_i = 0; arm_i < arm_size_; arm_i++){
                config_req_.payload.arm[arm_i].reset_interpolation = 0;
                config_req_.payload.arm[arm_i].reset_control_mem = 0;
            }
        }
        cmd_req_.payload.activated_control_strategy = strategy;
        cmd_req_.payload.enable_jog = 0;
    }

    /** Multi-waypoint Path APIs require QuinticPath; single-point APIs restore config default. */
    void applyPathArmMode(
        ControlStrategy strategy,
        ControlType target_type,
        ControlType actuator_mode,
        bool reset_control_mem = true){
        auto path_smoothing = PlatformSmoothingMethod::kQuinticPath;
        applyArmMode(
            strategy, target_type, actuator_mode,
            /*reset_interpolation=*/true,
            reset_control_mem,
            PlaybackState::kStop,
            &path_smoothing);
    }

    void applySinglePointArmMode(
        ControlStrategy strategy,
        ControlType target_type,
        ControlType actuator_mode,
        bool reset_interpolation = false,
        bool reset_control_mem = false,
        PlaybackState playback = PlaybackState::kStop){
        // QuinticPath cannot interpolate single targets; restore the configured default.
        applyArmMode(
            strategy, target_type, actuator_mode,
            reset_interpolation, reset_control_mem, playback,
            &default_smoothing_);
    }

    void fillHoldJointCommand(){
        auto state = snapshotState();
        cmd_req_.payload.target_count = 1;
        cmd_req_.payload.enable_jog = 0;
        auto& target = cmd_req_.payload.target[0];
        target.interpolation_t = 0.f;
        target.interpolation_speed_ratio = 0.f;
        for (int arm_i = 0; arm_i < arm_size_; arm_i++){
            for (int j = 0; j < arm_joint_size_[arm_i]; j++){
                target.arm_joint[arm_i][j] = state->payload.arm[arm_i].position[j];
            }
        }
        for (int gri_i = 0; gri_i < gripper_size_; gri_i++){
            for (int j = 0; j < gripper_joint_size_[gri_i]; j++){
                target.gripper_joint[gri_i][j] = state->payload.gripper[gri_i].position[j];
            }
        }
    }

    void fillPathTiming(int count, const Path<int>& v, const Path<float>& t){
        for (int i = 0; i < count; i++){
            cmd_req_.payload.target[i].interpolation_t = default_acc_t_;
            cmd_req_.payload.target[i].interpolation_speed_ratio = 0.f;
        }
        if (static_cast<int>(t.size()) == count){
            for (int i = 0; i < count; i++){
                cmd_req_.payload.target[i].interpolation_t = t[i];
            }
        }
        if (static_cast<int>(v.size()) == count){
            for (int i = 0; i < count; i++){
                cmd_req_.payload.target[i].interpolation_speed_ratio =
                    std::clamp(v[i] / 100.0f, 0.0f, 1.0f);
            }
        }
    }

    void shutdownTransport(){
        shutdown_.store(true);
        if (receive_thread_.joinable()){
            receive_thread_.join();
        }
        if (heartbeat_thread_.joinable()){
            heartbeat_thread_.join();
        }
        if (mcast_){
            mcast_->close();
            mcast_.reset();
        }
        if (monitoring_){
            monitoring_->close();
            monitoring_.reset();
        }
        first_state_received_ = false;
        started_ = false;
        session_id_ = 0;
    }
};

Spark2::Spark2(std::string config_prefix_path) : pimpl_(std::make_unique<Impl>()){
    pimpl_->config_prefix_path_ = config_prefix_path;
    const std::string config_path = config_prefix_path + "/config.yaml";
    YAML::Node config = robot::platform::loadYamlConfig(config_path);
    pimpl_->initialize(config);
    pimpl_->configurator_ = std::unique_ptr<Configurator>(new Configurator());
    pimpl_->kinematics_ = std::unique_ptr<Kinematics>(new Kinematics());
    pimpl_->kinematics_->pimpl_->initialize(config_prefix_path, config);

    pimpl_->configurator_->pimpl_->arm_joint_size_ =
        pimpl_->arm_joint_size_.empty() ? 6 : pimpl_->arm_joint_size_[0];
    pimpl_->configurator_->pimpl_->gripper_joint_size_ =
        pimpl_->gripper_joint_size_.empty() ? 1 : pimpl_->gripper_joint_size_[0];

    pimpl_->configurator_->pimpl_->registerCallback(
        [this]() -> SdkConfigReq& { return pimpl_->getConfigReq(); },
        [this]() -> const SdkConfigRes& { return pimpl_->getConfigRes(); },
        [this]() { pimpl_->sendConfigAndWait(); },
        [this]() { return pimpl_->isRobotStarted(); }
    );
}

Spark2::~Spark2(){
    try {
        stop();
    } catch (...) {}
    pimpl_->shutdownTransport();
    pimpl_->configurator_.reset();
    pimpl_->kinematics_.reset();
    pimpl_.reset();
}

void Spark2::start(){
    if (pimpl_->started_){
        return;
    }

    try {
        YAML::Node config = pimpl_->config_;
        std::filesystem::path keys_path;
        if (config["rt_control"]["keys_path"]){
            keys_path = std::filesystem::path(pimpl_->config_prefix_path_)
                / config["rt_control"]["keys_path"].as<std::string>();
        } else {
            // Default: configuration/keys.json next to config.yaml (SDK package; no rt_control tree).
            keys_path = std::filesystem::path(pimpl_->config_prefix_path_) / "keys.json";
        }
        keys_path = std::filesystem::weakly_canonical(keys_path);
        if (!robot::platform::loadClientKeys(keys_path.string())){
            throw std::runtime_error("Failed to load client keys from " + keys_path.string());
        }

        const int receive_ack_port = config["panel"]["port"].as<int>();
        const std::string configured_server_ip = config["rt_control"]["server_ip"]
            ? config["rt_control"]["server_ip"].as<std::string>() : std::string("auto");
        const int core_request_port = config["rt_control"]["core_request_port"].as<int>();
        const std::string multicast_ip = config["rt_control"]["multicast_ip"].as<std::string>();
        const int multicast_port = config["rt_control"]["multicast_port"].as<int>();
        const int monitoring_request_port = config["rt_control"]["monitoring_request_port"].as<int>();

        std::vector<std::string> probe_ips;
        const bool use_auto_discovery =
            configured_server_ip.empty()
            || configured_server_ip == "auto"
            || configured_server_ip == "discover";
        if (use_auto_discovery){
            probe_ips.push_back(multicast_ip);
            probe_ips.push_back("255.255.255.255");
            probe_ips.push_back("127.0.0.1");
        } else {
            probe_ips.push_back(configured_server_ip);
        }

        std::cout << "Discovering Robot..." << std::endl;
        const robot::platform::DiscoveredServer discovered =
            robot::platform::discoverRtServerViaHandshake(
                pimpl_->client_id_, core_request_port, receive_ack_port, probe_ips);
        pimpl_->session_id_ = discovered.session_id;
        const std::string server_ip = discovered.server_ip;

        pimpl_->shutdown_.store(false);
        pimpl_->mcast_ = std::make_unique<SdkMcastClient>(
            server_ip,
            core_request_port,
            multicast_ip,
            multicast_port,
            receive_ack_port,
            unpackSrvState,
            packCoreRequest,
            unpackCoreResponse,
            kAckPollMs);

        pimpl_->monitoring_ = std::make_unique<MonitoringUdp>(
            "",
            -1,
            server_ip,
            monitoring_request_port,
            nullptr,
            packMonitoringRequest);

        pimpl_->receive_thread_ = std::thread(&Impl::receiveLoop, pimpl_.get());
        pimpl_->heartbeat_thread_ = std::thread(&Impl::heartbeatLoop, pimpl_.get());

        std::cout << "Connecting to Robot at " << server_ip << "..." << std::endl;
        const auto connect_started = std::chrono::steady_clock::now();
        while (!pimpl_->first_state_received_.load()){
            if (std::chrono::steady_clock::now() - connect_started > std::chrono::seconds(30)){
                throw std::runtime_error("Timed out waiting for SrvState from robot");
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }

        pimpl_->config_req_.client_id = pimpl_->client_id_;
        pimpl_->config_req_.session_id = pimpl_->session_id_;
        pimpl_->cmd_req_.client_id = pimpl_->client_id_;
        pimpl_->cmd_req_.session_id = pimpl_->session_id_;
        pimpl_->config_req_.payload.read_only = 0;
        pimpl_->sendConfigAndWait();

        pimpl_->applyArmMode(
            ControlStrategy::kJoint,
            pimpl_->initial_target_type_,
            pimpl_->initial_actuator_mode_);
        pimpl_->fillHoldJointCommand();
        pimpl_->sendCommandAndWait();

        pimpl_->started_ = true;
        std::cout << "Successfully connected to Robot." << std::endl;
    } catch (const std::exception& e){
        pimpl_->shutdownTransport();
        throw std::runtime_error(std::string("Failed to start Spark2 SDK: ") + e.what());
    }
}

void Spark2::stop(){
    if (!pimpl_->started_){
        return;
    }
    pimpl_->started_ = false;
    pimpl_->shutdownTransport();
}

void Spark2::enableArmJoint(JointState6b arm_state){
    pimpl_->ensureRobotStarted();
    const int arm_i = 0;
    for (int joint_i = 0; joint_i < pimpl_->arm_joint_size_[arm_i]; joint_i++){
        pimpl_->config_req_.payload.arm[arm_i].enable_joint[joint_i] = arm_state[joint_i] ? 1 : 0;
    }
    pimpl_->sendConfigAndWait();
}

void Spark2::setArmSmoothingMethod(SmoothingMethod method){
    pimpl_->ensureRobotStarted();
    const auto platform_method = toPlatformSmoothing(method);
    pimpl_->default_smoothing_ = platform_method;
    for (int arm_i = 0; arm_i < pimpl_->arm_size_; arm_i++){
        pimpl_->config_req_.payload.arm[arm_i].filter_type = platform_method;
    }
    pimpl_->sendConfigAndWait();
}

void Spark2::movePos(const JointState6f& arm_pos, int v, float t){
    pimpl_->ensureRobotStarted();
    pimpl_->applySinglePointArmMode(
        ControlStrategy::kJoint,
        ControlType::kPosition,
        pimpl_->actuator_mode_under_position_control_);

    pimpl_->cmd_req_.payload.target_count = 1;
    pimpl_->cmd_req_.payload.enable_jog = 0;
    auto& target = pimpl_->cmd_req_.payload.target[0];
    target.interpolation_t = (t > 0.f) ? t : pimpl_->default_acc_t_;
    target.interpolation_speed_ratio = std::clamp(v / 100.0f, 0.0f, 1.0f);

    const int arm_i = 0;
    for (int j = 0; j < pimpl_->arm_joint_size_[arm_i]; j++){
        target.arm_joint[arm_i][j] = arm_pos[j] * kDegToRad;
    }
    pimpl_->sendCommandAndWait();
}

void Spark2::movePosPath(const Path<JointState6f>& arm_pos, Path<int> v, Path<float> t){
    pimpl_->ensureRobotStarted();
    pimpl_->validatePathSize("movePosPath", arm_pos, v, t);
    pimpl_->applyPathArmMode(
        ControlStrategy::kJoint,
        ControlType::kPosition,
        pimpl_->actuator_mode_under_position_control_,
        /*reset_control_mem=*/false);

    const int count = static_cast<int>(arm_pos.size());
    pimpl_->cmd_req_.payload.target_count = static_cast<uint8_t>(count);
    pimpl_->cmd_req_.payload.enable_jog = 0;
    const int arm_i = 0;
    for (int i = 0; i < count; i++){
        for (int j = 0; j < pimpl_->arm_joint_size_[arm_i]; j++){
            pimpl_->cmd_req_.payload.target[i].arm_joint[arm_i][j] = arm_pos[i][j] * kDegToRad;
        }
    }
    pimpl_->fillPathTiming(count, v, t);
    pimpl_->sendCommandAndWait();
}

void Spark2::moveVel(const JointState6f& arm_vel, int a, float t){
    pimpl_->ensureRobotStarted();
    pimpl_->applySinglePointArmMode(
        ControlStrategy::kJoint,
        ControlType::kVelocity,
        ControlType::kVelocity);

    pimpl_->cmd_req_.payload.target_count = 1;
    pimpl_->cmd_req_.payload.enable_jog = 0;
    auto& target = pimpl_->cmd_req_.payload.target[0];
    target.interpolation_t = t;
    target.interpolation_speed_ratio = std::clamp(a / 100.0f, 0.0f, 1.0f);

    const int arm_i = 0;
    for (int j = 0; j < pimpl_->arm_joint_size_[arm_i]; j++){
        target.arm_joint[arm_i][j] = arm_vel[j] * kDegToRad;
    }
    pimpl_->sendCommandAndWait();
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
    Path<Pose> tool_poses;
    tool_poses.reserve(arm_ee.size());
    Position tool_offset = pimpl_->configurator_->getToolOffset();
    for (const auto& ee : arm_ee){
        tool_poses.push_back(pimpl_->kinematics_->eeToToolPose(ee, tool_offset));
    }
    moveToolPointPath(tool_poses, std::move(v), std::move(t));
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
    Path<Pose> tool_poses;
    tool_poses.reserve(arm_ee.size());
    Position tool_offset = pimpl_->configurator_->getToolOffset();
    for (const auto& ee : arm_ee){
        tool_poses.push_back(pimpl_->kinematics_->eeToToolPose(ee, tool_offset));
    }
    moveToolLinePath(tool_poses, std::move(v), std::move(t));
}

void Spark2::moveToolPoint(const Pose& arm_tool, int v, float t){
    pimpl_->ensureRobotStarted();
    pimpl_->applySinglePointArmMode(
        ControlStrategy::kCartesian,
        ControlType::kPosition,
        pimpl_->actuator_mode_under_position_control_,
        /*reset_interpolation=*/false,
        /*reset_control_mem=*/true);

    pimpl_->cmd_req_.payload.target_count = 1;
    pimpl_->cmd_req_.payload.enable_jog = 0;
    auto& target = pimpl_->cmd_req_.payload.target[0];
    target.interpolation_t = (t > 0.f) ? t : pimpl_->default_acc_t_;
    target.interpolation_speed_ratio = std::clamp(v / 100.0f, 0.0f, 1.0f);
    fillCartesianFromPose(arm_tool, target.arm_tool_cartesian[0]);
    pimpl_->sendCommandAndWait();
}

void Spark2::moveToolPointPath(const Path<Pose>& arm_tool, Path<int> v, Path<float> t){
    pimpl_->ensureRobotStarted();
    pimpl_->validatePathSize("moveToolPointPath", arm_tool, v, t);
    pimpl_->applyPathArmMode(
        ControlStrategy::kCartesian,
        ControlType::kPosition,
        pimpl_->actuator_mode_under_position_control_);

    const int count = static_cast<int>(arm_tool.size());
    pimpl_->cmd_req_.payload.target_count = static_cast<uint8_t>(count);
    pimpl_->cmd_req_.payload.enable_jog = 0;
    for (int i = 0; i < count; i++){
        fillCartesianFromPose(arm_tool[i], pimpl_->cmd_req_.payload.target[i].arm_tool_cartesian[0]);
    }
    pimpl_->fillPathTiming(count, v, t);
    pimpl_->sendCommandAndWait();
}

void Spark2::moveToolLine(const Pose& arm_tool, int v, float t){
    pimpl_->ensureRobotStarted();
    pimpl_->applySinglePointArmMode(
        ControlStrategy::kCartesianLine,
        ControlType::kPosition,
        pimpl_->actuator_mode_under_position_control_,
        /*reset_interpolation=*/false,
        /*reset_control_mem=*/true);

    pimpl_->cmd_req_.payload.target_count = 1;
    pimpl_->cmd_req_.payload.enable_jog = 0;
    auto& target = pimpl_->cmd_req_.payload.target[0];
    target.interpolation_t = (t > 0.f) ? t : pimpl_->default_acc_t_;
    target.interpolation_speed_ratio = std::clamp(v / 100.0f, 0.0f, 1.0f);
    fillCartesianFromPose(arm_tool, target.arm_tool_cartesian[0]);
    pimpl_->sendCommandAndWait();
}

void Spark2::moveToolLinePath(const Path<Pose>& arm_tool, Path<int> v, Path<float> t){
    pimpl_->ensureRobotStarted();
    pimpl_->validatePathSize("moveToolLinePath", arm_tool, v, t);
    pimpl_->applyPathArmMode(
        ControlStrategy::kCartesianLine,
        ControlType::kPosition,
        pimpl_->actuator_mode_under_position_control_);

    const int count = static_cast<int>(arm_tool.size());
    pimpl_->cmd_req_.payload.target_count = static_cast<uint8_t>(count);
    pimpl_->cmd_req_.payload.enable_jog = 0;
    for (int i = 0; i < count; i++){
        fillCartesianFromPose(arm_tool[i], pimpl_->cmd_req_.payload.target[i].arm_tool_cartesian[0]);
    }
    pimpl_->fillPathTiming(count, v, t);
    pimpl_->sendCommandAndWait();
}

void Spark2::goHome(int v, float t){
    JointState6f home_pos{};
    movePos(home_pos, v, t);
}

void Spark2::moveGripperPos(const JointState1f& pos, int /*v*/, float /*t*/){
    pimpl_->ensureRobotStarted();
    pimpl_->cmd_req_.payload.target_count = 1;
    pimpl_->cmd_req_.payload.enable_jog = 0;
    auto& target = pimpl_->cmd_req_.payload.target[0];
    // Keep current arm joints while updating gripper.
    auto state = pimpl_->snapshotState();
    const int arm_i = 0;
    for (int j = 0; j < pimpl_->arm_joint_size_[arm_i]; j++){
        target.arm_joint[arm_i][j] = state->payload.arm[arm_i].position[j];
    }
    for (int gri_i = 0; gri_i < pimpl_->gripper_size_; gri_i++){
        for (int j = 0; j < pimpl_->gripper_joint_size_[gri_i]; j++){
            target.gripper_joint[gri_i][j] = pos[j];
        }
    }
    pimpl_->sendCommandAndWait();
}

void Spark2::startTeach(){
    pimpl_->ensureRobotStarted();
    // kRecord enables gravity-compensation behavior and RT-side trajectory recording.
    pimpl_->applySinglePointArmMode(
        ControlStrategy::kRecord,
        ControlType::kTorque,
        ControlType::kTorque);
    pimpl_->fillHoldJointCommand();
    pimpl_->sendCommandAndWait();
}

void Spark2::stopTeach(){
    pimpl_->ensureRobotStarted();
    pimpl_->applySinglePointArmMode(
        ControlStrategy::kJoint,
        ControlType::kPosition,
        pimpl_->actuator_mode_under_position_control_);
    pimpl_->fillHoldJointCommand();
    pimpl_->sendCommandAndWait();
}

void Spark2::startPlayback(){
    pimpl_->ensureRobotStarted();
    pimpl_->applySinglePointArmMode(
        ControlStrategy::kPlayback,
        ControlType::kPosition,
        pimpl_->actuator_mode_under_position_control_,
        /*reset_interpolation=*/false,
        /*reset_control_mem=*/false,
        PlaybackState::kStart);
    pimpl_->fillHoldJointCommand();
    pimpl_->cmd_req_.payload.target[0].interpolation_speed_ratio = 0.2f;
    pimpl_->sendCommandAndWait();
}

void Spark2::stopPlayback(){
    pimpl_->ensureRobotStarted();
    pimpl_->applySinglePointArmMode(
        ControlStrategy::kPlayback,
        ControlType::kPosition,
        pimpl_->actuator_mode_under_position_control_,
        false, false, PlaybackState::kStop);
    pimpl_->fillHoldJointCommand();
    pimpl_->sendCommandAndWait();
}

void Spark2::resetPlayback(){
    pimpl_->ensureRobotStarted();
    pimpl_->applySinglePointArmMode(
        ControlStrategy::kPlayback,
        ControlType::kPosition,
        pimpl_->actuator_mode_under_position_control_,
        false, false, PlaybackState::kReset);
    pimpl_->fillHoldJointCommand();
    pimpl_->sendCommandAndWait();
}

RobotJointStatef Spark2::getPos() const{
    pimpl_->ensureRobotStarted();
    auto state = pimpl_->snapshotState();
    RobotJointStatef out{};
    const int arm_i = 0;
    for (int i = 0; i < pimpl_->arm_joint_size_[arm_i]; i++){
        out.arm[i] = state->payload.arm[arm_i].position[i] * kRadToDeg;
    }
    if (pimpl_->gripper_size_ > 0){
        for (int i = 0; i < pimpl_->gripper_joint_size_[0]; i++){
            out.gripper[i] = state->payload.gripper[0].position[i];
        }
    }
    return out;
}

RobotJointStatef Spark2::getVel() const{
    pimpl_->ensureRobotStarted();
    auto state = pimpl_->snapshotState();
    RobotJointStatef out{};
    const int arm_i = 0;
    for (int i = 0; i < pimpl_->arm_joint_size_[arm_i]; i++){
        out.arm[i] = state->payload.arm[arm_i].velocity[i] * kRadToDeg;
    }
    return out;
}

RobotJointStatef Spark2::getTor() const{
    pimpl_->ensureRobotStarted();
    auto state = pimpl_->snapshotState();
    RobotJointStatef out{};
    const int arm_i = 0;
    for (int i = 0; i < pimpl_->arm_joint_size_[arm_i]; i++){
        out.arm[i] = state->payload.arm[arm_i].torque[i];
    }
    return out;
}

Pose Spark2::getEEPose() const{
    pimpl_->ensureRobotStarted();
    auto state = pimpl_->snapshotState();
    const int arm_i = 0;
    const int ee_idx = pimpl_->arm_joint_size_[arm_i] - 1;
    return poseFromCartesian(state->payload.arm[arm_i].joint_poses[ee_idx]);
}

Pose Spark2::getToolPose() const{
    pimpl_->ensureRobotStarted();
    auto state = pimpl_->snapshotState();
    return poseFromCartesian(state->payload.arm[0].tool_pose);
}

JointState6b Spark2::isArmJointEnabled() const{
    pimpl_->ensureRobotStarted();
    JointState6b enabled{};
    const int arm_i = 0;
    if (pimpl_->have_config_res_){
        for (int j = 0; j < pimpl_->arm_joint_size_[arm_i]; j++){
            enabled[j] = pimpl_->last_config_res_.payload.arm[arm_i].enabled_joint[j] != 0;
        }
    } else {
        for (int j = 0; j < pimpl_->arm_joint_size_[arm_i]; j++){
            enabled[j] = pimpl_->config_req_.payload.arm[arm_i].enable_joint[j] != 0;
        }
    }
    return enabled;
}

SystemStatus Spark2::getStatus() const{
    pimpl_->ensureRobotStarted();
    auto state = pimpl_->snapshotState();
    SystemStatus status{};
    status.robot_state = mapSystemState(state->payload.system_state);
    status.plan_result = static_cast<PlanResult>(state->payload.plan_result);
    status.robot_diagnostic_flags = state->payload.system_diagnostic_flags;
    const int arm_i = 0;
    for (int j = 0; j < pimpl_->arm_joint_size_[arm_i]; j++){
        status.arm_joint_diagnostic_flags[j] = state->payload.arm[arm_i].diagnostic_flags[j];
    }
    if (pimpl_->gripper_size_ > 0){
        for (int j = 0; j < pimpl_->gripper_joint_size_[0]; j++){
            status.gripper_joint_diagnostic_flags[j] = state->payload.gripper[0].diagnostic_flags[j];
        }
    }
    return status;
}

void Spark2::printStatus(const SystemStatus& status) const{
    std::cout << "Robot State: " << robotStateToString(status.robot_state) << std::endl;
    std::cout << "Plan Result: " << planResultToString(status.plan_result) << std::endl;
    std::cout << "Robot Diagnostic Flags: ";
    robot::platform::print_diagnostic_flags(status.robot_diagnostic_flags);

    std::cout << "Arm Joint Diagnostic Flags: ";
    bool has_flags = false;
    for (int j = 0; j < pimpl_->arm_joint_size_[0]; j++){
        if (status.arm_joint_diagnostic_flags[j] != DiagnosticFlags::kNone){
            std::cout << "Joint " << j << ": ";
            robot::platform::print_diagnostic_flags(status.arm_joint_diagnostic_flags[j]);
            has_flags = true;
        }
    }
    if (!has_flags){
        std::cout << "None\n";
    }

    if (pimpl_->gripper_size_ > 0){
        std::cout << "Gripper Joint Diagnostic Flags: ";
        has_flags = false;
        for (int j = 0; j < pimpl_->gripper_joint_size_[0]; j++){
            if (status.gripper_joint_diagnostic_flags[j] != DiagnosticFlags::kNone){
                std::cout << "Joint " << j << ": ";
                robot::platform::print_diagnostic_flags(status.gripper_joint_diagnostic_flags[j]);
                has_flags = true;
            }
        }
        if (!has_flags){
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

}  // namespace spark2
