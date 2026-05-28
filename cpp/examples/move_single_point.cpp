#include "spark.h"

#include <iostream>
#include <thread>
#include <chrono>

using namespace spark;

void printFeedback(const Spark& arm, float dt, float timeout, RobotJointStatef* joint_pos=nullptr, Pose* tool_pose=nullptr, std::string prefix_text=""){
    int elapsed_time_ms = 0;
    int dt_ms = static_cast<int>(dt * 1000);
    bool is_latest_target_received = false;
    bool is_idle = false;
    bool is_interrupted = false;
    SystemStatus sys_status;
    while (elapsed_time_ms <= timeout * 1000){
        std::cout << prefix_text;
        std::cout <<"Elapsed time: " << elapsed_time_ms / 1000.0f << "s\n";

        //Print current joint position
        if (joint_pos != nullptr){
            *joint_pos = arm.getPos();
            std::cout << "Current joint position";
            std::cout << "\n\tArm: ";
            for (int i=0; i<6; i++){
                std::cout << joint_pos->arm[i] << " ";
            }
            std::cout << ", Gripper: ";
            for (int i=0; i<1; i++){
                std::cout << joint_pos->gripper[i] << " ";
            }
            std::cout << std::endl;
        }

        //Print current tool pose
        if (tool_pose != nullptr){
            *tool_pose = arm.getToolPose();
            std::cout << "Current tool pose"
                    <<"\n\tposition(x, y, z): " << tool_pose->position.x << ", " << tool_pose->position.y << ", " << tool_pose->position.z << ","
                    <<"\n\torientation(w, x, y, z): " 
                    << tool_pose->orientation.w << ", " << tool_pose->orientation.x << ", " << tool_pose->orientation.y << ", " << tool_pose->orientation.z << std::endl;
        }
        
        //Print status
        arm.printStatus();
        sys_status = arm.getStatus();
        is_latest_target_received = arm.isLatestTargetReceived();
        is_idle = sys_status.robot_state == spark::RobotState::kIdle;
        is_interrupted = sys_status.plan_result != spark::PlanResult::kSuccess;
        std::cout <<"---------------------------------------------------------------\n";
        if (elapsed_time_ms == timeout * 1000 || (is_latest_target_received && is_idle)) break;
        std::this_thread::sleep_for(std::chrono::milliseconds(dt_ms));
        elapsed_time_ms += dt_ms;
    }
    std::cout << "is_latest_target_received: " << is_latest_target_received << std::endl;
    std::cout << "is_idle: " << is_idle << std::endl;
    if (is_interrupted) std::cout << "IS INTERRUPTED!!!" << std::endl;
    std::cout <<"===============================================================\n";
    std::this_thread::sleep_for(std::chrono::seconds(1));
}

int main(int argc, char *argv[]){ 
    std::string config_prefix_path = CONFIG_PREFIX_PATH;
    std::cout << "Config prefix path: " << config_prefix_path << std::endl;
    Spark arm(config_prefix_path);
    arm.start();
    arm.enableArmJoint({true, true, true, true, true, true});
    std::cout << "Arm started" << std::endl;

    int v = 0; //From 0 to 100
    float t = 3; // Override v if t is not 0
    std::string prefix_text = "";
    Pose current_pose;
    RobotJointStatef current_joint_pos;
    
    //--------------------------------movePos--------------------------------
    prefix_text = "==> movePos (10.0, 20.0, 30.0, 40.0, 50.0, 60.0)\n";
    arm.movePos({10.0, 20.0, 30.0, 40.0, 50.0, 60.0}, v, t);
    printFeedback(arm, 0.3, 15, &current_joint_pos, &current_pose, prefix_text);

    //--------------------------------moveToolPoint--------------------------------
    prefix_text = "==> moveToolPoint (position: (0.431085, -0.09439, 0.410693), orientation: (0.40558, 0.704416, -0.061629, 0.579228))\n";
    arm.moveToolPoint({{0.431085, -0.09439, 0.410693}, {0.40558, 0.704416, -0.061629, 0.579228}}, v, t);
    printFeedback(arm, 0.3, 15, &current_joint_pos, &current_pose, prefix_text);

    //--------------------------------moveToolLine--------------------------------
    prefix_text = "==> movePos (10.0, 20.0, 30.0, 40.0, 50.0, 60.0)\n";
    arm.movePos({10.0, 20.0, 30.0, 40.0, 50.0, 60.0}, 30, 0);
    printFeedback(arm, 0.3, 15, &current_joint_pos, &current_pose, prefix_text);
    
    prefix_text = "==> moveToolLine (position: (0.431085, -0.09439, 0.410693), orientation: (0.40558, 0.704416, -0.061629, 0.579228))\n";
    arm.moveToolLine({{0.431085, -0.09439, 0.410693}, {0.40558, 0.704416, -0.061629, 0.579228}}, 30, 0);
    printFeedback(arm, 0.3, 15, &current_joint_pos, &current_pose, prefix_text);

    //--------------------------------goHome--------------------------------
    std::cout <<"==> Go to home position (0.0, 0.0, 0.0, 0.0, 0.0, 0.0)" << std::endl;
    arm.goHome(v, t);
    printFeedback(arm, 0.3, 15, &current_joint_pos, &current_pose, prefix_text);
    
    std::cout << "Motion completed" << std::endl;
    arm.stop();
    return 0;
}