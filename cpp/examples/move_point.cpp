#include "spark2.h"

#include <iostream>
#include <thread>
#include <chrono>

using namespace spark2;

void printFeedback(const Spark2& arm, float dt, float timeout, RobotJointStatef* joint_pos=nullptr, Pose* tool_pose=nullptr, std::string prefix_text=""){
    int elapsed_time_ms = 0;
    int dt_ms = static_cast<int>(dt * 1000);
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
        sys_status = arm.getStatus();
        arm.printStatus(sys_status);
        is_idle = sys_status.robot_state == spark2::RobotState::kIdle;
        is_interrupted = sys_status.plan_result != spark2::PlanResult::kSuccess;
        std::cout <<"---------------------------------------------------------------\n";
        if (elapsed_time_ms == timeout * 1000 || is_idle) break;
        std::this_thread::sleep_for(std::chrono::milliseconds(dt_ms));
        elapsed_time_ms += dt_ms;
    }
    std::cout << "is_idle: " << is_idle << std::endl;
    if (is_interrupted) std::cout << "IS INTERRUPTED!!!" << std::endl;
    std::cout <<"===============================================================\n";
    std::this_thread::sleep_for(std::chrono::seconds(1));
}

int main(int argc, char *argv[]){ 
    std::string config_prefix_path = CONFIG_PREFIX_PATH;
    std::cout << "Config prefix path: " << config_prefix_path << std::endl;
    Spark2 arm(config_prefix_path);
    arm.start();
    arm.enableArmJoint({true, true, true, true, true, true});
    std::cout << "Arm started" << std::endl;

    int v = 30; //From 0 to 100
    float t = 5;
    float print_dt = 0.3;
    float timeout = 50;
    std::string prefix_text = "";
    Pose current_pose;
    RobotJointStatef current_joint_pos;
    
    //--------------------move joint from point to point---------------------
    prefix_text = "==> movePos (10.0, 20.0, 30.0, 40.0, 50.0, 60.0)\n";
    arm.movePos({10.0, 20.0, 30.0, 40.0, 50.0, 60.0}, v, 0);
    printFeedback(arm, print_dt, timeout, &current_joint_pos, &current_pose, prefix_text);

    //--------------------move tool from point to point----------------------
    prefix_text = "==> moveToolPoint (position: (0.431085, -0.09439, 0.410693), orientation: (0.40558, 0.704416, -0.061629, 0.579228))\n";
    arm.moveToolPoint({{0.431085, -0.09439, 0.410693}, {0.40558, 0.704416, -0.061629, 0.579228}}, v, 0);
    printFeedback(arm, print_dt, timeout, &current_joint_pos, &current_pose, prefix_text);

    //--------------------move tool in a straight line-----------------------
    prefix_text = "==> movePos (10.0, 20.0, 30.0, 40.0, 50.0, 60.0)\n";
    arm.movePos({10.0, 20.0, 30.0, 40.0, 50.0, 60.0}, v, 0);
    printFeedback(arm, print_dt, timeout, &current_joint_pos, &current_pose, prefix_text);
    
    prefix_text = "==> moveToolLine (position: (0.431085, -0.09439, 0.410693), orientation: (0.40558, 0.704416, -0.061629, 0.579228))\n";
    arm.moveToolLine({{0.431085, -0.09439, 0.410693}, {0.40558, 0.704416, -0.061629, 0.579228}}, 0, t);
    printFeedback(arm, print_dt, timeout, &current_joint_pos, &current_pose, prefix_text);

    //--------------------------------goHome--------------------------------
    std::cout <<"==> Go to home position (0.0, 0.0, 0.0, 0.0, 0.0, 0.0)" << std::endl;
    arm.goHome(20, t); //t would extend if leading joint exceeded v
    printFeedback(arm, print_dt, timeout, &current_joint_pos, &current_pose, prefix_text);
    
    std::cout << "Motion completed" << std::endl;
    arm.stop();
    return 0;
}