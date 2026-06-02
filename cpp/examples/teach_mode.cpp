#include "spark2.h"

#include <iostream>
#include <thread>
#include <chrono>
#include <termios.h>
#include <unistd.h>

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

int getch() {
    struct termios oldt, newt;
    int ch;
    tcgetattr(STDIN_FILENO, &oldt);
    newt = oldt;
    newt.c_lflag &= ~(ICANON | ECHO);          
    tcsetattr(STDIN_FILENO, TCSANOW, &newt);
    ch = getchar();
    tcsetattr(STDIN_FILENO, TCSANOW, &oldt); 
    return ch;
}

int main(int argc, char *argv[]){ 
    std::string config_prefix_path = CONFIG_PREFIX_PATH;
    std::cout << "Config prefix path: " << config_prefix_path << std::endl;
    Spark2 arm(config_prefix_path);
    arm.start();
    arm.enableArmJoint({true, true, true, true, true, true});
    std::cout << "Arm started" << std::endl;

    float print_dt = 0.3;
    float timeout = 15;
    std::string prefix_text = "";
    Pose current_pose;
    RobotJointStatef current_joint_pos;

    //-------------------------Start Teach---------------------------------
    prefix_text = "==> You can freely move the robot arm to any pose.\n";
    arm.startTeach();
    std::cout <<"Press any key to stop...";
    getch();
    std::cout <<"Key pressed. Hand off the robot now.\n";
    std::this_thread::sleep_for(std::chrono::milliseconds(500)); //Hand off reaction time

    //-------------------------Stop Teach----------------------------------
    std::cout <<"Stop teach mode and go home.\n";
    arm.stopTeach();
    
    //----------------------------Go Home----------------------------------
    prefix_text = "==> Go home.\n";
    arm.goHome();
    printFeedback(arm, print_dt, timeout, &current_joint_pos, nullptr, prefix_text);

    std::cout << "Motion completed" << std::endl;
    arm.stop();
    return 0;
}