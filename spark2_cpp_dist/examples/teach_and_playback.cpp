#include "spark2.h"

#include <iostream>
#include <thread>
#include <chrono>

#if defined(_WIN32)
    #include <conio.h>   // Native Windows console input
#else
    #include <termios.h> // Linux terminal control
    #include <unistd.h>  // Linux system calls
#endif


using namespace spark2;

void printFeedback(const Spark2& robot, float dt, float timeout, RobotJointStatef* joint_pos=nullptr, Pose* tool_pose=nullptr, std::string prefix_text=""){
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
            *joint_pos = robot.getPos();
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
            *tool_pose = robot.getToolPose();
            std::cout << "Current tool pose"
                    <<"\n\tposition(x, y, z): " << tool_pose->position.x << ", " << tool_pose->position.y << ", " << tool_pose->position.z << ","
                    <<"\n\torientation(w, x, y, z): " 
                    << tool_pose->orientation.w << ", " << tool_pose->orientation.x << ", " << tool_pose->orientation.y << ", " << tool_pose->orientation.z << std::endl;
        }
        
        //Print status
        sys_status = robot.getStatus();
        robot.printStatus(sys_status);
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

int waitForKeyPress() {
#if defined(_WIN32)
    return _getch(); 
#else
    struct termios oldt, newt;
    int ch;
    tcgetattr(STDIN_FILENO, &oldt);
    newt = oldt;
    newt.c_lflag &= ~(ICANON | ECHO);          
    tcsetattr(STDIN_FILENO, TCSANOW, &newt);
    ch = getchar();
    tcsetattr(STDIN_FILENO, TCSANOW, &oldt); 
    return ch;
#endif
}

int main(int argc, char *argv[]){ 
    std::string config_prefix_path = CONFIG_PREFIX_PATH;
    std::cout << "Config prefix path: " << config_prefix_path << std::endl;
    Spark2 robot(config_prefix_path);
    robot.start();
    robot.enableArmJoint({true, true, true, true, true, true});
    std::cout << "Arm started" << std::endl;

    float print_dt = 0.3;
    float timeout = 50;
    std::string prefix_text = "";
    Pose current_pose;
    RobotJointStatef current_joint_pos;

    //-------------------------Start Teach---------------------------------
    std::cout <<"Press any key to START TEACH...\n";
    waitForKeyPress();
    robot.startTeach();

    //-------------------------Stop Teach----------------------------------
    std::cout <<"Press any key to STOP TEACH...\n";
    waitForKeyPress();
    robot.stopTeach();

    //----------------------------Reset Playback---------------------------
    std::cout <<"Press any key to MOVE TO INITIAL POSITION of playback...\n";
    waitForKeyPress();
    robot.resetPlayback();
    prefix_text = "==> Go to initial position of playback.\n";
    printFeedback(robot, print_dt, timeout, &current_joint_pos, nullptr, prefix_text);

    //----------------------------Start Playback---------------------------
    std::cout <<"Press any key to START REPLAY...\n";
    waitForKeyPress();
    robot.startPlayback();

    //----------------------------Stop Playback---------------------------
    std::cout <<"Press any key to STOP REPLAY...\n";
    waitForKeyPress();
    robot.stopPlayback();

    //----------------------------Go Home-------------------------------
    std::cout <<"Press any key to GO HOME...\n";
    waitForKeyPress();
    robot.goHome(30, 0);
    prefix_text = "==> Go home.\n";
    printFeedback(robot, print_dt, timeout, &current_joint_pos, nullptr, prefix_text);

    std::cout << "Motion completed" << std::endl;
    robot.stop();
    return 0;
}