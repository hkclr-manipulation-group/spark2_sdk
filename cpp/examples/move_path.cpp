#include "spark2.h"

#include <iostream>
#include <thread>
#include <chrono>

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
    
    prefix_text = "==> movePos (10.0, 20.0, 30.0, 40.0, 50.0, 60.0)\n";
    robot.movePos({10.0, 20.0, 30.0, 40.0, 50.0, 60.0}, 0, 5);
    printFeedback(robot, print_dt, timeout, &current_joint_pos, &current_pose, prefix_text);

    //----------------------------move tool in a straight line---------------------------------
    prefix_text = "==> Move Tool Line\n";
    Path<Pose> path1 = {
        {{0.351085, -0.12439, 0.410693}, {0.40558, 0.704416, -0.061629, 0.579228}},
        {{0.301085, -0.16439, 0.340693}, {0.40558, 0.704416, -0.061629, 0.579228}}
    };
    Path<int> v1 = {30, 30};
    robot.moveToolLinePath(path1, v1, {});
    printFeedback(robot, print_dt, timeout, &current_joint_pos, &current_pose, prefix_text);

    prefix_text = "==> Go Home\n";
    robot.goHome(50, 5);
    printFeedback(robot, print_dt, timeout, &current_joint_pos, &current_pose, prefix_text);

    //--------------------------------move tool from point to point---------------------------
    prefix_text = "==> Move Tool Point\n";
    Path<Pose> path2 = {
        {{0.431085, -0.09439, 0.410693}, {0.40558, 0.704416, -0.061629, 0.579228}},
        {{0.193, 0.045, 0.777}, {0.707107, 0, 0, 0.707107}}
    };
    Path<float> t2 = {5, 5};
    robot.moveToolPointPath(path2, {}, t2);
    printFeedback(robot, print_dt, timeout, &current_joint_pos, &current_pose, prefix_text);

    //--------------------------------move joint from point to point--------------------------------
    prefix_text = "==> Move Joint Pos\n";
    Path<JointState6f> path3 = {
        {-10.0, -20.0, -30.0, -10.0, -10.0, -10.0},
        {0.0, 0.0, 0.0, 0.0, 0.0, 0.0}
    };
    Path<float> t3 = {10, 10};
    robot.movePosPath(path3, {}, t3);
    printFeedback(robot, print_dt, timeout, &current_joint_pos, &current_pose, prefix_text);

    std::cout << "Motion completed" << std::endl;
    robot.stop();
    return 0;
}