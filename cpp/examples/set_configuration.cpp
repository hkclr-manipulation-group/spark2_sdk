#include <iostream>
#include <thread>
#include <chrono>

#include "spark2.h"
#include "configurator.h"
#include "kinematics.h"

using namespace spark2;

int main(int argc, char *argv[]){ 
    std::string config_prefix_path = CONFIG_PREFIX_PATH;
    std::cout << "Config prefix path: " << config_prefix_path << std::endl;
    Spark2 robot(config_prefix_path);
    robot.start();
    std::cout << "Arm started" << std::endl;

    Configurator& configurator = robot.getConfigurator();
    Kinematics& kinematics = robot.getKinematics();

    Position tool_offset = configurator.getToolOffset();
    Pose tool_pose = robot.getToolPose();
    std::cout << "Before setToolOffset(x, y, z): " << tool_offset.x << ", " << tool_offset.y << ", " << tool_offset.z << std::endl;
    std::cout << "Before Tool pose"
            <<"\n\tposition(x, y, z): " << tool_pose.position.x << ", " << tool_pose.position.y << ", " << tool_pose.position.z << ","
            <<"\n\torientation(w, x, y, z): " 
            << tool_pose.orientation.w << ", " << tool_pose.orientation.x << ", " << tool_pose.orientation.y << ", " << tool_pose.orientation.z << std::endl;
    std::cout << "--------------------------------------------------------\n";


    configurator.setToolOffset({0.01, 0.02, 0.03});
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    Position tool_offset1 = configurator.getToolOffset();
    Pose tool_pose1 = robot.getToolPose();
    std::cout << "After setToolOffset(x, y, z): " << tool_offset1.x << ", " << tool_offset1.y << ", " << tool_offset1.z << std::endl;
    std::cout << "After Tool pose"
            <<"\n\tposition(x, y, z): " << tool_pose1.position.x << ", " << tool_pose1.position.y << ", " << tool_pose1.position.z << ","
            <<"\n\torientation(w, x, y, z): " 
            << tool_pose1.orientation.w << ", " << tool_pose1.orientation.x << ", " << tool_pose1.orientation.y << ", " << tool_pose1.orientation.z << std::endl;
    std::cout << "--------------------------------------------------------\n";
    

    Pose tool_pose2 = kinematics.eeToToolPose(tool_pose, tool_offset1);
    std::cout << "Verification"
            <<"\n\tposition(x, y, z): " << tool_pose2.position.x << ", " << tool_pose2.position.y << ", " << tool_pose2.position.z << ","
            <<"\n\torientation(w, x, y, z): " 
            << tool_pose2.orientation.w << ", " << tool_pose2.orientation.x << ", " << tool_pose2.orientation.y << ", " << tool_pose2.orientation.z << std::endl;
    return 0;
}