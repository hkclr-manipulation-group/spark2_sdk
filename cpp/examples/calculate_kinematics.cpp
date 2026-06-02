#include <iostream>

#include "spark2.h"
#include "kinematics.h"

using namespace spark2;

int main(int argc, char *argv[]){ 
    std::string config_prefix_path = CONFIG_PREFIX_PATH;
    std::cout << "Config prefix path: " << config_prefix_path << std::endl;
    Spark2 robot(config_prefix_path);
    const Kinematics& kinematics = robot.getKinematics();

    JointState6f joint_pos = {10.0, 10.0, 10.0, 0.0, 0.0, 0.0}; //Initial joint position(Degree)
    Pose ee_pose = {{0.431085, -0.09439, 0.410693}, {0.40558, 0.704416, -0.061629, 0.579228}};
    bool success = kinematics.inverseKinematics(ee_pose, joint_pos);
    Pose pose1 = kinematics.forwardKinematics(joint_pos);


    //--------------------------- Print IK and FK result --------------------------------
    std::cout <<"IK Target\n\tposition(x, y, z): " << ee_pose.position.x << ", " << ee_pose.position.y << ", " << ee_pose.position.z << std::endl;
    std::cout <<"\torientation(w, x, y, z): " << ee_pose.orientation.w << ", " << ee_pose.orientation.x << ", " << ee_pose.orientation.y << ", " << ee_pose.orientation.z << std::endl;
    if (success){
        std::cout << "IK result: " << joint_pos[0] << ", " << joint_pos[1] << ", " 
        << joint_pos[2] << ", " << joint_pos[3] << ", " << joint_pos[4] << ", " << joint_pos[5] << std::endl;
    }
    std::cout << "--------------------------------------------------------\n";

    std::cout << "FK result\n\tposition(x, y, z): " << pose1.position.x << ", " << pose1.position.y << ", " << pose1.position.z << std::endl;
    std::cout << "\torientation(w, x, y, z): " << pose1.orientation.w << ", " << pose1.orientation.x << ", " << pose1.orientation.y << ", " << pose1.orientation.z << std::endl;
    
    return 0;
}