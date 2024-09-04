#ifndef SYHAND_CONTROLLER_H
#define SYHAND_CONTROLLER_H

#include "shm_msgs.h"
#include "math_type_define.h"
#include <cmath>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Int32.h>

class SYhandController {
public:
    SYhandController(ros::NodeHandle nh_);

    void *PubThread();
    static void *PubStarter(void *context) { return ((SYhandController *)context)->PubThread(); }

    void oneFingerFlexionCalculation(float distance, float flex[]);

    void kinematicsCalculation_RHand(float actuator_values[], float joint_values[]);

    void hand_open_callback(const std_msgs::Int32ConstPtr &msg);

    float deg2rad(float degrees) { return degrees * M_PI / 180.0; }


    ros::Publisher hand_state_pub;
    ros::Subscriber hand_open_sub;

    SHMmsgs *shm_msgs_;    
    sensor_msgs::JointState hand_state_msgs;
    
    const std::string hand_joint_name[HAND_DOF] = {
        "aa2" , "mcp2", "pip2", "dip2", "act2",
        "aa1" , "mcp1", "pip1", "dip1", "act1",
        "aa3" , "mcp3", "pip3", "dip3", "act3",
        "aa4" , "mcp4", "pip4", "dip4", "act4"};
};

#endif
