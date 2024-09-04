#include "tocabi_controller/syhand_controller.h"
#include <iostream>

void print_array(float* arr, int len){
    for(int i = 0; i < len; i++){
        std::cout << arr[i] << " ";
    }
    std::cout << std::endl;
}

SYhandController::SYhandController(ros::NodeHandle nh_){
    std::cout << "initiate sy hand controller" << std::endl;
    
    // hand_state_pub = nh_.advertise<sensor_msgs::JointState>("/tocabi/handstates", 1);
    hand_open_sub = nh_.subscribe<std_msgs::Int32>("/mujoco_ros_interface/hand_open", 1, &SYhandController::hand_open_callback, this);

    // hand_state_msgs.name.resize(HAND_DOF);
    // for(int i = 0; i < HAND_DOF; i++){
    //     hand_state_msgs.name[i] = hand_joint_name[i];
    // }
    // hand_state_msgs.position.resize(HAND_DOF);
    // hand_state_msgs.velocity.resize(HAND_DOF);
    // hand_state_msgs.effort.resize(HAND_DOF);
}

void *SYhandController::PubThread(){
    int pub_count = 0;
    while (true){
        pub_count++;
        if (pub_count % 16 == 0){
            hand_state_msgs.header.stamp = ros::Time::now();
            for(int i = 0; i < HAND_DOF; i++){
                hand_state_msgs.position[i] = shm_msgs_->hand_pos[i];
                hand_state_msgs.velocity[i] = shm_msgs_->hand_vel[i];
                hand_state_msgs.effort[i] = shm_msgs_->hand_acc[i];
            }
            hand_state_pub.publish(hand_state_msgs);
        }
    }
}

void SYhandController::oneFingerFlexionCalculation(float distance, float flex[]) {
    // Constants
    const float l_p = 0.028;
    const float h = 0.0025;
    const float l_1 = 0.01686;
    const float l_2 = 0.02638;
    const float l_3 = 0.00638;
    const float l_4 = 0.03500;
    const float l_5 = 0.04000;
    const float l_6 = 0.00550;
    const float l_7 = 0.01000;
    const float l_8 = 0.03252;
    const float l_9 = 0.03420;
    const float l_10 = 0.01356;

    // Angles in radians
    const float alpha_prime = deg2rad(10.9);
    const float gamma_0 = deg2rad(37.8);
    const float zeta_prime = deg2rad(31.9);
    const float kappa_0 = deg2rad(24.1);

    // Length of the sliding screw
    float d = distance;

    float s_1 = std::sqrt(std::pow(d, 2) + std::pow(h, 2));
    float alpha = std::acos((std::pow(l_1, 2) + std::pow(l_2, 2) - std::pow(s_1, 2)) / (2 * l_1 * l_2));
    float beta = alpha + alpha_prime;
    float s_2 = std::sqrt(std::pow(l_3, 2) + std::pow(l_4, 2) - 2 * l_3 * l_4 * std::cos(beta));
    float gamma = std::acos((std::pow(l_5, 2) + std::pow(l_6, 2) - std::pow(s_2, 2)) / (2 * l_5 * l_6));
    float zeta = std::acos((std::pow(l_4, 2) + std::pow(s_2, 2) - std::pow(l_3, 2)) / (2 * l_4 * s_2))     
                    - std::acos((std::pow(l_6, 2) + std::pow(s_2, 2) - std::pow(l_5, 2)) / (2 * l_6 * s_2));
    float l_hk1 = std::sqrt(std::pow(l_6, 2) + std::pow(l_4, 2) - 2 * l_6 * l_4 * std::cos(zeta));
    float theta_hk1 = std::acos((std::pow(l_5, 2) + std::pow(l_3, 2) - std::pow(l_hk1, 2)) / (2 * l_5 * l_3));
    float eta = zeta + zeta_prime;
    float s_3 = std::sqrt(std::pow(l_7, 2) + std::pow(l_8, 2) - 2 * l_7 * l_8 * std::cos(eta));
    float kappa = std::acos((std::pow(l_10, 2) + std::pow(s_3, 2) - std::pow(l_9, 2)) / (2 * l_10 * s_3));
    float s_4 = std::sqrt(std::pow(l_7, 2) + std::pow(l_6, 2) - 2 * l_7 * l_6 * std::cos(eta));
    float kappa_2 = std::acos((std::pow(s_3, 2) + std::pow(l_p, 2) - std::pow(s_4, 2)) / (2 * s_3 * l_p));

    float theta_mcp = deg2rad(90) - (theta_hk1 - deg2rad(25));
    float theta_pip = gamma - gamma_0;
    float theta_dip = kappa + kappa_2 - kappa_0;

    // Heuristic compensation
    theta_mcp -= deg2rad(30);
    theta_pip += deg2rad(15);
    theta_dip -= deg2rad(50);

    flex[0] = theta_mcp;
    flex[1] = theta_pip;
    flex[2] = theta_dip;
    flex[3] = 0.8 * theta_mcp;
}

void SYhandController::kinematicsCalculation_RHand(float actuator_values[], float joint_values[]) {
    float DA11 = actuator_values[0];
    float DA12 = actuator_values[1];
    float DA21 = actuator_values[2];
    float DA22 = actuator_values[3];
    float DA31 = actuator_values[4];
    float DA32 = actuator_values[5];
    float DA41 = actuator_values[6];
    float DA42 = actuator_values[7];

    float finger_1_flex[4];
    oneFingerFlexionCalculation(DA12, finger_1_flex);
    joint_values[0] = DA11;
    std::copy(finger_1_flex, finger_1_flex+3, joint_values+1);

    float finger_2_flex[4];
    oneFingerFlexionCalculation(DA22, finger_2_flex);
    joint_values[4] = DA21;
    std::copy(finger_2_flex, finger_2_flex+3, joint_values+5);

    float finger_3_flex[4];
    oneFingerFlexionCalculation(DA32, finger_3_flex);
    joint_values[8] = DA31;
    std::copy(finger_3_flex, finger_3_flex+3, joint_values+9);

    float finger_4_flex[4];
    oneFingerFlexionCalculation(DA42, finger_4_flex);
    joint_values[12] = DA41;
    std::copy(finger_4_flex, finger_4_flex+3, joint_values+13);

    float finger_act[] = {finger_1_flex[3], finger_2_flex[3], finger_3_flex[3], finger_4_flex[3]};
    std::copy(finger_act, finger_act+4, joint_values+16);
}

void SYhandController::hand_open_callback(const std_msgs::Int32ConstPtr &msg){
    std::cout << (msg->data ? "close" : "open") << std::endl;

    float sin_d = 0.027 + 0.011*msg->data;
    float aa_value = 0.3*msg->data;

    std::cout << sin_d << " " << aa_value << std::endl;
    float Actuator_values[] = {0.0, sin_d, aa_value*2, sin_d, 0.0, sin_d, 0.0, sin_d};
    // float Actuator_values[] = {aa_value*2, sin_d, aa_value*2, sin_d, 0.0, sin_d, -aa_value*2, sin_d};
    float hand_command[HAND_DOF];
    kinematicsCalculation_RHand(Actuator_values, hand_command);

    print_array(hand_command, HAND_DOF);

    float hand_init[HAND_DOF];
    std::copy(shm_msgs_->handCommand, shm_msgs_->handCommand + HAND_DOF, hand_init);
    ros::Time init = ros::Time::now();
    double t;
    do{
        t = (ros::Time::now()-init).toSec();
        for(int i = 0; i < HAND_DOF; i++){
            shm_msgs_->handCommand[i] = DyrosMath::cubic(t, 0.0, 0.5, hand_init[i], hand_command[i], 0.0, 0.0);
        }
    }
    while(t < 0.5);
}

int main(int argc, char **argv)
{
    // :: ROS CUSTUM :: initialize ros
    ros::init(argc, argv, "syhand_controller");
    ros::NodeHandle nh("~");

    SYhandController syhand_controller(nh);
    int shm_msg_id;
    init_shm(shm_msg_key, shm_msg_id, &syhand_controller.shm_msgs_);

    // pthread_t pubThread;
    // pthread_attr_t pubattrs;
    // struct sched_param param_pub;
    // param_pub.sched_priority = 41 + 50;

    // pthread_attr_init(&pubattrs);
    // if (pthread_attr_setschedpolicy(&pubattrs, SCHED_FIFO)){
    //     printf("attr pub setschedpolicy failed ");
    // }
    // if (pthread_attr_setschedparam(&pubattrs, &param_pub)){
    //     printf("attr pub setschedparam failed ");
    // }
    // if (pthread_attr_setinheritsched(&pubattrs, PTHREAD_EXPLICIT_SCHED)){
    //     printf("attr pub setinheritsched failed ");
    // }

    // auto error_code = pthread_create(&pubThread, &pubattrs, &SYhandController::PubStarter, &syhand_controller);
    // if (error_code){
    //     std::cout << error_code << ": ";
    //     printf("pub Thread create failed\n");
    // }
    // pthread_join(pubThread, NULL);

    ros::spin();

    deleteSharedMemory(shm_msg_id, syhand_controller.shm_msgs_);
}