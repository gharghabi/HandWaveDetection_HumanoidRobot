
#include "skeletonclient.h"



int main(int argc, char **argv)
{
    ros::init(argc, argv, "skeleton_client");
    std::cout<<"skeleton client started"<<std::endl;

    ros::Time::init();
    ros::Rate loop_rate(20);
    
    skeletonclient Client;

Client.gesture_detect_enable();
Client.get_position_orientation_enable();


// std::string get_right_hand_posture();//get right hand posture
//         std::string get_left_hand_posture();//get left hand posture
//         std::string get_right_leg_posture();//get right foot posture
//         std::string get_left_leg_posture();//ger left foot posture
//         geometry_msgs::Vector3 get_joint_position(std::string JointName);
//         geometry_msgs::Quaternion get_joint_orientation(std::string JointName);

    while (ros::ok())
    {
        ros::spinOnce();
        loop_rate.sleep();
        std::cout<<Client.get_joint_position("righ_shoulder").x<<std::endl;
         std::cout<<Client.get_joint_position("left_shoulder").y<<std::endl;
        std::cout<<Client.get_joint_position("torso").z<<std::endl;

        // std::cout<<"right_hand"<<Client.get_RH_posture()<<std::endl;
        // std::cout<<"left_hand"<<Client.get_LH_posture()<<std::endl;
        // std::cout<<"right_foot"<<Client.get_RF_posture()<<std::endl;
        // std::cout<<"left_foot"<<Client.get_LF_posture()<<std::endl;
    }
    return 0;
}

