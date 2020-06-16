#include "ros/ros.h"
#include <boost/thread.hpp>
#include "upperbodycore_msgs/Skeleton.h"
#include "upperbodycore_msgs/skeletonSrv.h"
#include <upperbodycore_msgs/jointPO.h>
#include <upperbodycore_msgs/GestureDetect.h>
#include "skeletonclient.h"

using namespace std;
ros::Publisher JointPOPublisher;
ros::Publisher gestureDetectPublisher;

skeletonclient *Client;

upperbodycore_msgs::jointPO jointPoOr;
upperbodycore_msgs::GestureDetect jestureDetectmsg;

int jointNum = 15;
bool GDS = false;
bool PAOS = false;

bool checkJointPORequest(upperbodycore_msgs::skeletonSrv::Request  &req,upperbodycore_msgs::skeletonSrv::Response &res);
bool checkgesturdetectRequest(upperbodycore_msgs::skeletonSrv::Request  &req, upperbodycore_msgs::skeletonSrv::Response &res);

int main(int argc, char **argv)
{
     ros::init(argc, argv, "core_skeleton");
     std::cout<<"core started"<<std::endl;


    ros::Time::init();
    ros::Rate loop_rate(20);
    ros::NodeHandle nh_[4];
	ros::ServiceServer service_gesturedetect = nh_[0].advertiseService("skeletonin_gesturedetect", checkgesturdetectRequest);
    ros::ServiceServer service_jointPO = nh_[1].advertiseService("skeletonin_jointPO", checkJointPORequest);
    gestureDetectPublisher = nh_[2].advertise<upperbodycore_msgs::GestureDetect>("skeletonout_gestureDetect", 1);
    JointPOPublisher= nh_[3].advertise<upperbodycore_msgs::jointPO>("skeletonout_jointPO",1);
    Client = new skeletonclient();
    Client->skeleton_enable(true);

    while (ros::ok())
    {
 			 jestureDetectmsg.right_hand = Client->get_right_hand_posture();
             cout<<Client->get_right_hand_posture()<<endl;
		     jestureDetectmsg.left_hand  = Client->get_left_hand_posture();
    		 jestureDetectmsg.right_foot = Client->get_right_leg_posture();
    		 jestureDetectmsg.left_foot  = Client->get_left_leg_posture();
    	   	 gestureDetectPublisher.publish(jestureDetectmsg);
 	 		for(int i = 0; i < jointNum; ++i)
 	 		{
 	 			jointPoOr.position.push_back(Client->get_skeleton_data().position[i]);
                //cout<<Client->get_skeleton_data().position[i]<<endl;
 	 			jointPoOr.orientation.push_back(Client->get_skeleton_data().orientation[i]);
 	 		}
 	 		JointPOPublisher.publish(jointPoOr);
 	 		jointPoOr.position.clear();
    		jointPoOr.orientation.clear();
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}

bool checkgesturdetectRequest(upperbodycore_msgs::skeletonSrv::Request  &req, upperbodycore_msgs::skeletonSrv::Response &res)
{
    if(req.requ)
        {    
            Client->skeleton_enable(true);
            GDS =true;
        }
    else
        {
        	Client->skeleton_enable(false);
            GDS = false;
        }
        return true;
}
 bool checkJointPORequest(upperbodycore_msgs::skeletonSrv::Request  &req,upperbodycore_msgs::skeletonSrv::Response &res)
 {
    if(req.requ)
    {
    	bool req = true;
         Client->skeleton_enable(true);
        PAOS = true; 
 	}
    else
    {
        Client->skeleton_enable(false);
        PAOS = false;
 	}
    return true;
 }


