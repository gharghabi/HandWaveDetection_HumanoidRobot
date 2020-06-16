#include "ros/ros.h"
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <stdio.h>
#include <stdlib.h>
#include "std_msgs/Int32.h"
#include "std_msgs/String.h"
#include <math.h>
#include <sstream>
#include <boost/thread.hpp>
#include <string>
#include <pcl/common/time.h>
#include <unistd.h>
#include <fstream>
#include <unistd.h>
#include <skeleton_markers/Skeleton.h>


using namespace std;

class TFTransClass
{
public:
    float x, y, z;
};

int open_ni_code = 0;
int open_ni_error_count = 0;
float open_ni_last_point = 0;
string open_ni_code_str = "0";

int follow_state = 0;
int robot_state = 0;
float desire_distance = -1.1;
bool StartTrack = false;
static double first = 0;
static double last = 0;
int averagesize = 100;
float bufferZ[200][15];
float bufferX[200][15];
float bufferY[200][15];
int counterBuffer = 0;
float person[3];
float head[3];
//float distance[7];
TFTransClass joints[15];
float HightRatio;

float OperatorFeaturesArray[7];

int g_Motor[3] = {0};
int s_Motor[3] = {0};
int p_Motor[3] = {0};


float kp_degree_d = 200;
float ki_degree_d = 0.75;

float kp_degree_a = 100;
float ki_degree_a = 1;

bool App_exit = false;

float pid_i_buffer_d[20] = {0};
float pid_i_buffer_a[20] = {0};

ros::Publisher chatter_pub[3];

float PersonHeight = 0;

void chatterCallback(const std_msgs::Int32::ConstPtr &msg)
{
    open_ni_code = msg->data;

    //ROS_INFO("get  %d", msg->data);
}
void chatterCallbackSkeleton(const skeleton_markers::Skeleton msg)
{
    int user_id =msg.user_id;
    //cout<<msg.confidence[14]<<" confidence "<<user_id<<" user id "<<endl;
    cout<<msg.position[0].x<<" position "<<endl;
    cout<<msg.position[8].x<<" hand x "<<msg.position[0].x<<" head x "<<endl;

    //***********************right hand**************************//

    if(msg.position[8].z<(msg.position[0].z-0.2))//be jaye 0.2 tafazole y torso va neck
    {
        cout<<"front right "<<endl;
    }
    else if(msg.position[8].y>(msg.position[0].y))
    {
        cout<<"up right "<<endl;
    }
    else if(abs(msg.position[8].x-msg.position[6].x)>abs(msg.position[6].x-msg.position[0].x))
    {
        cout<<"side right "<<endl;
    }
    else if(msg.position[8].y<(msg.position[2].y))
    {
        cout<<"down right"<<endl;
    }

    //***********************left hand**************************//

    if(msg.position[5].z<(msg.position[0].z-0.2))//be jaye 0.2 tafazole y torso va neck
    {
        cout<<"front left"<<endl;
    }
    else if(msg.position[5].y>(msg.position[0].y))
    {
        cout<<"up left"<<endl;
    }
    else if(abs(msg.position[5].x-msg.position[3].x)>abs(msg.position[3].x-msg.position[0].x))
    {
        cout<<"side left"<<endl;
    }
    else if(msg.position[5].y<(msg.position[2].y))
    {
        cout<<"down left "<<endl;
    }

}

int main(int argc, char **argv)
{

    //?boost::thread t(&logic);

    ros::init(argc, argv, "openniData");

    ros::NodeHandle n_listen;
    ros::NodeHandle n_listen2;
    ros::NodeHandle n_listen1;

    ros::NodeHandle n;

    //ros::Subscriber sub = n_listen.subscribe("/motor_states/pan_tilt_port", 1000, chatterCallback);
    ros::Subscriber sub1 = n_listen1.subscribe("skeleton", 1000, chatterCallbackSkeleton);
    ros::Subscriber sub2 = n_listen2.subscribe("sourena_userid", 1000, chatterCallback);

    tf::TransformListener listener;
    ros::Rate loop_rate(20);

    while (ros::ok() && App_exit == false)
    {
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}

bool checkJointPORequest(upperbodycore_msgs::skeletonSrv::Request  &req,upperbodycore_msgs::skeletonSrv::Response &res)
{
    cout<<"joint pos req 2"<<endl; 

    if(req.requ)
    {
        PAOS = true;
        cout<<"true"<<endl;
    }
    else
    {
        PAOS = false;
        cout<<"false"<<endl;
    }

    return true;
}


