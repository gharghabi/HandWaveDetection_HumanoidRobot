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


bool App_exit = false;
int  jointstate [8];
int open_ni_code = 0;
int confidencTekrar = 1;
void chatterCallback(const std_msgs::Int32::ConstPtr &msg)
{
    open_ni_code = msg->data;

    //ROS_INFO("get  %d", msg->data);
}
void chatterCallbackSkeleton(const skeleton_markers::Skeleton msg)
{
    int user_id =msg.user_id;
    //cout<<msg.confidence[14]<<" confidence "<<user_id<<" user id "<<endl;
    // cout<<msg.position[0].x<<" position "<<endl;
    // cout<<msg.position[8].z<<" hand z "<<msg.position[0].z<<" head z "<<endl;
    // cout<<" tafazole y "<<abs(msg.position[1].y-msg.position[2].y)<<endl;
    //***********************right hand**************************//


    if(msg.position[8].z<(msg.position[0].z-abs(msg.position[1].y-msg.position[2].y)))//be jaye 0.2 tafazole y torso va neck
    {
        //cout<<"front right "<<endl;
        ++jointstate[0];
        for (int i=0;i<8;++i)
        {
            if(i!=0)
            {
                jointstate[i] = 0;
            }
        }
    }
    else if(msg.position[8].y>(msg.position[0].y))
    {
        //cout<<"up right "<<endl;
        ++jointstate[1];
        for (int i=0;i<8;++i)
        {
            if(i!=1)
            {
                jointstate[i] = 0;
            }
        }
    }
    else if(abs(msg.position[8].x-msg.position[6].x)>((1.2)*abs(msg.position[6].x-msg.position[0].x)))
    {
        // cout<<"side right "<<endl;
        ++jointstate[2];
        for (int i=0;i<8;++i)
        {
            if(i!=2)
            {
                jointstate[i] = 0;
            }
        }
    }
    else if(msg.position[8].y<(msg.position[2].y))
    {
        // cout<<"down right"<<endl;
        ++jointstate[3];
        for (int i=0;i<8;++i)
        {
            if(i!=3)
            {
                jointstate[i] = 0;
            }
        }
    }


        if(jointstate[0]>confidencTekrar)
        {
            cout<<"front right "<<endl;
        }
        else if (jointstate[1]>confidencTekrar)
        {
            cout<<"up right "<<endl;

        }
        else if(jointstate[2]>confidencTekrar)
        {
            cout<<"side right "<<endl;

        }
        else if(jointstate[3]>confidencTekrar)
        {
            cout<<" down  right "<<endl;

        }
    //***********************left hand**************************//

    if(msg.position[5].z<(msg.position[0].z-abs(msg.position[1].y-msg.position[2].y)))//be jaye 0.2 tafazole y torso va neck
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



