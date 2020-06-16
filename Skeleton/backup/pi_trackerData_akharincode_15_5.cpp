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
#include <PGITIC_sourena_msgs/jointPO.h>
#include <PGITIC_sourena_msgs/GestureDetect.h>
#include <skeleton_markers/Skeleton.h>
#include "std_srvs/Empty.h"
#include "PGITIC_sourena_msgs/skeletonSrv.h"

using namespace std;

ros::Publisher JointPOPublisher;
ros::Publisher gestureDetectPublisher;
bool App_exit = false;
int  jointstate [8];
int open_ni_code = 0;
int selectfunction = 1;
int WaveCounter[6] = {0};
float last_x[6] = {0};
bool PAOS = false;
bool GDS = false;



// ros::ServiceClient client;
// athomerobot_msgs::maptools srv;

skeleton_markers::Skeleton SkData;
PGITIC_sourena_msgs::jointPO jointPoOr;
PGITIC_sourena_msgs::GestureDetect jestureDetectmsg;


ros::ServiceClient clientgesture,clientjoint;
PGITIC_sourena_msgs::skeletonSrv srv;

void waveDetect();
void gestureDetect();
void getPositionAndOrientation();
void help();
void chatterCallback(const std_msgs::Int32::ConstPtr &msg)
{
    open_ni_code = msg->data;
    //ROS_INFO("get  %d", msg->data);
}
void checkJointPORequest(PGITIC_sourena_msgs::skeletonSrv::Request  &req)
{
    if(req.requ)
        PAOS = true;
    else
        PAOS = false;
}
void checkgesturdetectRequest(PGITIC_sourena_msgs::skeletonSrv::Request  &req)
{
    if(req.requ)
        GDS = true;
    else
        GDS = false;
}
void chatterCallbackSkeleton(const skeleton_markers::Skeleton msg)
{
    int user_id =msg.user_id;
    SkData = msg;
    // cout<< msg.name[0]<<" name "<<endl;
    // cout<<msg.orientation[2]<<" head orientation"<<endl;
    if(GDS)
    {
      gestureDetect();  
    }
    if(PAOS)
        getPositionAndOrientation();
//    gestureDetect();
    // waveDetect();    
}
void chatterCallbackSDK(std_msgs::String msg)
{
    if(msg.data == "getPositionAndOrientation")
    {
        PAOS = true;
    }
    if(msg.data == "gestureDetect")
    {
        GDS = true;
    }
}
int main(int argc, char **argv)
{

    //?boost::thread t(&logic);

    ros::init(argc, argv, "openniData");

    ros::NodeHandle n_listen;
    ros::NodeHandle n_listen2;
    ros::NodeHandle n_listen1;
    ros::NodeHandle n_listen3;
    ros::NodeHandle NodeHandle2;
    ros::NodeHandle NodeHandle1;


    ros::NodeHandle n;

    //ros::Subscriber sub = n_listen.subscribe("/motor_states/pan_tilt_port", 1000, chatterCallback);
    ros::Subscriber sub1 = n_listen1.subscribe("skeleton", 1000, chatterCallbackSkeleton);
    ros::Subscriber sub2 = n_listen2.subscribe("sourena_userid", 1000, chatterCallback);
    ros::Subscriber sun3 = n_listen3.subscribe("skele_in",1000, chatterCallbackSDK);



    ros::NodeHandle n_service;
    ros::ServiceServer service_gesturedetect = n_service.advertiseService("skeletonin_gesturedetect", checkgesturdetectRequest);

    ros::NodeHandle n_service1;
    ros::ServiceServer service_jointPO = n_service1.advertiseService("skeletonin_jointPO", checkJointPORequest);


    JointPOPublisher = NodeHandle1.advertise<PGITIC_sourena_msgs::jointPO>("skeletonout_gestureDetect", 1);
    gestureDetectPublisher = NodeHandle2.advertise<PGITIC_sourena_msgs::GestureDetect>("skeletonout_jointPO",1);

    tf::TransformListener listener;
    ros::Rate loop_rate(20);

    while (ros::ok() && App_exit == false)
    {
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
void waveDetect()
{
     skeleton_markers::Skeleton msg = SkData;
         int user_id = msg.user_id;

        if(abs(msg.position[8].x-msg.position[6].x)>((1.5)*abs(msg.position[6].x-msg.position[0].x)))
            {
                 int velocity = (int)abs((last_x[user_id] - msg.position[8].x) / 0.13);
                    if(velocity>60)
                    {
                        ++WaveCounter[user_id];
                    }
                    else
                    {
                        WaveCounter[user_id]=0;                        
                    }
                    if(WaveCounter[user_id]>4)
                    {
                        cout<<"waveDetect for user "<<user_id<<endl;
                    }
                    last_x[user_id] = msg.position[8].x; 
            }

}
void gestureDetect()
{
    skeleton_markers::Skeleton msg = SkData;
            // ***********************right hand**************************//
        if(msg.position[8].y>(msg.position[0].y+0.2*abs(msg.position[0].y-msg.position[1].y)))
        {
            if((msg.confidence[8]>0.5)&&(msg.confidence[0]>0.5))
            {
                jestureDetectmsg.right_hand = "up";
                cout<<"right hand up "<<endl;

            }
        }
        else if(abs(msg.position[8].x-msg.position[6].x)>((1.5)*abs(msg.position[6].x-msg.position[0].x)))
        {
            if((msg.confidence[8]>0.5)&&(msg.confidence[6]>0.5))
            {
                jestureDetectmsg.right_hand = "side";
                cout<<"right hand side"<<endl;
            }
        }
        else if(msg.position[8].z<(msg.position[0].z-1.5*abs(msg.position[1].y-msg.position[2].y)))//be jaye 0.2 tafazole y torso va neck
        {
            if((msg.confidence[8]>0.5)&&(msg.confidence[0]>0.5)&&(msg.confidence[2]>0.5)&&(msg.confidence[1]>0.5))
            {
                jestureDetectmsg.right_hand = "front";
                cout<<"right hand front  "<<endl;
            }
        }
        else if(msg.position[8].y<(msg.position[2].y))
        {
            if((msg.confidence[8]>0.5)&&(msg.confidence[2]>0.5))
            {
                 jestureDetectmsg.right_hand = "down";
                cout<<"right hand down"<<endl;
            }
        }

        //***********************left hand**************************//
        if(msg.position[5].y>(msg.position[0].y+0.20*abs(msg.position[0].y-msg.position[1].y)))
        {
            if((msg.confidence[5]>0.5)&&(msg.confidence[0]>0.5))
            {
                cout<<"left hand up"<<endl;
                 jestureDetectmsg.left_hand = "up";

            }
        }
        else if(abs(msg.position[5].x-msg.position[3].x)>1.5*abs(msg.position[3].x-msg.position[0].x))
        {
            if((msg.confidence[5]>0.5)&&(msg.confidence[3]>0.5))
            {
                                jestureDetectmsg.left_hand = "side";

                cout<<"left hand side"<<endl;
            }
        }
        else if(msg.position[5].z<(msg.position[0].z-1.5*abs(msg.position[1].y-msg.position[2].y)))//be jaye 0.2 tafazole y torso va neck
        {
            if((msg.confidence[5]>0.5)&&(msg.confidence[0]>0.5)&&(msg.confidence[2]>0.5)&&(msg.confidence[1]>0.5))
            {
                cout<<"left hand front "<<endl;
                jestureDetectmsg.left_hand = "front";

            }
        }
        else if(msg.position[5].y<(msg.position[2].y))
        {
            if((msg.confidence[5]>0.5)&&(msg.confidence[2]>0.5))
            {
                jestureDetectmsg.left_hand = "down";
                cout<<"left hand down"<<endl;
            }
        }

        //***********************left foot**************************//
        if(msg.position[11].x<(msg.position[9].x-1.5*abs(msg.position[9].x-msg.position[2].x)))//paye chape man mishe paye raste in baraye dastesh in juri nabud
        {
            if((msg.confidence[11]>0.5)&&(msg.confidence[9]>0.5)&&(msg.confidence[2]>0.5))
            {
                jestureDetectmsg.left_foot = "side";
                 cout<<"left foot side "<<endl;
            }
        }
        else if(msg.position[11].z<(msg.position[2].z-abs(msg.position[9].x-msg.position[2].x)))//paye chape man mishe paye raste in baraye dastesh in juri nabud
        {
            if((msg.confidence[11]>0.5)&&(msg.confidence[9]>0.5)&&(msg.confidence[2]>0.5))
            {
                jestureDetectmsg.left_foot = "front";
                cout<<"left foot front "<<endl;
            }
        }
                //*************tashis back ba knee ********************//
        // else if(msg.position[10].z>(msg.position[2].z+2*abs(msg.position[9].x-msg.position[2].x)))//paye chape man mishe paye raste in baraye dastesh in juri nabud
        // {
        //     if((msg.confidence[10]>0.5)&&(msg.confidence[9]>0.5)&&(msg.confidence[2]>0.5))
        //     {
        //         cout<<"left foot back "<<endl;
        //     }
        // }

        // else if(msg.position[11].z>(msg.position[10].z+1.8*abs(msg.position[9].x-msg.position[2].x)))//paye chape man mishe paye raste in baraye dastesh in juri nabud
        // {
        //     if((msg.confidence[11]>0.5)&&(msg.confidence[9]>0.5)&&(msg.confidence[2]>0.5))
        //     {
        //         cout<<"left foot back "<<endl;
        //     }
        // }
        else 
        {
            if((msg.confidence[11]>0.5)&&(msg.confidence[9]>0.5)&&(msg.confidence[2]>0.5)&&(msg.confidence[14]>0.5))
            {
                jestureDetectmsg.left_foot = "down";
                cout<<"left foot down "<<endl;
            }
        }

        //***********************right foot**************************//

        if(msg.position[14].x>(msg.position[12].x+1.5*abs(msg.position[12].x-msg.position[2].x)))//paye chape man mishe paye raste in baraye dastesh in juri nabud
        {
            if((msg.confidence[14]>0.5)&&(msg.confidence[12]>0.5)&&(msg.confidence[2]>0.5))
            {
                jestureDetectmsg.right_foot = "side";
                 cout<<"right foot side "<<endl;
            }
        }
        else if(msg.position[14].z<(msg.position[2].z-abs(msg.position[12].x-msg.position[2].x)))//paye chape man mishe paye raste in baraye dastesh in juri nabud
        {
            if((msg.confidence[14]>0.5)&&(msg.confidence[9]>0.5)&&(msg.confidence[2]>0.5))
            {
                jestureDetectmsg.right_foot = "front";
                cout<<"right foot front "<<endl;
            }
        }

        // else if(msg.position[13].z>(msg.position[2].z+2*abs(msg.position[12].x-msg.position[2].x)))//paye chape man mishe paye raste in baraye dastesh in juri nabud
        // {
        //     if((msg.confidence[13]>0.5)&&(msg.confidence[12]>0.5)&&(msg.confidence[2]>0.5))
        //     {
        //         cout<<"right foot back "<<endl;
        //     }
        // }

        // else if(msg.position[14].z>(msg.position[13].z+1.8*abs(msg.position[12].x-msg.position[2].x)))//paye chape man mishe paye raste in baraye dastesh in juri nabud
        // {
        //     if((msg.confidence[14]>0.5)&&(msg.confidence[12]>0.5)&&(msg.confidence[2]>0.5))
        //     {
        //         cout<<"right foot back "<<endl;
        //     }
        // }
        else 
        {
            if((msg.confidence[14]>0.5)&&(msg.confidence[12]>0.5)&&(msg.confidence[2]>0.5)&&(msg.confidence[14]>0.5))
            {
                jestureDetectmsg.right_foot = "down";
                cout<<"rightt foot down "<<endl;
            }
        }
        gestureDetectPublisher.publish(jestureDetectmsg);
        boost::this_thread::sleep(boost::posix_time::milliseconds(500));

}

void getPositionAndOrientation()
{

    jointPoOr.position= SkData.position;
    jointPoOr.orientation = SkData.orientation;
    JointPOPublisher.publish(jointPoOr);

    for (int i=0;++i;i<SkData.user_id)
    {
        cout<<SkData.confidence[0]<<endl;
    }
    // if(name== "head")         
    // {
    //     cout<<"position in cartesian is "<<" x = "<<SkData.position[0].x<<" y = "<<SkData.position[0].y<<" z = "<<SkData.position[0].z<<endl;
    //     cout<<"orientation represent as a quaternions is "<<" x = "<<SkData.orientation[0].x<<" y = "<<SkData.orientation[0].y<<" z = "<<SkData.orientation[0].z<<" w = "<< SkData.orientation[0].w<<endl;
    // }
    // else if(name== "neck") 
    // {
    //     cout<<"position in cartesian is "<<" x = "<<SkData.position[1].x<<" y = "<<SkData.position[1].y<<" z = "<<SkData.position[1].z<<endl;
    //     cout<<"orientation represent as a quaternions is "<<" x = "<<SkData.orientation[1].x<<" y = "<<SkData.orientation[1].y<<" z = "<<SkData.orientation[1].z<<" w = "<< SkData.orientation[1].w<<endl;
    // }
    // else if(name== "torso") 
    // {
    //     cout<<"position in cartesian is "<<" x = "<<SkData.position[2].x<<" y = "<<SkData.position[2].y<<" z = "<<SkData.position[2].z<<endl;
    //     cout<<"orientation represent as a quaternions is "<<" x = "<<SkData.orientation[2].x<<" y = "<<SkData.orientation[2].y<<" z = "<<SkData.orientation[2].z<<" w = "<< SkData.orientation[2].w<<endl;
    // }
    // else if(name== "left_shoulder") 
    // {
    //     cout<<"position in cartesian is "<<" x = "<<SkData.position[3].x<<" y = "<<SkData.position[3].y<<" z = "<<SkData.position[3].z<<endl;
    //     cout<<"orientation represent as a quaternions is "<<" x = "<<SkData.orientation[3].x<<" y = "<<SkData.orientation[3].y<<" z = "<<SkData.orientation[3].z<<" w = "<< SkData.orientation[3].w<<endl;
    // }
    // else if(name== "left_elbow") 
    // {
    //     cout<<"position in cartesian is "<<" x = "<<SkData.position[4].x<<" y = "<<SkData.position[4].y<<" z = "<<SkData.position[4].z<<endl;
    //     cout<<"orientation represent as a quaternions is "<<" x = "<<SkData.orientation[4].x<<" y = "<<SkData.orientation[4].y<<" z = "<<SkData.orientation[4].z<<" w = "<< SkData.orientation[4].w<<endl;
    // }
    // else if(name== "left_hand") 
    // {
    //     cout<<"position in cartesian is "<<" x = "<<SkData.position[5].x<<" y = "<<SkData.position[5].y<<" z = "<<SkData.position[5].z<<endl;
    //     cout<<"orientation represent as a quaternions is "<<" x = "<<SkData.orientation[5].x<<" y = "<<SkData.orientation[5].y<<" z = "<<SkData.orientation[5].z<<" w = "<< SkData.orientation[5].w<<endl;
    // }
    // else if(name== "right_shoulder") 
    // {
    //     cout<<"position in cartesian is "<<" x = "<<SkData.position[6].x<<" y = "<<SkData.position[6].y<<" z = "<<SkData.position[6].z<<endl;
    //     cout<<"orientation represent as a quaternions is "<<" x = "<<SkData.orientation[6].x<<" y = "<<SkData.orientation[6].y<<" z = "<<SkData.orientation[6].z<<" w = "<< SkData.orientation[6].w<<endl;
    // }
    // else if(name== "right_elbow") 
    // {
    //     cout<<"position in cartesian is "<<" x = "<<SkData.position[7].x<<" y = "<<SkData.position[7].y<<" z = "<<SkData.position[7].z<<endl;
    //     cout<<"orientation represent as a quaternions is "<<" x = "<<SkData.orientation[7].x<<" y = "<<SkData.orientation[7].y<<" z = "<<SkData.orientation[7].z<<" w = "<< SkData.orientation[7].w<<endl;
    // }
    // else if(name== "right_hand") 
    // {
    //     cout<<"position in cartesian is "<<" x = "<<SkData.position[8].x<<" y = "<<SkData.position[8].y<<" z = "<<SkData.position[8].z<<endl;
    //     cout<<"orientation represent as a quaternions is "<<" x = "<<SkData.orientation[8].x<<" y = "<<SkData.orientation[8].y<<" z = "<<SkData.orientation[8].z<<" w = "<< SkData.orientation[8].w<<endl;
    // }
    // else if(name== "left_hip") 
    //   {
    //     cout<<"position in cartesian is "<<" x = "<<SkData.position[9].x<<" y = "<<SkData.position[9].y<<" z = "<<SkData.position[9].z<<endl;
    //     cout<<"orientation represent as a quaternions is "<<" x = "<<SkData.orientation[9].x<<" y = "<<SkData.orientation[9].y<<" z = "<<SkData.orientation[9].z<<" w = "<< SkData.orientation[9].w<<endl;
    // }
    // else if(name== "left_knee") 
    // {
    //     cout<<"position in cartesian is "<<" x = "<<SkData.position[10].x<<" y = "<<SkData.position[10].y<<" z = "<<SkData.position[10].z<<endl;
    //     cout<<"orientation represent as a quaternions is "<<" x = "<<SkData.orientation[10].x<<" y = "<<SkData.orientation[10].y<<" z = "<<SkData.orientation[10].z<<" w = "<< SkData.orientation[10].w<<endl;
    // }
    // else if(name== "left_foot") 
    // {
    //     cout<<"position in cartesian is "<<" x = "<<SkData.position[11].x<<" y = "<<SkData.position[11].y<<" z = "<<SkData.position[11].z<<endl;
    //     cout<<"orientation represent as a quaternions is "<<" x = "<<SkData.orientation[11].x<<" y = "<<SkData.orientation[11].y<<" z = "<<SkData.orientation[11].z<<" w = "<< SkData.orientation[11].w<<endl;
    // }
    // else if(name== "right_hip") 
    // {
    //     cout<<"position in cartesian is "<<" x = "<<SkData.position[12].x<<" y = "<<SkData.position[12].y<<" z = "<<SkData.position[12].z<<endl;
    //     cout<<"orientation represent as a quaternions is "<<" x = "<<SkData.orientation[12].x<<" y = "<<SkData.orientation[12].y<<" z = "<<SkData.orientation[0].z<<" w = "<< SkData.orientation[12].w<<endl;
    // }
    // else if(name== "right_knee") 
    // {
    //     cout<<"position in cartesian is "<<" x = "<<SkData.position[13].x<<" y = "<<SkData.position[13].y<<" z = "<<SkData.position[13].z<<endl;
    //     cout<<"orientation represent as a quaternions is "<<" x = "<<SkData.orientation[13].x<<" y = "<<SkData.orientation[13].y<<" z = "<<SkData.orientation[13].z<<" w = "<< SkData.orientation[13].w<<endl;
    // }
    // else if(name== "right_foot") 
    // {
    //     cout<<"position in cartesian is "<<" x = "<<SkData.position[14].x<<" y = "<<SkData.position[14].y<<" z = "<<SkData.position[14].z<<endl;
    //     cout<<"orientation represent as a quaternions is "<<" x = "<<SkData.orientation[14].x<<" y = "<<SkData.orientation[14].y<<" z = "<<SkData.orientation[14].z<<" w = "<< SkData.orientation[14].w<<endl;
    // }
    // else 
    //     {
    //         cout << "please enter a correct name from list below";
    //         cout<<"head"<<"\n"<<"neck"<<"\n"<<"torso"<<"\n"<<"right_shoulder"<<"\n"<<"right_elbow"<<"\n";
    //     }
}
void help()
{

}