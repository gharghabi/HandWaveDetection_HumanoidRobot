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
#include "std_srvs/Empty.h"
#include "upperbodycore_msgs/skeletonSrv.h"

#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>
#include <cv_bridge/cv_bridge.h>

#include <sensor_msgs/PointCloud2.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>

#include <upperbodycore_msgs/jointPO.h>
#include <upperbodycore_msgs/GestureDetect.h>

using namespace std;
using namespace cv;
ros::Publisher JointPOPublisher;
ros::Publisher gestureDetectPublisher;
bool App_exit = false;
int  jointstate [8];
int open_ni_code = 0;
int selectfunction = 1;
int WaveCounter[6] = {0};
float last_x[6] = {0};
bool PAOS = true;
bool GDS = true;

// struct  Coordinate
// {
//     float x, y, z;
// };
struct  CoordinateInMat
{
    int i;
    int j;
    /* data */
};
// ros::ServiceClient client;
// athomerobot_msgs::maptools srv;
CoordinateInMat MokhtasatInMat[15];
skeleton_markers::Skeleton SkData;
upperbodycore_msgs::jointPO jointPoOr;
upperbodycore_msgs::GestureDetect jestureDetectmsg;

pcl::PointCloud<pcl::PointXYZ> globalcloud;

ros::ServiceClient clientgesture,clientjoint;
upperbodycore_msgs::skeletonSrv srv;
void convert_pcd_ToImage();
void waveDetect();
void gestureDetect();
void getPositionAndOrientation();
void help();
void drawUser();
void chatterCallback(const std_msgs::Int32::ConstPtr &msg)
{
    open_ni_code = msg->data;
    //ROS_INFO("get  %d", msg->data);
}
void PointCloudCallBack(const sensor_msgs::PointCloud2::ConstPtr &msg)
{
    pcl::fromROSMsg(*msg, globalcloud);
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
bool checkgesturdetectRequest(upperbodycore_msgs::skeletonSrv::Request  &req, upperbodycore_msgs::skeletonSrv::Response &res)
{
    if(req.requ)
        {
             GDS = true;
                     cout<<"true"<<endl;
        }
    else
        {
              cout<<"false"<<endl;
            GDS = false;
        }
        return true;
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
      // cout<<"get gesuter"<<endl;
    }
    else if(!GDS)
         cout<<"false GDS"<<endl;

    if(PAOS)
    {
        getPositionAndOrientation();
        // cout<<" get position"<<endl;
    }
// if(globalcloud.size()!=0)
//     drawUser();
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
    ros::Subscriber sun3 = n_listen3.subscribe("skeleton_in",1000, chatterCallbackSDK);


    ros::NodeHandle nodeHandleForPC;
    ros::Subscriber PCSubscriber = nodeHandleForPC.subscribe("/camera/depth_registered/points", 1, PointCloudCallBack);



    ros::NodeHandle n_service;
    ros::ServiceServer service_gesturedetect = n_service.advertiseService("skeletonin_gesturedetect", checkgesturdetectRequest);

    ros::NodeHandle n_service1;
    ros::ServiceServer service_jointPO = n_service1.advertiseService("skeletonin_jointPO", checkJointPORequest);


    gestureDetectPublisher = NodeHandle1.advertise<upperbodycore_msgs::GestureDetect>("skeletonout_gestureDetect", 1);
    JointPOPublisher= NodeHandle2.advertise<upperbodycore_msgs::jointPO>("skeletonout_jointPO",1);

    tf::TransformListener listener;
    ros::Rate loop_rate(20);

    while (ros::ok() && App_exit == false)
    {
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
void drawUser()
{
    cv::Size sizepic(480,640);
    Mat skeletonImage = Mat::zeros(480,640, CV_8UC3);
    convert_pcd_ToImage();
    for(int k=0;k<15;++k)
    {
        cout<<MokhtasatInMat[k].i<<" i "<<MokhtasatInMat[k].j<<" j "<<endl;
        // if(MokhtasatInMat[k].i<240)
        //     MokhtasatInMat[k].i+=240;
        // else
        //     MokhtasatInMat[k].i-=240;

        // if(MokhtasatInMat[k].j<320)
        //     MokhtasatInMat[k].j+=320;
        // else
        //     MokhtasatInMat[k].j-=320;

        Point center( MokhtasatInMat[k].i,MokhtasatInMat[k].j );
            ellipse( skeletonImage, center, Size( 2, 2), 0, 0, 360, Scalar( 0, 0, 255 ), 4, 8, 0);  


// Point num1;
// num1.x=MokhtasatInMat[k].i;
//             line(Mat& img, Point pt1, Point pt2, const Scalar& color, int thickness=1, int lineType=8, int shift=0) 
    }     

    cv::Point2i p1(MokhtasatInMat[8].i, MokhtasatInMat[8].j);
    cv::Point2i p2(MokhtasatInMat[1].i, MokhtasatInMat[1].j);

    cv::line(skeletonImage, p1, p2, Scalar( 0, 0, 255 ), 1, CV_AA); // 1 pixel thick, CV_AA == Anti-aliased flag

    imshow("salam",skeletonImage);
    waitKey(10);
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
                // cout<<"right hand up "<<endl;

            }
        }
        else if(abs(msg.position[8].x-msg.position[6].x)>((1.5)*abs(msg.position[6].x-msg.position[0].x)))
        {
            if((msg.confidence[8]>0.5)&&(msg.confidence[6]>0.5))
            {
                jestureDetectmsg.right_hand = "side";
                // cout<<"right hand side"<<endl;
            }
        }
        else if(msg.position[8].z<(msg.position[0].z-1.5*abs(msg.position[1].y-msg.position[2].y)))//be jaye 0.2 tafazole y torso va neck
        {
            if((msg.confidence[8]>0.5)&&(msg.confidence[0]>0.5)&&(msg.confidence[2]>0.5)&&(msg.confidence[1]>0.5))
            {
                jestureDetectmsg.right_hand = "front";
                // cout<<"right hand front  "<<endl;
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
                // cout<<"left hand up"<<endl;
                 jestureDetectmsg.left_hand = "up";

            }
        }
        else if(abs(msg.position[5].x-msg.position[3].x)>1.5*abs(msg.position[3].x-msg.position[0].x))
        {
            if((msg.confidence[5]>0.5)&&(msg.confidence[3]>0.5))
            {
                                jestureDetectmsg.left_hand = "side";

                // cout<<"left hand side"<<endl;
            }
        }
        else if(msg.position[5].z<(msg.position[0].z-1.5*abs(msg.position[1].y-msg.position[2].y)))//be jaye 0.2 tafazole y torso va neck
        {
            if((msg.confidence[5]>0.5)&&(msg.confidence[0]>0.5)&&(msg.confidence[2]>0.5)&&(msg.confidence[1]>0.5))
            {
                // cout<<"left hand front "<<endl;
                jestureDetectmsg.left_hand = "front";

            }
        }
        else if(msg.position[5].y<(msg.position[2].y))
        {
            if((msg.confidence[5]>0.5)&&(msg.confidence[2]>0.5))
            {
                jestureDetectmsg.left_hand = "down";
                // cout<<"left hand down"<<endl;
            }
        }

        //***********************left foot**************************//
        if(msg.position[11].x<(msg.position[9].x-1.5*abs(msg.position[9].x-msg.position[2].x)))//paye chape man mishe paye raste in baraye dastesh in juri nabud
        {
            if((msg.confidence[11]>0.5)&&(msg.confidence[9]>0.5)&&(msg.confidence[2]>0.5))
            {
                jestureDetectmsg.left_foot = "side";
                 // cout<<"left foot side "<<endl;
            }
        }
        else if(msg.position[11].z<(msg.position[2].z-abs(msg.position[9].x-msg.position[2].x)))//paye chape man mishe paye raste in baraye dastesh in juri nabud
        {
            if((msg.confidence[11]>0.5)&&(msg.confidence[9]>0.5)&&(msg.confidence[2]>0.5))
            {
                jestureDetectmsg.left_foot = "front";
                // cout<<"left foot front "<<endl;
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
        cout<<jestureDetectmsg.right_foot<<"hhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhh"<<endl;
        //boost::this_thread::sleep(boost::posix_time::milliseconds(500));

}

void getPositionAndOrientation()
{

    for(int k =0;k<15;++k)
    {
        geometry_msgs::Vector3 mvector;

        mvector.x= SkData.position[k].x;
        mvector.y= SkData.position[k].y;
        mvector.z= SkData.position[k].z;
        jointPoOr.position.push_back(mvector);

        geometry_msgs::Quaternion nvector;
        nvector.x = SkData.orientation[k].x;
        nvector.y = SkData.orientation[k].y;
        nvector.z = SkData.orientation[k].z;
        nvector.w = SkData.orientation[k].w;
        jointPoOr.orientation.push_back(nvector);

    }
    
    JointPOPublisher.publish(jointPoOr);
    // boost::this_thread::sleep(boost::posix_time::milliseconds(500));
    jointPoOr.position.clear();
    jointPoOr.orientation.clear();

    // for (int i=0;++i;i<SkData.user_id)
    // {
        cout<<"SEND"<<endl;
    //}
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
// /i 500
void convert_pcd_ToImage()
{
    for(int  k=0;k<15;++k)
    {
        MokhtasatInMat[k].i =0;
        MokhtasatInMat[k].j =0;
    }
    for (int i = 0; i < 640; ++i)
    {
        for (int j = 0; j < 480; ++j)
        {
            for(int k = 0; k<15;++k)
            {
                if (abs(SkData.position[k].x - globalcloud.at(i, j).x)<0.5 && abs(-SkData.position[k].y - globalcloud.at(i, j).y)<0.5 && abs(SkData.position[k].z - globalcloud.at(i, j).z)<0.5)
                {
                    MokhtasatInMat[k].i = i;
                    MokhtasatInMat[k].j = j;
                }
            }
        }    
    }    
}