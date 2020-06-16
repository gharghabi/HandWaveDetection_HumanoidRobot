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


int main(int argc, char **argv)
{

    //?boost::thread t(&logic);

    ros::init(argc, argv, "openniData");

    ros::NodeHandle n_listen;
    ros::NodeHandle n_listen2;
    ros::NodeHandle n;

    //ros::Subscriber sub = n_listen.subscribe("/motor_states/pan_tilt_port", 1000, chatterCallback);
    ros::Subscriber sub2 = n_listen2.subscribe("sourena_userid", 1000, chatterCallback);

    tf::TransformListener listener;
    ros::Rate loop_rate(20);
    tf::StampedTransform transform[15];
    ofstream datax(argv[1]);
    ofstream datay(argv[2]);
    cout<<argv[2]<<"ssssssssssssssssssssssssssssss"<<argv[3]<<"fdsfdsfdsfsd"<<endl;
    ofstream dataz(argv[3]);
//    ofstream average(argv[2]);
     if (datax.is_open())
        {
            datax<<" head    "<<" neck   "<<" torso   "<<" right_hip"<<" left_hip  "<<" right_shoulder_"<<" right elbow   "<<" right hand    "<<" right_knee  "<<" right_foot  "<<" left shoulder   "<<" left elbow  "<<" left hand  "<<" left_knee  "<<" left_foot  "<<"\n";            
                        
        }
        else
        {
            cout<<"error1"<<endl;
        }
        if(datay.is_open())
        {
            datay<<" head    "<<" neck   "<<" torso   "<<" right_hip"<<" left_hip  "<<" right_shoulder_"<<" right elbow   "<<" right hand    "<<" right_knee  "<<" right_foot  "<<" left shoulder   "<<" left elbow  "<<" left hand  "<<" left_knee  "<<" left_foot  "<<"\n";            

        }
        else
        {
            cout<<"error2"<<endl;
        }

        if(dataz.is_open())
        {
            dataz<<" head    "<<" neck   "<<" torso   "<<" right_hip"<<" left_hip  "<<" right_shoulder_"<<" right elbow   "<<" right hand    "<<" right_knee  "<<" right_foot  "<<" left shoulder   "<<" left elbow  "<<" left hand  "<<" left_knee  "<<" left_foot  "<<"\n";            
        }
        else
        {
            cout<<"error3"<<endl;
        }


     // if (average.is_open())
     //            {
     //                average <<" head    "<<" neck   "<<" torso   "<<" right_hip"<<" left_hip  "<<" right_shoulder_"<<" right elbow   "<<" right hand    "<<" right_knee  "<<" right_foot  "<<" left shoulder   "<<" left elbow  "<<" left hand  "<<" left_knee  "<<" left_foot  "<<"\n";            
                            
     //            }

    while (ros::ok() && App_exit == false)
    {
        if ( open_ni_code > 300 )
        {
            //new user callibrated !!!
            int xxx = open_ni_code - 300;
            string s = "0";
            if ( xxx == 1 ) s = "1";
            if ( xxx == 2 ) s = "2";
            if ( xxx == 3 ) s = "3";
            if ( xxx == 4 ) s = "4";
            if ( xxx == 5 ) s = "5";
            if ( xxx == 6 ) s = "6";
            open_ni_code_str = s;
            cout << "TRACK START FOR " << s << endl;
            last = pcl::getTime();
            /****************** Operator features***********************/

            try
            {
        

                listener.lookupTransform("/head_" + open_ni_code_str, "/openni_depth_frame", ros::Time(0), transform[4]);
                listener.lookupTransform("/right_knee_" + open_ni_code_str, "/openni_depth_frame", ros::Time(0), transform[5]);
                listener.lookupTransform("/left_hand_" + open_ni_code_str, "/openni_depth_frame", ros::Time(0), transform[9]);
                listener.lookupTransform("/left_knee_" + open_ni_code_str, "/openni_depth_frame", ros::Time(0), transform[11]);
                listener.lookupTransform("/torso_" + open_ni_code_str, "/openni_depth_frame", ros::Time(0), transform[2]);
                listener.lookupTransform("/right_shoulder_" + open_ni_code_str, "/openni_depth_frame", ros::Time(0), transform[7]);
                listener.lookupTransform("/left_shoulder_" + open_ni_code_str, "/openni_depth_frame", ros::Time(0), transform[8]);
                listener.lookupTransform("/left_hip_" + open_ni_code_str, "/openni_depth_frame", ros::Time(0), transform[0]);
                listener.lookupTransform("/right_hip_" + open_ni_code_str, "/openni_depth_frame", ros::Time(0), transform[10]);
                listener.lookupTransform("/neck_" + open_ni_code_str, "/openni_depth_frame", ros::Time(0), transform[1]);
                listener.lookupTransform("/right_elbow_" + open_ni_code_str, "/openni_depth_frame", ros::Time(0), transform[3]);
                listener.lookupTransform("/left_elbow_" + open_ni_code_str, "/openni_depth_frame", ros::Time(0), transform[6]);
                listener.lookupTransform("/right_hand_" + open_ni_code_str, "/openni_depth_frame", ros::Time(0), transform[12]);
                listener.lookupTransform("/right_foot_" + open_ni_code_str, "/openni_depth_frame", ros::Time(0), transform[13]);
                listener.lookupTransform("/left_foot_" + open_ni_code_str, "/openni_depth_frame", ros::Time(0), transform[14]);
                float sumX[15];
                float sumY[15];
                float sumZ[15];
                for(int i=averagesize-2;i>=0;i--)
                     {
                        for (int j=0;j<15;++j)
                        {
                            sumX[j]=0;
                            sumZ[j]=0;
                            sumY[j]=0;
                            bufferZ[i+1][j]= bufferZ[i][j];
                         
                            bufferX[i+1][j]= bufferX[i][j];
                        
                            bufferY[i+1][j]= bufferY[i][j];
                        }

                     }


                    for (int j=0;j<15;++j)
                        {
                            bufferZ[0][j] = transform[j].getOrigin().z();
                          
                            bufferX[0][j] = transform[j].getOrigin().x();
                          
                            bufferY[0][j] = transform[j].getOrigin().y();
                          
                        }

                        for (int j =0;j<15;++j)
                        {    
                            for(int i=0;i<averagesize;++i)
                            {
                                   sumZ[j]+= bufferZ[i][j];
                                   sumX[j]+= bufferX[i][j];
                                   sumY[j]+= bufferY[i][j];

                            }
                            sumZ[j]/=averagesize;
                            sumY[j]/=averagesize;
                            sumX[j]/=averagesize;
                        }


// <<" head    "<<" neck   "<<" torso   "<<" right_hip"<<" right_shoulder_"<<" right elbow   "<<" right hand    "<<" right_knee  "<<" right_foot  "<<" left_hip  "<<" left shoulder   "<<" left elbow  "<<" left hand  "<<" left_knee  "<<" left_foot  "<<"\n";            
// 4,1,2,10,7,3,12,5,13,0,8,6,9,11,14               
                
                if (datax.is_open())
                    {
                    datax <<transform[4].getOrigin().x()<<","<<transform[1].getOrigin().x()<<","<<transform[2].getOrigin().x()<<","<<transform[10].getOrigin().x()<<","<<transform[7].getOrigin().x()<<","<<transform[3].getOrigin().x()<<","<<transform[12].getOrigin().x()<<","<<transform[5].getOrigin().x()<<","<<transform[13].getOrigin().x()<<","<<transform[0].getOrigin().x()<<","<<transform[8].getOrigin().x()<<","<<transform[6].getOrigin().x()<<","<<transform[9].getOrigin().x()<<","<<transform[11].getOrigin().x()<<","<<transform[14].getOrigin().x()<<"\n";            
                    }

                if (datay.is_open())
                    {
                    datay <<transform[4].getOrigin().y()<<","<<transform[1].getOrigin().y()<<","<<transform[2].getOrigin().y()<<","<<transform[10].getOrigin().y()<<","<<transform[7].getOrigin().y()<<","<<transform[3].getOrigin().y()<<","<<transform[12].getOrigin().y()<<","<<transform[5].getOrigin().y()<<","<<transform[13].getOrigin().y()<<","<<transform[0].getOrigin().y()<<","<<transform[8].getOrigin().y()<<","<<transform[6].getOrigin().y()<<","<<transform[9].getOrigin().y()<<","<<transform[11].getOrigin().y()<<","<<transform[14].getOrigin().y()<<"\n";            
                    }
                if (dataz.is_open())
                    {
                    dataz <<transform[4].getOrigin().z()<<","<<transform[1].getOrigin().z()<<","<<transform[2].getOrigin().z()<<","<<transform[10].getOrigin().z()<<","<<transform[7].getOrigin().z()<<","<<transform[3].getOrigin().z()<<","<<transform[12].getOrigin().z()<<","<<transform[5].getOrigin().z()<<","<<transform[13].getOrigin().z()<<","<<transform[0].getOrigin().z()<<","<<transform[8].getOrigin().z()<<","<<transform[6].getOrigin().z()<<","<<transform[9].getOrigin().z()<<","<<transform[11].getOrigin().z()<<","<<transform[14].getOrigin().z()<<"\n";            
                    }


                 // if (average.is_open())
                 //    {
                 //        average <<sumY[4]<<","<<sumY[1]<<","<<sumY[2]<<","<<sumY[3]<<","<<sumY[7]<<","<<sumY[12]<<"\n";            
                 //    }

                    // cout<<transform[1].getOrigin().y()<<" y neck "<<endl;
                    // cout<<transform[2].getOrigin().y()<<" y torso "<<endl;
                    // cout<<sumY[1]<<" sum y neck "<<endl;
                    // cout<<sumY[2]<<" sum y torso "<<endl;
                        // cout<<sumX[1]<<" average neck X "<<endl;
                    // cout<<sumX[12]<<" average right hand X "<<sumX[1]<<" average neck X "<<endl;
                    // cout<<sumY[12]<<" averahge right hand Y "<<sumY[1]<<" averahge neck Y "<<endl;
                    // cout<<sumZ[12]<<" averahge right hand Z "<<sumZ[1]<<" averahge neck Z "<<endl;

                    //cout<<sumX<<" average "<<endl;
                //cout<<transform[12].getOrigin().x()<<endl;
                if (!StartTrack)
                {
                    if (first == 0)
                    {
                        first = pcl::getTime();
                    }
                    //cout << "salam" << endl;
                    // joints[14].x = transform[14].getOrigin().x();
                    for (int i = 0; i < 15; ++i)
                    {
                        joints[i].x = transform[i].getOrigin().x();
                        joints[i].y = transform[i].getOrigin().y();
                        joints[i].z = transform[i].getOrigin().z();
                    }
                                       //cout << "avvalin frame" << endl;
                }
                last = pcl::getTime();
                cout<<StartTrack<<" start track "<<endl;
                if (((last - first) > 5) && (first != 0)) //?user id ham bayad check shavad
                {
                    StartTrack = true;
                    float TYNT = (abs(transform[1].getOrigin().y()-transform[2].getOrigin().y()));
                    cout<<TYNT<<"2/3 tafazole y neck va torso"<<endl;

                    if (abs(transform[12].getOrigin().x()-joints[12].x)>0.3)
                    {
                         cout<<" side "<<endl;       
                    }
                    else if ((abs(transform[7].getOrigin().x()-joints[7].x)>0.6))//shoulder az avvalie bozargtar az 60
                    {
                        cout<<" front "<<endl;
                    }
                    else if (abs(transform[12].getOrigin().y()<transform[4].getOrigin().y()))
                    {
                        cout<<" up"<<endl;
                    } 
                    else if (abs(transform[12].getOrigin().x()-transform[10].getOrigin().x())<0.3 && abs(transform[12].getOrigin().z()-transform[10].getOrigin().z())<0.3)
                    {
                        cout<<"down "<<endl;
                    }
                    //cout<<abs(transform[12].getOrigin().x()-transform[10].getOrigin().x())<< " tafazol " <<endl;
                    cout<<transform[4].getOrigin().z()<<" hip "<<endl;
                    cout<<transform[12].getOrigin().z()<< " dast "<<endl;
                    int trust = 0;
                    // if (abs(transform[7].getOrigin().y()-joints[7].y)<0.2)
                    // {
                    //     //cout<<" down "<<endl;
                    //     ++trust;
                    //      // cout<<" down "<<endl;
                    // }
                    // if (abs(transform[7].getOrigin().x()-joints[7].x)<0.2)
                    // {
                    //     ++trust;
                    //      // cout<<" down "<<endl;                           
                    // }
                    // if (abs(transform[7].getOrigin().z()-joints[7].z)<0.2)
                    // {
                    //     ++trust;
                    //      // cout<<" down "<<endl;
                    // }
                    // if(trust>2)
                    // {
                    //     cout<<"down"<<endl;
                    //     // for (int i = 0; i < 15; ++i)
                    //     // {
                    //     //     joints[i].x = transform[i].getOrigin().x();
                    //     //     joints[i].y = transform[i].getOrigin().y();
                    //     //     joints[i].z = transform[i].getOrigin().z();
                    //     // }
                    // }
                    trust = 0;
                    // if (abs(sumY[1] - sumY[12])>TYNT) //right down
                    // {
                    //      cout<<" down "<<endl;
                    //      cout << "neckY " << sumY[1] << " handY " << sumY[12] << " TYNT "<<TYNT<<endl;
                    //      // if (motors_info.is_open())
                    //      //    {
                    //      //        motors_info <<" down "<<"\n";
                    //      //        motors_info <<" neck y"<<sumY[1]<<" hand y "<<sumY[12]<<"\n";            
                            
                    //      //    }

                    // }
                    // if (abs(sumZ[1] - sumZ[12])>TYNT) //right front
                    // {
                    //      cout<<" front "<<endl;
                    //      cout << "neckZ " << sumZ[1] << " handZ " << sumZ[12] << " TYNT "<<TYNT<<endl;
                    //       // if (motors_info.is_open())
                    //       //   {
                    //       //       motors_info <<" front "<<"\n";
                    //       //       motors_info <<" neck Z"<<sumZ[1]<<" hand Z "<<sumZ[12]<<"\n";            
                    //       //   }

                    // }
                    // if (abs(sumX[1] - sumX[12])>TYNT) //right side
                    // {
                    //      cout<<" side "<<endl;
                    //     cout << "neckX " << sumX[1] << " handX " << sumX[12] << " TYNT "<<TYNT<<endl;
                    //      // if (motors_info.is_open())
                    //      //    {
                    //      //        motors_info <<" side "<<"\n";
                    //      //        motors_info <<" neck X "<<sumX[1]<<" hand X "<<sumX[12]<<"\n";            
                    //      //    }

                    // }
                }
                //OperatorFeaturesArray[0]=(transform[4].getOrigin().y()-((transform[5].getOrigin().y()+transform[11].getOrigin().y())/2))/torso.z;
                // cout << OperatorFeaturesArray[0] << " Operator Hight" << endl;
                // cout<<"head  ="<<transform[4].getOrigin().y()<<" y"<<transform[4].getOrigin().x()<<" x"<<transform[4].getOrigin().z()<<" z"<<endl;
                // cout << "left_hand_  =" << transform[9].getOrigin().y() << " y " << transform[9].getOrigin().x() << " x " << transform[9].getOrigin().z() << "z" << endl;
                // cout<<"left_knee_  ="<<transform[11].getOrigin().y()<<" y"<<transform[11].getOrigin().x()<<" x"<<transform[11].getOrigin().z()<<"z"<<endl;
                // cout<<"right_knee_  ="<<transform[5].getOrigin().y()<<" y"<<transform[5].getOrigin().x()<<" x"<<transform[5].getOrigin().z()<<" z"<<endl;

            }
            catch (tf::TransformException ex)
            {
                cout << "21" << "hello" << endl;

            }
        }
        else
        {
            StartTrack = false;
            first = 0;
        }
        ros::spinOnce();
        loop_rate.sleep();
    }
      datax.close();
      datay.close();
      dataz.close();

    return 0;
}



