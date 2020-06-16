#include "wave.h"
 
using namespace std;
wave::wave()
 {	
    sub_ = nh_.subscribe("skeleton", 1000, &wave::SkeletonData_CB,this);  
 	SkData.user_id = -1;
	confidence = 0.3;
    x_RH_prev = 0;
    x_LH_prev = 0;
    RH_wave_counter = 0;
    LH_wave_counter = 0;
    RH_wave_detect = false;
    LH_wave_detect = false;
 }
 wave::~wave()
 
 {
 	
 }
 void wave::SkeletonData_CB(const upperbodycore_msgs::Skeleton msg)
 {
 	SkData = msg;
    wave_detect();  
 }
 void wave::wave_detect()
 {
	upperbodycore_msgs::Skeleton msg = SkData;
    //int user_id = msg.user_id;
    if(msg.confidence[8]>confidence)
    {
        if(abs(msg.position[8].y>msg.position[1].y))
        {
             int velocity = (int)(abs((x_RH_prev - msg.position[8].x) / (0.03))*100);//har frame tu 0.03 sanie miad
                if(velocity>300*abs(msg.position[0].y - msg.position[1].y))
                {
                    ++RH_wave_counter;
                }
                else
                {
                    if(RH_wave_counter > 0)
                         --RH_wave_counter;  
                    else
                      RH_wave_detect = false;
                }
                if(RH_wave_counter>4)
                {
                    RH_wave_detect = true; 
                }
                x_RH_prev = msg.position[8].x; 
        }
        else
        {
            RH_wave_counter = 0;
            x_RH_prev = 0;
            RH_wave_detect = false;
        }
    }
    if(msg.confidence[5]>confidence)
    {
        if(abs(msg.position[5].y>msg.position[1].y))
        {
             int velocity = (int)(abs((x_LH_prev - msg.position[5].x) / (0.03))*100);//har frame tu 0.03 sanie miad
                if(velocity>300*abs(msg.position[0].y - msg.position[1].y))
                {
                    cout<<velocity<<" velocity "<< 300*abs(msg.position[0].y - msg.position[1].y)<< " paye"<<endl;
                    ++LH_wave_counter;
                }
                else
                {
                    if(LH_wave_counter > 0)
                         --LH_wave_counter;  
                    else
                    LH_wave_detect = false;   
                }
                if(LH_wave_counter>4)
                {
                    LH_wave_detect = true; 
                }
                x_LH_prev = msg.position[5].x; 
        }
        else
        {
            LH_wave_counter = 0;
            x_LH_prev = 0;
            LH_wave_detect = false;
        }
    }
 }
bool wave::right_hand_wave_detect()
{
    return RH_wave_detect;
}
bool wave::left_hand_wave_detect()
{
    return LH_wave_detect;
}