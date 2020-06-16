#include <ros/ros.h>
#include "upperbodycore_msgs/Skeleton.h"
class wave
{

	private:
	    	    
	    static int const jointNum =15;
	    ros::NodeHandle nh_;
	    ros::Subscriber sub_;
       upperbodycore_msgs::Skeleton SkData;
	    int RH_wave_counter;
	    int LH_wave_counter;
		float confidence;
		float x_RH_prev;
		float x_LH_prev;
		bool RH_wave_detect;
		bool LH_wave_detect;
        void SkeletonData_CB(const upperbodycore_msgs::Skeleton msg);
		void wave_detect();
	public:
	    wave();
	    ~wave();
		bool right_hand_wave_detect();
		bool left_hand_wave_detect();
};
