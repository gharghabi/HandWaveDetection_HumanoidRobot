#include <ros/ros.h>
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>

#include <upperbodycore_msgs/jointPO.h>
#include <upperbodycore_msgs/GestureDetect.h>

#include <skeleton_markers/Skeleton.h>
#include "upperbodycore_msgs/skeletonSrv.h"
#include "std_msgs/Int32.h"
#include "std_msgs/String.h"

class skeletonclient
{

	private:
	    ros::NodeHandle nh_[7];
	    ros::Subscriber sub_[4];
		ros::Publisher JointPOPublisher;
		ros::Publisher gestureDetectPublisher;
	    ros::ServiceServer service_gesturedetect;	    // upperbodycore_msgs::motorreset reset;
	    ros::ServiceServer service_jointPO;

	    skeleton_markers::Skeleton SkData;
	    std::string right_hand_posture;
	    std::string left_hand_posture;
	    std::string left_leg_posture;
	    std::string right_leg_posture;
		std::string JointsNameA[15];
		upperbodycore_msgs::GestureDetect jestureDetectmsg;
		upperbodycore_msgs::jointPO jointPoOr;

	    int user_id;
	    bool PAOS;//position and orientation start
	    bool GDS;//gesture detect start
	    void SkeletonData_CB(const skeleton_markers::Skeleton msg);
	    void GestureDetect();
	    void getPositionAndOrientation();

	public:
		bool checkJointPORequest(upperbodycore_msgs::skeletonSrv::Request  &req,upperbodycore_msgs::skeletonSrv::Response &res);
		bool checkgesturdetectRequest(upperbodycore_msgs::skeletonSrv::Request  &req, upperbodycore_msgs::skeletonSrv::Response &res);

		void gesture_detect_enable();//dar in halat tamame etelaate aza ruye topic khoriji midam
		void get_position_orientation_enable();
		
		int get_user_id();

		std::string get_right_hand_posture();//get right hand posture
		std::string get_left_hand_posture();//get left hand posture
		std::string get_right_leg_posture();//get right foot posture
		std::string get_left_leg_posture();//ger left foot posture
		geometry_msgs::Vector3 get_joint_position(std::string JointName);
		geometry_msgs::Quaternion get_joint_orientation(std::string JointName);

	
		skeleton_markers::Skeleton getSKData();
	    skeletonclient();
	    ~skeletonclient();

};
