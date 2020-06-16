	#include "skeletonclient.h"
 
 using namespace std;
skeletonclient::skeletonclient()
 {

 	
     sub_[0] = nh_[0].subscribe("skeleton", 1000, &skeletonclient::SkeletonData_CB,this);
    
    service_gesturedetect = nh_[3].advertiseService("skeletonin_gesturedetect", &skeletonclient::checkgesturdetectRequest,this);
    service_jointPO = nh_[4].advertiseService("skeletonin_jointPO", &skeletonclient::checkJointPORequest,this);

    gestureDetectPublisher = nh_[5].advertise<upperbodycore_msgs::GestureDetect>("skeletonout_gestureDetect", 1);
    JointPOPublisher= nh_[6].advertise<upperbodycore_msgs::jointPO>("skeletonout_jointPO",1);

    PAOS = false;
    GDS = false;
	SkData.user_id = -1;


    right_hand_posture = "NotSet";
    left_hand_posture = "NotSet";
    right_leg_posture = "NotSet";
    left_leg_posture = "NotSet";
    JointsNameA[0]="head";
    JointsNameA[1]="neck";
    JointsNameA[2] = "torso";
    JointsNameA[3] = "left_shoulder";
	JointsNameA[4] = "left_elbow";
	JointsNameA[5] = "left_hand";
	JointsNameA[6] = "right_shoulder";
	JointsNameA[7] = "right_elbow";
	JointsNameA[8] = "right_hand";
	JointsNameA[9] = "left_hip";
	JointsNameA[10]= "left_knee";
	JointsNameA[11]= "left_foot";
	JointsNameA[12]= "right_hip";
	JointsNameA[13]= "right_knee";
	JointsNameA[14]= "right_foot";
 }
 skeletonclient::~skeletonclient()
 {
 	
 }

int skeletonclient::get_user_id()
{
	return SkData.user_id;
}
 bool skeletonclient::checkJointPORequest(upperbodycore_msgs::skeletonSrv::Request  &req,upperbodycore_msgs::skeletonSrv::Response &res)
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
 bool skeletonclient::checkgesturdetectRequest(upperbodycore_msgs::skeletonSrv::Request  &req, upperbodycore_msgs::skeletonSrv::Response &res)
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
 
 void skeletonclient::gesture_detect_enable()
 {
 	GDS = true;
 }
 void skeletonclient::get_position_orientation_enable()
 {
 	PAOS = true;
 }
 void skeletonclient::SkeletonData_CB(const skeleton_markers::Skeleton msg)
 {
 	SkData = msg;
    // cout<< msg.name[0]<<" name "<<endl;
    // cout<<msg.orientation[2]<<" head orientation"<<endl;
    if(GDS)
    {
      GestureDetect();  
      // cout<<"get gesuter"<<endl;
    }
    else if(!GDS)
         cout<<"false GDS"<<endl;

    if(PAOS)
    {
        getPositionAndOrientation();
        // cout<<" get position"<<endl;
    }
 }

 void skeletonclient::getPositionAndOrientation()
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
	
 }
void skeletonclient::GestureDetect()
{
	skeleton_markers::Skeleton msg = SkData;
    // ***********************right hand**************************//
    right_hand_posture = "NotSet";
    left_hand_posture = "NotSet";
    right_leg_posture = "NotSet";
    left_leg_posture = "NotSet";

    if(msg.position[8].y>(msg.position[0].y+0.2*abs(msg.position[0].y-msg.position[1].y)))
    {
        if((msg.confidence[8]>0.5)&&(msg.confidence[0]>0.5))
        {
            right_hand_posture = "Up";
        }
    }
    else if(abs(msg.position[8].x-msg.position[6].x)>((1.5)*abs(msg.position[6].x-msg.position[0].x)))
    {
        if((msg.confidence[8]>0.5)&&(msg.confidence[6]>0.5))
        {
            right_hand_posture = "Side";
        }
    }
    else if(msg.position[8].z<(msg.position[0].z-1.5*abs(msg.position[1].y-msg.position[2].y)))//be jaye 0.2 tafazole y torso va neck
    {
        if((msg.confidence[8]>0.5)&&(msg.confidence[0]>0.5)&&(msg.confidence[2]>0.5)&&(msg.confidence[1]>0.5))
        {
            right_hand_posture = "Front";
        }
    }
    else if(msg.position[8].y<(msg.position[2].y))
    {
        if((msg.confidence[8]>0.5)&&(msg.confidence[2]>0.5))
        {
             right_hand_posture = "Down";
        }
    }

    //***********************left hand**************************//
    if(msg.position[5].y>(msg.position[0].y+0.20*abs(msg.position[0].y-msg.position[1].y)))
    {
        if((msg.confidence[5]>0.5)&&(msg.confidence[0]>0.5))
        {
             left_hand_posture = "Up";
        }
    }
    else if(abs(msg.position[5].x-msg.position[3].x)>1.5*abs(msg.position[3].x-msg.position[0].x))
    {
        if((msg.confidence[5]>0.5)&&(msg.confidence[3]>0.5))
        {
			   left_hand_posture = "Side";
        }
    }
    else if(msg.position[5].z<(msg.position[0].z-1.5*abs(msg.position[1].y-msg.position[2].y)))//be jaye 0.2 tafazole y torso va neck
    {
        if((msg.confidence[5]>0.5)&&(msg.confidence[0]>0.5)&&(msg.confidence[2]>0.5)&&(msg.confidence[1]>0.5))
        {
        	left_hand_posture = "Front";
        }
    }
    else if(msg.position[5].y<(msg.position[2].y))
    {
        if((msg.confidence[5]>0.5)&&(msg.confidence[2]>0.5))
        {
        	left_hand_posture = "Down";
        }
    }

    //***********************left foot**************************//
    if(msg.position[11].x<(msg.position[9].x-1.5*abs(msg.position[9].x-msg.position[2].x)))//paye chape man mishe paye raste in baraye dastesh in juri nabud
    {
        if((msg.confidence[11]>0.5)&&(msg.confidence[9]>0.5)&&(msg.confidence[2]>0.5))
        {
        	left_leg_posture = "Side";
        }
    }
    else if(msg.position[11].z<(msg.position[2].z-abs(msg.position[9].x-msg.position[2].x)))//paye chape man mishe paye raste in baraye dastesh in juri nabud
    {
        if((msg.confidence[11]>0.5)&&(msg.confidence[9]>0.5)&&(msg.confidence[2]>0.5))
        {
        	left_leg_posture = "Front";
        }
    }
    else 
    {
        if((msg.confidence[11]>0.5)&&(msg.confidence[9]>0.5)&&(msg.confidence[2]>0.5)&&(msg.confidence[14]>0.5))
        {
        	left_leg_posture = "Down";
        }
    }

    //***********************right foot**************************//

    if(msg.position[14].x>(msg.position[12].x+1.5*abs(msg.position[12].x-msg.position[2].x)))//paye chape man mishe paye raste in baraye dastesh in juri nabud
    {
        if((msg.confidence[14]>0.5)&&(msg.confidence[12]>0.5)&&(msg.confidence[2]>0.5))
        {
            right_leg_posture = "Side";
        }
    }
    else if(msg.position[14].z<(msg.position[2].z-abs(msg.position[12].x-msg.position[2].x)))//paye chape man mishe paye raste in baraye dastesh in juri nabud
    {
        if((msg.confidence[14]>0.5)&&(msg.confidence[9]>0.5)&&(msg.confidence[2]>0.5))
        {
            right_leg_posture = "Front";
        }
    }
    else 
    {
        if((msg.confidence[14]>0.5)&&(msg.confidence[12]>0.5)&&(msg.confidence[2]>0.5)&&(msg.confidence[14]>0.5))
        {
            right_leg_posture = "Down";
        }
    }

    jestureDetectmsg.right_hand = right_hand_posture;
    jestureDetectmsg.left_hand = left_hand_posture;
    jestureDetectmsg.right_foot = right_leg_posture;
    jestureDetectmsg.left_foot = left_leg_posture;
    gestureDetectPublisher.publish(jestureDetectmsg);

}

 geometry_msgs::Vector3 skeletonclient::get_joint_position(std::string JointName)
 {
 		geometry_msgs::Vector3 jointP;
 		jointP.x = 0;
 		jointP.y = 0;
 		jointP.z = 0;

 		if(SkData.user_id != -1)
 		{
	 		for(int k = 0; k<15; ++k)
	 		{
	 			if(JointsNameA[k]==JointName)
	 			{
	        		jointP.x= SkData.position[k].x;
	        		jointP.y= SkData.position[k].y;
	        		jointP.z= SkData.position[k].z;
	        	}
	        }
	    }
 	return jointP;
 }
 geometry_msgs::Quaternion skeletonclient::get_joint_orientation(std::string JointName)
 {
	 	geometry_msgs::Quaternion jointO;
	jointO.x = 0;
	jointO.y = 0;
	jointO.z = 0;
	jointO.w = 0;
 	if (SkData.user_id != -1)
 	{
	 	for(int k = 0; k<15; ++k)
	 		{
	 			if(JointsNameA[k]==JointName)
	 			{
			        jointO.x = SkData.orientation[k].x;
			        jointO.y = SkData.orientation[k].y;
			        jointO.z = SkData.orientation[k].z;
			        jointO.w = SkData.orientation[k].w;
	            }
			}
 	}
 	return jointO;
 }
 string skeletonclient::get_right_hand_posture(){
 	return right_hand_posture;
 }
 string skeletonclient::get_left_hand_posture(){
 	return left_hand_posture;
 }
 string skeletonclient::get_right_leg_posture(){
 	return right_leg_posture;
 }
 string skeletonclient::get_left_leg_posture(){
 	return left_leg_posture;
 }
 skeleton_markers::Skeleton skeletonclient::getSKData()
 {

 }
  