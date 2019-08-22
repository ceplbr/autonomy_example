#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <tf/tf.h>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

/* Target coordinates */
float position_x = -0.578879380226; 
float position_y = 0;
float quat_x = 0; 
float quat_y = 0;
float quat_z = 0; 
float quat_w = 1;

int main(int argc, char** argv){
	ros::init(argc, argv, "send_simple_goal");
	ros::NodeHandle nh("~");
	double param_x = (double)position_x;
	double param_y = (double)position_y;

	double param_qx = (double)quat_x;
	double param_qy = (double)quat_y;
	double param_qz = (double)quat_z;
	double param_qw = (double)quat_w;

	double param_a = 0;

	nh.getParam("x", param_x);
	nh.getParam("y", param_y);
	if(nh.getParam("a", param_a)){
		/* Orientation */
    	tf::Matrix3x3 obs_matS;
    	double target_angle = (double)param_a;
    	obs_matS.setEulerYPR(target_angle,0,0);
    	tf::Quaternion q_tfS;
    	obs_matS.getRotation(q_tfS);
		param_qx = q_tfS.getX();
		param_qy = q_tfS.getY();
		param_qz = q_tfS.getZ();
		param_qw = q_tfS.getW();
	}

	ROS_INFO("Using parameters: x=%f, y=%f, qx=%f, qy=%f, qz=%f, qw=%f", param_x, param_y, param_qx, param_qy, param_qz, param_qw);
	
	sleep(3);
	ROS_INFO("Running..");

	//tell the action client that we want to spin a thread by default
	MoveBaseClient ac("move_base", true);

	//wait for the action server to come up
	while(!ac.waitForServer(ros::Duration(5.0))){
	ROS_INFO("Waiting for the move_base action server to come up");
	}
	ROS_INFO("Server is online");
	ROS_INFO("--------------------------------");
	ROS_INFO("Sending goal");
	move_base_msgs::MoveBaseGoal goal;
	goal.target_pose.header.frame_id = "map";

	//goal.target_pose.header.stamp = ros::Time::now();
	goal.target_pose.header.stamp = ros::Time(0);

	/* Position */
	goal.target_pose.pose.position.x = param_x;
	goal.target_pose.pose.position.y = param_y;

	/* Orientation */
	goal.target_pose.pose.orientation.x = param_qx;
	goal.target_pose.pose.orientation.y = param_qy;
	goal.target_pose.pose.orientation.z = param_qz;
	goal.target_pose.pose.orientation.w = param_qw;

	/* Send goal */
	ac.sendGoal(goal);
	
	/* Wait for outcome */
	ac.waitForResult();

	/* Process result */
	if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED){
		ROS_INFO("Goal reached");
	}
	else {
		ROS_INFO("The base failed to move forward for some reason");
	}
	return 0;
}
