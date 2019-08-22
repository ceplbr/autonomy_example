#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <tf/tf.h>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

int main(int argc, char** argv){
	ros::init(argc, argv, "send_fixed_goal");
	ROS_INFO("Will send fixed goal, waiting for a second..");
	sleep(1);
	ROS_INFO("Running..");

	//tell the action client that we want to spin a thread by default
	MoveBaseClient ac("move_base", true);

	//wait for the action server to come up
	while(!ac.waitForServer(ros::Duration(5.0))){
	ROS_INFO("Waiting for the move_base action server to come up");
	}
	ROS_INFO("Server is online");
	ROS_INFO("--------------------------------");
	ROS_INFO("Sending fixed goal");
	move_base_msgs::MoveBaseGoal goal;
	goal.target_pose.header.frame_id = "map";

	//goal.target_pose.header.stamp = ros::Time::now();
	goal.target_pose.header.stamp = ros::Time(0);

	/* Position */
	goal.target_pose.pose.position.x = -3.93;
	goal.target_pose.pose.position.y = -2.82;

	/* Orientation */
	goal.target_pose.pose.orientation.x = 0;
	goal.target_pose.pose.orientation.y = 0;
	goal.target_pose.pose.orientation.z = 0.7;
	goal.target_pose.pose.orientation.w = 0.7;

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