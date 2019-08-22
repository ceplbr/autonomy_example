#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <tf/tf.h>


#define FIRSTPOS	1u
#define SECONDPOS	2u
#define THIRDPOS	3u

using namespace move_base_msgs;

volatile int target_state = FIRSTPOS;

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

void doneCb(const actionlib::SimpleClientGoalState& state,
            const MoveBaseResultConstPtr& result);
// Called once when the goal becomes active
void activeCb();
// Called every time feedback is received for the goal
void feedbackCb(const MoveBaseFeedbackConstPtr& feedback);

int main(int argc, char** argv){
	ros::init(argc, argv, "triangle_simple_master");
	ROS_INFO("Triangle simple master started, waiting for 3 seconds..");
	sleep(3);
	ROS_INFO("Running");

	//tell the action client that we want to spin a thread by default
	MoveBaseClient ac("move_base", true);

	//wait for the action server to come up
	while(!ac.waitForServer(ros::Duration(5.0))){
	ROS_INFO("Waiting for the move_base action server to come up");
	}
	while (ros::ok()){
		switch(target_state){
			case FIRSTPOS:
			{
				ROS_INFO("--------------------------------");
				ROS_INFO("Sending goal FIRSTPOS");
				move_base_msgs::MoveBaseGoal goal;
				//we'll send a goal to the robot to move 1 meter forward
				goal.target_pose.header.frame_id = "map";
				goal.target_pose.header.seq = 0;
				//goal.target_pose.header.stamp = ros::Time::now();
				goal.target_pose.header.stamp = ros::Time(0);

				/* Position */
				//goal.target_pose.pose.position.x = -0.035200715065;
				//goal.target_pose.pose.position.y = 1.52880525589;
				goal.target_pose.pose.position.x = -3.93;
				goal.target_pose.pose.position.y = -3;

				/* Orientation */
				goal.target_pose.pose.orientation.x = 0;
				goal.target_pose.pose.orientation.y = 0;
				goal.target_pose.pose.orientation.z = 0.7;
				goal.target_pose.pose.orientation.w = 0.7;
				ac.sendGoal(goal, &doneCb, &activeCb, &feedbackCb);

				ac.waitForResult();

				if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED){
					ROS_INFO("FIRSTPOS successful");
					target_state = SECONDPOS;
				}
			
				else {
					ROS_INFO("The base failed to move forward for some reason");
					target_state = FIRSTPOS;
				}
				break;

			}
			case SECONDPOS:
			{
				ROS_INFO("--------------------------------");
				ROS_INFO("Sending goal SECONDPOS");
				move_base_msgs::MoveBaseGoal goal;
				//we'll send a goal to the robot to move 1 meter forward
				goal.target_pose.header.frame_id = "map";
				//goal.target_pose.header.stamp = ros::Time::now();
				goal.target_pose.header.stamp = ros::Time(0);

				/* Position */
				goal.target_pose.pose.position.x = -5.32;
				goal.target_pose.pose.position.y = -1.9;//-0.15;
				//goal.target_pose.pose.position.y = 0.0450327682495; // old

				/* Orientation */
				goal.target_pose.pose.orientation.x = 0;
				goal.target_pose.pose.orientation.y = 0;
				goal.target_pose.pose.orientation.z = 0; //-0.7163849509;
				goal.target_pose.pose.orientation.w = 1; //0.697705240143;
				ac.sendGoal(goal, &doneCb, &activeCb, &feedbackCb);

				ac.waitForResult();
				if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED){
					ROS_INFO("SECONDPOS successful");
					target_state = THIRDPOS;	
				}
			
				else {
					ROS_INFO("The base failed to move forward for some reason");
					target_state = FIRSTPOS;
				}
				break;
			}
			case THIRDPOS:
			{
				ROS_INFO("--------------------------------");
				ROS_INFO("Sending goal THIRDPOS");
				move_base_msgs::MoveBaseGoal goal;
				//we'll send a goal to the robot to move 1 meter forward
				goal.target_pose.header.frame_id = "map";
				//goal.target_pose.header.stamp = ros::Time::now();
				goal.target_pose.header.stamp = ros::Time(0);

				/* Position */
				goal.target_pose.pose.position.x = -5.96;
				goal.target_pose.pose.position.y = -2.97;

				/* Orientation */
				goal.target_pose.pose.orientation.x = 0;
				goal.target_pose.pose.orientation.y = 0;
				goal.target_pose.pose.orientation.z = -0.00663892909955;
				goal.target_pose.pose.orientation.w = 0.999977962067;
				ac.sendGoal(goal, &doneCb, &activeCb, &feedbackCb);
				
				ac.waitForResult();

				if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED){
					ROS_INFO("THIRDPOS successful");
					target_state = FIRSTPOS;
				}
				else {
					ROS_INFO("The base failed to move forward for some reason");
					target_state = FIRSTPOS;
				}
				break;
			}
		}
	}
	return 0;
}

// Called once when the goal completes
void doneCb(const actionlib::SimpleClientGoalState& state,
            const MoveBaseResultConstPtr& result)
{
    ROS_INFO("Goal done");
}

// Called once when the goal becomes active
void activeCb()
{
    ROS_INFO("Goal just went active");
}

// Called every time feedback is received for the goal
void feedbackCb(const MoveBaseFeedbackConstPtr& feedback)
{
    ROS_INFO("Just got feedback");
    ROS_INFO("Got Feedback x: %f", feedback->base_position.pose.position.x);
    ROS_INFO("Got Feedback y: %f", feedback->base_position.pose.position.y);
}