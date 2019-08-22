#!/usr/bin/env python


#import roslib; roslib.load_manifest('rbx1_nav')
import rospy
import actionlib
from actionlib_msgs.msg import *
#from geometry_msgs.msg import Pose, Point, Quaternion, Twist
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

class SendGoal():
    def __init__(self):
        rospy.init_node('send_fixed_goal_py', anonymous=False)
            
        # Subscribe to the move_base action server
        self.move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        rospy.loginfo("Waiting for move_base action server...")
        
        # Wait 60 seconds for the action server to become available
        self.move_base.wait_for_server(rospy.Duration(60))
        
        rospy.loginfo("Connected to move base server")
        rospy.loginfo("Starting navigation test")
 
        # Intialize the waypoint goal
        goal = MoveBaseGoal()
        
        # Use the map frame to define goal poses
        goal.target_pose.header.frame_id = 'map'
        
        # Set the time stamp to "now"
        goal.target_pose.header.stamp = rospy.Time(0)

        # Position 
        goal.target_pose.pose.position.x = -3.93;
        goal.target_pose.pose.position.y = -2.82;

        # Orientation
        goal.target_pose.pose.orientation.x = 0;
        goal.target_pose.pose.orientation.y = 0;
        goal.target_pose.pose.orientation.z = 0.7;
        goal.target_pose.pose.orientation.w = 0.7;
            
        # Send the goal pose to the MoveBaseAction server
        self.move_base.send_goal(goal)
        
        # Allow 1 minute to get there
        finished_within_time = self.move_base.wait_for_result(rospy.Duration(60)) 
        
        # If we don't get there in time, abort the goal
        if not finished_within_time:
            self.move_base.cancel_goal()
            rospy.loginfo("Timed out achieving goal")
        else:
            # We made it!
            state = self.move_base.get_state()
            if state == GoalStatus.SUCCEEDED:
                rospy.loginfo("Goal succeeded!")

if __name__ == '__main__':
    try:
        SendGoal()
    except rospy.ROSInterruptException:
        rospy.loginfo("Autonomy test finished.")