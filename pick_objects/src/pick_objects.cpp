#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <std_msgs/Int8.h>



// Define a client for to send goal requests to the move_base server through a SimpleActionClient
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

int main(int argc, char** argv){
  // Initialize the simple_navigation_goals node
  ros::init(argc, argv, "pick_objects");

  //tell the action client that we want to spin a thread by default
  MoveBaseClient ac("move_base", true);

  // Wait 5 sec for move_base action server to come up
  while(!ac.waitForServer(ros::Duration(5.0))){
    ROS_INFO("Waiting for the move_base action server to come up");
  }

  move_base_msgs::MoveBaseGoal goal;

  // set up the frame parameters
  goal.target_pose.header.frame_id = "map";
  goal.target_pose.header.stamp = ros::Time::now();

  // Define a position and orientation for the robot to reach
  goal.target_pose.pose.position.x = 0;
  goal.target_pose.pose.position.y = -2;
  goal.target_pose.pose.orientation.w = 1.0;


  std_msgs::Int8 msg;
  
  ros::NodeHandle n;
  ros::Rate r(1);
  ros::Publisher location_state_pub = n.advertise<std_msgs::Int8>("location_state", 1);
 
 

   // Send the goal position and orientation for the robot to reach
  ROS_INFO("Sending goal");
  ac.sendGoal(goal);

  // Wait an infinite time for the results
  ac.waitForResult();

  // Check if the robot reached its goal
  if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
  {
    msg.data = 1;
    location_state_pub.publish(msg);    
    ROS_INFO("The robot has reached its pickup destination!");
  }
  else
    ROS_INFO("The robot failed to reach its pickup destination.");

  ros::Duration(5).sleep();

  // set up the frame parameters 
  goal.target_pose.header.stamp = ros::Time::now();

  // Define a position and orientation for the robot to reach
  goal.target_pose.pose.position.x = -7;
  goal.target_pose.pose.position.y = 0;
  goal.target_pose.pose.orientation.w = 1.0;

  ROS_INFO("The robot is going to the drop off location.");
  ac.sendGoal(goal);

 // Wait an infinite time for the results
  ac.waitForResult();

  // Check if the robot reached its goal
  if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
  {
    msg.data = 2;
    location_state_pub.publish(msg);  
    ROS_INFO("The robot has reached the drop off location!");
  }
  else
    ROS_INFO("The robot failed to reach the drop off location.");

  ros::Duration(5).sleep();

  return 0;
}
