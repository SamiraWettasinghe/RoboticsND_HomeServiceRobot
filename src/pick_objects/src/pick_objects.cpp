#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include "pick_objects/DriveLocation.h"

// Define a client for to send goal requests to the move_base server through a SimpleActionClient
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

// callback function for when service is requested
bool drive_location(pick_objects::DriveLocation::Request& req, pick_objects::DriveLocation::Response& res)
{
  ROS_INFO("Location Received - X: %1.2f, Y: %1.2f, W: %1.2f", (float)req.x_loc, (float)req.y_loc, (float)req.w_rot);

  if (req.first)
    MoveBaseClient ac("move_base", true);

    while(!ac.waitForServer(ros::Duration(5.0))){
      ROS_INFO("Waiting for the move_base action server to come up");
    }

    move_base_msgs::MoveBaseGoal goal;

    goal.target_pose.header.frame_id = "map";
    goal.target_pose.header.stamp = ros::Time::now();

  goal.target_pose.pose.position.x = req.x_loc;
  goal.target_pose.pose.position.y = req.y_loc;
  goal.target_pose.pose.orientation.w = req.w_rot;

  ROS_INFO("Sending location");
  ac.sendGoal(goal);
  ac.waitForResult();

  if (ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    ROS_INFO("Location reached successfully");
    return true;

  else
    ROS_INFO("The base failed to move");
    return false;

  ros::Duration(1.0).sleep(); 
}

int main(int argc, char** argv){
  ros::init(argc, argv, "pick_objects_node");
  ros::NodeHandle n;

  // start the drive location service
  ros::ServiceServer service = n.advertiseService("/add_markers/new_location", drive_location);
  ROS_INFO("Ready to receive the location");

  ros::spin();

  return 0;
}
