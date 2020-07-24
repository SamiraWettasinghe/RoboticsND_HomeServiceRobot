#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include "pick_objects/DriveLocation.h"

// http://wiki.ros.org/ROS/Tutorials/WritingServiceClient%28c%2B%2B%29

// Set up client to request to drive to new location
ros::ServiceClient client;
void drive_location(float x, float y, float w)
{
  add_markers::DriveLocation srv;
  srv.request.x_loc = x;
  srv.request.y_loc = y;
  srv.request.w_rot = w;

  if (!client.call(srv)) {
    ROS_ERROR("Failed to call service")
  }
}

int main( int argc, char** argv )
{
  // Set up cube visualization_marker
  ros::init(argc, argv, "add_markers");
  ros::NodeHandle n;
  ros::Rate r(1);
  ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1);
  client = n.ServiceClient<add_markers::DriveLocation>("/add_markers/new_location");

  uint32_t shape = visualization_msgs::Marker::CUBE;

  while (ros::ok())
  {
    visualization_msgs::Marker marker;
    marker.header.frame_id = "map";
    marker.header.stamp = ros::Time::now();

    marker.ns = "add_markers";
    marker.id = 0;

    marker.type = shape;

    marker.action = visualization_msgs::Marker::ADD;

    marker.pose.position.x = -6.6;
    marker.pose.position.y = 2.0;
    marker.pose.position.z = 0;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;

    marker.scale.x = 0.25;
    marker.scale.y = 0.25;
    marker.scale.z = 0.25;

    marker.color.r = 0.0f;
    marker.color.g = 0.0f;
    marker.color.b = 1.0f;
    marker.color.a = 1.0;

    marker.lifetime = ros::Duration();

    while (marker_pub.getNumSubscribers() < 1)
    {
      if (!ros::ok())
      {
        return 0;
      }
      ROS_WARN_ONCE("Please create a subscriber to the marker");
      sleep(1);
    }

    drive_location(-6.6, 2.0, 1.571);
    marker_pub.publish(marker);
    ros::spinOnce();

    marker.action = visualization_msgs::Marker::DELETE;
	  marker_pub.publish(marker);
    ROS_INFO("Marker deleted");

    ros::Duration(1.0).sleep();
    marker.pose.position.x = -0.8;
    marker.pose.position.y = -5;
    marker.action = visualization_msgs::Marker::ADD;
    ROS_INFO("Publishing drop off marker");
    marker_pub.publish(marker);
    drive_location(-0.8, -5, 1.571);

    ros::Duration(5.0).sleep();
    return 0;
  }
}
