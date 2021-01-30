/**
 * @file tf_lesson2.cpp
 * @brief Listen tf at current time, 1sec ago, and latest tf-available time
 */

#include "geometry_msgs/Transform.h"
#include "geometry_msgs/TransformStamped.h"
#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <visualization_msgs/MarkerArray.h>

void tf_listener_example()
{
  static tf2_ros::Buffer tf_buffer;
  static tf2_ros::TransformListener tf_listener(tf_buffer);

  ros::Rate loop(0.5); // 1loop for 2sec
  while (ros::ok())
  {
    geometry_msgs::TransformStamped tf_now;
    try
    {
      tf_now = tf_buffer.lookupTransform("world", "base_link", ros::Time::now());
      ROS_INFO("current-time tf has been listened");
    }
    catch(tf2::TransformException &ex)
    {
      ROS_WARN("current-time tf couldn't be listened: %s", ex.what());
    }
    geometry_msgs::TransformStamped tf_1sec_ago;
    try
    {
      tf_now = tf_buffer.lookupTransform("world", "base_link", ros::Time::now()-ros::Duration(1.0));
      ROS_INFO("one-sec-ago tf has been listened");
    }
    catch(tf2::TransformException &ex)
    {
      ROS_WARN("one-sec-ago time tf couldn't be listened: %s", ex.what());
    }
    geometry_msgs::TransformStamped tf_maybe_available;
    try
    {
      tf_now = tf_buffer.lookupTransform("world", "base_link", ros::Time(0));
      ROS_INFO("maybe-available tf has been listened");
    }
    catch(tf2::TransformException &ex)
    {
      ROS_WARN("maybe-available ago time tf couldn't be listened: %s", ex.what());
    }

    loop.sleep();
  }

  ROS_INFO("Listened tf");
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "tf_lesson2");
  ros::NodeHandle nh;

  tf_listener_example();

  ros::spin();
  return 0;
}
