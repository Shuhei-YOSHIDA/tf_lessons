/**
 * @file tf_lesson3.cpp
 * @breif periodic tf broadcaster
 */

#include <ros/ros.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>

void tf_periodic_example()
{
  // 内部でtfの情報をkeepするので寿命を長く取ること．
  // tfのbroadcastはtopicのpublishと同じ処理であって，TransformBroadcasterでは
  // ros::NodeHandle::advertise<tf2_msgs::TFMessage>("/tf", 100, false)でlatch=falseになっている
  static tf2_ros::TransformBroadcaster br;

  geometry_msgs::TransformStamped tf;
  //tf.header.stamp = ros::Time::now();
  tf.header.frame_id = "world";
  tf.child_frame_id = "base_link";
  tf.transform.translation.x = 1.;
  tf.transform.translation.y = 2.;
  tf.transform.translation.z = 3.;
  tf.transform.rotation.w = 1.0;
  tf.transform.rotation.x = 0.0;
  tf.transform.rotation.y = 0.0;
  tf.transform.rotation.z = 0.0;

  ros::Rate loop(1);
  while (ros::ok())
  {
    tf.header.stamp = ros::Time::now();
    br.sendTransform(tf);
    loop.sleep();
  }
  ROS_INFO("Broadcasted tf");
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "tf_lesson3");
  ros::NodeHandle nh;

  tf_periodic_example();

  ros::spin();
  return 0;
}
