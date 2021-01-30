/**
 * @file tf_lesson1.cpp
 * @breif static tf broadcaster
 */

#include "ros/node_handle.h"
#include <ros/ros.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>

void tf_static_example()
{
  // 内部でtfの情報をkeepするので寿命を長く取ること．
  // このstaticを消して，この関数の最後にsleep(10)など入れて
  // tfがどれだけ利用可能か調べてみよう
  // tfのbroadcastはtopicのpublishと同じ処理であって，StaticTransformBroadcasterでは
  // ros::NodeHandle::advertise<tf2_msgs::TFMessage>("/tf_static", 100, true)でlatch=trueになっている
  static tf2_ros::StaticTransformBroadcaster s_br;

  geometry_msgs::TransformStamped tf;
  tf.header.stamp = ros::Time::now();
  tf.header.frame_id = "world";
  tf.child_frame_id = "base_link";
  tf.transform.translation.x = 1.;
  tf.transform.translation.y = 2.;
  tf.transform.translation.z = 3.;
  tf.transform.rotation.w = 1.0;
  tf.transform.rotation.x = 0.0;
  tf.transform.rotation.y = 0.0;
  tf.transform.rotation.z = 0.0;

  s_br.sendTransform(tf);
  ROS_INFO("Broadcasted tf");
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "tf_lesson1");
  ros::NodeHandle nh;

  tf_static_example();

  ros::spin();
  return 0;
}
