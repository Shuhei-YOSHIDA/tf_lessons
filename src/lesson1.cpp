/**
 * @file lesson1.cpp
 * @breif fundamental method of tf broadcaster
 */

#include <ros/ros.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>

using namespace std;

void tf_static_example()
{
  ROS_INFO("tf static example lesson1");
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

void tf_non_static_example()
{
  ROS_INFO("tf non-static example lesson1");
  static tf2_ros::TransformBroadcaster s_br;

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

  for (int i = 0; i < 11; i++)
  {
    tf.header.stamp = ros::Time::now();
    s_br.sendTransform(tf);
    ROS_INFO("Broadcasted (non-static) tf");
    ros::Duration(1).sleep();
  }
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "lesson1");
  ros::NodeHandle nh;

  if (argc != 2)
  {
    tf_static_example();
  }
  else
  {
    if (string(argv[1]) == "non-static") tf_non_static_example();
    else tf_static_example();
  }

  ros::spin();
  return 0;
}
