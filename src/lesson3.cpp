/**
 * @file lesson3.cpp
 * @breif periodic tf broadcaster
 */

#include <ros/ros.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <geometry_msgs/TransformStamped.h>

using namespace std;

void tf_periodic_example()
{
  // 内部でtfの情報をkeepするので寿命を長く取ること．
  // tfのbroadcastはtopicのpublishと同じ処理であって，TransformBroadcasterでは
  // ros::NodeHandle::advertise<tf2_msgs::TFMessage>("/tf", 100, false)でlatch=falseになっている
  static tf2_ros::TransformBroadcaster br;

  vector<geometry_msgs::TransformStamped> tfs;
  double diff_angle = 36;
  for (double angle = 0; angle < 360; angle+=diff_angle)
  {
    double rad = 5.0;
    geometry_msgs::TransformStamped tf;
    tf.header.frame_id = "world";
    tf.child_frame_id = "base_link";
    tf.transform.translation.x = rad*cos(angle*M_PI/180.);
    tf.transform.translation.y = rad*sin(angle*M_PI/180.);
    tf.transform.translation.z = 0;
    tf2::Quaternion q;
    q.setRPY(0, 0, (180+angle)*M_PI/180.);
    tf.transform.rotation.w = q.w();
    tf.transform.rotation.x = q.x();
    tf.transform.rotation.y = q.y();
    tf.transform.rotation.z = q.z();
    tfs.push_back(tf);
  }

  ros::Rate loop(1);
  int index = 0;
  while (ros::ok())
  {
    geometry_msgs::TransformStamped tf = tfs[index];
    index = (index == tfs.size()-1) ? 0 : index + 1;
    tf.header.stamp = ros::Time::now();
    br.sendTransform(tf);
    loop.sleep();
  }
  ROS_INFO("Broadcasted tf");
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "lesson3");
  ros::NodeHandle nh;

  tf_periodic_example();

  ros::spin();
  return 0;
}
