/**
 * @file lesson2.cpp
 * @brief Listen tf at current time, 1sec ago, and latest tf-available time
 */

#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <visualization_msgs/MarkerArray.h>

using namespace std;
using namespace visualization_msgs;

Marker testMarker(const geometry_msgs::TransformStamped& tf, string color, int id, bool is_delete=false)
{
  if (is_delete)
  {
    Marker m;
    m.id = id;
    m.action = Marker::DELETE;
    m.header.stamp = ros::Time::now();
    return m;
  }

  std_msgs::ColorRGBA color_msg;
  if (color == "red")
  {
    color_msg.r = 1; color_msg.g = 0; color_msg.b = 0; color_msg.a = 1;
  }
  else if (color == "green")
  {
    color_msg.r = 0; color_msg.g = 1; color_msg.b = 0; color_msg.a = 1;
  }
  else if (color == "blue")
  {
    color_msg.r = 0; color_msg.g = 0; color_msg.b = 1; color_msg.a = 1;
  }
  else
  {
    // black
    color_msg.r = 0; color_msg.g = 0; color_msg.b = 0; color_msg.a = 1;
  }

  Marker msg;
  msg.header = tf.header;
  msg.header.stamp = ros::Time::now();
  msg.id = id;
  msg.pose.position.x = tf.transform.translation.x;
  msg.pose.position.y = tf.transform.translation.y;
  msg.pose.position.z = tf.transform.translation.z;
  msg.pose.orientation = tf.transform.rotation;
  msg.type = Marker::MESH_RESOURCE;
  msg.mesh_resource = "package://tf_lessons/rviz/xyz_marker.stl";
  msg.mesh_use_embedded_materials = true;
  msg.color = color_msg;
  msg.scale.x = 5.0;
  msg.scale.y = 5.0;
  msg.scale.z = 5.0;

  return msg;
}

void tf_listener_example()
{
  // tf2_ros::Bufferにtfの履歴を保存する．
  // 履歴保存の長さのデフォルトは10sであり，コンストラクタの引数で変更可能．
  // tf2_ros::TransformListernerのメソッドを実行することは無いが/tf,/tf_staticをListen(subscribe)
  // するために必要なので実行すること．
  // ここでのstatic宣言はlesson1と違い，whileでこの関数を抜けないのでstaticはなくてよい
  static tf2_ros::Buffer tf_buffer;
  static tf2_ros::TransformListener tf_listener(tf_buffer);

  ros::NodeHandle nh;
  ros::Publisher marker_pub = nh.advertise<MarkerArray>("test_marker", 1);
  MarkerArray mrk_msg;
  mrk_msg.markers.resize(3);

  //ros::Rate loop(0.5); // 1loop for 2sec
  ros::Rate loop(10); // 10hz
  while (ros::ok())
  {
    geometry_msgs::TransformStamped tf_now;
    try
    {
      tf_now = tf_buffer.lookupTransform("world", "base_link", ros::Time::now());
      auto stamp = tf_now.header.stamp;
      ROS_INFO("current-time<%d.%09d> tf has been listened", stamp.sec, stamp.nsec);
    }
    catch(tf2::TransformException &ex)
    {
      ROS_WARN("current-time tf couldn't be listened: %s", ex.what());
    }

    geometry_msgs::TransformStamped tf_maybe_available;
    try
    {
      tf_maybe_available = tf_buffer.lookupTransform("world", "base_link", ros::Time(0));
      auto stamp = tf_maybe_available.header.stamp;
      ROS_INFO("maybe-available<%d.%09d> tf has been listened", stamp.sec, stamp.nsec);
      mrk_msg.markers[0] = testMarker(tf_maybe_available, "red", 0);
    }
    catch(tf2::TransformException &ex)
    {
      ROS_WARN("maybe-available ago time tf couldn't be listened: %s", ex.what());
      mrk_msg.markers[0] = testMarker(tf_maybe_available, "", 0, true);
    }

    geometry_msgs::TransformStamped tf_1sec_ago;
    try
    {
      tf_1sec_ago = tf_buffer.lookupTransform("world", "base_link", ros::Time::now()-ros::Duration(1.0));
      auto stamp = tf_1sec_ago.header.stamp;
      ROS_INFO("one-sec-ago<%d.%09d> tf has been listened", stamp.sec, stamp.nsec);
      mrk_msg.markers[1] = testMarker(tf_1sec_ago, "green", 1);
    }
    catch(tf2::TransformException &ex)
    {
      ROS_WARN("one-sec-ago time tf couldn't be listened: %s", ex.what());
      mrk_msg.markers[1] = testMarker(tf_1sec_ago, "", 1, true);
    }

    geometry_msgs::TransformStamped tf_10sec_ago;
    try
    {
      tf_10sec_ago = tf_buffer.lookupTransform("world", "base_link", ros::Time::now()-ros::Duration(10.0));
      auto stamp = tf_10sec_ago.header.stamp;
      ROS_INFO("ten-sec-ago<%d.%09d> tf has been listened", stamp.sec, stamp.nsec);
      mrk_msg.markers[2] = testMarker(tf_10sec_ago, "blue", 2);
    }
    catch(tf2::TransformException &ex)
    {
      ROS_WARN("ten-sec-ago time tf couldn't be listened: %s", ex.what());
      mrk_msg.markers[2] = testMarker(tf_10sec_ago, "", 2, true);
    }

    marker_pub.publish(mrk_msg);

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
