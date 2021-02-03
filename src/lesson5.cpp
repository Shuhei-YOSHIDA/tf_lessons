/**
 * @file lesson5.cpp
 */

#include <ros/ros.h>
#include <tf2_ros/transform_broadcaster.h>

using namespace std;

string base_name = "base_";
const double hz = 100;

void onetf(string world_frame_id, string base_frame_id, double x)
{
  static tf2_ros::TransformBroadcaster tf_br;

  ros::Rate loop(hz);
  //double t = 0;
  while (ros::ok())
  {
    geometry_msgs::TransformStamped tf;
    tf.header.frame_id = world_frame_id;
    tf.header.stamp = ros::Time::now();
    tf.child_frame_id = base_frame_id;
    tf.transform.rotation.w = 1.0;
    tf.transform.translation.y = x;

    //if (t > 2*M_PI) t = 0;
    //t += 2*M_PI/hz;
    //tf.transform.translation.x = x*sin(t);

    tf_br.sendTransform(tf);
    loop.sleep();
  }
}

void multitf(string world_frame_id, int num, double x)
{
  static tf2_ros::TransformBroadcaster tf_br;
  vector<geometry_msgs::TransformStamped> tfs;

  geometry_msgs::TransformStamped tf;
  tf.header.frame_id = world_frame_id;
  tf.child_frame_id = base_name + to_string(1);
  tf.transform.rotation.w = 1.0;
  tf.transform.translation.y = x;
  tfs.push_back(tf);
  for (int i = 1; i < num; i++)
  {
    geometry_msgs::TransformStamped tf;
    tf.header.frame_id = base_name + to_string(i);
    tf.child_frame_id = base_name + to_string(i+1);
    tf.transform.rotation.w = 1.0;
    tf.transform.translation.y = x;

    tfs.push_back(tf);
  }

  ros::Rate loop(hz);
  //double t = 0;
  while (ros::ok())
  {
    auto stamp = ros::Time::now();
    //if (t > 2*M_PI) t = 0;
    //t += 2*M_PI/hz;
    for (auto&& tf : tfs)
    {
      tf.header.stamp = stamp;
      //tf.transform.translation.x = x*sin(t);
    }
    tf_br.sendTransform(tfs);
    loop.sleep();
  }

}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "lesson5");
  ros::NodeHandle nh;

  if (argc == 5)
  {
    if (string(argv[1]) == "onetf")
    {
      string world_frame_id = string(argv[2]);
      string base_frame_id = string(argv[3]);
      double y = atof(argv[4]);
      ROS_INFO("Broadcast from %s to %s", world_frame_id.c_str(), base_frame_id.c_str());
      onetf(world_frame_id, base_frame_id, y);
    }
    else if (string(argv[1]) == "multitf")
    {
      string world_frame_id = string(argv[2]);
      int num = atoi(argv[3]);
      double y = atof(argv[4]);
      ROS_INFO("Broadcast multi-tf[%d] from %s to ...", num,  world_frame_id.c_str());
      multitf(world_frame_id, num, y);
    }
    else
    {
      cerr << "example:" << endl
           << "$ rosrun tf_lesson lesson5 onetf world base_link 0.5 # single tf broadcast" << endl
           << "$ rosrun tf_lesson lesson5 multitf world 20 0.5 # multi tf broadcast" << endl;
    }
  }
  else
  {
    cerr << "example:" << endl
         << "$ rosrun tf_lesson lesson5 onetf world base_link 0.5 # single tf broadcast" << endl
         << "$ rosrun tf_lesson lesson5 multitf world 20 0.5 # multi tf broadcast" << endl;
  }

  return 0;
}
