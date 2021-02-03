#!/usr/bin/env python3
import rospy

import tf2_ros
import geometry_msgs.msg

if __name__ == "__main__":
    rospy.init_node('lesson7')

    br = tf2_ros.TransformBroadcaster()
    t = geometry_msgs.msg.TransformStamped()

    t.header.frame_id = "world"
    t.child_frame_id = "added_link"
    t.transform.translation.x = -1
    t.transform.translation.y = -1
    t.transform.translation.z = -1
    t.transform.rotation.w = 1.0

    r = rospy.Rate(1) #1Hz
    while not rospy.is_shutdown():
        t.header.stamp = rospy.Time.now()
        t.transform.translation.x *= -1
        br.sendTransform(t)
        r.sleep()
