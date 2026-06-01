#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import tf
from geometry_msgs.msg import PoseWithCovarianceStamped

class FakeInitialPose:
    def __init__(self):
        rospy.init_node('fake_initial_pose')
        self.br = tf.TransformBroadcaster()
        
        # 默认停在原点
        self.x, self.y, self.z = 0.0, 0.0, 0.0
        self.qx, self.qy, self.qz, self.qw = 0.0, 0.0, 0.0, 1.0
        
        # 监听 RViz 里的 "2D Pose Estimate" 工具下发的话题
        rospy.Subscriber('/initialpose', PoseWithCovarianceStamped, self.pose_cb)
        
        # 以 20Hz 的频率持续广播 map 到 odom 的坐标变换
        self.timer = rospy.Timer(rospy.Duration(0.05), self.publish_tf)
        rospy.loginfo("等待在 RViz 中使用 2D Pose Estimate 选取起始位置...")

    def pose_cb(self, msg):
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        self.x, self.y, self.z = p.x, p.y, 0.0
        self.qx, self.qy, self.qz, self.qw = q.x, q.y, q.z, q.w
        rospy.loginfo("✅ 已接收到新的起始位置并更新！")

    def publish_tf(self, event):
        # 动态发布 map -> odom
        self.br.sendTransform((self.x, self.y, self.z),
                              (self.qx, self.qy, self.qz, self.qw),
                              rospy.Time.now(),
                              "odom",
                              "map")

if __name__ == '__main__':
    try:
        FakeInitialPose()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
