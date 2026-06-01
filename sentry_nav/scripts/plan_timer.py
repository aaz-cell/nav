#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import time
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path

class PlanTimer:
    def __init__(self):
        # 初始化 ROS 节点
        rospy.init_node('plan_timer_node', anonymous=True)
        self.start_time = 0.0
        
        # 监听 RViz 发出的目标点指令 (通常是这个话题)
        rospy.Subscriber('/move_base_simple/goal', PoseStamped, self.goal_cb)
        
        # 监听 A* 算法规划出的路径
        # 注意：如果你的路径话题叫别的名字，请在这里修改！
        rospy.Subscriber('/move_base1/GlobalPlanner/plan', Path, self.plan_cb)
        
        rospy.loginfo("计时小助手已启动！正在等待 RViz 下发目标点...")

    def goal_cb(self, msg):
        # 记录收到目标点的时间戳 (使用系统最高精度时间)
        self.start_time = time.time()
        rospy.loginfo("🎯 收到新目标，A* 开始计算...")

    def plan_cb(self, msg):
        # 只要接收到了路径，立刻计算时间差
        if self.start_time > 0:
            calc_time = time.time() - self.start_time
            path_length = len(msg.poses) # 路径由多少个坐标点组成
            
            rospy.loginfo("\n" + "="*40)
            rospy.loginfo("✅ 路径规划完成！")
            rospy.loginfo("⏱️  A* 算法耗时 : {:.5f} 秒".format(calc_time))
            rospy.loginfo("📏 路径包含点数 : {} 个点".format(path_length))
            rospy.loginfo("="*40 + "\n")
            
            # 重置时间，等待下一次点击
            self.start_time = 0.0

if __name__ == '__main__':
    try:
        PlanTimer()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
