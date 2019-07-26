# !/usr/bin/env python
# -*- coding: utf-8 -*-
import roslib; roslib.load_manifest('teleop_twist_keyboard')
import rospy
from geometry_msgs.msg import Twist, PoseStamped
from teleop_controller.msg import EnemyPos, ShootCmd, ModeSW

class Controller():
    def __init__(self):
        # 速度指令:仿真环境
        self.vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=100)
        # self.vel_pub = rospy.Publisher('robot_0/cmd_vel', Twist, queue_size=1)
        self.vel_pub_robot1 = rospy.Publisher('robot_1/cmd_vel', Twist, queue_size=1)

        #射击指令
        self.shoot_pub = rospy.Publisher('shoot_cmd', ShootCmd, queue_size=1)
        self.shoot_pub_robot1 = rospy.Publisher('robot_1/shoot_cmd', ShootCmd, queue_size=1)

        #模式切换
        self.modesw_pub = rospy.Publisher('switch_mode', ModeSW, queue_size=1)
        self.modesw_pub_robot1 = rospy.Publisher('robot_1/switch_mode', ModeSW, queue_size=1)

        #发布目标点
        self.nav_goal_pub = rospy.Publisher('move_base_simple/goal', PoseStamped, queue_size=10)
        self.nav_goal_pub_robot1 = rospy.Publisher('robot_1/move_base_simple/goal', PoseStamped, queue_size=1)

    # sendgoal
    def send_goal(self, goal):
        self.nav_goal_pub.publish(goal)

    def send_goal_robot1(self, goal):
        self.nav_goal_pub_robot1.publish(goal)

    # sendvel
    def send_vel(self, vel):
        self.vel_pub.publish(vel)

    def send_vel_robot1(self, vel):
        self.vel_pub_robot1.publish(vel)

    # shoot
    def shoot(self,shoot):
        self.shoot_pub.publish(shoot)

    def shoot_robot1(self,shoot):
        self.shoot_pub_robot1.publish(shoot)

    # mode
    def mode_switch(self, mode):
        self.modesw_pub.publish(mode)

    def mode_switch_robot1(self, mode):
        self.modesw_pub_robot1.publish(mode)


if __name__ == "__main__":
    teleop = Controller()

