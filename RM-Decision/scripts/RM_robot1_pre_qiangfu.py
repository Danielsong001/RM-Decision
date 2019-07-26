#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
from Battle import BattleEnv
from teleop_control import Controller
from nav_msgs.msg import Odometry
from teleop_controller.msg import EnemyPos, ShootCmd, ModeSW, RFID, YoloEnemy, Hurt, GameInfo, EnemyArea,Communite
import math
import numpy as np
import tf
from pi_trees_ros.pi_trees_ros import *
from geometry_msgs.msg import Twist, PoseStamped

# 定义静止命令
cmdvel_stop = Twist()
cmdvel_stop.linear.x = 0
cmdvel_stop.linear.y = 0
cmdvel_stop.linear.z = 0
cmdvel_stop.angular.x = 0
cmdvel_stop.angular.y = 0
cmdvel_stop.angular.z = 0

# 推车命令
cmdvel_blockedpush = Twist()
# for test

class BlackBoard():
    def __init__(self):
        self.sended_close = False
        self.sended_patrol = False
        self.patrol_flag = 0  # ptl点

        self.getbufftime1 = 0  # 到了6s就执行抢符
        self.getbufftimeflag = False  # 不要重复计时标志位

class BiuldTree():
    def __init__(self):
        self.blackboard = BlackBoard()
        rate = rospy.Rate(10)
        NORMALBAHAVE = Selector("NORMALBAHAVE")

        CLOSE = Sequence("CLOSE")
        PATROL = Sequence("PATROL")

        NORMALBAHAVE.add_child(CLOSE)
        NORMALBAHAVE.add_child(PATROL)

        ISCLOSE = IsClose("ISCLOSE", blackboard=self.blackboard)
        CLOSESHOOT = CloseShoot("CLOSESHOOT", blackboard=self.blackboard)
        CLOSE.add_child(ISCLOSE)
        CLOSE.add_child(CLOSESHOOT)

        ISPATROL = IsPatrol("ISPATROL", blackboard=self.blackboard)
        RANDOM = Random("RANDOM", blackboard=self.blackboard)
        PATROL.add_child(ISPATROL)
        PATROL.add_child(RANDOM)

        print "NORMALBAHAVE Tree"
        print_tree(NORMALBAHAVE)

        while 1:
            NORMALBAHAVE.status = NORMALBAHAVE.run()
            rate.sleep()

class IsClose(Task):
    def __init__(self, name, blackboard=None):
        super(IsClose, self).__init__(name)
        self.name = name
        self.blackboard = blackboard

    def run(self):
        if env.EnemyPoseSave_robot1.enemy_dist > 0:
            print 'r1 is close'
            return TaskStatus.SUCCESS
        else:
            return TaskStatus.FAILURE

    def reset(self):
        pass

class CloseShoot(Task):
    def __init__(self, name, blackboard=None):
        super(CloseShoot, self).__init__(name)
        self.name = name
        self.blackboard = blackboard
        self.goal_x = 0
        self.goal_y = 0
        self.goal_yaw = 0
        self.time1 = 0

    def run(self):
        self.blackboard.getbufftimeflag = False
        self.blackboard.sended_patrol = False
        if env.beentreeflag == 1:
            # 发布进入主树的消息
            if env.EnemyPoseSave_robot1.enemy_dist == 0:  # 当前无敌人；退一下
                goal_x = 3.85
                goal_y = 3.45
                goal_yaw = -65
                if env.isActionAvaliable(goal_x, goal_y, goal_yaw):  # front1
                    controller.send_goal_robot1(env.navgoal)
                rospy.sleep(1.5)
            print '!!!!!!!!!go into master tree'
            while 1:
                rospy.sleep(1)
            while 1:
                rospy.sleep(1)
        if self.blackboard.sended_close == False:
            self.goal_x = env.MyPose_robot1['x']
            self.goal_y = env.MyPose_robot1['y']
            if env.EnemyPoseSave_robot1.enemy_yaw > 0:  #枪管在左
                self.goal_yaw = env.MyPose_robot1['theta']+env.Gimbal_robot1.enemy_yaw-35  #与枪管成30度
            else:
                self.goal_yaw = env.MyPose_robot1['theta']+env.Gimbal_robot1.enemy_yaw+35  #与枪管成30度
            if self.goal_yaw > 180:
                self.goal_yaw = self.goal_yaw - 360
            if self.goal_yaw < -180:
                self.goal_yaw = self.goal_yaw + 360
            if env.isActionAvaliable(self.goal_x, self.goal_y, self.goal_yaw):  # 指向敌人
                controller.send_goal_robot1(env.navgoal)
                self.blackboard.sended_close = True
                self.time1 = rospy.Time.now().secs
                print 'r1： turn to enemy!!!!!!!!!!1'
            else:
                print 'r1: cant turn to enemy goal is %s, %s, %s' % (self.goal_x, self.goal_y, self.goal_yaw)
        if rospy.Time.now().secs - self.time1 > 3.5:
            self.blackboard.sended_close = False
        return TaskStatus.SUCCESS

    def reset(self):
        pass



class IsPatrol(Task):
    def __init__(self, name, blackboard=None):
        super(IsPatrol, self).__init__(name)
        self.name = name
        self.blackboard = blackboard

    def run(self):
        if env.EnemyPoseSave_robot1.enemy_dist == 0:  #看不到敌人或它们加到符了
            return TaskStatus.SUCCESS
        else:
            return TaskStatus.FAILURE

    def reset(self):
        pass

class Random(Task):
    def __init__(self, name, blackboard=None):
        super(Random, self).__init__(name)
        self.name = name
        self.count = 0
        self.patrol_yaw = True
        self.blackboard = blackboard
        self.goal_x = 0
        self.goal_y = 0
        self.goal_yaw = 0
        self.time1 = 0

    def run(self):
        self.blackboard.sended_close = False
        if env.beentreeflag == 1:
            # 发布进入主树的消息
            if env.EnemyPoseSave_robot1.enemy_dist == 0:  # 当前无敌人；退一下
                goal_x = 3.85
                goal_y = 3.45
                goal_yaw = -65
                if env.isActionAvaliable(goal_x, goal_y, goal_yaw):  # front1
                    controller.send_goal_robot1(env.navgoal)
                rospy.sleep(1.5)
            print '!!!!!!!!!go into master tree'
            while 1:
                rospy.sleep(1)
            while 1:
                rospy.sleep(1)
        else:  #正常的ptl点
            if self.blackboard.sended_patrol == False:
                if self.blackboard.patrol_flag == 0:
                    self.goal_x = 3.75
                    self.goal_y = 3.55
                    self.goal_yaw = -65
                    print 'ptl1 ttttttttttttttttttttttttttttttttttttttt'
                elif self.blackboard.patrol_flag == 1:
                    self.goal_x = 4.25  # 攻击点
                    self.goal_y = 3.15
                    self.goal_yaw = -50
                    print 'ptl2 ttttttttttttttttttttttttttttttttttttttttttt'
                else:
                    print 'ptl error!!!!!!!!!!!!!!!!'
                if env.isActionAvaliable(self.goal_x, self.goal_y, self.goal_yaw):  # front1
                    controller.send_goal_robot1(env.navgoal)
                    # print 'r0 patrol; goal is %s, %s, %s' % (self.goal_x, self.goal_y, self.goal_yaw)
                    self.time1 = rospy.Time.now().secs
                    self.blackboard.sended_patrol = True
                else:
                    print 'ptl error!!!!!!!!!!!!!!!!!!!!!!!!!!'
            else:
                if rospy.Time.now().secs - self.time1 > 2.5:
                    self.blackboard.sended_patrol = False
                    self.blackboard.patrol_flag = self.blackboard.patrol_flag + 1
                    if self.blackboard.patrol_flag == 2:
                        self.blackboard.patrol_flag = 0
                        print 'ptl end!!!!!!!!!!!!!!!!!!!!!!!!'
        return TaskStatus.SUCCESS

    def reset(self):
        pass

if __name__ == '__main__':
    rospy.init_node('RM_robot1_Pre')
    # 启动控制器和环境类
    controller = Controller()
    env = BattleEnv()
    tflistener = tf.TransformListener()

    # 标志位
    time1 = 0  # 记录进入buff后的时间
    time2 = 0  # 记录执行时间

    # 接受机器人状态消息
    rospy.Subscriber('gimbalpos', EnemyPos, env.getGimbalPoseCallback)
    rospy.Subscriber('robot_1/gimbalpos', EnemyPos, env.getGimbalPoseCallback_robot1)

    rospy.Subscriber('my_pose', Odometry, env.getSelfPoseCallback)
    rospy.Subscriber('robot_1/my_pose', Odometry, env.getSelfPoseCallback_robot1)
    # rospy.Subscriber('/robot_1/base_pose_ground_truth', Odometry, env.getSelfPoseCallback_robot1)

    rospy.Subscriber('enemy_pos', EnemyPos, env.getEnemyPoseCallback)
    rospy.Subscriber('robot_1/enemy_pos', EnemyPos, env.getEnemyPoseCallback_robot1)

    # 机器人0的裁判信息
    rospy.Subscriber('rfid', RFID, env.getRFIDCallback)
    rospy.Subscriber('hurt', Hurt, env.getHurtInfoCallback)
    rospy.Subscriber('gameinfo', GameInfo, env.getMyHPCallback)
    rospy.Subscriber('robot_1/gameinfo', GameInfo, env.getMyHPCallback_robot1)
    # rospy.Subscriber('yolo_enemy', YoloEnemy, env.getYoloEnemyCallback)  #不测试别用
    rospy.Subscriber('communite', Communite, env.getCommunite)

    # while 1:
    #     if env.fiveseconds == 3:
    #         rospy.sleep(4.85)
    #         break
    #     else:
    #         print 'wait start'
    #     rospy.sleep(0.01)

    print 'Start the RM!!!!!!!!!!!!!!!!!!1'
    goal_x = 2  #zhongjiandian
    goal_y = 3.25
    goal_yaw = 0
    if env.isActionAvaliable(goal_x, goal_y, goal_yaw):  # front1
        controller.send_goal_robot1(env.navgoal)
        print 'seng goal success1'
    rospy.sleep(0.15)
    if env.isActionAvaliable(goal_x, goal_y, goal_yaw):  # front1
        controller.send_goal_robot1(env.navgoal)
        print 'seng goal success2'
    rospy.sleep(3)
    goal_x = 4.25  # 攻击点
    goal_y = 3.15
    goal_yaw = -50
    if env.isActionAvaliable(goal_x, goal_y, goal_yaw):  # front1
        controller.send_goal_robot1(env.navgoal)
        print 'seng goal success1'
    rospy.sleep(0.15)
    if env.isActionAvaliable(goal_x, goal_y, goal_yaw):  # front1
        controller.send_goal_robot1(env.navgoal)
        print 'seng goal success2'
    rospy.sleep(4)  #等待进入助攻区域
    tree = BiuldTree()  # 已经加buff进入tree