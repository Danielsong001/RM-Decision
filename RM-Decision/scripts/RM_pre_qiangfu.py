#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
from Battle import BattleEnv
from teleop_control import Controller
from nav_msgs.msg import Odometry
from teleop_controller.msg import EnemyPos, ShootCmd, ModeSW, RFID, YoloEnemy, Hurt, GameInfo, EnemyArea, Communite, GameBuff
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
cmdvel_blockedretreat = Twist()
# for test

if __name__ == '__main__':
    rospy.init_node('RM_Pre')
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
    rospy.Subscriber('gamebuff', GameBuff, env.getGameBuffCallback)
    # rospy.Subscriber('yolo_enemy', YoloEnemy, env.getYoloEnemyCallback)  #不测试别用

    communite_pub = rospy.Publisher('communite', Communite, queue_size=1)
    communitemsg = Communite()

    # while 1:
    #     if env.fiveseconds == 3:
    #         rospy.sleep(5)
    #         break
    #     else:
    #         print 'wait start'
    #     rospy.sleep(0.01)

    print 'Start the RM!!!!!!!!!!!!!!!!!!1'
    goal_x = 4
    goal_y = 2.5
    goal_yaw = 0
    if env.isActionAvaliable(goal_x, goal_y, goal_yaw):  # front1
        controller.send_goal(env.navgoal)
        print 'seng goal success1'
    rospy.sleep(0.15)
    if env.isActionAvaliable(goal_x, goal_y, goal_yaw):  # front1
        controller.send_goal(env.navgoal)
        print 'seng goal success2'
    cmdvel_blockedpush.linear.x = 0.2
    # cmdvel_blockedpush.linear.y = 0.2 * np.sin(30 * 3.1416 / 180)
    cmdvel_blockedpush.linear.y = 0
    cmdvel_blockedpush.linear.z = 0
    cmdvel_blockedpush.angular.x = 0
    cmdvel_blockedpush.angular.y = 0
    cmdvel_blockedpush.angular.z = 0
    time2 = rospy.Time.now().secs
    while np.square(env.MyPose['x'] - goal_x) + np.square(env.MyPose['y'] - goal_y) > 0.03 or np.abs(
            env.MyPose['theta'] - goal_yaw) > 15:  # 开始计时
        print np.square(env.MyPose['x'] - goal_x) + np.square(env.MyPose['y'] - goal_y)
        print np.abs(env.MyPose['theta'] - goal_yaw)
        rospy.sleep(0.01)
        if rospy.Time.now().secs - time2 > 7:
            print 'time is out 5555555555555555555555555555555'
            break
    time2 = rospy.Time.now().secs
    while rospy.Time.now().secs - time2 < 5:  # 抢符时间
        print 'start in buff 2222222'
        rospy.sleep(0.02)
        controller.send_vel(cmdvel_blockedpush)
        # if env.Rfid == 1:
        if np.square(env.MyPose['x'] - 4) + np.square(env.MyPose['y'] - 2.5) < 0.01:  # 进入符的标志
            rospy.sleep(0.03)
            controller.send_vel(cmdvel_stop)
            controller.send_vel(cmdvel_stop)
            # print 'Been in buff'
            time1 = rospy.Time.now().secs
            # while rospy.Time.now().secs - time1 < 7 and env.Iget != 1:  # 得到符标志
            while rospy.Time.now().secs - time1 < 5.5:  # 得到符标志
                print 'wait buff!!!!!!!!!!!!'
                rospy.sleep(0.02)
                controller.send_vel(cmdvel_stop)
                controller.send_vel(cmdvel_stop)
            break
    controller.send_vel(cmdvel_stop)
    controller.send_vel(cmdvel_stop)
    print 'Success buff!!!!!!!!!!!!!!!!!!!!!!!1'
    communitemsg.beentreeflag = 1  # 可以进入数了
    communitemsg.beentreeflag_robot1 = 1  # 可以进入数了
    communite_pub.publish(communitemsg)
    if env.EnemyPoseSave.enemy_dist == 0:  #当前无敌人；退一下
        goal_x = 3.15
        goal_y = 2.65
        goal_yaw = -50
        if env.isActionAvaliable(goal_x, goal_y, goal_yaw):  # front1
            controller.send_goal(env.navgoal)
        rospy.sleep(1.5)
    while 1:
        communitemsg.beentreeflag = 1  # 可以进入数了
        communitemsg.beentreeflag_robot1 = 1  # 可以进入数了
        communite_pub.publish(communitemsg)
        rospy.sleep(0.5)
