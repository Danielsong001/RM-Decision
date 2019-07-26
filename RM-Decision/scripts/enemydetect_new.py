#!/usr/bin/env python
# -*- coding: UTF-8 -*-
import tf
import rospy
import math
import numpy as np
from teleop_controller.msg import EnemyPos
from PIL import Image
from nav_msgs.msg import Odometry
from Battle import BattleEnv

envdetect = BattleEnv()
EnemyPose_robot2 = {'x': 0, 'y': 0, 'theta': 0}
EnemyPose_robot3 = {'x': 0, 'y': 0, 'theta': 0}

rospy.Subscriber('my_pose', Odometry, envdetect.getSelfPoseCallback)
# rospy.Subscriber('robot_1/my_pose', Odometry, envdetect.getSelfPoseCallback_robot1)
rospy.Subscriber('robot_1/my_pose', Odometry, envdetect.getSelfPoseCallback_robot1)


def getEnemyPoseCallback_robot2(data):
    EnemyPose_robot2['x'] = data.pose.pose.position.x
    EnemyPose_robot2['y'] = data.pose.pose.position.y
    qx = data.pose.pose.orientation.x
    qy = data.pose.pose.orientation.y
    qz = data.pose.pose.orientation.z
    qw = data.pose.pose.orientation.w
    angle = math.atan2(2 * (qx * qy + qz * qw), qw * qw + qx * qx - qy * qy - qz * qz) * 180 / 3.1415926
    EnemyPose_robot2['theta'] = angle


def getEnemyPoseCallback_robot3(data):
    EnemyPose_robot3['x'] = data.pose.pose.position.x
    EnemyPose_robot3['y'] = data.pose.pose.position.y
    qx = data.pose.pose.orientation.x
    qy = data.pose.pose.orientation.y
    qz = data.pose.pose.orientation.z
    qw = data.pose.pose.orientation.w
    angle = math.atan2(2 * (qx * qy + qz * qw), qw * qw + qx * qx - qy * qy - qz * qz) * 180 / 3.1415926
    EnemyPose_robot3['theta'] = angle

rospy.Subscriber('/robot_2/base_pose_ground_truth', Odometry, getEnemyPoseCallback_robot2)
rospy.Subscriber('/robot_3/base_pose_ground_truth', Odometry, getEnemyPoseCallback_robot3)

if __name__ == '__main__':
    rospy.init_node('enemydetect')
    rate = rospy.Rate(50)
    map = np.array(Image.open("icra.pgm"))
    enemy_pub = rospy.Publisher('enemy_pos', EnemyPos, queue_size=1)
    enemy_pub_robot1 = rospy.Publisher('robot_1/enemy_pos', EnemyPos, queue_size=1)
    enemy_pos = EnemyPos()
    while not rospy.is_shutdown():
    # 主机器人检测结果：取距离近的发布
        # 主机器人检测robot2
        obstacle2 = False
        x0s = int(envdetect.MyPose['x'] / 0.05)
        y0s = int(envdetect.MyPose['y'] / 0.05)
        x0e = int(EnemyPose_robot2['x'] / 0.05)
        y0e = int(EnemyPose_robot2['y'] / 0.05)
        A = np.mat([[x0s, 1], [x0e, 1]])
        B = np.mat([[y0s], [y0e]])
        if (np.linalg.det(A)!=0):
            ab = np.linalg.solve(A, B)  # 检测两点直线y=ax+b
            a = ab[0, 0]
            b = ab[1, 0]
            if x0s > x0e:
                t = x0s
                x0s = x0e
                x0e = t
            for x in range(x0s, x0e):
                y = int(a * x + b)
                if map[101 - y, x] < 255:
                    obstacle2 = True
                    break
        else:
            if y0s > y0e:
                t = y0s
                y0s = y0e
                y0e = t
            for y_1 in range(y0s, y0e):
                if map[101 - y_1, int(x0s)] < 255:
                    obstacle2 = True
                    break
        # 主机器人检测robot3
        obstacle3 = False
        x0s = int(envdetect.MyPose['x'] / 0.05)
        y0s = int(envdetect.MyPose['y'] / 0.05)
        x0e = int(EnemyPose_robot3['x'] / 0.05)
        y0e = int(EnemyPose_robot3['y'] / 0.05)
        A = np.mat([[x0s, 1], [x0e, 1]])
        B = np.mat([[y0s], [y0e]])
        if (np.linalg.det(A)!=0):
            ab = np.linalg.solve(A, B)  # 检测两点直线y=ax+b
            a = ab[0, 0]
            b = ab[1, 0]
            if x0s > x0e:
                t = x0s
                x0s = x0e
                x0e = t
            for x in range(x0s, x0e):
                y = int(a * x + b)
                if map[101 - y, x] < 255:
                    obstacle3 = True
                    break
        else:
            if y0s > y0e:
                t = y0s
                y0s = y0e
                y0e = t
            for y_1 in range(y0s, y0e):
                if map[101 - y_1, int(x0s)] < 255:
                    obstacle3 = True
                    break
        # 主机器人选择较近者发布
        if obstacle2 == False and obstacle3 == False:
            enemy_yaw2 = math.atan2((EnemyPose_robot2['y']-envdetect.MyPose['y']), (EnemyPose_robot2['x']-envdetect.MyPose['x'])) * 180 / 3.1416 - envdetect.MyPose['theta']
            enemy_dist2 = math.sqrt(np.square(EnemyPose_robot2['x']-envdetect.MyPose['x']) + np.square(EnemyPose_robot2['y']-envdetect.MyPose['y']))

            enemy_yaw3 = math.atan2((EnemyPose_robot3['y']-envdetect.MyPose['y']), (EnemyPose_robot3['x']-envdetect.MyPose['x'])) * 180 / 3.1416 - envdetect.MyPose['theta']
            enemy_dist3 = math.sqrt(np.square(EnemyPose_robot3['x'] - envdetect.MyPose['x']) + np.square(EnemyPose_robot3['y'] - envdetect.MyPose['y']))
            if enemy_dist2 < enemy_dist3:
                enemy_pos.enemy_yaw = - enemy_yaw2
                enemy_pos.enemy_dist = enemy_dist2 * 1000
                enemy_pos.enemy_pitch = 0
            else:
                enemy_pos.enemy_yaw = - enemy_yaw3
                enemy_pos.enemy_dist = enemy_dist3 * 1000
                enemy_pos.enemy_pitch = 0
            enemy_pub.publish(enemy_pos)
            print 'robot0(small): %s\t%s\t\n' % (-enemy_pos.enemy_yaw, enemy_pos.enemy_dist)

        elif obstacle2 == True and obstacle3 == False:
            enemy_yaw3 = math.atan2((EnemyPose_robot3['y'] - envdetect.MyPose['y']), (EnemyPose_robot3['x'] - envdetect.MyPose['x'])) * 180 / 3.1416 - envdetect.MyPose['theta']
            enemy_dist3 = math.sqrt(np.square(EnemyPose_robot3['x'] - envdetect.MyPose['x']) + np.square(EnemyPose_robot3['y'] - envdetect.MyPose['y']))
            enemy_pos.enemy_yaw = -enemy_yaw3
            enemy_pos.enemy_dist = enemy_dist3 * 1000
            enemy_pos.enemy_pitch = 0
            enemy_pub.publish(enemy_pos)
            print 'robot0: %s\t%s\t\n' % (-enemy_pos.enemy_yaw, enemy_pos.enemy_dist)

        elif obstacle2 == False and obstacle3 == True:
            enemy_yaw2 = math.atan2((EnemyPose_robot2['y'] - envdetect.MyPose['y']), (EnemyPose_robot2['x'] - envdetect.MyPose['x'])) * 180 / 3.1416 - envdetect.MyPose['theta']
            enemy_dist2 = math.sqrt(np.square(EnemyPose_robot2['x'] - envdetect.MyPose['x']) + np.square(EnemyPose_robot2['y'] - envdetect.MyPose['y']))
            enemy_pos.enemy_yaw = -enemy_yaw2
            enemy_pos.enemy_dist = enemy_dist2 * 1000
            enemy_pos.enemy_pitch = 0
            enemy_pub.publish(enemy_pos)
            print 'robot0: %s\t%s\t\n' % (-enemy_pos.enemy_yaw, enemy_pos.enemy_dist)

        else:
            enemy_pos.enemy_yaw = 0
            enemy_pos.enemy_dist = 0
            enemy_pos.enemy_pitch = 0
            enemy_pub.publish(enemy_pos)
            print 'robot0: %s\t%s\t\n' % (enemy_pos.enemy_yaw, enemy_pos.enemy_dist)

    # 副机器人检测结果：取距离近的发布
        # 副机器人检测robot2
        obstacle2 = False
        x0s = int(envdetect.MyPose_robot1['x'] / 0.05)
        y0s = int(envdetect.MyPose_robot1['y'] / 0.05)
        x0e = int(EnemyPose_robot2['x'] / 0.05)
        y0e = int(EnemyPose_robot2['y'] / 0.05)
        A = np.mat([[x0s, 1], [x0e, 1]])
        B = np.mat([[y0s], [y0e]])
        if (np.linalg.det(A)!=0):
            ab = np.linalg.solve(A, B)  # 检测两点直线y=ax+b
            a = ab[0, 0]
            b = ab[1, 0]
            if x0s > x0e:
                t = x0s
                x0s = x0e
                x0e = t
            for x in range(x0s, x0e):
                y = int(a * x + b)
                if map[101 - y, x] < 255:
                    obstacle2 = True
                    break
        else:
            if y0s > y0e:
                t = y0s
                y0s = y0e
                y0e = t
            for y_1 in range(y0s, y0e):
                if map[101 - y_1, int(x0s)] < 255:
                    obstacle2 = True
                    break
        # 副机器人检测robot3
        obstacle3 = False
        x0s = int(envdetect.MyPose_robot1['x'] / 0.05)
        y0s = int(envdetect.MyPose_robot1['y'] / 0.05)
        x0e = int(EnemyPose_robot3['x'] / 0.05)
        y0e = int(EnemyPose_robot3['y'] / 0.05)
        A = np.mat([[x0s, 1], [x0e, 1]])
        B = np.mat([[y0s], [y0e]])
        if (np.linalg.det(A)!=0):
            ab = np.linalg.solve(A, B)  # 检测两点直线y=ax+b
            a = ab[0, 0]
            b = ab[1, 0]
            if x0s > x0e:
                t = x0s
                x0s = x0e
                x0e = t
            for x in range(x0s, x0e):
                y = int(a * x + b)
                if map[101 - y, x] < 255:
                    obstacle3 = True
                    break
        else:
            if y0s > y0e:
                t = y0s
                y0s = y0e
                y0e = t
            for y_1 in range(y0s, y0e):
                if map[101 - y_1, int(x0s)] < 255:
                    obstacle3 = True
                    break
        # 副机器人选择较近者发布
        if obstacle2 == False and obstacle3 == False:
            enemy_yaw2 = math.atan2((EnemyPose_robot2['y']-envdetect.MyPose_robot1['y']), (EnemyPose_robot2['x']-envdetect.MyPose_robot1['x'])) * 180 / 3.1416 - envdetect.MyPose_robot1['theta']
            enemy_dist2 = math.sqrt(np.square(EnemyPose_robot2['x']-envdetect.MyPose_robot1['x']) + np.square(EnemyPose_robot2['y']-envdetect.MyPose_robot1['y']))

            enemy_yaw3 = math.atan2((EnemyPose_robot3['y']-envdetect.MyPose_robot1['y']), (EnemyPose_robot3['x']-envdetect.MyPose_robot1['x'])) * 180 / 3.1416 - envdetect.MyPose_robot1['theta']
            enemy_dist3 = math.sqrt(np.square(EnemyPose_robot3['x'] - envdetect.MyPose_robot1['x']) + np.square(EnemyPose_robot3['y'] - envdetect.MyPose_robot1['y']))
            if enemy_dist2 < enemy_dist3:
                enemy_pos.enemy_yaw = -enemy_yaw2
                enemy_pos.enemy_dist = enemy_dist2 * 1000
                enemy_pos.enemy_pitch = 0
            else:
                enemy_pos.enemy_yaw = -enemy_yaw3
                enemy_pos.enemy_dist = enemy_dist3 * 1000
                enemy_pos.enemy_pitch = 0
            enemy_pub_robot1.publish(enemy_pos)
            print 'robot1(small): %s\t%s\t\n' % (-enemy_pos.enemy_yaw, enemy_pos.enemy_dist)

        elif obstacle2 == True and obstacle3 == False:
            enemy_yaw3 = math.atan2((EnemyPose_robot3['y'] - envdetect.MyPose_robot1['y']), (EnemyPose_robot3['x'] - envdetect.MyPose_robot1['x'])) * 180 / 3.1416 - envdetect.MyPose_robot1['theta']
            enemy_dist3 = math.sqrt(np.square(EnemyPose_robot3['x'] - envdetect.MyPose_robot1['x']) + np.square(EnemyPose_robot3['y'] - envdetect.MyPose_robot1['y']))
            enemy_pos.enemy_yaw = -enemy_yaw3
            enemy_pos.enemy_dist = enemy_dist3 * 1000
            enemy_pos.enemy_pitch = 0
            enemy_pub_robot1.publish(enemy_pos)
            print 'robot1: %s\t%s\t\n' % (-enemy_pos.enemy_yaw, enemy_pos.enemy_dist)

        elif obstacle2 == False and obstacle3 == True:
            enemy_yaw2 = math.atan2((EnemyPose_robot2['y'] - envdetect.MyPose_robot1['y']), (EnemyPose_robot2['x'] - envdetect.MyPose_robot1['x'])) * 180 / 3.1416 - envdetect.MyPose_robot1['theta']
            enemy_dist2 = math.sqrt(np.square(EnemyPose_robot2['x'] - envdetect.MyPose_robot1['x']) + np.square(EnemyPose_robot2['y'] - envdetect.MyPose_robot1['y']))
            enemy_pos.enemy_yaw = -enemy_yaw2
            enemy_pos.enemy_dist = enemy_dist2 * 1000
            enemy_pos.enemy_pitch = 0
            enemy_pub_robot1.publish(enemy_pos)
            print 'robot1: %s\t%s\t\n' % (-enemy_pos.enemy_yaw, enemy_pos.enemy_dist)

        else:
            enemy_pos.enemy_yaw = 0
            enemy_pos.enemy_dist = 0
            enemy_pos.enemy_pitch = 0
            enemy_pub_robot1.publish(enemy_pos)
            print 'robot1: %s\t%s\t\n' % (enemy_pos.enemy_yaw, enemy_pos.enemy_dist)

        rospy.sleep(0.05)

