# -*- coding: utf-8 -*-
import sys
import numpy as np
from PIL import Image
import rospy
from geometry_msgs.msg import PoseStamped, Quaternion, PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from teleop_controller.msg import EnemyPos, ShootCmd, ModeSW, RFID, YoloEnemy, Hurt, GameInfo, EnemyArea
import time
import math
import tf
from teleop_control import Controller


class BattleEnv():
    def __init__(self):
        self.map = np.array(Image.open("icra2.pgm"))
        self.shoot_pub = rospy.Publisher('shoot_cmd', ShootCmd, queue_size=1)
        self.goal_pub = rospy.Publisher('move_base_simple/goal', PoseStamped, queue_size=1)

        self.navgoal = PoseStamped()
        self.navgoal.header.frame_id = 'map'
        self.shoot = ShootCmd()
        self.MyPose = {'x': 1, 'y': 1, 'theta': 0}
        self.MyPose_robot1 = {'x': 1, 'y': 1.5, 'theta': 0}
        self.EnemyPoseSave = EnemyPos()
        self.EnemyPoseSave_last = EnemyPos()
        self.EnemyPoseSave_robot1 = EnemyPos()
        self.EnemyPoseSave_last_robot1 = EnemyPos()
        self.Gimbal = EnemyPos()
        self.Gimbal_robot1 = EnemyPos()
        self.witch_armor = -1
        self.witch_armor_robot1 = -1

        self.num_no_enemy = 0
        self.num_no_enemy_robot1 = 0
        self.goal_x = 0
        self.goal_y = 0
        self.goal_yaw = 0
        self.goal_x_robot1 = 0
        self.goal_y_robot1 = 0
        self.goal_yaw_robot1 = 0
        self.choosedgoal_x = 0
        self.choosedgoal_y = 0
        self.choosedgoal_yaw = 0
        self.choosedgoal_x_robot1 = 0
        self.choosedgoal_y_robot1 = 0
        self.choosedgoal_yaw_robot1 = 0
        self.farchoosegoal_x = 0
        self.farchoosegoal_y = 0
        self.farchoosegoal_yaw = 0
        self.farchoosegoal_x_robot1 = 0
        self.farchoosegoal_y_robot1 = 0
        self.farchoosegoal_yaw_robot1 = 0

        self.enemyblock_num = 0
        self.enemyblock_attacknum = [[2.3, 3.2, -45], [2.3, 3.2, 45], [1, 3.15, -90]]
        self.enemyblock_num_robot1 = 0
        self.enemyblock_attacknum_robot1 = [[5.7, 1.8, 145], [5.7, 1.8, -145], [7, 1.85, 90]]

        self.distfromhim = []
        self.distfromme = []
        self.distfromhim_robot1 = []
        self.distfromme_robot1 = []
        self.followgoal_x = 0
        self.followgoal_y = 0
        self.followgoal_yaw = 0
        self.followgoal_x_robot1 = 0
        self.followgoal_y_robot1 = 0
        self.followgoal_yaw_robot1 = 0

        self.armor_nonecount = 0
        self.armor_nonecount_robot1 = 0
        self.deputy_nonecount = 0
        self.deputy_nonecount_robot1 = 0

        self.blockedthetas = []
        self.blockedthetas_robot1 = []

        # 裁判系统
        self.Rfid = 0
        self.Rfid_robot1 = 0
        self.MyHP = 2000
        self.MyHP_robot1 = 2000
        self.fiveseconds = 0
        self.beentreeflag = 0
        self.beentreeflag_robot1 = 0
        self.Iget = 0
        self.Eget = 0

    def getSelfPoseCallback(self, data):
        self.MyPose['x'] = data.pose.pose.position.x
        self.MyPose['y'] = data.pose.pose.position.y
        qx = data.pose.pose.orientation.x
        qy = data.pose.pose.orientation.y
        qz = data.pose.pose.orientation.z
        qw = data.pose.pose.orientation.w
        angle = math.atan2(2 * (qx * qy + qz * qw), qw * qw + qx * qx - qy * qy - qz * qz)* 180 / 3.1415926
        self.MyPose['theta'] = angle
        self.Blockedpose_Analysis(self.MyPose['x'], self.MyPose['y'], self.MyPose['theta'])

    def getSelfPoseCallback_robot1(self, data):
        self.MyPose_robot1['x'] = data.pose.pose.position.x
        self.MyPose_robot1['y'] = data.pose.pose.position.y
        qx = data.pose.pose.orientation.x
        qy = data.pose.pose.orientation.y
        qz = data.pose.pose.orientation.z
        qw = data.pose.pose.orientation.w
        angle = math.atan2(2 * (qx * qy + qz * qw), qw * qw + qx * qx - qy * qy - qz * qz)* 180 / 3.1415926
        self.MyPose_robot1['theta'] = angle
        self.Blockedpose_Analysis_robot1(self.MyPose_robot1['x'],self.MyPose_robot1['y'],self.MyPose_robot1['theta'])

    def Blockedpose_Analysis(self, x, y, theta):
        if (x>2.15 and x<3.15 and y>0 and y<2.5) or (x>0.85 and x<1.85 and y>1.2 and y<2.5) or (x>4.85 and x<5.85 and y>2.5 and y<5) or (x>6.15 and x<7.15 and y>2.5 and y<3.8):
            for k in range(0, len(self.blockedthetas)):
                del self.blockedthetas[0]
            self.blockedthetas.append(90)
            self.blockedthetas.append(-90)
        elif (x>0 and x<2.15 and y>2.7 and y<3.7) or (x>0 and x<2.15 and y>4 and y<5) or (x>5.85 and x<8 and y>1.3 and y<2.3) or (x>5.85 and x<8 and y>0 and y<1):
            for k in range(0, len(self.blockedthetas)):
                del self.blockedthetas[0]
            self.blockedthetas.append(0)
            self.blockedthetas.append(178)
            self.blockedthetas.append(-178)
        else:
            for k in range(0, len(self.blockedthetas)):
                del self.blockedthetas[0]
            self.blockedthetas.append(0)
            self.blockedthetas.append(90)
            self.blockedthetas.append(178)
            self.blockedthetas.append(-178)
            self.blockedthetas.append(-90)

    def Blockedpose_Analysis_robot1(self, x, y, theta):
        if (x>2.15 and x<3.15 and y>0 and y<2.5) or (x>0.85 and x<1.85 and y>1.2 and y<2.5) or (x>4.85 and x<5.85 and y>2.5 and y<5) or (x>6.15 and x<7.15 and y>2.5 and y<3.8):
            for k in range(0, len(self.blockedthetas_robot1)):
                del self.blockedthetas_robot1[0]
            self.blockedthetas_robot1.append(90)
            self.blockedthetas_robot1.append(-90)
        elif (x>0 and x<2.15 and y>2.7 and y<3.7) or (x>0 and x<2.15 and y>4 and y<5) or (x>5.85 and x<8 and y>1.3 and y<2.3) or (x>5.85 and x<8 and y>0 and y<1):
            for k in range(0, len(self.blockedthetas_robot1)):
                del self.blockedthetas_robot1[0]
            self.blockedthetas_robot1.append(0)
            self.blockedthetas_robot1.append(178)
            self.blockedthetas_robot1.append(-178)
        else:
            for k in range(0, len(self.blockedthetas_robot1)):
                del self.blockedthetas_robot1[0]
            self.blockedthetas_robot1.append(0)
            self.blockedthetas_robot1.append(90)
            self.blockedthetas_robot1.append(178)
            self.blockedthetas_robot1.append(-178)
            self.blockedthetas_robot1.append(-90)


    def getEnemyPoseCallback(self, data):
        if data.enemy_dist > 0 and data.enemy_dist <= 3000:
            self.num_no_enemy = 0
            self.EnemyPoseSave.enemy_yaw = - data.enemy_yaw
            self.EnemyPoseSave.enemy_pitch = data.enemy_pitch
            self.EnemyPoseSave.enemy_dist = data.enemy_dist / 1000
            self.EnemyPoseSave_last = self.EnemyPoseSave.enemy_yaw

            dist = self.EnemyPoseSave.enemy_dist-1.1
            yaw = self.EnemyPoseSave.enemy_yaw * 3.1416 / 180
            alpha = self.MyPose['theta'] * 3.1416 / 180
            x = self.MyPose['x']
            y = self.MyPose['y']
            dx = dist * math.cos(yaw)
            dy = dist * math.sin(yaw)
            Ts0 = np.mat(
                [[math.cos(alpha), -math.sin(alpha), 0, x], [math.sin(alpha), math.cos(alpha), 0, y], [0, 0, 1, 0],
                 [0, 0, 0, 1]])
            Tse = np.mat([[math.cos(yaw), -math.sin(yaw), 0, dx], [math.sin(yaw), math.cos(yaw), 0, dy], [0, 0, 1, 0],
                          [0, 0, 0, 1]])
            T = Ts0 * Tse
            self.goal_x = T[0, 3]
            self.goal_y = T[1, 3]
            self.goal_yaw = (np.arctan2(T[1, 0], T[0, 0])) * 180 / 3.1415926

            # 用于计算follow
            dist = self.EnemyPoseSave.enemy_dist
            yaw = self.EnemyPoseSave.enemy_yaw * 3.1416 / 180
            alpha = self.MyPose['theta'] * 3.1416 / 180
            x = self.MyPose['x']
            y = self.MyPose['y']
            dx = dist * math.cos(yaw)
            dy = dist * math.sin(yaw)
            Ts0 = np.mat(
                [[math.cos(alpha), -math.sin(alpha), 0, x], [math.sin(alpha), math.cos(alpha), 0, y], [0, 0, 1, 0],
                 [0, 0, 0, 1]])
            Tse = np.mat([[math.cos(yaw), -math.sin(yaw), 0, dx], [math.sin(yaw), math.cos(yaw), 0, dy], [0, 0, 1, 0],
                          [0, 0, 0, 1]])
            T = Ts0 * Tse
            self.followgoal_x = T[0, 3]
            self.followgoal_y = T[1, 3]
            self.followgoal_yaw = (np.arctan2(T[1, 0], T[0, 0])) * 180 / 3.1415926

            self.Enemypose_Analysis(self.followgoal_x, self.followgoal_y, self.followgoal_yaw)
            # print 'r1 should !!follow: num is %s, goal is %s %s, %s dist is %s  yaw is %s' % (self.enemyblock_num, self.choosedgoal_x_robot1, self.choosedgoal_y_robot1, self.choosedgoal_yaw_robot1, self.EnemyPoseSave.enemy_dist,self.EnemyPoseSave.enemy_yaw)

        else:
            self.num_no_enemy = self.num_no_enemy + 1
        if self.num_no_enemy > 5:  #超过1s都没数据;太长或导致撞车
            self.EnemyPoseSave.enemy_yaw = 0
            self.EnemyPoseSave.enemy_pitch = 0
            self.EnemyPoseSave.enemy_dist = 0
            self.num_no_enemy = 0

    def getEnemyPoseCallback_robot1(self, data):
        if data.enemy_dist > 0 and data.enemy_dist <= 3000:
            self.num_no_enemy_robot1 = 0
            self.EnemyPoseSave_robot1.enemy_yaw = - data.enemy_yaw
            self.EnemyPoseSave_robot1.enemy_pitch = data.enemy_pitch
            self.EnemyPoseSave_robot1.enemy_dist = data.enemy_dist / 1000
            self.EnemyPoseSave_last_robot1 = self.EnemyPoseSave_robot1.enemy_yaw

            dist_robot1 = self.EnemyPoseSave_robot1.enemy_dist-1.1
            yaw_robot1 = self.EnemyPoseSave_robot1.enemy_yaw * 3.1416 / 180
            alpha_robot1 = self.MyPose_robot1['theta'] * 3.1416 / 180
            x_robot1 = self.MyPose_robot1['x']
            y_robot1 = self.MyPose_robot1['y']
            dx_robot1 = dist_robot1 * math.cos(yaw_robot1)
            dy_robot1 = dist_robot1 * math.sin(yaw_robot1)
            Ts0_robot1 = np.mat(
                [[math.cos(alpha_robot1), -math.sin(alpha_robot1), 0, x_robot1], [math.sin(alpha_robot1), math.cos(alpha_robot1), 0, y_robot1], [0, 0, 1, 0],
                 [0, 0, 0, 1]])
            Tse_robot1 = np.mat([[math.cos(yaw_robot1), -math.sin(yaw_robot1), 0, dx_robot1], [math.sin(yaw_robot1), math.cos(yaw_robot1), 0, dy_robot1], [0, 0, 1, 0],
                          [0, 0, 0, 1]])
            T_robot1 = Ts0_robot1 * Tse_robot1
            self.goal_x_robot1 = T_robot1[0, 3]
            self.goal_y_robot1 = T_robot1[1, 3]
            self.goal_yaw_robot1 = (np.arctan2(T_robot1[1, 0], T_robot1[0, 0])) * 180 / 3.1415926

            # 用于follow
            dist_robot1 = self.EnemyPoseSave_robot1.enemy_dist
            yaw_robot1 = self.EnemyPoseSave_robot1.enemy_yaw * 3.1416 / 180
            alpha_robot1 = self.MyPose_robot1['theta'] * 3.1416 / 180
            x_robot1 = self.MyPose_robot1['x']
            y_robot1 = self.MyPose_robot1['y']
            dx_robot1 = dist_robot1 * math.cos(yaw_robot1)
            dy_robot1 = dist_robot1 * math.sin(yaw_robot1)
            Ts0_robot1 = np.mat(
                [[math.cos(alpha_robot1), -math.sin(alpha_robot1), 0, x_robot1],
                 [math.sin(alpha_robot1), math.cos(alpha_robot1), 0, y_robot1], [0, 0, 1, 0],
                 [0, 0, 0, 1]])
            Tse_robot1 = np.mat([[math.cos(yaw_robot1), -math.sin(yaw_robot1), 0, dx_robot1],
                                 [math.sin(yaw_robot1), math.cos(yaw_robot1), 0, dy_robot1], [0, 0, 1, 0],
                                 [0, 0, 0, 1]])
            T_robot1 = Ts0_robot1 * Tse_robot1
            self.followgoal_x_robot1 = T_robot1[0, 3]
            self.followgoal_y_robot1 = T_robot1[1, 3]
            self.followgoal_yaw_robot1 = (np.arctan2(T_robot1[1, 0], T_robot1[0, 0])) * 180 / 3.1415926
            self.Enemypose_Analysis_robot1(self.followgoal_x_robot1, self.followgoal_y_robot1, self.followgoal_yaw_robot1)
            # print 'r0 should !!follow: num is %s, goal is %s, %s, %s dist is %s,  yaw is %s' % (self.enemyblock_num_robot1,self.choosedgoal_x, self.choosedgoal_y, self.choosedgoal_yaw, self.EnemyPoseSave_robot1.enemy_dist,self.EnemyPoseSave_robot1.enemy_yaw)

        else:
            self.num_no_enemy_robot1 = self.num_no_enemy_robot1 + 1
        if self.num_no_enemy_robot1 > 5:
            self.EnemyPoseSave_robot1.enemy_yaw = 0
            self.EnemyPoseSave_robot1.enemy_pitch = 0
            self.EnemyPoseSave_robot1.enemy_dist = 0
            self.num_no_enemy_robot1 = 0

    def Enemypose_Analysis(self, goal_x, goal_y, goal_yaw):
        if goal_x >= 0 and goal_x <= 1.9 and goal_y >= 0 and goal_y <= 2.35:
            self.enemyblock_num = 1
            # 清空并引入块集
            for k in range(0, len(self.enemyblock_attacknum)): # 清空数组
                del self.enemyblock_attacknum[0]
            self.enemyblock_attacknum.append([1.3, 2.15, -95])
            self.enemyblock_attacknum.append([1.9, 0.8, 170])
            # 清空并引入距离数组
            for k in range(0, len(self.distfromme)):
                del self.distfromme[0]
            for k in range(0, len(self.distfromhim)):
                del self.distfromhim[0]
            for k in enumerate(self.enemyblock_attacknum):
                self.distfromhim.append(
                    np.square(k[1][0] - self.MyPose_robot1['x']) + np.square(k[1][1] - self.MyPose_robot1['y']))
                self.distfromme.append(np.square(k[1][0] - self.MyPose['x']) + np.square(k[1][1] - self.MyPose['y']))
            if self.distfromme.index(min(self.distfromme)) != self.distfromhim.index(min(self.distfromhim)):
                self.choosedgoal_x_robot1 = self.enemyblock_attacknum[self.distfromhim.index(min(self.distfromhim))][0]
                self.choosedgoal_y_robot1 = self.enemyblock_attacknum[self.distfromhim.index(min(self.distfromhim))][1]
                self.choosedgoal_yaw_robot1 = self.enemyblock_attacknum[self.distfromhim.index(min(self.distfromhim))][2]
                self.farchoosegoal_x = self.enemyblock_attacknum[self.distfromme.index(min(self.distfromme))][0]
                self.farchoosegoal_y = self.enemyblock_attacknum[self.distfromme.index(min(self.distfromme))][1]
                self.farchoosegoal_yaw = self.enemyblock_attacknum[self.distfromme.index(min(self.distfromme))][2]
            else:
                del self.enemyblock_attacknum[self.distfromme.index(min(self.distfromme))]
                del self.distfromhim[self.distfromhim.index(min(self.distfromhim))]
                del self.distfromme[self.distfromme.index(min(self.distfromme))]
                self.choosedgoal_x_robot1 = self.enemyblock_attacknum[self.distfromhim.index(min(self.distfromhim))][0]
                self.choosedgoal_y_robot1 = self.enemyblock_attacknum[self.distfromhim.index(min(self.distfromhim))][1]
                self.choosedgoal_yaw_robot1 = self.enemyblock_attacknum[self.distfromhim.index(min(self.distfromhim))][2]
                self.farchoosegoal_x = self.enemyblock_attacknum[self.distfromme.index(min(self.distfromme))][0]
                self.farchoosegoal_y = self.enemyblock_attacknum[self.distfromme.index(min(self.distfromme))][1]
                self.farchoosegoal_yaw = self.enemyblock_attacknum[self.distfromme.index(min(self.distfromme))][2]

        elif goal_x >=0 and goal_x <= 1.9 and goal_y > 2.35 and goal_y <= 4:
            self.enemyblock_num = 2
            for k in range(0, len(self.enemyblock_attacknum)): # 清空数组
                del self.enemyblock_attacknum[0]
            self.enemyblock_attacknum.append([1.35, 2.4, 100])
            self.enemyblock_attacknum.append([0.65, 4, -75])
            self.enemyblock_attacknum.append([2.35, 3.25, 178])
            for k in range(0, len(self.distfromme)):
                del self.distfromme[0]
            for k in range(0, len(self.distfromhim)):
                del self.distfromhim[0]
            for k in enumerate(self.enemyblock_attacknum):
                self.distfromhim.append(
                    np.square(k[1][0] - self.MyPose_robot1['x']) + np.square(k[1][1] - self.MyPose_robot1['y']))
                self.distfromme.append(np.square(k[1][0] - self.MyPose['x']) + np.square(k[1][1] - self.MyPose['y']))
            if self.distfromme.index(min(self.distfromme)) != self.distfromhim.index(min(self.distfromhim)):
                self.choosedgoal_x_robot1 = self.enemyblock_attacknum[self.distfromhim.index(min(self.distfromhim))][0]
                self.choosedgoal_y_robot1 = self.enemyblock_attacknum[self.distfromhim.index(min(self.distfromhim))][1]
                self.choosedgoal_yaw_robot1 = self.enemyblock_attacknum[self.distfromhim.index(min(self.distfromhim))][2]
                self.farchoosegoal_x = self.enemyblock_attacknum[self.distfromme.index(min(self.distfromme))][0]
                self.farchoosegoal_y = self.enemyblock_attacknum[self.distfromme.index(min(self.distfromme))][1]
                self.farchoosegoal_yaw = self.enemyblock_attacknum[self.distfromme.index(min(self.distfromme))][2]
            else:
                del self.enemyblock_attacknum[self.distfromme.index(min(self.distfromme))]
                del self.distfromhim[self.distfromhim.index(min(self.distfromhim))]
                del self.distfromme[self.distfromme.index(min(self.distfromme))]
                self.choosedgoal_x_robot1 = self.enemyblock_attacknum[self.distfromhim.index(min(self.distfromhim))][0]
                self.choosedgoal_y_robot1 = self.enemyblock_attacknum[self.distfromhim.index(min(self.distfromhim))][1]
                self.choosedgoal_yaw_robot1 = self.enemyblock_attacknum[self.distfromhim.index(min(self.distfromhim))][2]
                self.farchoosegoal_x = self.enemyblock_attacknum[self.distfromme.index(min(self.distfromme))][0]
                self.farchoosegoal_y = self.enemyblock_attacknum[self.distfromme.index(min(self.distfromme))][1]
                self.farchoosegoal_yaw = self.enemyblock_attacknum[self.distfromme.index(min(self.distfromme))][2]
        elif goal_x >= 0 and goal_x <= 1.9 and goal_y > 4 and goal_y <= 5:
            self.enemyblock_num = 3
            for k in range(0, len(self.enemyblock_attacknum)): # 清空数组
                del self.enemyblock_attacknum[0]
            self.enemyblock_attacknum.append([0.65, 3.7, 70])
            self.enemyblock_attacknum.append([2.35, 4.5, 178])
            for k in range(0, len(self.distfromme)):
                del self.distfromme[0]
            for k in range(0, len(self.distfromhim)):
                del self.distfromhim[0]
            for k in enumerate(self.enemyblock_attacknum):
                self.distfromhim.append(
                    np.square(k[1][0] - self.MyPose_robot1['x']) + np.square(k[1][1] - self.MyPose_robot1['y']))
                self.distfromme.append(np.square(k[1][0] - self.MyPose['x']) + np.square(k[1][1] - self.MyPose['y']))
            if self.distfromme.index(min(self.distfromme)) != self.distfromhim.index(min(self.distfromhim)):
                self.choosedgoal_x_robot1 = self.enemyblock_attacknum[self.distfromhim.index(min(self.distfromhim))][0]
                self.choosedgoal_y_robot1 = self.enemyblock_attacknum[self.distfromhim.index(min(self.distfromhim))][1]
                self.choosedgoal_yaw_robot1 = self.enemyblock_attacknum[self.distfromhim.index(min(self.distfromhim))][2]
                self.farchoosegoal_x = self.enemyblock_attacknum[self.distfromme.index(min(self.distfromme))][0]
                self.farchoosegoal_y = self.enemyblock_attacknum[self.distfromme.index(min(self.distfromme))][1]
                self.farchoosegoal_yaw = self.enemyblock_attacknum[self.distfromme.index(min(self.distfromme))][2]
            else:
                del self.enemyblock_attacknum[self.distfromme.index(min(self.distfromme))]
                del self.distfromhim[self.distfromhim.index(min(self.distfromhim))]
                del self.distfromme[self.distfromme.index(min(self.distfromme))]
                self.choosedgoal_x_robot1 = self.enemyblock_attacknum[self.distfromhim.index(min(self.distfromhim))][0]
                self.choosedgoal_y_robot1 = self.enemyblock_attacknum[self.distfromhim.index(min(self.distfromhim))][1]
                self.choosedgoal_yaw_robot1 = self.enemyblock_attacknum[self.distfromhim.index(min(self.distfromhim))][2]
                self.farchoosegoal_x = self.enemyblock_attacknum[self.distfromme.index(min(self.distfromme))][0]
                self.farchoosegoal_y = self.enemyblock_attacknum[self.distfromme.index(min(self.distfromme))][1]
                self.farchoosegoal_yaw = self.enemyblock_attacknum[self.distfromme.index(min(self.distfromme))][2]
        elif (goal_x > 1.9 and goal_x <= 4.7 and goal_y >= 3 and goal_y <= 5) or (goal_x > 1.9 and goal_x <= 4 and goal_y >= 2 and goal_y <= 3):
            self.enemyblock_num = 4
            for k in range(0, len(self.enemyblock_attacknum)): # 清空数组
                del self.enemyblock_attacknum[0]
            self.enemyblock_attacknum.append([2, 4.5, -55])
            self.enemyblock_attacknum.append([2, 3.2, 0])
            self.enemyblock_attacknum.append([2.65, 2.55, 45])
            self.enemyblock_attacknum.append([4, 2.5, 135])
            for k in range(0, len(self.distfromme)):
                del self.distfromme[0]
            for k in range(0, len(self.distfromhim)):
                del self.distfromhim[0]
            for k in enumerate(self.enemyblock_attacknum):
                self.distfromhim.append(
                    np.square(k[1][0] - self.MyPose_robot1['x']) + np.square(k[1][1] - self.MyPose_robot1['y']))
                self.distfromme.append(np.square(k[1][0] - self.MyPose['x']) + np.square(k[1][1] - self.MyPose['y']))
            if self.distfromme.index(min(self.distfromme)) != self.distfromhim.index(min(self.distfromhim)):
                self.choosedgoal_x_robot1 = self.enemyblock_attacknum[self.distfromhim.index(min(self.distfromhim))][0]
                self.choosedgoal_y_robot1 = self.enemyblock_attacknum[self.distfromhim.index(min(self.distfromhim))][1]
                self.choosedgoal_yaw_robot1 = self.enemyblock_attacknum[self.distfromhim.index(min(self.distfromhim))][2]
                self.farchoosegoal_x = self.enemyblock_attacknum[self.distfromme.index(min(self.distfromme))][0]
                self.farchoosegoal_y = self.enemyblock_attacknum[self.distfromme.index(min(self.distfromme))][1]
                self.farchoosegoal_yaw = self.enemyblock_attacknum[self.distfromme.index(min(self.distfromme))][2]
            else:
                del self.enemyblock_attacknum[self.distfromme.index(min(self.distfromme))]
                del self.distfromhim[self.distfromhim.index(min(self.distfromhim))]
                del self.distfromme[self.distfromme.index(min(self.distfromme))]
                self.choosedgoal_x_robot1 = self.enemyblock_attacknum[self.distfromhim.index(min(self.distfromhim))][0]
                self.choosedgoal_y_robot1 = self.enemyblock_attacknum[self.distfromhim.index(min(self.distfromhim))][1]
                self.choosedgoal_yaw_robot1 = self.enemyblock_attacknum[self.distfromhim.index(min(self.distfromhim))][2]
                self.farchoosegoal_x = self.enemyblock_attacknum[self.distfromme.index(min(self.distfromme))][0]
                self.farchoosegoal_y = self.enemyblock_attacknum[self.distfromme.index(min(self.distfromme))][1]
                self.farchoosegoal_yaw = self.enemyblock_attacknum[self.distfromme.index(min(self.distfromme))][2]
        elif goal_x >1.9 and goal_x <= 3.15 and goal_y >= 0 and goal_y <= 2:
            self.enemyblock_num = 5
            for k in range(0, len(self.enemyblock_attacknum)): # 清空数组
                del self.enemyblock_attacknum[0]
            self.enemyblock_attacknum.append([1.1, 0.75, 0])
            self.enemyblock_attacknum.append([2.65, 1.67, -90])
            for k in range(0, len(self.distfromme)):
                del self.distfromme[0]
            for k in range(0, len(self.distfromhim)):
                del self.distfromhim[0]
            for k in enumerate(self.enemyblock_attacknum):
                self.distfromhim.append(
                    np.square(k[1][0] - self.MyPose_robot1['x']) + np.square(k[1][1] - self.MyPose_robot1['y']))
                self.distfromme.append(np.square(k[1][0] - self.MyPose['x']) + np.square(k[1][1] - self.MyPose['y']))
            if self.distfromme.index(min(self.distfromme)) != self.distfromhim.index(min(self.distfromhim)):
                self.choosedgoal_x_robot1 = self.enemyblock_attacknum[self.distfromhim.index(min(self.distfromhim))][0]
                self.choosedgoal_y_robot1 = self.enemyblock_attacknum[self.distfromhim.index(min(self.distfromhim))][1]
                self.choosedgoal_yaw_robot1 = self.enemyblock_attacknum[self.distfromhim.index(min(self.distfromhim))][2]
                self.farchoosegoal_x = self.enemyblock_attacknum[self.distfromme.index(min(self.distfromme))][0]
                self.farchoosegoal_y = self.enemyblock_attacknum[self.distfromme.index(min(self.distfromme))][1]
                self.farchoosegoal_yaw = self.enemyblock_attacknum[self.distfromme.index(min(self.distfromme))][2]
            else:
                del self.enemyblock_attacknum[self.distfromme.index(min(self.distfromme))]
                del self.distfromhim[self.distfromhim.index(min(self.distfromhim))]
                del self.distfromme[self.distfromme.index(min(self.distfromme))]
                self.choosedgoal_x_robot1 = self.enemyblock_attacknum[self.distfromhim.index(min(self.distfromhim))][0]
                self.choosedgoal_y_robot1 = self.enemyblock_attacknum[self.distfromhim.index(min(self.distfromhim))][1]
                self.choosedgoal_yaw_robot1 = self.enemyblock_attacknum[self.distfromhim.index(min(self.distfromhim))][2]
                self.farchoosegoal_x = self.enemyblock_attacknum[self.distfromme.index(min(self.distfromme))][0]
                self.farchoosegoal_y = self.enemyblock_attacknum[self.distfromme.index(min(self.distfromme))][1]
                self.farchoosegoal_yaw = self.enemyblock_attacknum[self.distfromme.index(min(self.distfromme))][2]
        elif (goal_x >= 3.15 and goal_x <= 6 and goal_y >= 0 and goal_y <= 2) or (goal_x >= 4 and goal_x <= 6 and goal_y >= 2 and goal_y <= 3):
            self.enemyblock_num = 6
            for k in range(0, len(self.enemyblock_attacknum)): # 清空数组
                del self.enemyblock_attacknum[0]
            self.enemyblock_attacknum.append([4, 2.5, -45])
            self.enemyblock_attacknum.append([5.5, 2.47, -135])
            self.enemyblock_attacknum.append([6, 1.8, -165])
            self.enemyblock_attacknum.append([6.1, 0.5, 135])
            for k in range(0, len(self.distfromme)):
                del self.distfromme[0]
            for k in range(0, len(self.distfromhim)):
                del self.distfromhim[0]
            for k in enumerate(self.enemyblock_attacknum):
                self.distfromhim.append(
                    np.square(k[1][0] - self.MyPose_robot1['x']) + np.square(k[1][1] - self.MyPose_robot1['y']))
                self.distfromme.append(np.square(k[1][0] - self.MyPose['x']) + np.square(k[1][1] - self.MyPose['y']))
            if self.distfromme.index(min(self.distfromme)) != self.distfromhim.index(min(self.distfromhim)):
                self.choosedgoal_x_robot1 = self.enemyblock_attacknum[self.distfromhim.index(min(self.distfromhim))][0]
                self.choosedgoal_y_robot1 = self.enemyblock_attacknum[self.distfromhim.index(min(self.distfromhim))][1]
                self.choosedgoal_yaw_robot1 = self.enemyblock_attacknum[self.distfromhim.index(min(self.distfromhim))][2]
                self.farchoosegoal_x = self.enemyblock_attacknum[self.distfromme.index(min(self.distfromme))][0]
                self.farchoosegoal_y = self.enemyblock_attacknum[self.distfromme.index(min(self.distfromme))][1]
                self.farchoosegoal_yaw = self.enemyblock_attacknum[self.distfromme.index(min(self.distfromme))][2]
            else:
                del self.enemyblock_attacknum[self.distfromme.index(min(self.distfromme))]
                del self.distfromhim[self.distfromhim.index(min(self.distfromhim))]
                del self.distfromme[self.distfromme.index(min(self.distfromme))]
                self.choosedgoal_x_robot1 = self.enemyblock_attacknum[self.distfromhim.index(min(self.distfromhim))][0]
                self.choosedgoal_y_robot1 = self.enemyblock_attacknum[self.distfromhim.index(min(self.distfromhim))][1]
                self.choosedgoal_yaw_robot1 = self.enemyblock_attacknum[self.distfromhim.index(min(self.distfromhim))][2]
                self.farchoosegoal_x = self.enemyblock_attacknum[self.distfromme.index(min(self.distfromme))][0]
                self.farchoosegoal_y = self.enemyblock_attacknum[self.distfromme.index(min(self.distfromme))][1]
                self.farchoosegoal_yaw = self.enemyblock_attacknum[self.distfromme.index(min(self.distfromme))][2]
        elif goal_x > 6 and goal_x <= 8 and goal_y >= 0 and goal_y <= 1:
            self.enemyblock_num = 7
            for k in range(0, len(self.enemyblock_attacknum)): # 清空数组
                del self.enemyblock_attacknum[0]
            self.enemyblock_attacknum.append([5.4, 0.5, 0])
            self.enemyblock_attacknum.append([7.45, 1.8, -90])
            for k in range(0, len(self.distfromme)):
                del self.distfromme[0]
            for k in range(0, len(self.distfromhim)):
                del self.distfromhim[0]
            for k in enumerate(self.enemyblock_attacknum):
                self.distfromhim.append(
                    np.square(k[1][0] - self.MyPose_robot1['x']) + np.square(k[1][1] - self.MyPose_robot1['y']))
                self.distfromme.append(np.square(k[1][0] - self.MyPose['x']) + np.square(k[1][1] - self.MyPose['y']))
            if self.distfromme.index(min(self.distfromme)) != self.distfromhim.index(min(self.distfromhim)):
                self.choosedgoal_x_robot1 = self.enemyblock_attacknum[self.distfromhim.index(min(self.distfromhim))][0]
                self.choosedgoal_y_robot1 = self.enemyblock_attacknum[self.distfromhim.index(min(self.distfromhim))][1]
                self.choosedgoal_yaw_robot1 = self.enemyblock_attacknum[self.distfromhim.index(min(self.distfromhim))][2]
                self.farchoosegoal_x = self.enemyblock_attacknum[self.distfromme.index(min(self.distfromme))][0]
                self.farchoosegoal_y = self.enemyblock_attacknum[self.distfromme.index(min(self.distfromme))][1]
                self.farchoosegoal_yaw = self.enemyblock_attacknum[self.distfromme.index(min(self.distfromme))][2]
            else:
                del self.enemyblock_attacknum[self.distfromme.index(min(self.distfromme))]
                del self.distfromhim[self.distfromhim.index(min(self.distfromhim))]
                del self.distfromme[self.distfromme.index(min(self.distfromme))]
                self.choosedgoal_x_robot1 = self.enemyblock_attacknum[self.distfromhim.index(min(self.distfromhim))][0]
                self.choosedgoal_y_robot1 = self.enemyblock_attacknum[self.distfromhim.index(min(self.distfromhim))][1]
                self.choosedgoal_yaw_robot1 = self.enemyblock_attacknum[self.distfromhim.index(min(self.distfromhim))][2]
                self.farchoosegoal_x = self.enemyblock_attacknum[self.distfromme.index(min(self.distfromme))][0]
                self.farchoosegoal_y = self.enemyblock_attacknum[self.distfromme.index(min(self.distfromme))][1]
                self.farchoosegoal_yaw = self.enemyblock_attacknum[self.distfromme.index(min(self.distfromme))][2]
        elif goal_x > 6 and goal_x <= 8 and goal_y >= 1 and goal_y <= 2.6:
            self.enemyblock_num = 8
            for k in range(0, len(self.enemyblock_attacknum)): # 清空数组
                del self.enemyblock_attacknum[0]
            self.enemyblock_attacknum.append([7.45, 1.1, 120])
            self.enemyblock_attacknum.append([5.5, 1.8, 0])
            self.enemyblock_attacknum.append([6.75, 2.65, -60])
            for k in range(0, len(self.distfromme)):
                del self.distfromme[0]
            for k in range(0, len(self.distfromhim)):
                del self.distfromhim[0]
            for k in enumerate(self.enemyblock_attacknum):
                self.distfromhim.append(
                    np.square(k[1][0] - self.MyPose_robot1['x']) + np.square(k[1][1] - self.MyPose_robot1['y']))
                self.distfromme.append(np.square(k[1][0] - self.MyPose['x']) + np.square(k[1][1] - self.MyPose['y']))
            if self.distfromme.index(min(self.distfromme)) != self.distfromhim.index(min(self.distfromhim)):
                self.choosedgoal_x_robot1 = self.enemyblock_attacknum[self.distfromhim.index(min(self.distfromhim))][0]
                self.choosedgoal_y_robot1 = self.enemyblock_attacknum[self.distfromhim.index(min(self.distfromhim))][1]
                self.choosedgoal_yaw_robot1 = self.enemyblock_attacknum[self.distfromhim.index(min(self.distfromhim))][2]
                self.farchoosegoal_x = self.enemyblock_attacknum[self.distfromme.index(min(self.distfromme))][0]
                self.farchoosegoal_y = self.enemyblock_attacknum[self.distfromme.index(min(self.distfromme))][1]
                self.farchoosegoal_yaw = self.enemyblock_attacknum[self.distfromme.index(min(self.distfromme))][2]
            else:
                del self.enemyblock_attacknum[self.distfromme.index(min(self.distfromme))]
                del self.distfromhim[self.distfromhim.index(min(self.distfromhim))]
                del self.distfromme[self.distfromme.index(min(self.distfromme))]
                self.choosedgoal_x_robot1 = self.enemyblock_attacknum[self.distfromhim.index(min(self.distfromhim))][0]
                self.choosedgoal_y_robot1 = self.enemyblock_attacknum[self.distfromhim.index(min(self.distfromhim))][1]
                self.choosedgoal_yaw_robot1 = self.enemyblock_attacknum[self.distfromhim.index(min(self.distfromhim))][2]
                self.farchoosegoal_x = self.enemyblock_attacknum[self.distfromme.index(min(self.distfromme))][0]
                self.farchoosegoal_y = self.enemyblock_attacknum[self.distfromme.index(min(self.distfromme))][1]
                self.farchoosegoal_yaw = self.enemyblock_attacknum[self.distfromme.index(min(self.distfromme))][2]
        elif goal_x > 6 and goal_x <= 8 and goal_y >= 2.6 and goal_y <= 5:
            self.enemyblock_num = 9
            for k in range(0, len(self.enemyblock_attacknum)): # 清空数组
                del self.enemyblock_attacknum[0]
            self.enemyblock_attacknum.append([6.7, 2.75, 65])
            self.enemyblock_attacknum.append([5.75, 4.3, -30])
            for k in range(0, len(self.distfromme)):
                del self.distfromme[0]
            for k in range(0, len(self.distfromhim)):
                del self.distfromhim[0]
            for k in enumerate(self.enemyblock_attacknum):
                self.distfromhim.append(
                    np.square(k[1][0] - self.MyPose_robot1['x']) + np.square(k[1][1] - self.MyPose_robot1['y']))
                self.distfromme.append(np.square(k[1][0] - self.MyPose['x']) + np.square(k[1][1] - self.MyPose['y']))
            if self.distfromme.index(min(self.distfromme)) != self.distfromhim.index(min(self.distfromhim)):
                self.choosedgoal_x_robot1 = self.enemyblock_attacknum[self.distfromhim.index(min(self.distfromhim))][0]
                self.choosedgoal_y_robot1 = self.enemyblock_attacknum[self.distfromhim.index(min(self.distfromhim))][1]
                self.choosedgoal_yaw_robot1 = self.enemyblock_attacknum[self.distfromhim.index(min(self.distfromhim))][2]
                self.farchoosegoal_x = self.enemyblock_attacknum[self.distfromme.index(min(self.distfromme))][0]
                self.farchoosegoal_y = self.enemyblock_attacknum[self.distfromme.index(min(self.distfromme))][1]
                self.farchoosegoal_yaw = self.enemyblock_attacknum[self.distfromme.index(min(self.distfromme))][2]
            else:
                del self.enemyblock_attacknum[self.distfromme.index(min(self.distfromme))]
                del self.distfromhim[self.distfromhim.index(min(self.distfromhim))]
                del self.distfromme[self.distfromme.index(min(self.distfromme))]
                self.choosedgoal_x_robot1 = self.enemyblock_attacknum[self.distfromhim.index(min(self.distfromhim))][0]
                self.choosedgoal_y_robot1 = self.enemyblock_attacknum[self.distfromhim.index(min(self.distfromhim))][1]
                self.choosedgoal_yaw_robot1 = self.enemyblock_attacknum[self.distfromhim.index(min(self.distfromhim))][2]
                self.farchoosegoal_x = self.enemyblock_attacknum[self.distfromme.index(min(self.distfromme))][0]
                self.farchoosegoal_y = self.enemyblock_attacknum[self.distfromme.index(min(self.distfromme))][1]
                self.farchoosegoal_yaw = self.enemyblock_attacknum[self.distfromme.index(min(self.distfromme))][2]
        elif goal_x > 4.7 and goal_x <= 6 and goal_y >= 3 and goal_y <= 5:
            self.enemyblock_num = 10
            for k in range(0, len(self.enemyblock_attacknum)): # 清空数组
                del self.enemyblock_attacknum[0]
            self.enemyblock_attacknum.append([5.45, 2.9, 90])
            self.enemyblock_attacknum.append([7, 4.3, -175])
            for k in range(0, len(self.distfromme)):
                del self.distfromme[0]
            for k in range(0, len(self.distfromhim)):
                del self.distfromhim[0]
            for k in enumerate(self.enemyblock_attacknum):
                self.distfromhim.append(
                    np.square(k[1][0] - self.MyPose_robot1['x']) + np.square(k[1][1] - self.MyPose_robot1['y']))
                self.distfromme.append(np.square(k[1][0] - self.MyPose['x']) + np.square(k[1][1] - self.MyPose['y']))
            if self.distfromme.index(min(self.distfromme)) != self.distfromhim.index(min(self.distfromhim)):
                self.choosedgoal_x_robot1 = self.enemyblock_attacknum[self.distfromhim.index(min(self.distfromhim))][0]
                self.choosedgoal_y_robot1 = self.enemyblock_attacknum[self.distfromhim.index(min(self.distfromhim))][1]
                self.choosedgoal_yaw_robot1 = self.enemyblock_attacknum[self.distfromhim.index(min(self.distfromhim))][2]
                self.farchoosegoal_x = self.enemyblock_attacknum[self.distfromme.index(min(self.distfromme))][0]
                self.farchoosegoal_y = self.enemyblock_attacknum[self.distfromme.index(min(self.distfromme))][1]
                self.farchoosegoal_yaw = self.enemyblock_attacknum[self.distfromme.index(min(self.distfromme))][2]
            else:
                del self.enemyblock_attacknum[self.distfromme.index(min(self.distfromme))]
                del self.distfromhim[self.distfromhim.index(min(self.distfromhim))]
                del self.distfromme[self.distfromme.index(min(self.distfromme))]
                self.choosedgoal_x_robot1 = self.enemyblock_attacknum[self.distfromhim.index(min(self.distfromhim))][0]
                self.choosedgoal_y_robot1 = self.enemyblock_attacknum[self.distfromhim.index(min(self.distfromhim))][1]
                self.choosedgoal_yaw_robot1 = self.enemyblock_attacknum[self.distfromhim.index(min(self.distfromhim))][2]
                self.farchoosegoal_x = self.enemyblock_attacknum[self.distfromme.index(min(self.distfromme))][0]
                self.farchoosegoal_y = self.enemyblock_attacknum[self.distfromme.index(min(self.distfromme))][1]
                self.farchoosegoal_yaw = self.enemyblock_attacknum[self.distfromme.index(min(self.distfromme))][2]
        else:
            print 'enemy_analysis out of map!!!!!!!!!!!1'

    def Enemypose_Analysis_robot1(self, goal_x, goal_y, goal_yaw):
        if goal_x >= 0 and goal_x <= 1.9 and goal_y >= 0 and goal_y <= 2.35:
            self.enemyblock_num_robot1 = 1
            for k in range(0, len(self.enemyblock_attacknum_robot1)): # 清空数组
                del self.enemyblock_attacknum_robot1[0]
            self.enemyblock_attacknum_robot1.append([1.3, 2.15, -95])
            self.enemyblock_attacknum_robot1.append([1.9, 0.8, 170])
            for k in range(0, len(self.distfromme_robot1)):
                del self.distfromme_robot1[0]
            for k in range(0, len(self.distfromhim_robot1)):
                del self.distfromhim_robot1[0]
            for k in enumerate(self.enemyblock_attacknum_robot1):
                self.distfromhim_robot1.append(
                    np.square(k[1][0] - self.MyPose['x']) + np.square(k[1][1] - self.MyPose['y']))
                self.distfromme_robot1.append(np.square(k[1][0] - self.MyPose_robot1['x']) + np.square(k[1][1] - self.MyPose_robot1['y']))
            if self.distfromme_robot1.index(min(self.distfromme_robot1)) != self.distfromhim_robot1.index(min(self.distfromhim_robot1)):
                self.choosedgoal_x = self.enemyblock_attacknum_robot1[self.distfromhim_robot1.index(min(self.distfromhim_robot1))][0]
                self.choosedgoal_y = self.enemyblock_attacknum_robot1[self.distfromhim_robot1.index(min(self.distfromhim_robot1))][1]
                self.choosedgoal_yaw = self.enemyblock_attacknum_robot1[self.distfromhim_robot1.index(min(self.distfromhim_robot1))][2]
                self.farchoosegoal_x_robot1 = \
                self.enemyblock_attacknum_robot1[self.distfromme_robot1.index(min(self.distfromme_robot1))][0]
                self.farchoosegoal_y_robot1 = \
                self.enemyblock_attacknum_robot1[self.distfromme_robot1.index(min(self.distfromme_robot1))][1]
                self.farchoosegoal_yaw_robot1 = \
                self.enemyblock_attacknum_robot1[self.distfromme_robot1.index(min(self.distfromme_robot1))][2]
            else:
                del self.enemyblock_attacknum_robot1[self.distfromme_robot1.index(min(self.distfromme_robot1))]
                del self.distfromhim_robot1[self.distfromhim_robot1.index(min(self.distfromhim_robot1))]
                del self.distfromme_robot1[self.distfromme_robot1.index(min(self.distfromme_robot1))]
                self.choosedgoal_x = self.enemyblock_attacknum_robot1[self.distfromhim_robot1.index(min(self.distfromhim_robot1))][0]
                self.choosedgoal_y = self.enemyblock_attacknum_robot1[self.distfromhim_robot1.index(min(self.distfromhim_robot1))][1]
                self.choosedgoal_yaw = self.enemyblock_attacknum_robot1[self.distfromhim_robot1.index(min(self.distfromhim_robot1))][2]
                self.farchoosegoal_x_robot1 = \
                self.enemyblock_attacknum_robot1[self.distfromme_robot1.index(min(self.distfromme_robot1))][0]
                self.farchoosegoal_y_robot1 = \
                self.enemyblock_attacknum_robot1[self.distfromme_robot1.index(min(self.distfromme_robot1))][1]
                self.farchoosegoal_yaw_robot1 = \
                self.enemyblock_attacknum_robot1[self.distfromme_robot1.index(min(self.distfromme_robot1))][2]
        elif goal_x >=0 and goal_x <= 1.9 and goal_y > 2.35 and goal_y <= 4:
            self.enemyblock_num_robot1 = 2
            for k in range(0, len(self.enemyblock_attacknum_robot1)): # 清空数组
                del self.enemyblock_attacknum_robot1[0]
            self.enemyblock_attacknum_robot1.append([1.35, 2.4, 100])
            self.enemyblock_attacknum_robot1.append([0.65, 4, -75])
            self.enemyblock_attacknum_robot1.append([2.35, 3.25, 178])
            for k in range(0, len(self.distfromme_robot1)):
                del self.distfromme_robot1[0]
            for k in range(0, len(self.distfromhim_robot1)):
                del self.distfromhim_robot1[0]
            for k in enumerate(self.enemyblock_attacknum_robot1):
                self.distfromhim_robot1.append(
                    np.square(k[1][0] - self.MyPose['x']) + np.square(k[1][1] - self.MyPose['y']))
                self.distfromme_robot1.append(np.square(k[1][0] - self.MyPose_robot1['x']) + np.square(k[1][1] - self.MyPose_robot1['y']))
            if self.distfromme_robot1.index(min(self.distfromme_robot1)) != self.distfromhim_robot1.index(min(self.distfromhim_robot1)):
                self.choosedgoal_x = self.enemyblock_attacknum_robot1[self.distfromhim_robot1.index(min(self.distfromhim_robot1))][0]
                self.choosedgoal_y = self.enemyblock_attacknum_robot1[self.distfromhim_robot1.index(min(self.distfromhim_robot1))][1]
                self.choosedgoal_yaw = self.enemyblock_attacknum_robot1[self.distfromhim_robot1.index(min(self.distfromhim_robot1))][2]
                self.farchoosegoal_x_robot1 = \
                    self.enemyblock_attacknum_robot1[self.distfromme_robot1.index(min(self.distfromme_robot1))][0]
                self.farchoosegoal_y_robot1 = \
                    self.enemyblock_attacknum_robot1[self.distfromme_robot1.index(min(self.distfromme_robot1))][1]
                self.farchoosegoal_yaw_robot1 = \
                    self.enemyblock_attacknum_robot1[self.distfromme_robot1.index(min(self.distfromme_robot1))][2]
            else:
                del self.enemyblock_attacknum_robot1[self.distfromme_robot1.index(min(self.distfromme_robot1))]
                del self.distfromhim_robot1[self.distfromhim_robot1.index(min(self.distfromhim_robot1))]
                del self.distfromme_robot1[self.distfromme_robot1.index(min(self.distfromme_robot1))]
                self.choosedgoal_x = self.enemyblock_attacknum_robot1[self.distfromhim_robot1.index(min(self.distfromhim_robot1))][0]
                self.choosedgoal_y = self.enemyblock_attacknum_robot1[self.distfromhim_robot1.index(min(self.distfromhim_robot1))][1]
                self.choosedgoal_yaw = self.enemyblock_attacknum_robot1[self.distfromhim_robot1.index(min(self.distfromhim_robot1))][2]
                self.farchoosegoal_x_robot1 = \
                    self.enemyblock_attacknum_robot1[self.distfromme_robot1.index(min(self.distfromme_robot1))][0]
                self.farchoosegoal_y_robot1 = \
                    self.enemyblock_attacknum_robot1[self.distfromme_robot1.index(min(self.distfromme_robot1))][1]
                self.farchoosegoal_yaw_robot1 = \
                    self.enemyblock_attacknum_robot1[self.distfromme_robot1.index(min(self.distfromme_robot1))][2]
        elif goal_x >= 0 and goal_x <= 1.9 and goal_y > 4 and goal_y <= 5:
            self.enemyblock_num_robot1 = 3
            for k in range(0, len(self.enemyblock_attacknum_robot1)): # 清空数组
                del self.enemyblock_attacknum_robot1[0]
            self.enemyblock_attacknum_robot1.append([0.65, 3.7, 70])
            self.enemyblock_attacknum_robot1.append([2.35, 4.5, 178])
            for k in range(0, len(self.distfromme_robot1)):
                del self.distfromme_robot1[0]
            for k in range(0, len(self.distfromhim_robot1)):
                del self.distfromhim_robot1[0]
            for k in enumerate(self.enemyblock_attacknum_robot1):
                self.distfromhim_robot1.append(
                    np.square(k[1][0] - self.MyPose['x']) + np.square(k[1][1] - self.MyPose['y']))
                self.distfromme_robot1.append(np.square(k[1][0] - self.MyPose_robot1['x']) + np.square(k[1][1] - self.MyPose_robot1['y']))
            if self.distfromme_robot1.index(min(self.distfromme_robot1)) != self.distfromhim_robot1.index(min(self.distfromhim_robot1)):
                self.choosedgoal_x = self.enemyblock_attacknum_robot1[self.distfromhim_robot1.index(min(self.distfromhim_robot1))][0]
                self.choosedgoal_y = self.enemyblock_attacknum_robot1[self.distfromhim_robot1.index(min(self.distfromhim_robot1))][1]
                self.choosedgoal_yaw = self.enemyblock_attacknum_robot1[self.distfromhim_robot1.index(min(self.distfromhim_robot1))][2]
                self.farchoosegoal_x_robot1 = \
                    self.enemyblock_attacknum_robot1[self.distfromme_robot1.index(min(self.distfromme_robot1))][0]
                self.farchoosegoal_y_robot1 = \
                    self.enemyblock_attacknum_robot1[self.distfromme_robot1.index(min(self.distfromme_robot1))][1]
                self.farchoosegoal_yaw_robot1 = \
                    self.enemyblock_attacknum_robot1[self.distfromme_robot1.index(min(self.distfromme_robot1))][2]
            else:
                del self.enemyblock_attacknum_robot1[self.distfromme_robot1.index(min(self.distfromme_robot1))]
                del self.distfromhim_robot1[self.distfromhim_robot1.index(min(self.distfromhim_robot1))]
                del self.distfromme_robot1[self.distfromme_robot1.index(min(self.distfromme_robot1))]
                self.choosedgoal_x = self.enemyblock_attacknum_robot1[self.distfromhim_robot1.index(min(self.distfromhim_robot1))][0]
                self.choosedgoal_y = self.enemyblock_attacknum_robot1[self.distfromhim_robot1.index(min(self.distfromhim_robot1))][1]
                self.choosedgoal_yaw = self.enemyblock_attacknum_robot1[self.distfromhim_robot1.index(min(self.distfromhim_robot1))][2]
                self.farchoosegoal_x_robot1 = \
                    self.enemyblock_attacknum_robot1[self.distfromme_robot1.index(min(self.distfromme_robot1))][0]
                self.farchoosegoal_y_robot1 = \
                    self.enemyblock_attacknum_robot1[self.distfromme_robot1.index(min(self.distfromme_robot1))][1]
                self.farchoosegoal_yaw_robot1 = \
                    self.enemyblock_attacknum_robot1[self.distfromme_robot1.index(min(self.distfromme_robot1))][2]
        elif (goal_x > 1.9 and goal_x <= 4.7 and goal_y >= 3 and goal_y <= 5) or (goal_x > 1.9 and goal_x <= 4 and goal_y >= 2 and goal_y <= 3):
            self.enemyblock_num_robot1 = 4
            for k in range(0, len(self.enemyblock_attacknum_robot1)): # 清空数组
                del self.enemyblock_attacknum_robot1[0]
            self.enemyblock_attacknum_robot1.append([2, 4.5, -55])
            self.enemyblock_attacknum_robot1.append([2, 3.2, 0])
            self.enemyblock_attacknum_robot1.append([2.65, 2.55, 45])
            self.enemyblock_attacknum_robot1.append([4, 2.5, 135])
            for k in range(0, len(self.distfromme_robot1)):
                del self.distfromme_robot1[0]
            for k in range(0, len(self.distfromhim_robot1)):
                del self.distfromhim_robot1[0]
            for k in enumerate(self.enemyblock_attacknum_robot1):
                self.distfromhim_robot1.append(
                    np.square(k[1][0] - self.MyPose['x']) + np.square(k[1][1] - self.MyPose['y']))
                self.distfromme_robot1.append(np.square(k[1][0] - self.MyPose_robot1['x']) + np.square(k[1][1] - self.MyPose_robot1['y']))
            if self.distfromme_robot1.index(min(self.distfromme_robot1)) != self.distfromhim_robot1.index(min(self.distfromhim_robot1)):
                self.choosedgoal_x = self.enemyblock_attacknum_robot1[self.distfromhim_robot1.index(min(self.distfromhim_robot1))][0]
                self.choosedgoal_y = self.enemyblock_attacknum_robot1[self.distfromhim_robot1.index(min(self.distfromhim_robot1))][1]
                self.choosedgoal_yaw = self.enemyblock_attacknum_robot1[self.distfromhim_robot1.index(min(self.distfromhim_robot1))][2]
                self.farchoosegoal_x_robot1 = \
                    self.enemyblock_attacknum_robot1[self.distfromme_robot1.index(min(self.distfromme_robot1))][0]
                self.farchoosegoal_y_robot1 = \
                    self.enemyblock_attacknum_robot1[self.distfromme_robot1.index(min(self.distfromme_robot1))][1]
                self.farchoosegoal_yaw_robot1 = \
                    self.enemyblock_attacknum_robot1[self.distfromme_robot1.index(min(self.distfromme_robot1))][2]
            else:
                del self.enemyblock_attacknum_robot1[self.distfromme_robot1.index(min(self.distfromme_robot1))]
                del self.distfromhim_robot1[self.distfromhim_robot1.index(min(self.distfromhim_robot1))]
                del self.distfromme_robot1[self.distfromme_robot1.index(min(self.distfromme_robot1))]
                self.choosedgoal_x = self.enemyblock_attacknum_robot1[self.distfromhim_robot1.index(min(self.distfromhim_robot1))][0]
                self.choosedgoal_y = self.enemyblock_attacknum_robot1[self.distfromhim_robot1.index(min(self.distfromhim_robot1))][1]
                self.choosedgoal_yaw = self.enemyblock_attacknum_robot1[self.distfromhim_robot1.index(min(self.distfromhim_robot1))][2]
                self.farchoosegoal_x_robot1 = \
                    self.enemyblock_attacknum_robot1[self.distfromme_robot1.index(min(self.distfromme_robot1))][0]
                self.farchoosegoal_y_robot1 = \
                    self.enemyblock_attacknum_robot1[self.distfromme_robot1.index(min(self.distfromme_robot1))][1]
                self.farchoosegoal_yaw_robot1 = \
                    self.enemyblock_attacknum_robot1[self.distfromme_robot1.index(min(self.distfromme_robot1))][2]
        elif goal_x >1.9 and goal_x <= 3.15 and goal_y >= 0 and goal_y <= 2:
            self.enemyblock_num_robot1 = 5
            for k in range(0, len(self.enemyblock_attacknum_robot1)): # 清空数组
                del self.enemyblock_attacknum_robot1[0]
            self.enemyblock_attacknum_robot1.append([1.1, 0.75, 0])
            self.enemyblock_attacknum_robot1.append([2.65, 1.67, -90])
            for k in range(0, len(self.distfromme_robot1)):
                del self.distfromme_robot1[0]
            for k in range(0, len(self.distfromhim_robot1)):
                del self.distfromhim_robot1[0]
            for k in enumerate(self.enemyblock_attacknum_robot1):
                self.distfromhim_robot1.append(
                    np.square(k[1][0] - self.MyPose['x']) + np.square(k[1][1] - self.MyPose['y']))
                self.distfromme_robot1.append(np.square(k[1][0] - self.MyPose_robot1['x']) + np.square(k[1][1] - self.MyPose_robot1['y']))
            if self.distfromme_robot1.index(min(self.distfromme_robot1)) != self.distfromhim_robot1.index(min(self.distfromhim_robot1)):
                self.choosedgoal_x = self.enemyblock_attacknum_robot1[self.distfromhim_robot1.index(min(self.distfromhim_robot1))][0]
                self.choosedgoal_y = self.enemyblock_attacknum_robot1[self.distfromhim_robot1.index(min(self.distfromhim_robot1))][1]
                self.choosedgoal_yaw = self.enemyblock_attacknum_robot1[self.distfromhim_robot1.index(min(self.distfromhim_robot1))][2]
                self.farchoosegoal_x_robot1 = \
                    self.enemyblock_attacknum_robot1[self.distfromme_robot1.index(min(self.distfromme_robot1))][0]
                self.farchoosegoal_y_robot1 = \
                    self.enemyblock_attacknum_robot1[self.distfromme_robot1.index(min(self.distfromme_robot1))][1]
                self.farchoosegoal_yaw_robot1 = \
                    self.enemyblock_attacknum_robot1[self.distfromme_robot1.index(min(self.distfromme_robot1))][2]
            else:
                del self.enemyblock_attacknum_robot1[self.distfromme_robot1.index(min(self.distfromme_robot1))]
                del self.distfromhim_robot1[self.distfromhim_robot1.index(min(self.distfromhim_robot1))]
                del self.distfromme_robot1[self.distfromme_robot1.index(min(self.distfromme_robot1))]
                self.choosedgoal_x = self.enemyblock_attacknum_robot1[self.distfromhim_robot1.index(min(self.distfromhim_robot1))][0]
                self.choosedgoal_y = self.enemyblock_attacknum_robot1[self.distfromhim_robot1.index(min(self.distfromhim_robot1))][1]
                self.choosedgoal_yaw = self.enemyblock_attacknum_robot1[self.distfromhim_robot1.index(min(self.distfromhim_robot1))][2]
                self.farchoosegoal_x_robot1 = \
                    self.enemyblock_attacknum_robot1[self.distfromme_robot1.index(min(self.distfromme_robot1))][0]
                self.farchoosegoal_y_robot1 = \
                    self.enemyblock_attacknum_robot1[self.distfromme_robot1.index(min(self.distfromme_robot1))][1]
                self.farchoosegoal_yaw_robot1 = \
                    self.enemyblock_attacknum_robot1[self.distfromme_robot1.index(min(self.distfromme_robot1))][2]
        elif (goal_x >= 3.15 and goal_x <= 6 and goal_y >= 0 and goal_y <= 2) or (goal_x >= 4 and goal_x <= 6 and goal_y >= 2 and goal_y <= 3):
            self.enemyblock_num_robot1 = 6
            for k in range(0, len(self.enemyblock_attacknum_robot1)): # 清空数组
                del self.enemyblock_attacknum_robot1[0]
            self.enemyblock_attacknum_robot1.append([4, 2.5, -45])
            self.enemyblock_attacknum_robot1.append([5.5, 2.47, -135])
            self.enemyblock_attacknum_robot1.append([6, 1.8, -165])
            self.enemyblock_attacknum_robot1.append([6.1, 0.5, 135])
            for k in range(0, len(self.distfromme_robot1)):
                del self.distfromme_robot1[0]
            for k in range(0, len(self.distfromhim_robot1)):
                del self.distfromhim_robot1[0]
            for k in enumerate(self.enemyblock_attacknum_robot1):
                self.distfromhim_robot1.append(
                    np.square(k[1][0] - self.MyPose['x']) + np.square(k[1][1] - self.MyPose['y']))
                self.distfromme_robot1.append(np.square(k[1][0] - self.MyPose_robot1['x']) + np.square(k[1][1] - self.MyPose_robot1['y']))
            if self.distfromme_robot1.index(min(self.distfromme_robot1)) != self.distfromhim_robot1.index(min(self.distfromhim_robot1)):
                self.choosedgoal_x = self.enemyblock_attacknum_robot1[self.distfromhim_robot1.index(min(self.distfromhim_robot1))][0]
                self.choosedgoal_y = self.enemyblock_attacknum_robot1[self.distfromhim_robot1.index(min(self.distfromhim_robot1))][1]
                self.choosedgoal_yaw = self.enemyblock_attacknum_robot1[self.distfromhim_robot1.index(min(self.distfromhim_robot1))][2]
                self.farchoosegoal_x_robot1 = \
                    self.enemyblock_attacknum_robot1[self.distfromme_robot1.index(min(self.distfromme_robot1))][0]
                self.farchoosegoal_y_robot1 = \
                    self.enemyblock_attacknum_robot1[self.distfromme_robot1.index(min(self.distfromme_robot1))][1]
                self.farchoosegoal_yaw_robot1 = \
                    self.enemyblock_attacknum_robot1[self.distfromme_robot1.index(min(self.distfromme_robot1))][2]
            else:
                del self.enemyblock_attacknum_robot1[self.distfromme_robot1.index(min(self.distfromme_robot1))]
                del self.distfromhim_robot1[self.distfromhim_robot1.index(min(self.distfromhim_robot1))]
                del self.distfromme_robot1[self.distfromme_robot1.index(min(self.distfromme_robot1))]
                self.choosedgoal_x = self.enemyblock_attacknum_robot1[self.distfromhim_robot1.index(min(self.distfromhim_robot1))][0]
                self.choosedgoal_y = self.enemyblock_attacknum_robot1[self.distfromhim_robot1.index(min(self.distfromhim_robot1))][1]
                self.choosedgoal_yaw = self.enemyblock_attacknum_robot1[self.distfromhim_robot1.index(min(self.distfromhim_robot1))][2]
                self.farchoosegoal_x_robot1 = \
                    self.enemyblock_attacknum_robot1[self.distfromme_robot1.index(min(self.distfromme_robot1))][0]
                self.farchoosegoal_y_robot1 = \
                    self.enemyblock_attacknum_robot1[self.distfromme_robot1.index(min(self.distfromme_robot1))][1]
                self.farchoosegoal_yaw_robot1 = \
                    self.enemyblock_attacknum_robot1[self.distfromme_robot1.index(min(self.distfromme_robot1))][2]
        elif goal_x > 6 and goal_x <= 8 and goal_y >= 0 and goal_y <= 1:
            self.enemyblock_num_robot1 = 7
            for k in range(0, len(self.enemyblock_attacknum_robot1)): # 清空数组
                del self.enemyblock_attacknum_robot1[0]
            self.enemyblock_attacknum_robot1.append([5.4, 0.5, 0])
            self.enemyblock_attacknum_robot1.append([7.45, 1.8, -90])
            for k in range(0, len(self.distfromme_robot1)):
                del self.distfromme_robot1[0]
            for k in range(0, len(self.distfromhim_robot1)):
                del self.distfromhim_robot1[0]
            for k in enumerate(self.enemyblock_attacknum_robot1):
                self.distfromhim_robot1.append(
                    np.square(k[1][0] - self.MyPose['x']) + np.square(k[1][1] - self.MyPose['y']))
                self.distfromme_robot1.append(np.square(k[1][0] - self.MyPose_robot1['x']) + np.square(k[1][1] - self.MyPose_robot1['y']))
            if self.distfromme_robot1.index(min(self.distfromme_robot1)) != self.distfromhim_robot1.index(min(self.distfromhim_robot1)):
                self.choosedgoal_x = self.enemyblock_attacknum_robot1[self.distfromhim_robot1.index(min(self.distfromhim_robot1))][0]
                self.choosedgoal_y = self.enemyblock_attacknum_robot1[self.distfromhim_robot1.index(min(self.distfromhim_robot1))][1]
                self.choosedgoal_yaw = self.enemyblock_attacknum_robot1[self.distfromhim_robot1.index(min(self.distfromhim_robot1))][2]
                self.farchoosegoal_x_robot1 = \
                    self.enemyblock_attacknum_robot1[self.distfromme_robot1.index(min(self.distfromme_robot1))][0]
                self.farchoosegoal_y_robot1 = \
                    self.enemyblock_attacknum_robot1[self.distfromme_robot1.index(min(self.distfromme_robot1))][1]
                self.farchoosegoal_yaw_robot1 = \
                    self.enemyblock_attacknum_robot1[self.distfromme_robot1.index(min(self.distfromme_robot1))][2]
            else:
                del self.enemyblock_attacknum_robot1[self.distfromme_robot1.index(min(self.distfromme_robot1))]
                del self.distfromhim_robot1[self.distfromhim_robot1.index(min(self.distfromhim_robot1))]
                del self.distfromme_robot1[self.distfromme_robot1.index(min(self.distfromme_robot1))]
                self.choosedgoal_x = self.enemyblock_attacknum_robot1[self.distfromhim_robot1.index(min(self.distfromhim_robot1))][0]
                self.choosedgoal_y = self.enemyblock_attacknum_robot1[self.distfromhim_robot1.index(min(self.distfromhim_robot1))][1]
                self.choosedgoal_yaw = self.enemyblock_attacknum_robot1[self.distfromhim_robot1.index(min(self.distfromhim_robot1))][2]
                self.farchoosegoal_x_robot1 = \
                    self.enemyblock_attacknum_robot1[self.distfromme_robot1.index(min(self.distfromme_robot1))][0]
                self.farchoosegoal_y_robot1 = \
                    self.enemyblock_attacknum_robot1[self.distfromme_robot1.index(min(self.distfromme_robot1))][1]
                self.farchoosegoal_yaw_robot1 = \
                    self.enemyblock_attacknum_robot1[self.distfromme_robot1.index(min(self.distfromme_robot1))][2]
        elif goal_x > 6 and goal_x <= 8 and goal_y >= 1 and goal_y <= 2.6:
            self.enemyblock_num_robot1 = 8
            for k in range(0, len(self.enemyblock_attacknum_robot1)): # 清空数组
                del self.enemyblock_attacknum_robot1[0]
            self.enemyblock_attacknum_robot1.append([7.45, 1.1, 120])
            self.enemyblock_attacknum_robot1.append([5.5, 1.8, 0])
            self.enemyblock_attacknum_robot1.append([6.75, 2.65, -60])
            for k in range(0, len(self.distfromme_robot1)):
                del self.distfromme_robot1[0]
            for k in range(0, len(self.distfromhim_robot1)):
                del self.distfromhim_robot1[0]
            for k in enumerate(self.enemyblock_attacknum_robot1):
                self.distfromhim_robot1.append(
                    np.square(k[1][0] - self.MyPose['x']) + np.square(k[1][1] - self.MyPose['y']))
                self.distfromme_robot1.append(np.square(k[1][0] - self.MyPose_robot1['x']) + np.square(k[1][1] - self.MyPose_robot1['y']))
            if self.distfromme_robot1.index(min(self.distfromme_robot1)) != self.distfromhim_robot1.index(min(self.distfromhim_robot1)):
                self.choosedgoal_x = self.enemyblock_attacknum_robot1[self.distfromhim_robot1.index(min(self.distfromhim_robot1))][0]
                self.choosedgoal_y = self.enemyblock_attacknum_robot1[self.distfromhim_robot1.index(min(self.distfromhim_robot1))][1]
                self.choosedgoal_yaw = self.enemyblock_attacknum_robot1[self.distfromhim_robot1.index(min(self.distfromhim_robot1))][2]
                self.farchoosegoal_x_robot1 = \
                    self.enemyblock_attacknum_robot1[self.distfromme_robot1.index(min(self.distfromme_robot1))][0]
                self.farchoosegoal_y_robot1 = \
                    self.enemyblock_attacknum_robot1[self.distfromme_robot1.index(min(self.distfromme_robot1))][1]
                self.farchoosegoal_yaw_robot1 = \
                    self.enemyblock_attacknum_robot1[self.distfromme_robot1.index(min(self.distfromme_robot1))][2]
            else:
                del self.enemyblock_attacknum_robot1[self.distfromme_robot1.index(min(self.distfromme_robot1))]
                del self.distfromhim_robot1[self.distfromhim_robot1.index(min(self.distfromhim_robot1))]
                del self.distfromme_robot1[self.distfromme_robot1.index(min(self.distfromme_robot1))]
                self.choosedgoal_x = self.enemyblock_attacknum_robot1[self.distfromhim_robot1.index(min(self.distfromhim_robot1))][0]
                self.choosedgoal_y = self.enemyblock_attacknum_robot1[self.distfromhim_robot1.index(min(self.distfromhim_robot1))][1]
                self.choosedgoal_yaw = self.enemyblock_attacknum_robot1[self.distfromhim_robot1.index(min(self.distfromhim_robot1))][2]
                self.farchoosegoal_x_robot1 = \
                    self.enemyblock_attacknum_robot1[self.distfromme_robot1.index(min(self.distfromme_robot1))][0]
                self.farchoosegoal_y_robot1 = \
                    self.enemyblock_attacknum_robot1[self.distfromme_robot1.index(min(self.distfromme_robot1))][1]
                self.farchoosegoal_yaw_robot1 = \
                    self.enemyblock_attacknum_robot1[self.distfromme_robot1.index(min(self.distfromme_robot1))][2]
        elif goal_x > 6 and goal_x <= 8 and goal_y >= 2.6 and goal_y <= 5:
            self.enemyblock_num_robot1 = 9
            for k in range(0, len(self.enemyblock_attacknum_robot1)): # 清空数组
                del self.enemyblock_attacknum_robot1[0]
            self.enemyblock_attacknum_robot1.append([6.7, 2.75, 65])
            self.enemyblock_attacknum_robot1.append([5.75, 4.3, -30])
            for k in range(0, len(self.distfromme_robot1)):
                del self.distfromme_robot1[0]
            for k in range(0, len(self.distfromhim_robot1)):
                del self.distfromhim_robot1[0]
            for k in enumerate(self.enemyblock_attacknum_robot1):
                self.distfromhim_robot1.append(
                    np.square(k[1][0] - self.MyPose['x']) + np.square(k[1][1] - self.MyPose['y']))
                self.distfromme_robot1.append(np.square(k[1][0] - self.MyPose_robot1['x']) + np.square(k[1][1] - self.MyPose_robot1['y']))
            if self.distfromme_robot1.index(min(self.distfromme_robot1)) != self.distfromhim_robot1.index(min(self.distfromhim_robot1)):
                self.choosedgoal_x = self.enemyblock_attacknum_robot1[self.distfromhim_robot1.index(min(self.distfromhim_robot1))][0]
                self.choosedgoal_y = self.enemyblock_attacknum_robot1[self.distfromhim_robot1.index(min(self.distfromhim_robot1))][1]
                self.choosedgoal_yaw = self.enemyblock_attacknum_robot1[self.distfromhim_robot1.index(min(self.distfromhim_robot1))][2]
                self.farchoosegoal_x_robot1 = \
                    self.enemyblock_attacknum_robot1[self.distfromme_robot1.index(min(self.distfromme_robot1))][0]
                self.farchoosegoal_y_robot1 = \
                    self.enemyblock_attacknum_robot1[self.distfromme_robot1.index(min(self.distfromme_robot1))][1]
                self.farchoosegoal_yaw_robot1 = \
                    self.enemyblock_attacknum_robot1[self.distfromme_robot1.index(min(self.distfromme_robot1))][2]
            else:
                del self.enemyblock_attacknum_robot1[self.distfromme_robot1.index(min(self.distfromme_robot1))]
                del self.distfromhim_robot1[self.distfromhim_robot1.index(min(self.distfromhim_robot1))]
                del self.distfromme_robot1[self.distfromme_robot1.index(min(self.distfromme_robot1))]
                self.choosedgoal_x = self.enemyblock_attacknum_robot1[self.distfromhim_robot1.index(min(self.distfromhim_robot1))][0]
                self.choosedgoal_y = self.enemyblock_attacknum_robot1[self.distfromhim_robot1.index(min(self.distfromhim_robot1))][1]
                self.choosedgoal_yaw = self.enemyblock_attacknum_robot1[self.distfromhim_robot1.index(min(self.distfromhim_robot1))][2]
                self.farchoosegoal_x_robot1 = \
                    self.enemyblock_attacknum_robot1[self.distfromme_robot1.index(min(self.distfromme_robot1))][0]
                self.farchoosegoal_y_robot1 = \
                    self.enemyblock_attacknum_robot1[self.distfromme_robot1.index(min(self.distfromme_robot1))][1]
                self.farchoosegoal_yaw_robot1 = \
                    self.enemyblock_attacknum_robot1[self.distfromme_robot1.index(min(self.distfromme_robot1))][2]
        elif goal_x > 4.7 and goal_x <= 6 and goal_y >= 3 and goal_y <= 5:
            self.enemyblock_num_robot1 = 10
            for k in range(0, len(self.enemyblock_attacknum_robot1)): # 清空数组
                del self.enemyblock_attacknum_robot1[0]
            self.enemyblock_attacknum_robot1.append([5.45, 2.9, 90])
            self.enemyblock_attacknum_robot1.append([7, 4.3, -175])
            for k in range(0, len(self.distfromme_robot1)):
                del self.distfromme_robot1[0]
            for k in range(0, len(self.distfromhim_robot1)):
                del self.distfromhim_robot1[0]
            for k in enumerate(self.enemyblock_attacknum_robot1):
                self.distfromhim_robot1.append(
                    np.square(k[1][0] - self.MyPose['x']) + np.square(k[1][1] - self.MyPose['y']))
                self.distfromme_robot1.append(np.square(k[1][0] - self.MyPose_robot1['x']) + np.square(k[1][1] - self.MyPose_robot1['y']))
            if self.distfromme_robot1.index(min(self.distfromme_robot1)) != self.distfromhim_robot1.index(min(self.distfromhim_robot1)):
                self.choosedgoal_x = self.enemyblock_attacknum_robot1[self.distfromhim_robot1.index(min(self.distfromhim_robot1))][0]
                self.choosedgoal_y = self.enemyblock_attacknum_robot1[self.distfromhim_robot1.index(min(self.distfromhim_robot1))][1]
                self.choosedgoal_yaw = self.enemyblock_attacknum_robot1[self.distfromhim_robot1.index(min(self.distfromhim_robot1))][2]
                self.farchoosegoal_x_robot1 = \
                    self.enemyblock_attacknum_robot1[self.distfromme_robot1.index(min(self.distfromme_robot1))][0]
                self.farchoosegoal_y_robot1 = \
                    self.enemyblock_attacknum_robot1[self.distfromme_robot1.index(min(self.distfromme_robot1))][1]
                self.farchoosegoal_yaw_robot1 = \
                    self.enemyblock_attacknum_robot1[self.distfromme_robot1.index(min(self.distfromme_robot1))][2]
            else:
                del self.enemyblock_attacknum_robot1[self.distfromme_robot1.index(min(self.distfromme_robot1))]
                del self.distfromhim_robot1[self.distfromhim_robot1.index(min(self.distfromhim_robot1))]
                del self.distfromme_robot1[self.distfromme_robot1.index(min(self.distfromme_robot1))]
                self.choosedgoal_x = self.enemyblock_attacknum_robot1[self.distfromhim_robot1.index(min(self.distfromhim_robot1))][0]
                self.choosedgoal_y = self.enemyblock_attacknum_robot1[self.distfromhim_robot1.index(min(self.distfromhim_robot1))][1]
                self.choosedgoal_yaw = self.enemyblock_attacknum_robot1[self.distfromhim_robot1.index(min(self.distfromhim_robot1))][2]
                self.farchoosegoal_x_robot1 = \
                    self.enemyblock_attacknum_robot1[self.distfromme_robot1.index(min(self.distfromme_robot1))][0]
                self.farchoosegoal_y_robot1 = \
                    self.enemyblock_attacknum_robot1[self.distfromme_robot1.index(min(self.distfromme_robot1))][1]
                self.farchoosegoal_yaw_robot1 = \
                    self.enemyblock_attacknum_robot1[self.distfromme_robot1.index(min(self.distfromme_robot1))][2]
        else:
            print 'enemy_analysis_robot1 out of map!!!!!!!!!!!1'


    def getGimbalPoseCallback(self, data):
        self.Gimbal.enemy_yaw = data.enemy_yaw
        self.Gimbal.enemy_pitch = data.enemy_pitch
        self.Gimbal.enemy_dist = data.enemy_dist

    def getGimbalPoseCallback_robot1(self, data):
        self.Gimbal_robot1.enemy_yaw = data.enemy_yaw
        self.Gimbal_robot1.enemy_pitch = data.enemy_pitch
        self.Gimbal_robot1.enemy_dist = data.enemy_dist

    # robot0的裁判系统
    def getHurtInfoCallback(self, data):
        if data.armor_type == 0 and data.hurt_type == 0:
            self.witch_armor = 0
            self.armor_nonecount = 0
            print 'witch 0'
        elif data.armor_type == 1 and data.hurt_type == 0:
            self.witch_armor = 1
            self.armor_nonecount = 0
            print 'witch 1'
        elif data.armor_type == 2 and data.hurt_type == 0:
            self.witch_armor = 2
            self.armor_nonecount = 0
            print 'witch 2'
        elif data.armor_type == 3 and data.hurt_type == 0:
            self.witch_armor = 3
            self.armor_nonecount = 0
            print 'witch 3'
        else:
            if self.armor_nonecount == 2:  #希望是3s内检测不被打才清0
                self.witch_armor = -1  #没被打
                self.armor_nonecount = 0
                print 'witch -1'
            else:
                self.armor_nonecount = self.armor_nonecount + 1

    # robot1的裁判系统
    def getHurtInfoCallback_robot1(self, data):
        if data.armor_type == 0 and data.hurt_type == 0:
            self.witch_armor_robot1 = 0
            self.armor_nonecount_robot1 = 0
        elif data.armor_type == 1 and data.hurt_type == 0:
            self.witch_armor_robot1 = 1
            self.armor_nonecount_robot1 = 0
        elif data.armor_type == 2 and data.hurt_type == 0:
            self.witch_armor_robot1 = 2
            self.armor_nonecount_robot1 = 0
        elif data.armor_type == 3 and data.hurt_type == 0:
            self.witch_armor_robot1 = 3
            self.armor_nonecount_robot1 = 0
        else:
            if self.armor_nonecount_robot1 == 30:  # 60次检测不被打才清0
                self.witch_armor_robot1 = -1  # 没被打
                self.armor_nonecount_robot1 = 0
            else:
                self.armor_nonecount_robot1 = self.armor_nonecount_robot1 + 1

    def getRFIDCallback(self, data):
        self.Rfid = data


    def getRFIDCallback_robot1(self, data):
        self.Rfid_robot1 = data

    def getMyHPCallback(self, data):
        # data = GameInfo()
        self.MyHP = data
        self.fiveseconds = data.game_process

    def getMyHPCallback_robot1(self, data):
        # data = GameInfo()
        self.MyHP_robot1 = data
        # self.fiveseconds = data.game_process


    def getGameBuffCallback(self, data):
        if data.buff_mask >> 13 == 1:
            self.Iget = 1
        if data.buff_mask >> 14 == 1:
            self.Eget = 1


    def getYoloEnemyCallback(self, data):
        # print 'camero is %s, %s' % (data.leftcamera,data.rightcamera)
        # self.yolo_enemy = data
        # data = YoloEnemy()
        # print self.witch_armor
        if self.witch_armor == -1:  #armor检测不到才执行
            if data.leftcamera > 0:
                self.witch_armor = 4
                print 'detect armor 4'
            elif data.rightcamera > 0:
                self.witch_armor = 5
                print 'detect armor 5'
            else:  #都没看到
                self.witch_armor = -1
        else:
            pass

    def getYoloEnemyCallback_robot1(self, data):
        # self.yolo_enemy = data
        # data = YoloEnemy()
        if self.witch_armor_robot1 == -1:  #armor检测不到才执行
            if data.leftcamera > 0:
                self.witch_armor_robot1 = 4
            elif data.rightcamera > 0:
                self.witch_armor_robot1 = 5
            else:  #都没看到
                self.witch_armor_robot1 = -1
        else:
            pass


    def getCommunite(self, data):
        self.beentreeflag = data.beentreeflag
        self.beentreeflag_robot1 = data.beentreeflag_robot1

    def getCommunite_robot1(self, data):
        self.beentreeflag = data.beentreeflag
        self.beentreeflag_robot1 = data.beentreeflag_robot1

    def isActionAvaliable(self, px, py, theta):
        ok = False
        x_goal = int(px / 0.05)
        y_goal = int(py / 0.05)
        quat = tf.transformations.quaternion_from_euler(0, 0, theta*3.1416/180)
        if (y_goal<=5) or (y_goal >= 96) or (x_goal >= 156) or (x_goal <= 5):
            pass
        elif self.map[101 - y_goal +10, x_goal + 10] == 255:  # 是否在地图可行区域
            self.navgoal.pose.position.x = x_goal * 0.05  # 101X161地图栅格
            self.navgoal.pose.position.y = y_goal * 0.05
            self.navgoal.pose.orientation.x = quat[0]
            self.navgoal.pose.orientation.y = quat[1]
            self.navgoal.pose.orientation.z = quat[2]
            self.navgoal.pose.orientation.w = quat[3]
            self.navgoal.header.stamp = rospy.Time().now()
            ok = True
        else:
            pass
        return ok

    def shoot_fric_wheel(self,shoot_cmd,controller):
        shoot_cmd.fric_wheel_spd = 1200  # 摩擦轮转速设置
    #发弹命令
    def shooting(self,shoot_cmd,controller):
        shoot_cmd.fric_wheel_run = 1#开关摩擦轮
        shoot_cmd.shoot_cmd = 1#单发命令
        shoot_cmd.c_shoot_cmd =0#连发命令
        controller.shoot(shoot_cmd)#发布射击命令

    def shooting_robot1(self,shoot_cmd,controller):
        shoot_cmd.fric_wheel_run = 1#开关摩擦轮
        shoot_cmd.shoot_cmd = 1#单发命令
        shoot_cmd.c_shoot_cmd =0#连发命令
        controller.shoot_robot1(shoot_cmd)#发布射击命令

    def shooting_plus(self, shoot_cmd, controller):
      #  shoot_cmd.fric_wheel_spd = 1215  # 摩擦轮转速设置
        shoot_cmd.fric_wheel_run = 1  # 开关摩擦轮
        shoot_cmd.shoot_cmd = 0  # 单发命令
        shoot_cmd.c_shoot_cmd = 1  # 连发命令
        controller.shoot(shoot_cmd)  # 发布射击命令

    def shooting_plus_robot1(self, shoot_cmd, controller):
      #  shoot_cmd.fric_wheel_spd = 1215  # 摩擦轮转速设置
        shoot_cmd.fric_wheel_run = 1  # 开关摩擦轮
        shoot_cmd.shoot_cmd = 0  # 单发命令
        shoot_cmd.c_shoot_cmd = 1  # 连发命令
        controller.shoot_robot1(shoot_cmd)  # 发布射击命令

    def shooting_stop(self, shoot_cmd, controller):
      #  shoot_cmd.fric_wheel_spd = 1215# 摩擦轮转速设置
        shoot_cmd.fric_wheel_run = 1  # 开关摩擦轮
        shoot_cmd.shoot_cmd = 0  # 单发命令
        shoot_cmd.c_shoot_cmd = 0  # 连发命令
        controller.shoot(shoot_cmd)  # 发布射击命令

    def shooting_stop_robot1(self, shoot_cmd, controller):
      #  shoot_cmd.fric_wheel_spd = 1215# 摩擦轮转速设置
        shoot_cmd.fric_wheel_run = 1  # 开关摩擦轮
        shoot_cmd.shoot_cmd = 0  # 单发命令
        shoot_cmd.c_shoot_cmd = 0  # 连发命令
        controller.shoot_robot1(shoot_cmd)  # 发布射击命令


if __name__ == '__main__':
    env = BattleEnv()
