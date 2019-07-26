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

# 慢速向前命令
cmdvel_slowfront = Twist()
cmdvel_slowfront.linear.x = 0.205
cmdvel_slowfront.linear.y = 0
cmdvel_slowfront.linear.z = 0
cmdvel_slowfront.angular.x = 0
cmdvel_slowfront.angular.y = 0
cmdvel_slowfront.angular.z = 0
# 慢速向后命令
cmdvel_slowback = Twist()
cmdvel_slowback.linear.x = -0.205
cmdvel_slowback.linear.y = 0
cmdvel_slowback.linear.z = 0
cmdvel_slowback.angular.x = 0
cmdvel_slowback.angular.y = 0
cmdvel_slowback.angular.z = 0

# 慢速正转
cmdvel_slowzuotwist = Twist()
cmdvel_slowzuotwist.linear.x = 0
cmdvel_slowzuotwist.linear.y = 0
cmdvel_slowzuotwist.linear.z = 0
cmdvel_slowzuotwist.angular.x = 0
cmdvel_slowzuotwist.angular.y = 0
cmdvel_slowzuotwist.angular.z = 1.2

# 慢速反转
cmdvel_slowyoutwist = Twist()
cmdvel_slowyoutwist.linear.x = 0
cmdvel_slowyoutwist.linear.y = 0
cmdvel_slowyoutwist.linear.z = 0
cmdvel_slowyoutwist.angular.x = 0
cmdvel_slowyoutwist.angular.y = 0
cmdvel_slowyoutwist.angular.z = -1.2

# 中速向前命令
cmdvel_middlefront = Twist()
cmdvel_middlefront.linear.x = 0.6
cmdvel_middlefront.linear.y = 0
cmdvel_middlefront.linear.z = 0
cmdvel_middlefront.angular.x = 0
cmdvel_middlefront.angular.y = 0
cmdvel_middlefront.angular.z = 0
# 中速向后命令
cmdvel_middleback = Twist()
cmdvel_middleback.linear.x = -0.5
cmdvel_middleback.linear.y = 0
cmdvel_middleback.linear.z = 0
cmdvel_middleback.angular.x = 0
cmdvel_middleback.angular.y = 0
cmdvel_middleback.angular.z = 0

# 最快速向前命令
cmdvel_fastfront = Twist()
cmdvel_fastfront.linear.x = 3.5
cmdvel_fastfront.linear.y = 0
cmdvel_fastfront.linear.z = 0
cmdvel_fastfront.angular.x = 0
cmdvel_fastfront.angular.y = 0
cmdvel_fastfront.angular.z = 0
# 最快速向左命令
cmdvel_fastleft = Twist()
cmdvel_fastleft.linear.x = 0
cmdvel_fastleft.linear.y = 3.5
cmdvel_fastleft.linear.z = 0
cmdvel_fastleft.angular.x = 0
cmdvel_fastleft.angular.y = 0
cmdvel_fastleft.angular.z = 0
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

class BlackBoard():
    def __init__(self):
        self.sended_close = False
        self.sended_far = False
        self.sended_beated = False
        self.sended_follow = False
        self.sended_patrol = False
        self.sended_blocked = False
        self.follow_flag = 0  # 0:跟随；1：左转；2：右转
        self.patrol_flag = 0  # 0:第一个点; 1：第二个点； 2：第三个点...
        self.patrol_flag1 = 0  # 0:先自旋360； 1：开始ptl
        self.patrol_flag2 = 0  # ptl点
        self.leftarea = False

        self.goallastforptl_x = 0
        self.goallastforptl_y = 0

        self.Blocked = False
        self.Blocked_Prevent_close = False
        self.Blocked_Prevent_far = False
        self.Blocked_Prevent_follow = False
        self.Blocked_Prevent_patrol = False
        self.Blocked_excuteflag = 0  # 0:转正；1：推；2：拯救点
        self.BlockedPoseSaveX = 0
        self.BlockedPoseSaveY = 0
        self.BlockedPoseSaveYaw = 0
        self.Blocked_GoalYaw = 0

        # 定义拯救点数组:5个点
        self.PreRescuePoint = ((1.35, 2.6, 45), (2.25, 4.5, -45), (6.65, 2.4, -135), (5.75, 0.5, 135))
        self.RescuePoint = ((4, 2.5, 0), (4.8, 1.8, 179), (3.2, 3.2, 0))
        self.avoidloop_far = 0
        self.avoidloop_follow = 0
        self.avoidloop_patrol = 0

class BiuldTree():
    def __init__(self):
        self.blackboard = BlackBoard()
        rate = rospy.Rate(10)
        NORMALBAHAVE = Selector("NORMALBAHAVE")

        CLOSE = Sequence("CLOSE")
        FAR = Sequence("FAR")
        BEATED = Sequence("BEATED")
        PATROL = Sequence("PATROL")
        FOLLOW = Sequence("FOLLOW")
        BLOCKED = Sequence("BLOCKED")

        NORMALBAHAVE.add_child(CLOSE)
        NORMALBAHAVE.add_child(FAR)
        NORMALBAHAVE.add_child(BEATED)
        NORMALBAHAVE.add_child(PATROL)
        NORMALBAHAVE.add_child(FOLLOW)
        NORMALBAHAVE.add_child(BLOCKED)

        ISCLOSE = IsClose("ISCLOSE", blackboard=self.blackboard)
        CLOSESHOOT = CloseShoot("CLOSESHOOT", blackboard=self.blackboard)
        CLOSE.add_child(ISCLOSE)
        CLOSE.add_child(CLOSESHOOT)

        IS_FAR = IsFar("IS_FAR", blackboard=self.blackboard)
        FARSHOOT = FarShoot("FARSHOOT", blackboard=self.blackboard)
        FAR.add_child(IS_FAR)
        FAR.add_child(FARSHOOT)

        IS_BEATED = IsBeated("IS_BEATED", blackboard=self.blackboard)
        BEATEDSHOOT = BeatedShoot("BEATEDSHOOT", blackboard=self.blackboard)
        BEATED.add_child(IS_BEATED)
        BEATED.add_child(BEATEDSHOOT)

        ISFOLLOW = IsFollow("ISFOLLOW", blackboard=self.blackboard)
        FOLLOW_SHOOT = Follow_Shoot("FOLLOW_SHOOT", blackboard=self.blackboard)
        FOLLOW.add_child(ISFOLLOW)
        FOLLOW.add_child(FOLLOW_SHOOT)

        ISPATROL = IsPatrol("ISPATROL", blackboard=self.blackboard)
        RANDOM = Random("RANDOM", blackboard=self.blackboard)
        PATROL.add_child(ISPATROL)
        PATROL.add_child(RANDOM)

        ISBLOCKED = IsBlocked("ISBLOCKED", blackboard=self.blackboard)
        BLOCKED_SHOOT = Blocked_Shoot("BLOCKED_SHOOT", blackboard=self.blackboard)
        BLOCKED.add_child(ISBLOCKED)
        BLOCKED.add_child(BLOCKED_SHOOT)

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
        if env.EnemyPoseSave.enemy_dist < 1.58 and env.EnemyPoseSave.enemy_dist > 0 and self.blackboard.Blocked_Prevent_close == False:
            self.blackboard.sended_far = False
            self.blackboard.sended_beated = False
            self.blackboard.sended_patrol = False
            self.blackboard.sended_follow = False
            self.blackboard.sended_blocked = False
            self.blackboard.patrol_flag = 0
            self.blackboard.follow_flag = 0
            self.blackboard.patrol_flag1 = 0
            self.blackboard.patrol_flag2 = 0

            self.blackboard.Blocked = False
            self.blackboard.Blocked_Prevent_close = False
            self.blackboard.Blocked_Prevent_far = False
            self.blackboard.Blocked_Prevent_follow = False
            self.blackboard.Blocked_Prevent_patrol = False
            self.blackboard.Blocked_excuteflag = 0

            self.blackboard.avoidloop_far = 0
            self.blackboard.avoidloop_follow = 0
            self.blackboard.avoidloop_patrol = 0
            print 'r0 dist < 1.58'
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
        self.blackboard.goallastforptl_x = env.followgoal_x
        self.blackboard.goallastforptl_y = env.followgoal_y
        if env.EnemyPoseSave.enemy_dist > 0.8:  # 否则后退一些
            if self.blackboard.sended_close == False:
                self.goal_x = env.MyPose['x']
                self.goal_y = env.MyPose['y']
                if env.EnemyPoseSave.enemy_yaw > 0:  #枪管在左
                    self.goal_yaw = env.MyPose['theta']+env.Gimbal.enemy_yaw-35  #与枪管成30度
                    print '---------------------theta-----------------------------------'
                else:
                    self.goal_yaw = env.MyPose['theta']+env.Gimbal.enemy_yaw+35  #与枪管成30度
                    print '+++++++++++++++++++++++++++++++++++theta+++++++++++++++++++++++++++++++++++'
                if self.goal_yaw > 180:
                    self.goal_yaw = self.goal_yaw - 360
                if self.goal_yaw < -180:
                    self.goal_yaw = self.goal_yaw + 360
                if env.isActionAvaliable(self.goal_x, self.goal_y, self.goal_yaw):  # 指向敌人
                    controller.send_goal(env.navgoal)
                    self.blackboard.sended_close = True
                    self.time1 = rospy.Time.now().secs
                    print 'r0： turn to enemy!!!!!!!!!!1 goal is %s, %s' % (env.EnemyPoseSave.enemy_yaw, self.goal_yaw)
                else:
                    print 'r0: cant turn to enemy %s, %s, %s' % (self.goal_x, self.goal_y, self.goal_yaw)
            if rospy.Time.now().secs - self.time1 > 3.5:
                self.blackboard.sended_close = False
        else:
            if self.blackboard.sended_close == False:
                controller.send_vel(cmdvel_slowback)
                self.blackboard.sended_close = True
                self.time1 = rospy.Time.now().secs
                print 'r0: retreat!!!!!!!!!!!!!!!!%s, %s, %s' % (self.goal_x, self.goal_y, self.goal_yaw)
            if rospy.Time.now().secs - self.time1 > 0.2:
                self.blackboard.sended_close = False
                controller.send_vel(cmdvel_stop)
                controller.send_vel(cmdvel_stop)
                print 'retreat end!!!!!!!!!!!!!!!'
        return TaskStatus.SUCCESS

    def reset(self):
        pass

class IsFar(Task):
    def __init__(self, name, blackboard=None):
        super(IsFar, self).__init__(name)
        self.name = name
        self.blackboard = blackboard

    def run(self):
        if env.EnemyPoseSave.enemy_dist > 1.58 and self.blackboard.Blocked_Prevent_far == False:
            self.blackboard.sended_close = False
            self.blackboard.sended_beated = False
            self.blackboard.sended_patrol = False
            self.blackboard.sended_follow = False
            self.blackboard.sended_blocked = False
            self.blackboard.patrol_flag = 0
            self.blackboard.follow_flag = 0
            self.blackboard.patrol_flag1 = 0
            self.blackboard.patrol_flag2 = 0

            self.blackboard.Blocked = False
            self.blackboard.Blocked_Prevent_close = False
            self.blackboard.Blocked_Prevent_far = False
            self.blackboard.Blocked_Prevent_follow = False
            self.blackboard.Blocked_Prevent_patrol = False
            self.blackboard.Blocked_excuteflag = 0

            self.blackboard.avoidloop_follow = 0
            self.blackboard.avoidloop_patrol = 0
            print 'r0 dist > 1.58'
            return TaskStatus.SUCCESS
        else:
            return TaskStatus.FAILURE

    def reset(self):
        pass

class FarShoot(Task):
    def __init__(self, name, blackboard=None):
        super(FarShoot, self).__init__(name)
        self.name = name
        self.blackboard = blackboard
        self.goal_x = 0
        self.goal_y = 0
        self.goal_yaw = 0
        self.time1 = 0
        self.time2 = 0

    def run(self):
        self.blackboard.goallastforptl_x = env.followgoal_x
        self.blackboard.goallastforptl_y = env.followgoal_y
        if self.blackboard.sended_far == False:
            if env.isActionAvaliable(env.goal_x, env.goal_y, env.goal_yaw):
                self.goal_x = env.goal_x
                self.goal_y = env.goal_y
                self.goal_yaw = env.goal_yaw
            else:
                self.goal_x = env.farchoosegoal_x
                self.goal_y = env.farchoosegoal_y
                self.goal_yaw = env.farchoosegoal_yaw
                print 'farchoose known point!!!!!!!!!!!!!!'
            self.blackboard.BlockedPoseSaveX = env.MyPose['x']
            self.blackboard.BlockedPoseSaveY = env.MyPose['y']
            self.blackboard.BlockedPoseSaveYaw = env.MyPose['theta']
            if env.isActionAvaliable(self.goal_x, self.goal_y, self.goal_yaw):  # 指向敌人
                controller.send_goal(env.navgoal)
                self.time1 = rospy.Time.now().secs
                self.blackboard.sended_far = True
                print 'r0: Far track!!!!!!!!!!!!!! goal is %s,%s,%s ;enemy is in %s, %s ' % (self.goal_x, self.goal_y, self.goal_yaw,env.EnemyPoseSave.enemy_dist,env.EnemyPoseSave.enemy_yaw)
            else:
                self.time1 = rospy.Time.now().secs
                print 'r0: cant track %s, %s, %s' % (self.goal_x, self.goal_y, self.goal_yaw)
        if rospy.Time.now().secs - self.time1 > 0.3:
            self.blackboard.sended_far = False
        return TaskStatus.SUCCESS

    def reset(self):
        pass

class IsBeated(Task):
    def __init__(self, name, blackboard=None):
        super(IsBeated, self).__init__(name)
        self.name = name
        self.blackboard = blackboard

    def run(self):
        if env.witch_armor > 0:
            self.blackboard.sended_close = False
            self.blackboard.sended_far = False
            self.blackboard.sended_patrol = False
            self.blackboard.sended_follow = False
            self.blackboard.sended_blocked = False
            self.blackboard.patrol_flag = 0
            self.blackboard.follow_flag = 0
            self.blackboard.patrol_flag1 = 0
            self.blackboard.patrol_flag2 = 0

            self.blackboard.Blocked = False
            self.blackboard.Blocked_Prevent_close = False
            self.blackboard.Blocked_Prevent_far = False
            self.blackboard.Blocked_Prevent_follow = False
            self.blackboard.Blocked_Prevent_patrol = False
            self.blackboard.Blocked_excuteflag = 0

            self.blackboard.avoidloop_far = 0
            self.blackboard.avoidloop_follow = 0
            self.blackboard.avoidloop_patrol = 0
            print 'r0 beated!!!!!!!!!!!!!!!!!!!!!!!'
            return TaskStatus.SUCCESS
        else:
            return TaskStatus.FAILURE

    def reset(self):
        pass

class BeatedShoot(Task):
    def __init__(self, name, blackboard=None):
        super(BeatedShoot, self).__init__(name)
        self.name = name
        self.blackboard = blackboard
        self.goal_x = 0
        self.goal_y = 0
        self.goal_yaw = 0
        self.time1 = 0

    def run(self):
        if self.blackboard.sended_beated == False:
            self.goal_x = env.MyPose['x']
            self.goal_y = env.MyPose['y']
            if env.witch_armor == 0:  # 主 beated
                if env.EnemyPoseSave_last >= 0:
                    self.goal_yaw = env.MyPose['theta'] + 30
                else:
                    self.goal_yaw = env.MyPose['theta'] - 30
            elif env.witch_armor == 1:  # 左
                self.goal_yaw = env.MyPose['theta'] + 120
            elif env.witch_armor == 2:  # 后
                self.goal_yaw = env.MyPose['theta'] + 178
            elif env.witch_armor == 3:  #右
                self.goal_yaw = env.MyPose['theta'] - 120
            elif env.witch_armor == 4:  #左摄像头
                self.goal_yaw = env.MyPose['theta'] + 130
                print 'camera zuo move!!!!!!!!!!11'
            elif env.witch_armor == 5:  #右摄像头
                self.goal_yaw = env.MyPose['theta'] - 130
                print 'camera you move!!!!!!!!!!'
            else:
                print 'armor detect error!!!!!!!!!!!!'
            # 防止过度
            if self.goal_yaw > 180:
                self.goal_yaw = self.goal_yaw - 360
            elif self.goal_yaw < -180:
                self.goal_yaw = self.goal_yaw + 360
            else:
                pass
            if env.isActionAvaliable(self.goal_x, self.goal_y, self.goal_yaw):  # 指向敌人
                controller.send_goal(env.navgoal)
                self.blackboard.sended_beated = True
                self.time1 = rospy.Time.now().secs
                # print 'r0： beated move!!!!!!!!!!1'
            else:
                print 'r0: cant beated move %s, %s, %s' % (self.goal_x, self.goal_y, self.goal_yaw)
        if rospy.Time.now().secs - self.time1 > 3.5:
            self.blackboard.sended_beated = False
            env.witch_armor = -1
        return TaskStatus.SUCCESS

    def reset(self):
        pass


class IsFollow(Task):
    def __init__(self, name, blackboard=None):
        super(IsFollow, self).__init__(name)
        self.name = name
        self.blackboard = blackboard

    def run(self):
        if env.EnemyPoseSave.enemy_dist == 0 and env.EnemyPoseSave_robot1.enemy_dist > 0 and self.blackboard.Blocked_Prevent_follow == False:
            self.blackboard.sended_far = False
            self.blackboard.sended_close = False
            self.blackboard.sended_beated = False
            self.blackboard.sended_patrol = False
            self.blackboard.sended_blocked = False
            self.blackboard.patrol_flag = 0
            self.blackboard.patrol_flag1 = 0
            self.blackboard.patrol_flag2 = 0

            self.blackboard.Blocked = False
            self.blackboard.Blocked_Prevent_close = False
            self.blackboard.Blocked_Prevent_far = False
            self.blackboard.Blocked_Prevent_follow = False
            self.blackboard.Blocked_Prevent_patrol = False
            self.blackboard.Blocked_excuteflag = 0

            self.blackboard.avoidloop_far = 0
            self.blackboard.avoidloop_patrol = 0
            print 'r0 dist == 0 r1 dist > 1.58'
            return TaskStatus.SUCCESS
        else:
            return TaskStatus.FAILURE

    def reset(self):
        pass


class Follow_Shoot(Task):
    def __init__(self, name, blackboard=None):
        super(Follow_Shoot, self).__init__(name)
        self.name = name
        self.blackboard = blackboard
        self.goal_x = 0
        self.goal_y = 0
        self.goal_yaw = 0
        self.time1 = 0

    def run(self):
        if self.blackboard.follow_flag == 0:
            if self.blackboard.sended_follow == False:
                self.goal_x = env.choosedgoal_x
                self.goal_y = env.choosedgoal_y
                self.goal_yaw = env.choosedgoal_yaw
                self.blackboard.BlockedPoseSaveX = env.MyPose['x']
                self.blackboard.BlockedPoseSaveY = env.MyPose['y']
                self.blackboard.BlockedPoseSaveYaw = env.MyPose['theta']
                if env.isActionAvaliable(self.goal_x, self.goal_y, self.goal_yaw):  # 指向敌人
                    controller.send_goal(env.navgoal)
                    self.time1 = rospy.Time.now().secs
                    print 'r0: is follow !!!!!!!!!!!!!!%s,%s,%s' % (self.goal_x,self.goal_y,self.goal_yaw)
                    self.blackboard.sended_follow = True
                else:
                    self.time1 = rospy.Time.now().secs
                    print 'r0: cant follow %s, %s, %s' % (self.goal_x, self.goal_y, self.goal_yaw)
            if rospy.Time.now().secs - self.time1 > 2:
                # 0.1m不走10度不转
                if np.square(self.blackboard.BlockedPoseSaveX - env.MyPose['x']) + np.square(self.blackboard.BlockedPoseSaveY - env.MyPose['y']) < 0.0225 and np.abs(self.blackboard.BlockedPoseSaveYaw-env.MyPose['theta']) < 10:
                    self.blackboard.avoidloop_follow = self.blackboard.avoidloop_follow + 1
                    print 'follow blocked!!!!!!!,%s' % (self.blackboard.avoidloop_follow)
                    self.blackboard.Blocked_GoalYaw = np.arctan2((self.goal_y - env.MyPose['y']), self.goal_x - env.MyPose['x']) * 180 / 3.1416
                    self.blackboard.Blocked = True
                    self.blackboard.Blocked_Prevent_follow = True
                else:
                    self.blackboard.sended_follow = False
                    if np.square(self.goal_x - env.MyPose['x']) + np.square(self.goal_y - env.MyPose['y']) < 0.01:  #到了
                        self.blackboard.follow_flag = 1
            return TaskStatus.SUCCESS
        elif self.blackboard.follow_flag == 1:
            if self.blackboard.sended_follow == False:
                self.goal_x = env.choosedgoal_x
                self.goal_y = env.choosedgoal_y
                self.goal_yaw = env.choosedgoal_yaw + 30
                if self.goal_yaw > 180:
                    self.goal_yaw = self.goal_yaw - 360
                if env.isActionAvaliable(self.goal_x, self.goal_y, self.goal_yaw):  # 判断目标点是否可行
                    controller.send_goal(env.navgoal)
                    self.time1 = rospy.Time.now().secs
                    self.blackboard.sended_follow = True
                else:
                    print 'r0 cant move %s, %s, %s' % (self.goal_x, self.goal_y, self.goal_yaw)
            if rospy.Time.now().secs - self.time1 > 2.5:
                self.blackboard.sended_follow = False
                self.blackboard.follow_flag = 2
            else:
                return TaskStatus.SUCCESS
        elif self.blackboard.follow_flag == 2:
            if self.blackboard.sended_follow == False:
                self.goal_x = env.choosedgoal_x
                self.goal_y = env.choosedgoal_y
                self.goal_yaw = env.choosedgoal_yaw
                self.goal_yaw = self.goal_yaw - 30
                if self.goal_yaw < -180:
                    self.goal_yaw = self.goal_yaw + 360
                if env.isActionAvaliable(self.goal_x, self.goal_y, self.goal_yaw):  # 判断目标点是否可行
                    controller.send_goal(env.navgoal)
                    self.time1 = rospy.Time.now().secs
                    self.blackboard.sended_follow = True
                else:
                    print 'r0 cant move %s, %s, %s' % (self.goal_x, self.goal_y, self.goal_yaw)
            if rospy.Time.now().secs - self.time1 > 2.5:
                self.blackboard.sended_follow = False
                self.blackboard.follow_flag = 0
            else:
                return TaskStatus.SUCCESS
        else:
            print 'follow error!!!!!!!!!!!!!'

    def reset(self):
        pass


class IsPatrol(Task):
    def __init__(self, name, blackboard=None):
        super(IsPatrol, self).__init__(name)
        self.name = name
        self.blackboard = blackboard

    def run(self):
        if env.EnemyPoseSave.enemy_dist == 0 and env.EnemyPoseSave_robot1.enemy_dist == 0 and self.blackboard.Blocked_Prevent_patrol == False:
            self.blackboard.sended_far = False
            self.blackboard.sended_close = False
            self.blackboard.sended_beated = False
            self.blackboard.sended_follow = False
            self.blackboard.sended_blocked = False
            self.blackboard.follow_flag = 0

            self.blackboard.Blocked = False
            self.blackboard.Blocked_Prevent_close = False
            self.blackboard.Blocked_Prevent_far = False
            self.blackboard.Blocked_Prevent_follow = False
            self.blackboard.Blocked_Prevent_patrol = False
            self.blackboard.Blocked_excuteflag = 0

            self.blackboard.avoidloop_far = 0
            self.blackboard.avoidloop_follow = 0

            print 'r0 dist ==0; r1 dist == 0'
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
        if self.blackboard.patrol_flag1 == 0:  #先自转
            if self.blackboard.goallastforptl_x >= env.MyPose['x'] and self.blackboard.goallastforptl_y >= env.MyPose['y']:
                self.goal_x = env.MyPose['x'] - 0.15
                self.goal_y = env.MyPose['y'] - 0.15
                print 'ptl1 ttttttttttttttttttttttttttttttttttttttt'
            elif self.blackboard.goallastforptl_x >= env.MyPose['x'] and self.blackboard.goallastforptl_y < env.MyPose['y']:
                self.goal_x = env.MyPose['x'] - 0.15
                self.goal_y = env.MyPose['y'] + 0.15
                print 'ptl2 ttttttttttttttttttttttttttttttttttttttttttt'
            elif self.blackboard.goallastforptl_x < env.MyPose['x'] and self.blackboard.goallastforptl_y >= env.MyPose['y']:
                self.goal_x = env.MyPose['x'] + 0.15
                self.goal_y = env.MyPose['y'] - 0.15
                print 'ptl3 ttttttttttttttttttttttttttttttttttttttttttt'
            elif self.blackboard.goallastforptl_x < env.MyPose['x'] and self.blackboard.goallastforptl_y < env.MyPose['y']:
                self.goal_x = env.MyPose['x'] + 0.15
                self.goal_y = env.MyPose['y'] + 0.15
                print 'ptl4 tttttttttttttttttttttttttttttttttttttttt'
            else:
                print 'ptl1 error!!!!!!!!!!!!!!!!'
            if env.isActionAvaliable(self.goal_x, self.goal_y, 0):  # front1
                self.goal_x = env.MyPose['x']
                self.goal_y = env.MyPose['y']
            else:
                self.goal_x = env.MyPose['x']
                self.goal_y = env.MyPose['y']
            if env.EnemyPoseSave_last >= 0:  #左转
                if self.blackboard.patrol_flag2 == 0:  # 第一次转60度
                    self.goal_yaw = env.MyPose['theta'] + 135  #一次转90度；4次
                elif self.blackboard.patrol_flag2 == 1:  # 第2次转90度
                    self.goal_yaw = env.MyPose['theta'] + 135  # 一次转90度；4次
                elif self.blackboard.patrol_flag2 == 2:  # 第3次转90度
                    self.goal_yaw = env.MyPose['theta'] + 135  #一次转90度；4次
                else:
                    print 'zixuan error!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!'
            else:  #右转
                if self.blackboard.patrol_flag2 == 0:  # 第一次转60度
                    self.goal_yaw = env.MyPose['theta'] - 135  #一次转90度；4次
                elif self.blackboard.patrol_flag2 == 1:  # 第2次转90度
                    self.goal_yaw = env.MyPose['theta'] - 135  # 一次转90度；4次
                elif self.blackboard.patrol_flag2 == 2:  # 第3次转90度
                    self.goal_yaw = env.MyPose['theta'] - 135  #一次转90度；4次
                else:
                    print 'zixuan error!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!'
            if self.goal_yaw > 180:
                self.goal_yaw = self.goal_yaw - 360
            if self.goal_yaw < -180:
                self.goal_yaw = self.goal_yaw + 360
            if self.blackboard.sended_patrol == False:
                if env.isActionAvaliable(self.goal_x, self.goal_y, self.goal_yaw):  # front1
                    controller.send_goal(env.navgoal)
                    print 'r0 is patrol_spin; goal is %s, %s, %s' % (self.goal_x, self.goal_y, self.goal_yaw)
                    self.time1 = rospy.Time.now().secs
                    self.blackboard.sended_patrol = True
            else:
                if rospy.Time.now().secs - self.time1 > 2.5 or np.abs(env.MyPose['theta'] - self.goal_yaw) < 10:
                    self.blackboard.sended_patrol = False
                    self.blackboard.patrol_flag2 = self.blackboard.patrol_flag2 + 1
                    if self.blackboard.patrol_flag2 == 3:
                        self.blackboard.patrol_flag2 = 0
                        self.blackboard.patrol_flag1 = 1  # 旋转结束
                        print 'zixuan end!!!!!!!!!!!!!!!!!!!!!!!!'
        else:  #ptl
            if self.blackboard.patrol_flag == 0:  # 第一点：移动
                if self.blackboard.sended_patrol == False:
                    if env.MyHP_robot1 > 0:  #另一台车是活的
                        self.goal_x = 6
                        self.goal_y = 1.8
                        self.goal_yaw = 155
                    else:  #另车是死的
                        if self.blackboard.leftarea == False:  #走you边
                            self.goal_x = 6
                            self.goal_y = 1.8
                            self.goal_yaw = 155
                        else:
                            self.goal_x = 2
                            self.goal_y = 3.2
                            self.goal_yaw = -35
                    self.blackboard.BlockedPoseSaveX = env.MyPose['x']
                    self.blackboard.BlockedPoseSaveY = env.MyPose['y']
                    self.blackboard.BlockedPoseSaveYaw = env.MyPose['theta']
                    if env.isActionAvaliable(self.goal_x, self.goal_y, self.goal_yaw):  # front1
                        controller.send_goal(env.navgoal)
                        print 'r0 is patrol_front; goal is %s, %s, %s' % (self.goal_x, self.goal_y, self.goal_yaw)
                        self.time1 = rospy.Time.now().secs
                        self.blackboard.sended_patrol = True
                else:
                    if rospy.Time.now().secs - self.time1 > 2.5:
                        if np.square(self.blackboard.BlockedPoseSaveX - env.MyPose['x']) + np.square(self.blackboard.BlockedPoseSaveY - env.MyPose['y']) < 0.0225 and np.abs(self.blackboard.BlockedPoseSaveYaw - env.MyPose['theta']) < 10:
                            self.blackboard.avoidloop_patrol = self.blackboard.avoidloop_patrol + 1
                            print 'ptlfront blocked!!!!!!!  %s' % (self.blackboard.avoidloop_patrol)
                            self.blackboard.Blocked_GoalYaw = np.arctan2((self.goal_y - env.MyPose['y']),self.goal_x - env.MyPose['x']) * 180 / 3.1416
                            self.blackboard.Blocked = True
                            self.blackboard.Blocked_Prevent_patrol = True
                        else:
                            self.blackboard.sended_patrol = False
                            if np.square(self.goal_x - env.MyPose['x']) + np.square(self.goal_y - env.MyPose['y']) < 0.0169:  # 到了
                                self.blackboard.patrol_flag = 1
            elif self.blackboard.patrol_flag == 1:  #转
                if self.blackboard.sended_patrol == False:
                    if env.MyHP_robot1 > 0:
                        self.goal_x = 6.2
                        self.goal_y = 1.8
                        self.goal_yaw = -145
                    else:  #另车是死的
                        if self.blackboard.leftarea == False:  #走you边
                            self.goal_x = 6.2
                            self.goal_y = 1.8
                            self.goal_yaw = -145
                        else:
                            self.goal_x = 1.8
                            self.goal_y = 3.2
                            self.goal_yaw = 35
                    if env.isActionAvaliable(self.goal_x, self.goal_y, self.goal_yaw):  # front1
                        controller.send_goal(env.navgoal)
                        print 'r0 is patrol_front1; goal is %s, %s, %s' % (self.goal_x, self.goal_y, self.goal_yaw)
                        self.time1 = rospy.Time.now().secs
                        self.blackboard.sended_patrol = True
                else:
                    if rospy.Time.now().secs - self.time1 > 3:
                        self.blackboard.sended_patrol = False
                        self.blackboard.patrol_flag = 2
            elif self.blackboard.patrol_flag == 2:  #转
                if self.blackboard.sended_patrol == False:
                    if env.MyHP_robot1 > 0:
                        self.goal_x = 6.1
                        self.goal_y = 1.8
                        self.goal_yaw = 0
                    else:  #另车是死的
                        if self.blackboard.leftarea == False:  #走you边
                            self.goal_x = 6.1
                            self.goal_y = 1.8
                            self.goal_yaw = 0
                        else:
                            self.goal_x = 1.9
                            self.goal_y = 3.2
                            self.goal_yaw = 178
                    if env.isActionAvaliable(self.goal_x, self.goal_y, self.goal_yaw):  # front1
                        controller.send_goal(env.navgoal)
                        print 'r0 is patrol_front2; goal is %s, %s, %s' % (self.goal_x, self.goal_y, self.goal_yaw)
                        self.time1 = rospy.Time.now().secs
                        self.blackboard.sended_patrol = True
                else:
                    if rospy.Time.now().secs - self.time1 > 3:
                        self.blackboard.sended_patrol = False
                        self.blackboard.patrol_flag = 3
            elif self.blackboard.patrol_flag == 3:  #移动
                if self.blackboard.sended_patrol == False:
                    if env.MyHP_robot1 > 0:
                        self.goal_x = 7.1
                        self.goal_y = 1.9
                        self.goal_yaw = -75
                    else:  #另车是死的
                        if self.blackboard.leftarea == False:  #走you边
                            self.goal_x = 7.1
                            self.goal_y = 1.9
                            self.goal_yaw = -75
                        else:
                            self.goal_x = 0.9
                            self.goal_y = 3.1
                            self.goal_yaw = 105
                    self.blackboard.BlockedPoseSaveX = env.MyPose['x']
                    self.blackboard.BlockedPoseSaveY = env.MyPose['y']
                    self.blackboard.BlockedPoseSaveYaw = env.MyPose['theta']
                    if env.isActionAvaliable(self.goal_x, self.goal_y, self.goal_yaw):  # front1
                        controller.send_goal(env.navgoal)
                        print 'r0 is patrol_back1; goal is %s, %s, %s' % (self.goal_x, self.goal_y, self.goal_yaw)
                        self.time1 = rospy.Time.now().secs
                        self.blackboard.sended_patrol = True
                else:
                    if rospy.Time.now().secs - self.time1 > 2:
                        if np.square(self.blackboard.BlockedPoseSaveX - env.MyPose['x']) + np.square(self.blackboard.BlockedPoseSaveY - env.MyPose['y']) < 0.0225 and np.abs(self.blackboard.BlockedPoseSaveYaw - env.MyPose['theta']) < 10:
                            self.blackboard.avoidloop_patrol = self.blackboard.avoidloop_patrol + 1
                            print 'ptlback1 blocked!!!!!!!  %s' % (self.blackboard.avoidloop_patrol)
                            self.blackboard.Blocked_GoalYaw = np.arctan2((self.goal_y - env.MyPose['y']),self.goal_x - env.MyPose['x']) * 180 / 3.1416
                            self.blackboard.Blocked = True
                            self.blackboard.Blocked_Prevent_patrol = True
                        else:
                            self.blackboard.sended_patrol = False
                            if np.square(self.goal_x - env.MyPose['x']) + np.square(self.goal_y - env.MyPose['y']) < 0.0225:  # 到了
                                self.blackboard.patrol_flag = 4


            elif self.blackboard.patrol_flag == 4:  #移动
                if self.blackboard.sended_patrol == False:
                    if env.MyHP_robot1 > 0:
                        self.goal_x = 6.65
                        self.goal_y = 3
                        self.goal_yaw = 70
                    else:  # 另车是死的
                        if self.blackboard.leftarea == False:  # 走you边
                            self.goal_x = 6.65
                            self.goal_y = 3
                            self.goal_yaw = 70
                        else:
                            self.goal_x = 1.35
                            self.goal_y = 2
                            self.goal_yaw = -110
                    self.blackboard.BlockedPoseSaveX = env.MyPose['x']
                    self.blackboard.BlockedPoseSaveY = env.MyPose['y']
                    self.blackboard.BlockedPoseSaveYaw = env.MyPose['theta']
                    if env.isActionAvaliable(self.goal_x, self.goal_y, self.goal_yaw):  # front1
                        controller.send_goal(env.navgoal)
                        print 'r0 is patrol_back2; goal is %s, %s, %s' % (
                        self.goal_x, self.goal_y, self.goal_yaw)
                        self.time1 = rospy.Time.now().secs
                        self.blackboard.sended_patrol = True
                else:
                    if rospy.Time.now().secs - self.time1 > 2.8:
                        if np.square(self.blackboard.BlockedPoseSaveX - env.MyPose['x']) + np.square(self.blackboard.BlockedPoseSaveY - env.MyPose['y']) < 0.0225 and np.abs(self.blackboard.BlockedPoseSaveYaw - env.MyPose['theta']) < 10:
                            self.blackboard.avoidloop_patrol = self.blackboard.avoidloop_patrol + 1
                            print 'ptlback2 blocked!!!!!!!  %s' % (self.blackboard.avoidloop_patrol)
                            self.blackboard.Blocked_GoalYaw = np.arctan2((self.goal_y - env.MyPose['y']),self.goal_x - env.MyPose['x']) * 180 / 3.1416
                            self.blackboard.Blocked = True
                            self.blackboard.Blocked_Prevent_patrol = True
                        else:
                            self.blackboard.sended_patrol = False
                            if np.square(self.goal_x - env.MyPose['x']) + np.square(self.goal_y - env.MyPose['y']) < 0.0225:  # 到了
                                self.blackboard.patrol_flag = 5
            elif self.blackboard.patrol_flag == 5:  #移动
                if self.blackboard.sended_patrol == False:
                    if env.MyHP_robot1 > 0:
                        self.goal_x = 5.8
                        self.goal_y = 1.8
                        self.goal_yaw = 150
                    else:  # 另车是死的
                        if self.blackboard.leftarea == False:  # 走you边
                            self.goal_x = 5.8
                            self.goal_y = 1.8
                            self.goal_yaw = 150
                        else:
                            self.goal_x = 2.2
                            self.goal_y = 3.2
                            self.goal_yaw = -30
                    self.blackboard.BlockedPoseSaveX = env.MyPose['x']
                    self.blackboard.BlockedPoseSaveY = env.MyPose['y']
                    self.blackboard.BlockedPoseSaveYaw = env.MyPose['theta']
                    if env.isActionAvaliable(self.goal_x, self.goal_y, self.goal_yaw):  # front1
                        controller.send_goal(env.navgoal)
                        print 'r0 is patrol_behind; goal is %s, %s, %s' % (self.goal_x, self.goal_y, self.goal_yaw)
                        self.time1 = rospy.Time.now().secs
                        self.blackboard.sended_patrol = True
                else:
                    if rospy.Time.now().secs - self.time1 > 2:
                        if np.square(self.blackboard.BlockedPoseSaveX - env.MyPose['x']) + np.square(self.blackboard.BlockedPoseSaveY - env.MyPose['y']) < 0.0225 and np.abs(self.blackboard.BlockedPoseSaveYaw - env.MyPose['theta']) < 10:
                            self.blackboard.avoidloop_patrol = self.blackboard.avoidloop_patrol + 1
                            print 'ptlback blocked!!!!!!!  %s' % (self.blackboard.avoidloop_patrol)
                            self.blackboard.Blocked_GoalYaw = np.arctan2((self.goal_y - env.MyPose['y']),self.goal_x - env.MyPose['x']) * 180 / 3.1416
                            self.blackboard.Blocked = True
                            self.blackboard.Blocked_Prevent_patrol = True
                        else:
                            self.blackboard.sended_patrol = False
                            if np.square(self.goal_x - env.MyPose['x']) + np.square(self.goal_y - env.MyPose['y']) < 0.0225:  # 到了
                                self.blackboard.patrol_flag = 0
                                self.blackboard.leftarea = ~self.blackboard.leftarea
            else:
                print 'patrol error!!!'
        return TaskStatus.SUCCESS

    def reset(self):
        pass

class IsBlocked(Task):
    def __init__(self, name, blackboard=None):
        super(IsBlocked, self).__init__(name)
        self.name = name
        self.blackboard = blackboard

    def run(self):
        if self.blackboard.Blocked == True:
            print 'r0 blocked................'
            self.blackboard.sended_close = False
            self.blackboard.sended_far = False
            self.blackboard.sended_beated = False
            self.blackboard.sended_patrol = False
            self.blackboard.sended_follow = False
            return TaskStatus.SUCCESS
        else:
            return TaskStatus.FAILURE

    def reset(self):
        pass

class Blocked_Shoot(Task):
    def __init__(self, name, blackboard=None):
        super(Blocked_Shoot, self).__init__(name)
        self.name = name
        self.blackboard = blackboard
        self.goal_x = 0
        self.goal_y = 0
        self.goal_yaw = 0
        self.time1 = 0
        self.distfromeme = []
        self.abstheta = []  # 及角度差绝对值

    def run(self):  #只有进入其他三种情况或执行完这个才可以跳出
        if self.blackboard.Blocked_excuteflag == 0:  #转正
            if self.blackboard.sended_blocked == False:
                self.goal_x = env.MyPose['x']
                self.goal_y = env.MyPose['y']
                for k in range(0, len(self.abstheta)):
                    del self.abstheta[0]
                for k in range(0, len(env.blockedthetas)):
                    self.abstheta.append(abs(self.blackboard.Blocked_GoalYaw-env.blockedthetas[k]))
                    print env.blockedthetas
                    print self.abstheta
                self.goal_yaw = env.blockedthetas[self.abstheta.index(min(self.abstheta))]
                if np.abs(env.MyPose['theta']-(self.goal_yaw+30)) < np.abs(env.MyPose['theta']-(self.goal_yaw-30)):  #右转
                    self.goal_yaw = self.goal_yaw + 30
                    cmdvel_blockedpush.linear.x = 0.205*np.cos(30*3.1416/180)
                    cmdvel_blockedpush.linear.y = -0.205*np.sin(30*3.1416/180)
                    cmdvel_blockedpush.linear.z = 0
                    cmdvel_blockedpush.angular.x = 0
                    cmdvel_blockedpush.angular.y = 0
                    cmdvel_blockedpush.angular.z = 0
                    print 'qiantui zuo %s %s' % (cmdvel_blockedpush.linear.x,cmdvel_blockedpush.linear.y)

                    cmdvel_blockedretreat.linear.x = -0.205 * np.cos(30*3.1416/180)
                    cmdvel_blockedretreat.linear.y = 0.205 * np.sin(30*3.1416/180)
                    cmdvel_blockedretreat.linear.z = 0
                    cmdvel_blockedretreat.angular.x = 0
                    cmdvel_blockedretreat.angular.y = 0
                    cmdvel_blockedretreat.angular.z = 0
                else:
                    self.goal_yaw = self.goal_yaw - 30
                    cmdvel_blockedpush.linear.x = 0.205 * np.cos(30 * 3.1416 / 180)
                    cmdvel_blockedpush.linear.y = 0.205 * np.sin(30 * 3.1416 / 180)
                    cmdvel_blockedpush.linear.z = 0
                    cmdvel_blockedpush.angular.x = 0
                    cmdvel_blockedpush.angular.y = 0
                    cmdvel_blockedpush.angular.z = 0
                    print 'qiantui you %s %s' % (cmdvel_blockedpush.linear.x, cmdvel_blockedpush.linear.y)

                    cmdvel_blockedretreat.linear.x = -0.205 * np.cos(30 * 3.1416 / 180)
                    cmdvel_blockedretreat.linear.y = -0.205 * np.sin(30 * 3.1416 / 180)
                    cmdvel_blockedretreat.linear.z = 0
                    cmdvel_blockedretreat.angular.x = 0
                    cmdvel_blockedretreat.angular.y = 0
                    cmdvel_blockedretreat.angular.z = 0
                if env.isActionAvaliable(self.goal_x, self.goal_y, self.goal_yaw):  # 指向敌人
                    controller.send_goal(env.navgoal)
                    self.time1 = rospy.Time.now().secs
                    self.blackboard.sended_blocked = True
            if (rospy.Time.now().secs - self.time1 > 5) or (np.abs(env.MyPose['theta'] - self.goal_yaw) < 5):  # 可以下一步啦
                print 'turn front Done!!!!!!!!!!! %s, %s, %s' % (self.goal_x, self.goal_y, self.goal_yaw)
                self.blackboard.sended_blocked = False
                self.blackboard.Blocked_excuteflag = 1
                self.time1 = rospy.Time.now().secs
        elif self.blackboard.Blocked_excuteflag == 1:  # 钱推
            if rospy.Time.now().secs - self.time1 < 3:
                controller.send_vel(cmdvel_blockedpush)
            else:
                controller.send_vel(cmdvel_stop)
                controller.send_vel(cmdvel_stop)
                print 'qiantui Done!!!!!!!'
                if np.square(env.MyPose['x']-self.blackboard.BlockedPoseSaveX)+np.square(env.MyPose['y']-self.blackboard.BlockedPoseSaveY) > 0.04 and self.blackboard.avoidloop_far < 3 and self.blackboard.avoidloop_follow < 3 and self.blackboard.avoidloop_patrol < 5:  #前推成功
                    self.blackboard.Blocked_excuteflag = 3  #跳过拯救点执行完毕
                    self.time1 = rospy.Time.now().secs
                else:
                    self.blackboard.Blocked_excuteflag = 2  #进入拯救点(先后退)
                    self.time1= rospy.Time.now().secs
        elif self.blackboard.Blocked_excuteflag == 2:  # 后退
            if rospy.Time.now().secs - self.time1 < 2:
                controller.send_vel(cmdvel_blockedretreat)
            else:
                controller.send_vel(cmdvel_stop)
                controller.send_vel(cmdvel_stop)
                print 'houtui_Failture !!!!!!!'
                self.blackboard.Blocked_excuteflag = 4  #进入预拯救点
                self.time1= rospy.Time.now().secs
        elif self.blackboard.Blocked_excuteflag == 3:  # 后退
            if rospy.Time.now().secs - self.time1 < 2:
                controller.send_vel(cmdvel_blockedretreat)
            else:
                print 'houtui_Success !!!!!!!'
                self.blackboard.Blocked_excuteflag = 50  #跳出
                controller.send_vel(cmdvel_stop)
                controller.send_vel(cmdvel_stop)
        elif self.blackboard.Blocked_excuteflag == 4:  # 预拯救点
            if self.blackboard.sended_blocked == False:
                for k in range(0,len(self.distfromeme)):
                    del self.distfromeme[0]
                for k in enumerate(self.blackboard.PreRescuePoint):
                    self.distfromeme.append(np.square(env.MyPose['x']-k[1][0])+np.square(env.MyPose['y']-k[1][1]))
                # print self.distfromeme
                # self.blackboard.PreRescuePoint[self.distfromeme.index(min(self.distfromeme))]
                self.goal_x = self.blackboard.PreRescuePoint[self.distfromeme.index(min(self.distfromeme))][0]
                self.goal_y = self.blackboard.PreRescuePoint[self.distfromeme.index(min(self.distfromeme))][1]
                self.goal_yaw = self.blackboard.PreRescuePoint[self.distfromeme.index(min(self.distfromeme))][2]
                print 'Pre zhengjiudian is in %s %s %s' % (self.goal_x, self.goal_y, self.goal_yaw)
                if env.isActionAvaliable(self.goal_x, self.goal_y, self.goal_yaw):  # 指向敌人
                    controller.send_goal(env.navgoal)
                    self.blackboard.sended_blocked = True
                    self.time1 = rospy.Time.now().secs
            if (rospy.Time.now().secs - self.time1 > 4) or (np.square(env.MyPose['x'] - self.goal_x)+np.square(env.MyPose['y']-self.goal_y) < 0.09):  #执行完啦
                print ' Pre zhengjiu Done!!!!!!!'
                self.blackboard.sended_blocked = False
                self.blackboard.Blocked_excuteflag = 5
        elif self.blackboard.Blocked_excuteflag == 5:  # 拯救点
            if self.blackboard.sended_blocked == False:
                for k in range(0,len(self.distfromeme)):
                    del self.distfromeme[0]
                for k in enumerate(self.blackboard.RescuePoint):
                    self.distfromeme.append(np.square(env.MyPose['x']-k[1][0])+np.square(env.MyPose['y']-k[1][1]))
                # print self.distfromeme
                # self.blackboard.RescuePoint[self.distfromeme.index(min(self.distfromeme))]
                self.goal_x = self.blackboard.RescuePoint[self.distfromeme.index(min(self.distfromeme))][0]
                self.goal_y = self.blackboard.RescuePoint[self.distfromeme.index(min(self.distfromeme))][1]
                self.goal_yaw = self.blackboard.RescuePoint[self.distfromeme.index(min(self.distfromeme))][2]
                print 'zhengjiudian is in %s %s %s' % (self.goal_x, self.goal_y, self.goal_yaw)
                if env.isActionAvaliable(self.goal_x, self.goal_y, self.goal_yaw):  # 指向敌人
                    controller.send_goal(env.navgoal)
                    self.blackboard.sended_blocked = True
                    self.time1 = rospy.Time.now().secs
            if (rospy.Time.now().secs - self.time1 > 4) or (np.square(env.MyPose['x'] - self.goal_x)+np.square(env.MyPose['y']-self.goal_y) < 0.09):  #执行完啦
                print 'zhengjiu Done!!!!!!!'
                self.blackboard.avoidloop_far = 0
                self.blackboard.avoidloop_follow = 0
                self.blackboard.avoidloop_patrol = 0
                self.blackboard.Blocked_excuteflag = 50
        else:  #执行完
            self.blackboard.Blocked_Prevent_close = False
            self.blackboard.Blocked_Prevent_far = False
            self.blackboard.Blocked_Prevent_follow = False
            self.blackboard.Blocked_Prevent_patrol = False
            self.blackboard.Blocked = False
            print 'End clear!!!!!!!!!!!!!!!!!!'
        return TaskStatus.SUCCESS

    def reset(self):
        pass


if __name__ == '__main__':
    rospy.init_node('RM')
    # 启动控制器和环境类
    controller = Controller()
    env = BattleEnv()
    tflistener = tf.TransformListener()

    # 标志位
    time1 = 0  # 记录进入buff后的时间
    time2 = 0  # 记录执行时间
    gettedbuff = False

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
    # rospy.Subscriber('yolo_enemy', YoloEnemy, env.getYoloEnemyCallback)

    rospy.Subscriber('communite', Communite, env.getCommunite)


    while 1:
        if env.beentreeflag == 1:
            tree = BiuldTree()  # 已经加buff进入tree
        else:
            print 'wait start'
            print env.beentreeflag
        rospy.sleep(0.01)
