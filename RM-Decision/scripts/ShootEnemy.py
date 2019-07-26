#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
from Battle import BattleEnv
from teleop_control import Controller
from teleop_controller.msg import EnemyPos, ShootCmd, ModeSW
from pi_trees_ros.pi_trees_ros import *

controller = Controller()
env = BattleEnv()
shoot_cmd = ShootCmd()
if __name__ == '__main__':
    rospy.init_node('ShootEnemy')
    rospy.Subscriber('enemy_pos', EnemyPos, env.getEnemyPoseCallback)
    rospy.Subscriber('robot_1/enemy_pos', EnemyPos, env.getEnemyPoseCallback_robot1)
    # 开两机器人摩擦轮
    shoot_cmd.fric_wheel_spd = 1225
    shoot_cmd.fric_wheel_run = 1
    shoot_cmd.c_shoot_cmd = 0
    shoot_cmd.shoot_cmd = 0
    controller.shoot(shoot_cmd)
    controller.shoot_robot1(shoot_cmd)
    print 'start'

    while 1:
        if env.EnemyPoseSave.enemy_dist > 2.2:
            env.shooting(shoot_cmd, controller)
            print 'shoot================================='
        elif env.EnemyPoseSave.enemy_dist > 0 and env.EnemyPoseSave.enemy_dist <= 2.2:
            env.shooting_plus(shoot_cmd, controller)
            print 'shoot++++++++++++++++++++++++++++++++'
        else:
            env.shooting_stop(shoot_cmd, controller)
            env.shooting_stop(shoot_cmd, controller)
            print 'stop!!!!!!!!!!!!!!!!!!!!!!!!!!!!!1!!'
        # print 'enemy_dist: is %s' % (env.EnemyPoseSave.enemy_dist)

        if env.EnemyPoseSave_robot1.enemy_dist > 2.2:
            env.shooting_robot1(shoot_cmd, controller)
            print 'shoot_robot1================================='
        elif env.EnemyPoseSave_robot1.enemy_dist > 0 and env.EnemyPoseSave_robot1.enemy_dist <= 2.2:
            env.shooting_plus_robot1(shoot_cmd, controller)
            print 'shoot_robot1++++++++++++++++++++++++++++++++'
        else:
            env.shooting_stop_robot1(shoot_cmd, controller)
            env.shooting_stop_robot1(shoot_cmd, controller)
            # print 'stop_robot1!!!!!!!!!!!!!!!!!!!!!!!!!!!!!1!!'
        # print 'enemy_dist_robot1: is %s' % (env.EnemyPoseSave_robot1.enemy_dist)
        rospy.sleep(0.15)

