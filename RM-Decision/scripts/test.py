#!/usr/bin/env python
# -*- coding: utf-8 -*-
import math
import numpy as np
import tf

acc = [(4,2.5,-5),(3,2.5,5)]
dist = []

if __name__ == '__main__':
    # acc.append(2.5)
    # acc.append(2.6)
    # acc.append(2.7)
    # for k in range(0,len(acc)):
    #     del acc[0]
    print np.arctan2(2,3)*180/3.1416
    print np.cos(60 * 3.1416 / 180)
    # for k in enumerate(acc):
    #     print k
    # print min(dist)
    # print dist.index(min(dist))
    # print len(acc)
    # print acc[1][0]
    # for i in range(0, 3):
    #     dist.append(i)
    # print dist




# def Pose_Analysis(self, goal_x, goal_y, goal_yaw):
    #     if goal_x >= 0 and goal_x <= 1 and goal_y >= 0 and goal_y <=1:
    #     elif goal_x >1 and goal_x <= 2 and goal_y >= 0 and goal_y <=1:
    #         self.enemyblock_num = 2
    #         self.enemyblock_attacknum.append(1)
    #         self.enemyblock_attacknum.append(9)
    #         self.enemyblock_attacknum.append(10)
    #         self.enemyblock_attacknum.append(11)
    #         self.enemyblock_attacknum.append(3)
    #     elif goal_x > 2 and goal_x <= 3 and goal_y >= 0 and goal_y <=1:
    #         self.enemyblock_num = 3
    #         self.enemyblock_attacknum.append(2)
    #         self.enemyblock_attacknum.append(10)
    #         self.enemyblock_attacknum.append(11)
    #     elif goal_x > 3 and goal_x <= 4 and goal_y >= 0 and goal_y <=1:
    #         self.enemyblock_num = 4
    #         self.enemyblock_attacknum.append(12)
    #         self.enemyblock_attacknum.append(13)
    #         self.enemyblock_attacknum.append(5)
    #     elif goal_x > 4 and goal_x <= 5 and goal_y >= 0 and goal_y <=1:
    #         self.enemyblock_num = 5
    #         self.enemyblock_attacknum.append(4)
    #         self.enemyblock_attacknum.append(12)
    #         self.enemyblock_attacknum.append(13)
    #         self.enemyblock_attacknum.append(14)
    #         self.enemyblock_attacknum.append(6)
    #     elif goal_x > 5 and goal_x <= 6 and goal_y >= 0 and goal_y <=1:
    #         self.enemyblock_num = 6
    #         self.enemyblock_attacknum.append(5)
    #         self.enemyblock_attacknum.append(13)
    #         self.enemyblock_attacknum.append(14)
    #         self.enemyblock_attacknum.append(7)
    #     elif goal_x > 6 and goal_x <= 7 and goal_y >= 0 and goal_y <= 1:
    #         self.enemyblock_num = 7
    #         self.enemyblock_attacknum.append(6)
    #         self.enemyblock_attacknum.append(14)
    #         self.enemyblock_attacknum.append(16)
    #         self.enemyblock_attacknum.append(8)
    #     elif goal_x > 7 and goal_x <= 8 and goal_y >= 0 and goal_y <=1:
    #         self.enemyblock_num = 8
    #         self.enemyblock_attacknum.append(7)
    #         self.enemyblock_attacknum.append(16)
    #
    #     elif goal_x >= 0 and goal_x <= 1 and goal_y > 1 and goal_y <=2:
    #         self.enemyblock_num = 9
    #         self.enemyblock_attacknum.append(18)
    #         self.enemyblock_attacknum.append(10)
    #         self.enemyblock_attacknum.append(2)
    #         self.enemyblock_attacknum.append(1)
    #     elif goal_x > 1 and goal_x <= 2 and goal_y > 1 and goal_y <=2:
    #         self.enemyblock_num = 10
    #         self.enemyblock_attacknum.append(0)
    #         self.enemyblock_attacknum.append(0)
    #     elif goal_x > 2 and goal_x <= 3 and goal_y > 1 and goal_y <=2:
    #         self.enemyblock_num = 11
    #         self.enemyblock_attacknum.append(0)
    #         self.enemyblock_attacknum.append(0)
    #     elif goal_x > 3 and goal_x <= 4 and goal_y > 1 and goal_y <=2:
    #         self.enemyblock_num = 12
    #         self.enemyblock_attacknum.append(0)
    #         self.enemyblock_attacknum.append(0)
    #     elif goal_x > 4 and goal_x <= 5 and goal_y > 1 and goal_y <=2:
    #         self.enemyblock_num = 13
    #         self.enemyblock_attacknum.append(0)
    #         self.enemyblock_attacknum.append(0)
    #     elif goal_x > 5 and goal_x <= 6 and goal_y > 1 and goal_y <=2:
    #         self.enemyblock_num = 14
    #         self.enemyblock_attacknum.append(0)
    #         self.enemyblock_attacknum.append(0)
    #     elif goal_x > 6 and goal_x <= 7 and goal_y > 1 and goal_y <=2:
    #         self.enemyblock_num = 15
    #         self.enemyblock_attacknum.append(0)
    #         self.enemyblock_attacknum.append(0)
    #     elif goal_x > 7 and goal_x <= 8 and goal_y > 1 and goal_y <=2:
    #         self.enemyblock_num = 16
    #         self.enemyblock_attacknum.append(0)
    #         self.enemyblock_attacknum.append(0)
    #
    #     elif goal_x >= 0 and goal_x <= 1 and goal_y > 2 and goal_y <=3:
    #         self.enemyblock_num = 17
    #         self.enemyblock_attacknum.append(0)
    #         self.enemyblock_attacknum.append(0)
    #     elif goal_x > 1 and goal_x <= 2 and goal_y > 2 and goal_y <=3:
    #         self.enemyblock_num = 18
    #         self.enemyblock_attacknum.append(0)
    #         self.enemyblock_attacknum.append(0)
    #     elif goal_x > 2 and goal_x <= 3 and goal_y > 2 and goal_y <=3:
    #         self.enemyblock_num = 19
    #         self.enemyblock_attacknum.append(0)
    #         self.enemyblock_attacknum.append(0)
    #     elif goal_x > 3 and goal_x <= 4 and goal_y > 2 and goal_y <=3:
    #         self.enemyblock_num = 20
    #         self.enemyblock_attacknum.append(0)
    #         self.enemyblock_attacknum.append(0)
    #     elif goal_x > 4 and goal_x <= 5 and goal_y > 2 and goal_y <=3:
    #         self.enemyblock_num = 21
    #         self.enemyblock_attacknum.append(0)
    #         self.enemyblock_attacknum.append(0)
    #     elif goal_x > 5 and goal_x <= 6 and goal_y > 2 and goal_y <=3:
    #         self.enemyblock_num = 22
    #         self.enemyblock_attacknum.append(0)
    #         self.enemyblock_attacknum.append(0)
    #     elif goal_x > 6 and goal_x <= 7 and goal_y > 2 and goal_y <=3:
    #         self.enemyblock_num = 23
    #         self.enemyblock_attacknum.append(0)
    #         self.enemyblock_attacknum.append(0)
    #     elif goal_x > 7 and goal_x <= 8 and goal_y > 2 and goal_y <=3:
    #         self.enemyblock_num = 24
    #         self.enemyblock_attacknum.append(0)
    #         self.enemyblock_attacknum.append(0)
    #
    #     elif goal_x >= 0 and goal_x <= 1 and goal_y > 3 and goal_y <=4:
    #         self.enemyblock_num = 25
    #         self.enemyblock_attacknum.append(0)
    #         self.enemyblock_attacknum.append(0)
    #     elif goal_x > 1 and goal_x <= 2 and goal_y > 3 and goal_y <= 4:
    #         self.enemyblock_num = 26
    #         self.enemyblock_attacknum.append(0)
    #         self.enemyblock_attacknum.append(0)
    #     elif goal_x > 2 and goal_x <= 3 and goal_y > 3 and goal_y <= 4:
    #         self.enemyblock_num = 27
    #         self.enemyblock_attacknum.append(0)
    #         self.enemyblock_attacknum.append(0)
    #     elif goal_x > 3 and goal_x <= 4 and goal_y > 3 and goal_y <= 4:
    #         self.enemyblock_num = 28
    #         self.enemyblock_attacknum.append(0)
    #         self.enemyblock_attacknum.append(0)
    #     elif goal_x > 4 and goal_x <= 5 and goal_y > 3 and goal_y <= 4:
    #         self.enemyblock_num = 29
    #         self.enemyblock_attacknum.append(0)
    #         self.enemyblock_attacknum.append(0)
    #     elif goal_x > 5 and goal_x <= 6 and goal_y > 3 and goal_y <= 4:
    #         self.enemyblock_num = 30
    #         self.enemyblock_attacknum.append(0)
    #         self.enemyblock_attacknum.append(0)
    #     elif goal_x > 6 and goal_x <= 7 and goal_y > 3 and goal_y <= 4:
    #         self.enemyblock_num = 31
    #         self.enemyblock_attacknum.append(0)
    #         self.enemyblock_attacknum.append(0)
    #     elif goal_x > 7 and goal_x <= 8 and goal_y > 3 and goal_y <= 4:
    #         self.enemyblock_num = 32
    #         self.enemyblock_attacknum.append(0)
    #         self.enemyblock_attacknum.append(0)
    #
    #     elif goal_x >= 0 and goal_x <= 1 and goal_y > 4 and goal_y <= 5:
    #         self.enemyblock_num = 33
    #         self.enemyblock_attacknum.append(0)
    #         self.enemyblock_attacknum.append(0)
    #     elif goal_x > 1 and goal_x <= 2 and goal_y > 4 and goal_y <= 5:
    #         self.enemyblock_num = 34
    #         self.enemyblock_attacknum.append(0)
    #         self.enemyblock_attacknum.append(0)
    #     elif goal_x > 2 and goal_x <= 3 and goal_y > 4 and goal_y <= 5:
    #         self.enemyblock_num = 35
    #         self.enemyblock_attacknum.append(0)
    #         self.enemyblock_attacknum.append(0)
    #     elif goal_x > 3 and goal_x <= 4 and goal_y > 4 and goal_y <= 5:
    #         self.enemyblock_num = 36
    #         self.enemyblock_attacknum.append(0)
    #         self.enemyblock_attacknum.append(0)
    #     elif goal_x > 4 and goal_x <= 5 and goal_y > 4 and goal_y <= 5:
    #         self.enemyblock_num = 37
    #         self.enemyblock_attacknum.append(0)
    #         self.enemyblock_attacknum.append(0)
    #     elif goal_x > 5 and goal_x <= 6 and goal_y > 4 and goal_y <= 5:
    #         self.enemyblock_num = 38
    #         self.enemyblock_attacknum.append(0)
    #         self.enemyblock_attacknum.append(0)
    #     elif goal_x > 6 and goal_x <= 7 and goal_y > 4 and goal_y <= 5:
    #         self.enemyblock_num = 39
    #         self.enemyblock_attacknum.append(0)
    #         self.enemyblock_attacknum.append(0)
    #     elif goal_x > 7 and goal_x <= 8 and goal_y > 4 and goal_y <= 5:
    #         self.enemyblock_num = 40
    #         self.enemyblock_attacknum.append(0)
    #         self.enemyblock_attacknum.append(0)
    #     else:
    #         print 'block calculate error!!!!!!!!!!'









