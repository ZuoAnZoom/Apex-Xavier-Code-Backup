#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
from geometry_msgs.msg import Twist
import sys, select, termios, tty

msg = """
--------------------------------------------------------
                        车辆控制
--------------------------------------------------------
                             i    o     
                        j         l
                             ,    .
                      i/, : 加速/减速
                      j/l : 左转/右转
                      o   : 刹车急停
                      .   : 解除刹车
                      q   : 退出
--------------------------------------------------------
"""

def getKey():
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''

    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

def printInfo():
    print '转向:',
    print '{:<2}'.format(str(target_turn)),
    if target_turn == max_turn:
        print '{:<9}'.format('(max)'),
    elif target_turn == min_turn:
        print '{:<9}'.format('(min)'),
    else:
        print '{:<9}'.format(' '),

    print '速度:',
    print '{:<1}'.format(str(target_vel)),
    if target_vel == max_vel:
        print '{:<9}'.format('(max)'),
    elif target_vel == min_vel:
        print '{:<9}'.format('(min)'),
    else: 
        print '{:<9}'.format(' '),

    print '刹车(%):',
    print '{:<1}'.format(str(target_brake)),
    if target_brake == max_brake:
        print('(max)')
    elif target_brake == 0:
        print('(min)')
    else: 
        print(' ')
    return 0

if __name__=="__main__":
    settings = termios.tcgetattr(sys.stdin)
    
    rospy.init_node('robotcar_teleop')
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)

    target_vel = 0
    target_vel_th = 0
    max_vel_th = 25
    min_vel_th = -25
    max_vel = 2
    min_vel = -2
    max_brake = 100
    cycles = 0

    target_turn = 0
    target_brake = 0
    max_turn = 25
    min_turn = -25

    try:
        # print msg
        while(1):
            key = getKey()

            # 左转
            if key == 'j':
                if target_turn < max_turn:
                    target_turn += 1
                printInfo()
                cycles += 1
            
            # 右转
            elif key == 'l':
                if target_turn > min_turn:
                    target_turn -= 1
                printInfo()
                cycles += 1

            # 速度增加
            elif key == 'i':
                if target_vel < max_vel:
                    target_vel += 0.1
                printInfo()
                cycles += 1

            # 速度减小
            elif key == ',':
                if target_vel > min_vel:
                    target_vel -= 0.1
                printInfo()
                cycles += 1

            # 刹车急停
            elif key == 'o':
                target_vel = 0
                target_turn = 0
                cmd = Twist()
                cmd.linear.x = 0
                cmd.linear.y = 1  # 指代刹车急停
                cmd.linear.z = 0
                cmd.angular.x = 0
                cmd.angular.y = 0
                cmd.angular.z = 0
                pub.publish(cmd)
                printInfo()
                cycles += 1
                continue

            # 刹车解除
            elif key == '.':
                cmd = Twist()
                cmd.linear.x = 0
                cmd.linear.y = 2  # 指代刹车解除
                cmd.linear.z = 0
                cmd.angular.x = 0
                cmd.angular.y = 0
                cmd.angular.z = 0
                pub.publish(cmd)
                continue

            elif key == 'q':
                break
            
            if cycles % 15 == 0:
                print msg
                cycles += 1
            
            # 创建并发布twist消息
            cmd = Twist()
            cmd.linear.x = target_vel
            cmd.linear.y = 0
            cmd.linear.z = 0
            cmd.angular.x = 0
            cmd.angular.y = 0
            cmd.angular.z = target_turn #target_vel_th
            pub.publish(cmd)

    finally:
        cmd = Twist()
        cmd.linear.x = 0
        cmd.linear.y = 0
        cmd.linear.z = 0
        cmd.angular.x = 0
        cmd.angular.y = 0
        cmd.angular.z = 0
        pub.publish(cmd)

    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
