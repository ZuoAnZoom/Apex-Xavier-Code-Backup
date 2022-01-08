#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
from std_msgs.msg import Float32MultiArray
import sys, select, termios, tty

msg = """
--------------------------------------------------------
                        车辆控制
--------------------------------------------------------
                        u    i    o     
                        j    k    l
                        m    ,    .
                      i/, : 加速/减速
                      j/l : 左转/右转
                      u/m : 加/减刹车
                      k   : 缓停
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
    print '转向(°):',
    print '{:<3}'.format(str(target_turn)),
    if target_turn == max_turn:
        print '{:<10}'.format('(max)'),
    elif target_turn == min_turn:
        print '{:<10}'.format('(min)'),
    else:
        print '{:<10}'.format(' '),

    print '油门(%):',
    print '{:<2}'.format(str(target_throttle)),
    if target_throttle == max_throttle:
        print '{:<10}'.format('(max)'),
    elif target_throttle == min_throttle:
        print '{:<10}'.format('(min)'),
    else: 
        print '{:<10}'.format(' '),

    print '刹车(%):',
    print '{:<2}'.format(str(target_brake)),
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
    pub = rospy.Publisher('/cmd_chassis', Float32MultiArray, queue_size=1)

    target_throttle = 0
    target_turn = 0
    target_brake = 0
    max_turn = 25
    min_turn = -25
    max_throttle = 20
    min_throttle = -20
    max_brake = 100
    cycles = 0

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

            # 加速
            elif key == 'i':
                if target_throttle < max_throttle:
                    target_throttle += 1
                printInfo()
                cycles += 1

            # 减速
            elif key == ',':
                if target_throttle > min_throttle:
                    target_throttle -= 1
                printInfo()
                cycles += 1

            # 刹车增大
            elif key == 'u':
                if target_brake < max_brake:
                    target_brake += 1
                printInfo()
                cycles += 1

            # 刹车减小
            elif key == 'm':
                if target_brake > 0:
                    target_brake -= 1
                printInfo()
                cycles += 1

            # 刹车急停
            elif key == 'o':
                target_brake = 100
                target_throttle = 0
                printInfo()
                cycles += 1

            # 刹车解除
            elif key == '.':
                target_brake = 0
                printInfo()
                cycles += 1

            # 停止键
            elif key == 'k':
                target_turn = 0
                target_throttle = 0
                printInfo()
                cycles += 1

            elif key == 'q':
                break
            
            if cycles % 15 == 0:
                print msg
                cycles += 1
            
            # 创建并发布twist消息
            cmd = Float32MultiArray()
            cmd.data.append(target_turn)            # 第一位：转角
            cmd.data.append(target_throttle)        # 第二位：油门
            cmd.data.append(target_brake)           # 第三位：刹车
            pub.publish(cmd)

    finally:
        cmd = Float32MultiArray()
        cmd.data.append(0)
        cmd.data.append(0)
        cmd.data.append(0)
        pub.publish(cmd)

    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
