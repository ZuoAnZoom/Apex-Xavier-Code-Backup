#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
from geometry_msgs.msg import Twist
import sys, select, termios, tty

msg = """
--------------------------------------------------------
                        车辆控制
--------------------------------------------------------
                      o: 接管并停车
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

if __name__=="__main__":
    settings = termios.tcgetattr(sys.stdin)
    
    rospy.init_node('robotcar_teleop')
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)

    target_vel = 0
    target_vel_th = 0
    target_turn = 0

    print msg

    # print msg
    while(1):
        key = getKey()

        # 刹车急停
        if key == 'o':
            target_vel = 0
            target_turn = 0
            cmd = Twist()
            cmd.linear.x = 0
            cmd.linear.y = 1  # 指代刹车急停
            cmd.linear.z = 0
            cmd.angular.x = 0
            cmd.angular.y = 0
            cmd.angular.z = 0
            while(1):
                pub.publish(cmd)