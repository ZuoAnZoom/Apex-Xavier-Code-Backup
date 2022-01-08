#!/usr/bin/python3

#coding:utf-8

import roslib;  
import rosbag
import rospy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from cv_bridge import CvBridgeError

# [需要修改的位置1], 注意一定要以/结尾
# path='/media/nvidia/Extreme SSD/211105/left/' #存放图片的位置
path='/home/nvidia/workspace/211105/left/' #存放图片的位置

class ImageCreator():


   def __init__(self):
       self.bridge = CvBridge()
       i = 0
       # [需要修改的位置2]
       with rosbag.Bag('/home/nvidia/workspace/2021-11-05-17-24-40.bag', 'r') as bag:   #要读取的bag文件；
           for topic,msg,t in bag.read_messages():
               # [需要修改的位置3]
               if topic == "/miivii_gmsl_ros_A/camera1":  #图像的topic；
                       try:
                           cv_image = self.bridge.imgmsg_to_cv2(msg,"bgr8")
                       except CvBridgeError as e:
                           print(e)
                       timestr = "%.6f" %  msg.header.stamp.to_sec()
                       #%.6f表示小数点后带有6位，可根据精确度需要修改；
                       image_name = str(i)+ ".png" #图像命名：时间戳.jpg
                       cv2.imwrite(path+image_name, cv_image)  #保存；
                       i += 1

if __name__ == '__main__':

   #rospy.init_node(PKG)

   try:
       image_creator = ImageCreator()
   except rospy.ROSInterruptException:
       pass
