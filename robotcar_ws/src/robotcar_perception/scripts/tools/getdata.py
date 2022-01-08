#！/usr/bin/env python2
from cv_bridge import CvBridge

def getdata(msg):
    bridge = CvBridge()
    data = bridge.imgmsg_to_cv2(msg, 'bgr8')
    return data