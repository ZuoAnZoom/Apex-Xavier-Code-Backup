#!/usr/bin/env python3.6
# -*- coding: utf-8 -*-
import sys
sys.path.append('/home/h/catkin_ws/src/robotcar_perception/scripts')
import numpy as np
import cv2
import time
import rospy
import tensorflow as tf
from robotcar_perception.msg import lanes
from lanenet_model import lanenet
from lanenet_model import lanenet_postprocess3
from sensor_msgs.msg import Image
from tools import test_lanenet
from cv_bridge import CvBridge

from local_utils.config_utils import parse_config_utils
CFG = parse_config_utils.lanenet_cfg



if __name__ == '__main__':
    rospy.init_node('robotcar_perception')
    pub = rospy.Publisher("/perception/lanes", lanes, queue_size=10)

    weights_path = '/home/h/catkin_ws/src/robotcar_perception/scripts/Weights/tusimple_lanet_bise/tusimple_lanenet.ckpt'

    input_tensor = tf.placeholder(dtype=tf.float32, shape=[1, 256, 512, 3], name='input_tensor')

    net = lanenet.LaneNet(phase='test')
    binary_seg_ret, instance_seg_ret = net.inference(input_tensor=input_tensor, name='LaneNet')

    postprocessor = lanenet_postprocess3.LaneNetPostProcessor()

    sess_config = tf.ConfigProto()
    sess_config.gpu_options.per_process_gpu_memory_fraction = 0.9  # CFG.TEST.GPU_MEMORY_FRACTION
    sess_config.gpu_options.allow_growth = True  # CFG.TRAIN.TF_ALLOW_GROWTH
    sess_config.gpu_options.allocator_type = 'BFC'

    sess = tf.Session(config=sess_config)

    saver = tf.train.Saver()

    with sess.as_default():
        saver.restore(sess=sess, save_path=weights_path)

        while 1:
            transform_result = lanes()
            transform_result.stamp = rospy.Time.now()

            msg = rospy.wait_for_message("/miivii_gmsl_ros/camera1", Image, timeout=None)
            bridge = CvBridge()
            data = bridge.imgmsg_to_cv2(msg, "bgr8")
            image = np.reshape(data, 480, 720, 3))
            # image = cv2.imread('/home/st/Tusimple测试/图片/2.jpg', cv2.IMREAD_COLOR)
            image_vis = image
            image = cv2.resize(image, (512, 256), interpolation=cv2.INTER_LINEAR)
            image = image / 127.5 - 1.0

            binary_seg_image, instance_seg_image = sess.run(
                [binary_seg_ret, instance_seg_ret],
                feed_dict={input_tensor: [image]})

            mask_image = postprocessor.cluster(
                binary_seg_result=binary_seg_image[0],
                instance_seg_result=instance_seg_image[0]
            )
            if not np.any(mask_image):
                transform_result.num = 0
                pub.publish(transform_result)
                continue
                
            postprocess_result = postprocessor.postprocess_2(source_image=image_vis)

            result = postprocess_result['position_now']
            transform_result.num = len(result)
            for i in range(len(result)):
                for j in range(9):
                    exec('transform_result.y_{}.append(result[{}][{}][0])'.format(i, i, j))
                    exec('transform_result.x_{}.append(result[{}][{}][1])'.format(i, i, j))
            pub.publish(transform_result)

            BEV_image = postprocess_result['BEV_image']
            cv2.imshow("BEV_image", BEV_image)
            cv2.waitKey(10)

    # sess.close()