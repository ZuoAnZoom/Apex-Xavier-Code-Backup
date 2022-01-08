#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# @Time    : 18-5-23 上午11:33
# @Author  : MaybeShewill-CV
# @Site    : https://github.com/MaybeShewill-CV/lanenet-lane-detection
# @File    : test_lanenet.py
# @IDE: PyCharm Community Edition
"""
test LaneNet model on single image
"""
import argparse
import os.path as ops
import time

import cv2
import matplotlib.pyplot as plt
import numpy as np
import tensorflow as tf

from lanenet_model import lanenet
from lanenet_model import lanenet_postprocess
from local_utils.config_utils import parse_config_utils
from local_utils.log_util import init_logger

CFG = parse_config_utils.lanenet_cfg
LOG = init_logger.get_logger(log_file_name_prefix='lanenet_test')


def init_args():
    """

    :return:
    """
    parser = argparse.ArgumentParser()
    parser.add_argument('--image_path', type=str, help='The image path or the src image save dir')
    parser.add_argument('--weights_path', type=str, help='The model weights path')

    return parser.parse_args()


def args_str2bool(arg_value):
    """

    :param arg_value:
    :return:
    """
    if arg_value.lower() in ('yes', 'true', 't', 'y', '1'):
        return True

    elif arg_value.lower() in ('no', 'false', 'f', 'n', '0'):
        return False
    else:
        raise argparse.ArgumentTypeError('Unsupported value encountered.')


def minmax_scale(input_arr):
    """

    :param input_arr:
    :return:
    """
    min_val = np.min(input_arr)
    max_val = np.max(input_arr)

    output_arr = (input_arr - min_val) * 255.0 / (max_val - min_val)

    return output_arr


def test_lanenet(src_dir, weights_path):
    """

    :param src_dir:
    :param weights_path:
    :return:
    """
    assert ops.exists(src_dir), '{:s} not exist'.format(src_dir)



    input_tensor = tf.placeholder(dtype=tf.float32, shape=[1, 256, 512, 3], name='input_tensor')

    net = lanenet.LaneNet(phase='test')
    binary_seg_ret, instance_seg_ret = net.inference(input_tensor=input_tensor, name='LaneNet')

    postprocessor = lanenet_postprocess.LaneNetPostProcessor()

    # Set sess configuration
    sess_config = tf.ConfigProto()
    sess_config.gpu_options.per_process_gpu_memory_fraction = CFG.GPU.GPU_MEMORY_FRACTION
    sess_config.gpu_options.allow_growth = CFG.GPU.TF_ALLOW_GROWTH
    sess_config.gpu_options.allocator_type = 'BFC'

    sess = tf.Session(config=sess_config)

    # define moving average version of the learned variables for eval
    # with tf.variable_scope(name_or_scope='moving_avg'):
    #     variable_averages = tf.train.ExponentialMovingAverage(
    #         CFG.SOLVER.MOVING_AVE_DECAY)
    #     variables_to_restore = variable_averages.variables_to_restore()

    # define saver
    saver = tf.train.Saver()

    with sess.as_default():
        saver.restore(sess=sess, save_path=weights_path)
        avg_time_cost = []

        for i in range(1, 1000):
            # image_path = ops.join(src_dir,'frame0'+str(i)+'.jpg')
            image_path = ops.join(src_dir, str(i) + '.jpg')
            image = cv2.imread(image_path, cv2.IMREAD_COLOR)
            image_vis = image
            image = cv2.resize(image, (512, 256), interpolation=cv2.INTER_LINEAR)
            cv2.imwrite('/media/st/4297-ED06/Data/src_img/' + str(i) + '.jpg', image)

            # image_vis = cv2.resize(image_vis, (512, 256), interpolation=cv2.INTER_LINEAR)
            # image_vis = image_vis / 127.5 - 1.0

            print(i)
            image = image / 127.5 - 1.0

            t_start = time.time()
            binary_seg_image, instance_seg_image = sess.run(
                [binary_seg_ret, instance_seg_ret],
                feed_dict={input_tensor: [image]}
            )

            coords_xy = []
            binary_seg_result = np.array(binary_seg_image[0] * 255, dtype=np.uint8)
            lane_coords = np.where(binary_seg_result == 255)
            lane_coords_y = lane_coords[0]
            lane_coords_x = lane_coords[1]
            for index, x in enumerate(lane_coords_x):
                coords_xy.append(str([x, lane_coords_y[index]]))

            output_file_path = '/media/st/4297-ED06/Data/coords.txt'

            f = open(output_file_path, "a")

            f.writelines(coords_xy)
            f.write('\n')



            # # mask_image = postprocessor.cluster(
            # #     binary_seg_result=binary_seg_image[0],
            # #     instance_seg_result=instance_seg_image[0]
            # # )
            # # postprocess_result = postprocessor.postprocess_2(source_image=image_vis)
            #
            # postprocess_result = postprocessor.postprocess(
            #     binary_seg_result=binary_seg_image[0],
            #     instance_seg_result=instance_seg_image[0],
            #     source_image=image_vis
            # )
            #
            #
            #
            # # mask_image = postprocess_result['mask_image']
            #
            #
            # # img = postprocess_result['BEV_image']
            # # cv2.imshow('detection', img)
            # # cv2.waitKey(10)
            #
            # img = postprocess_result['source_image']
            # cv2.imshow('detection', img)
            # cv2.waitKey(10)
            #
            # # print(postprocess_result['position_now'])
            # avg_time_cost.append(time.time() - t_start)
            # index = i
            # if index % 10 == 0:
            #     LOG.info('Mean inference time every single image: {:.5f}s'.format(np.mean(avg_time_cost)))
            #     avg_time_cost.clear()


    return



if __name__ == '__main__':
    """
    test code
    """
    # init args
    args = init_args()
    args.image_path = '/media/st/8CEA-E915/xy_gc/left'
    args.weights_path = '/media/st/8CEA-E915/Weights/CU_xy_500/tusimple_train.ckpt'

    test_lanenet(args.image_path, args.weights_path)
