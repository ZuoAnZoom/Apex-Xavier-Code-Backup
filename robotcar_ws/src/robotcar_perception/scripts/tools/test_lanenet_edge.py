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
import time

import cv2
import numpy as np
import tensorflow as tf
import matplotlib.pyplot as plt

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


def test_lanenet(image_path, weights_path):
    """

    :param image_path:
    :param weights_path:
    :return:
    """
    # assert ops.exists(image_path), '{:s} not exist'.format(image_path)

    LOG.info('Start reading image and preprocessing')
    t_start = time.time()
    # image = plt.imread(image_path)
    image = cv2.imread(image_path, cv2.IMREAD_COLOR)

    image_vis = image
    image = cv2.resize(image, (512, 256), interpolation=cv2.INTER_LINEAR)
    image = image / 127.5 - 1.0

    #加一维边缘检测
    image_gray = cv2.imread(image_path, cv2.IMREAD_GRAYSCALE)
    image_edge = cv2.Canny(image_gray, 70, 150)
    image_edge = cv2.resize(image_edge, (512, 256), interpolation=cv2.INTER_LINEAR)
    image_edge = image_edge / 127.5 - 1.0
    image_edge = np.expand_dims(image_edge, -1)

    input_array = np.concatenate((image, image_edge) , axis=-1)

    LOG.info('Image load complete, cost time: {:.5f}s'.format(time.time() - t_start))

    input_tensor = tf.placeholder(dtype=tf.float32, shape=[1, 256, 512, 4], name='input_tensor')

    net = lanenet.LaneNet(phase='test')
    binary_seg_ret, instance_seg_ret = net.inference(input_tensor=input_tensor, name='LaneNet')

    postprocessor = lanenet_postprocess.LaneNetPostProcessor()

    # Set sess configuration
    sess_config = tf.ConfigProto()
    sess_config.gpu_options.per_process_gpu_memory_fraction = CFG.GPU.GPU_MEMORY_FRACTION
    sess_config.gpu_options.allow_growth = CFG.GPU.TF_ALLOW_GROWTH
    sess_config.gpu_options.allocator_type = 'BFC'

    sess = tf.Session(config=sess_config)

    # define saver
    saver = tf.train.Saver()

    with sess.as_default():
        saver.restore(sess=sess, save_path=weights_path)

        t_start = time.time()
        binary_seg_image, instance_seg_image = sess.run(
            [binary_seg_ret, instance_seg_ret],
            feed_dict={input_tensor: [input_array]}
        )
        t_cost = time.time() - t_start
        LOG.info('Single imgae inference cost time: {:.5f}s'.format(t_cost))

        # mask_image = postprocessor.cluster(
        #     binary_seg_result=binary_seg_image[0],
        #     instance_seg_result=instance_seg_image[0]
        # )
        # postprocess_result = postprocessor.postprocess_2(source_image=image_vis)


        postprocess_result = postprocessor.postprocess(
            binary_seg_result=binary_seg_image[0],
            instance_seg_result=instance_seg_image[0],
            source_image=image_vis
        )

        mask_image = postprocess_result['mask_image']

        for i in range(CFG.MODEL.EMBEDDING_FEATS_DIMS):
            instance_seg_image[0][:, :, i] = minmax_scale(instance_seg_image[0][:, :, i])
        embedding_image = np.array(instance_seg_image[0], np.uint8)

        # BEV_image = postprocess_result['BEV_image']
        # cv2.imshow("BEV_image", BEV_image)
        # cv2.waitKey()

        plt.figure('mask_image')
        plt.imshow(mask_image[:, :, (2, 1, 0)])
        plt.figure('src_image')
        plt.imshow(image_vis[:, :, (2, 1, 0)])
        plt.figure('instance_image')
        plt.imshow(embedding_image[:, :, (2, 1, 0)])
        plt.figure('binary_image')
        plt.imshow(binary_seg_image[0] * 255, cmap='gray')
        # plt.figure('BEV_image')
        # plt.imshow(BEV_image[:, :, (2, 1, 0)])
        plt.show()


    sess.close()

    return


if __name__ == '__main__':
    """
    test code
    """
    # init args
    args = init_args()
    args.image_path = '/media/st/H/xiaoyuan/test_set/src/17.jpg'
    args.weights_path = '/media/st/9C20-06C6/Weights/CU_xy_edge/tusimple_train.ckpt'

    test_lanenet(args.image_path, args.weights_path)