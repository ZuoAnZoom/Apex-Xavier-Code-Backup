"""
直接对binary的结果进行分类
"""
# import pyximport;pyximport.install(pyimport=True)
import numpy as np
import cv2
import time
import math
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from tools import dbscan, cal_position, my_cluster
from numba import jit
from local_utils.config_utils import parse_config_utils

CFG = parse_config_utils.lanenet_cfg


def _morphological_process(image, kernel_size=5):
    """
    morphological process to fill the hole in the binary segmentation result
    :param image:
    :param kernel_size:
    :return:
    """
    if len(image.shape) == 3:
        raise ValueError('Binary segmentation result image should be a single channel image')

    if image.dtype is not np.uint8:
        image = np.array(image, np.uint8)

    kernel = cv2.getStructuringElement(shape=cv2.MORPH_ELLIPSE, ksize=(kernel_size, kernel_size))

    # close operation fill hole
    closing = cv2.morphologyEx(image, cv2.MORPH_CLOSE, kernel, iterations=1)

    return closing


def _connect_components_analysis(image):
    """
    connect components analysis to remove the small components
    :param image:
    :return:
    """
    if len(image.shape) == 3:
        gray_image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    else:
        gray_image = image

    return cv2.connectedComponentsWithStats(gray_image, connectivity=8, ltype=cv2.CV_32S)


#@jit(nopython=True)
@jit # by csy 2021-12-14
def cal_x_centers(nonzerox):
    """
    计算分割的图片的中心x坐标，放外面才能用numba加速
    :param nonzerox:
    :return:
    """
    x_centers_each = []
    for x in nonzerox:
        x_center_exist = False
        for index, x_center in enumerate(x_centers_each):
            # 灰度重心法对车道线像素下采样，那不同的路段，这个距离限制50可能就不一样
            if abs(x - x_center) <= 50:
                x_centers_each[index] = (x + x_center) / 2
                x_center_exist = True
                break
        if not x_center_exist:
            x_centers_each.append(x)
    return x_centers_each


@jit(nopython=True)
def pixel_IMP2posiiton_real(points, labels_of_points, Minv, H, center, f_x, f_y):
    """

    :param points:
    :param labels_of_points:
    :param lane_positions:
    :param Minv:
    :param H:
    :param center:
    :param f_x:
    :param f_y:
    :return:
    """
    lane_positions = []
    unique_labels = np.unique(labels_of_points)
    for index, label in enumerate(unique_labels):
        if label == -1:
            continue
        label_indexs = np.reshape(np.array(np.where(labels_of_points == label)), (-1,))
        single_lane = []
        for _ in label_indexs:
            # 从俯视图投影回正视图
            point_src_mat = np.matmul(Minv, np.array([points[_][1], points[_][0], 1]))
            # 归一化
            point_src = [point_src_mat[0] / point_src_mat[2], point_src_mat[1] / point_src_mat[2]]
            # 根据正视图中的点的坐标，成像的相似三角形，计算真实世界中的坐标
            point_position_real = cal_position.related_pos(H, center, point_src, f_x, f_y)

            single_lane.append(point_position_real)
        lane_positions.append(single_lane)
    return lane_positions



@jit
def fitting_and_related_position(lanes_coords, dis_pix_y, dis_real_zong):
    """
    根据每条线的重心点，拟合在鸟瞰图中的车道线方程，并给出车道线的相对位置
    想用numba加速，但是并不支持polyfit函数
    :param lanes_coords:
    :param dis_pix_y:
    :param dis_real_zong:
    :return:
    """
    fit_params = []
    lane_positions = []
    for lane_index, coords in enumerate(lanes_coords):
        coords = np.array(coords)
        plot_y = coords[:, 0]
        plot_x = coords[:, 1]
        # 二次方程进行拟合
        fit_param = np.polyfit(plot_y, plot_x, 2)
        fit_params.append(fit_param)

        # 计算车道线相对车的坐标
        dst_real = []
        dis_pix_x = fit_param[0] * dis_pix_y ** 2 + \
                    fit_param[1] * dis_pix_y + fit_param[2]
        # 仿真环境中是横向120个像素对应一个车道，3米
        dis_real_heng = (dis_pix_x - 256) * 3 / 120
        for index in range(len(dis_real_zong)):
            dst_real.append([dis_real_heng[index], dis_real_zong[index]])
        lane_positions.append(dst_real)

    return fit_params, lane_positions


class PostProcessor(object):
    """
    得到二值图后进行后处理
    """
    def __init__(self, M, Minv, H, center, f_x, f_y):
        """

        :param M: 正视图到俯视图的变换矩阵
        :param Minv:
        :param H: 摄像机高度
        :param center: 成像中心
        :param f_x: 去畸变、图像缩放后的相机焦距，pixel
        :param f_y:
        """
        self._color_map = [np.array([255, 0, 0]),
                           np.array([0, 255, 0]),
                           np.array([0, 0, 255]),
                           np.array([125, 125, 0]),
                           np.array([0, 125, 125]),
                           np.array([125, 0, 125]),
                           np.array([50, 100, 50]),
                           np.array([100, 50, 100])]
        self.M = M
        self.Minv = Minv
        self.H = H
        self.center = center
        self.f_x = f_x
        self.f_y = f_y


        # 在另一个函数中预先算出来的，避免每次都调用最小二乘
        # 世界坐标系中距离车5,6,7,8,9,10,11米，对应的像素坐标的y值
        # 不再采用这种测量，计算5~11米对应俯视图哪个像素的方法
        # 假设相机水平，路面平坦，直接用相似三角形计算
        # 原理上应该没有问题，关键就是这个相机内参，因为经过了去畸变、图像缩放等操作
        # self.dis_pix_y = np.array(dis_pix_y)
        # self.dis_real_zong = dis_real_zong

        self.lane_positions = []
        self.BEV_lane_pts = []
        self.fit_params = []

    def show_result(self, source_image, fit_params):
        """
        展示拟合结果作图的效果
        :param source_image:
        :param fit_params:
        :return:
        """
        IPM_source_img = cv2.warpPerspective(source_image, self.M, source_image.shape[1::-1], flags=cv2.INTER_LINEAR)

        [BEV_image_height, BEV_image_width] = list(IPM_source_img.shape)[:2]

        for fit_param in fit_params:
            plot_y = np.linspace(10, BEV_image_height, BEV_image_height - 10)
            fit_x = fit_param[0] * plot_y ** 2 + fit_param[1] * plot_y + fit_param[2]

            lane_points = []
            for index in range(0, plot_y.shape[0], 5):
                if fit_x[index] < 0 or fit_x[index] > BEV_image_width:
                    continue
                else:
                    lane_points.append([fit_x[index], plot_y[index]])
            self.BEV_lane_pts.append(lane_points)

            for index, single_lane_pts in enumerate(self.BEV_lane_pts):
                single_lane_pt_x = np.array(single_lane_pts, dtype=np.float32)[:, 0]
                single_lane_pt_y = np.array(single_lane_pts, dtype=np.float32)[:, 1]

                # 作图范围
                start_plot_y = 10
                end_plot_y = 250

                step = int(math.floor((end_plot_y - start_plot_y) / 10))
                for plot_y in np.linspace(start_plot_y, end_plot_y, step):
                    diff = single_lane_pt_y - plot_y
                    fake_diff_bigger_than_zero = diff.copy()
                    fake_diff_smaller_than_zero = diff.copy()
                    fake_diff_bigger_than_zero[np.where(diff <= 0)] = float('inf')
                    fake_diff_smaller_than_zero[np.where(diff > 0)] = float('-inf')
                    idx_low = np.argmax(fake_diff_smaller_than_zero)
                    idx_high = np.argmin(fake_diff_bigger_than_zero)

                    previous_src_pt_x = single_lane_pt_x[idx_low]
                    previous_src_pt_y = single_lane_pt_y[idx_low]
                    last_src_pt_x = single_lane_pt_x[idx_high]
                    last_src_pt_y = single_lane_pt_y[idx_high]

                    if previous_src_pt_y < start_plot_y or last_src_pt_y < start_plot_y or \
                            fake_diff_smaller_than_zero[idx_low] == float('-inf') or \
                            fake_diff_bigger_than_zero[idx_high] == float('inf'):
                        continue

                    interpolation_src_pt_x = (abs(previous_src_pt_y - plot_y) * previous_src_pt_x +
                                              abs(last_src_pt_y - plot_y) * last_src_pt_x) / \
                                             (abs(previous_src_pt_y - plot_y) + abs(last_src_pt_y - plot_y))
                    interpolation_src_pt_y = (abs(previous_src_pt_y - plot_y) * previous_src_pt_y +
                                              abs(last_src_pt_y - plot_y) * last_src_pt_y) / \
                                             (abs(previous_src_pt_y - plot_y) + abs(last_src_pt_y - plot_y))

                    if interpolation_src_pt_x > BEV_image_width or interpolation_src_pt_x < 10:
                        continue

                    lane_color = self._color_map[index].tolist()
                    cv2.circle(IPM_source_img, (int(interpolation_src_pt_x), int(interpolation_src_pt_y)),
                               4, lane_color, -1)

        cv2.imshow('image', IPM_source_img)
        cv2.waitKey(10)

        self.BEV_lane_pts = []

    def postprocess(self, binary_seg_result, source_image, N, min_area_threshold = 100):
        """

        :param binary_seg_result:
        :param source_image:
        :param N:
        :param min_area_threshold:
        :return:
        """
        # cv2.imshow('source_image', source_image)
        # cv2.waitKey(10)
        IPM_source_img = cv2.warpPerspective(source_image, self.M, source_image.shape[1::-1],
                                             flags=cv2.INTER_LINEAR)
        # cv2.imshow('image', IPM_source_img)
        # cv2.waitKey(10)

        # 二值分割结果的处理
        binary_seg_result = np.array(binary_seg_result * 255, dtype=np.uint8)
        # 应用形态学变化填充黑洞，去除小的区域
        morphological_ret = _morphological_process(binary_seg_result, kernel_size=5)
        # 连通域分析，去除小的连通域
        connect_components_analysis_ret = _connect_components_analysis(image=morphological_ret)
        labels = connect_components_analysis_ret[1]
        stats = connect_components_analysis_ret[2]
        for index, stat in enumerate(stats):
            if stat[4] <= min_area_threshold:
                idx = np.where(labels == index)
                morphological_ret[idx] = 0
        # 太远处的结果忽略，这个之后根据相机的俯仰角计算更合适
        morphological_ret[0:CFG.POSTPROCESS.VALID_DISTANCE, :] = 0
        # cv2.imshow('binary_seg_result', morphological_ret)
        # cv2.waitKey(10)

        # 对二值图进行投影变换
        IPM_binary_image = cv2.warpPerspective(morphological_ret, self.M,
                                               morphological_ret.shape[1::-1], flags=cv2.INTER_NEAREST)
        # cv2.imshow('binary_seg_IPM', IPM_binary_image)
        # cv2.waitKey(10)

        # 提取重心
        image_shape = IPM_binary_image.shape
        height_of_part = int(np.floor(image_shape[0] / N))
        centers = []
        for i in range(N):
            # time1 = time.time()
            # 不同的位置，腐蚀程度应该不同，如果不仅是根据纵向距离就更好了
            # 究其原因，在真值图中没有做到远小近大，以至于投影时，越远被放大得越严重
            # 腐蚀的kernel最大设10，最小设1, 反比例
            kernel_size = int((1 - 10) / N * i + 10)
            # kernel_size = int((40 - i) / 5)
            kernel = np.ones((kernel_size, kernel_size), np.uint8)
            # time2 = time.time()
            if i == N - 1:
                part_img = IPM_binary_image[i * height_of_part: image_shape[0], :]
                erosion = cv2.erode(part_img, kernel)
                IPM_binary_image[i * height_of_part: image_shape[0], :] = erosion
                edge_image = cv2.Canny(erosion, 50, 150, apertureSize=3)
            else:
                part_img = IPM_binary_image[i * height_of_part: (i + 1) * height_of_part, :]
                erosion = cv2.erode(part_img, kernel)
                IPM_binary_image[i * height_of_part: (i + 1) * height_of_part, :] = erosion
                edge_image = cv2.Canny(erosion, 50, 150, apertureSize=3)

            # # 模拟断连
            # if i == 10 or i == 11 or i == 12 or i == 13:
            #     part_edge_img = np.where(part_edge_img < 100, part_edge_img, 0)

            nonzero = edge_image.nonzero()
            nonzeroy = np.array(nonzero[0])
            nonzerox = np.array(nonzero[1])
            # 没有线后面就不需要继续了
            if len(nonzeroy) == 0:
                continue
            # y值还要加上每一part的起始值
            y_mean = np.mean(nonzeroy) + i * height_of_part
            # time3 = time.time()
            x_centers_each = cal_x_centers(nonzerox)
            # IPM_source_img_1 = IPM_source_img
            for x in x_centers_each:
                # # 从俯视图投影回正视图
                # point_src_mat = np.matmul(self.Minv, np.array([x, y_mean, 1]))
                # # 归一化
                # point_src = [point_src_mat[0] / point_src_mat[2], point_src_mat[1] / point_src_mat[2]]
                # cv2.circle(source_image, (int(point_src[0]), int(point_src[1])),
                #            4, lane_color, -1)

                centers.append([int(y_mean), int(x)])
                # lane_color = [0, 0, 0]
                # cv2.circle(IPM_source_img_1, (int(x), int(y_mean)),
                #            4, lane_color, -1)

            # cv2.imshow('IPM_source_img_1', IPM_source_img_1)
            # cv2.waitKey(10)

            # time4 = time.time()
            # print('1', time2 - time1)
            # print('2', time3 - time2)
            # print('3', time4 - time3)

        # cv2.imshow('bianry_IPM_erode', IPM_binary_image)
        # cv2.waitKey(10)

        # # 提取到的重心用DBSCAN分类，EPS和MIN_SAMPLES有待商榷
        # labels_of_centers = dbscan.dbscan_simple(np.array(centers), 70, 5)

        # 提取到的重心进行聚类
        centers = centers[::-1]
        labels_of_centers = np.array(my_cluster.cluster(
            np.array(centers), CFG.POSTPROCESS.MY_CLUSTER_EPS, CFG.POSTPROCESS.MY_CLUSTER_MIN_SAMPLES))

        # 聚类结果呈现在src_IPM上
        for index, label in enumerate(labels_of_centers):
            if label == -1:
                continue
            lane_color = self._color_map[label].tolist()
            center = centers[index]
            cv2.circle(IPM_source_img, (center[1], center[0]), 4, lane_color, -1)

        # self.lane_positions = pixel_IMP2posiiton_real(points=centers, labels_of_points=labels_of_centers,
        #                                               Minv=self.Minv,
        #                                               H=self.H, center=self.center, f_x=self.f_x,f_y=self.f_y)

        # 根据分类结果将重心重新分别存储
        # 所有车道线的点还要投影到正视图中，因为投影的相似三角形关系并不是与俯视图的
        unique_labels = np.unique(labels_of_centers)
        # lanes_coords = []
        for index, label in enumerate(unique_labels):
            if label == -1:
                continue
            label_indexs = np.reshape(np.array(np.where(labels_of_centers == label)), (-1, ))
            single_lane = []
            for _ in label_indexs:
                # 从俯视图投影回正视图
                point_src_mat = np.matmul(self.Minv, np.array([centers[_][1], centers[_][0], 1]))
                # 归一化
                point_src = [point_src_mat[0] / point_src_mat[2], point_src_mat[1] / point_src_mat[2]]
                # cv2.circle(source_image, (int(point_src[0]), int(point_src[1])),
                #            4, [0, 0, 0], -1)
                # 根据正视图中的点的坐标，成像的相似三角形，计算真实世界中的坐标
                point_position_real = cal_position.related_pos(self.H, self.center, point_src, self.f_x, self.f_y)

                single_lane.append(point_position_real)

            # 原来是存俯视图中的像素坐标，现在直接存真实世界的位置
            # lanes_coords.append(single_lane)
            self.lane_positions.append(single_lane)


        # # IPM_source_img = cv2.warpPerspective(source_image, self.M, source_image.shape[1::-1],
        # #                                      flags=cv2.INTER_LINEAR)


        # # 直接将点给出去，不再拟合
        # # 根据每条线的重心点，拟合在鸟瞰图中的车道线方程，并给出车道线的相对位置
        # for lane_index, coords in enumerate(lanes_coords):
        #     coords = np.array(coords)
        #     plot_y = coords[:, 0]
        #     plot_x = coords[:, 1]
        #     fit_param = np.polyfit(plot_y, plot_x, 2)
        #     self.fit_params.append(fit_param)
        #
        #     # lane_color = self._color_map[lane_index].tolist()
        #     # # lane_color = [0, 0, 0]
        #     # for coord in coords:
        #     #     cv2.circle(IPM_source_img, (int(coord[1]), int(coord[0])),
        #     #                4, lane_color, -1)
        #
        #     # 计算车道线相对车的坐标
        #     # 不再采用这种等比关系计算的方式
        #     dst_real = []
        #     dis_pix_x = fit_param[0] * self.dis_pix_y ** 2 + \
        #                 fit_param[1] * self.dis_pix_y + fit_param[2]
        #     # 仿真环境是横向120个像素对应一个车道，3米
        #     dis_real_heng = (dis_pix_x - 256) * 3 / 120
        #     for index in range(len(self.dis_real_zong)):
        #         dst_real.append([dis_real_heng[index], self.dis_real_zong[index]])
        #     self.lane_positions.append(dst_real)
        #
        # self.show_result(source_image=source_image, fit_params=self.fit_params)

        # cv2.imshow('source_image', source_image)
        # cv2.waitKey(10)
        # cv2.imshow('image_final', IPM_source_img)
        # cv2.waitKey(10)
        pub = rospy.Publisher("/perception/lanes_image", Image, queue_size=10)
        bridge = CvBridge()
        image_final = bridge.cv2_to_imgmsg(IPM_source_img, "bgr8")
        pub.publish(image_final)

        ret = {
            'positions': self.lane_positions
        }
        # print(self.lane_positions)
        self.lane_positions = []
        self.fit_params = []

        return ret

