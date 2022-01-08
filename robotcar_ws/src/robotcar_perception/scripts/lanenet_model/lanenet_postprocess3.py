import math
import cv2
import glog as log
import numpy as np
import matplotlib.pyplot as plt
from sklearn.cluster import DBSCAN
from sklearn.preprocessing import StandardScaler

from local_utils.config_utils import parse_config_utils
from tools import cal_position

CFG = parse_config_utils.lanenet_cfg



def _morphological_process(image, kernel_size=5):
    """
    morphological(形态学的) process to fill the hole in the binary segmentation result
    形态学变换一般只作用于二值图像
    :param image:
    :param kernel_size:
    :return:
    """
    if len(image.shape) == 3:
        raise ValueError('Binary segmentation result image should be a single channel image')

    if image.dtype is not np.uint8:
        image = np.array(image, np.uint8)

    kernel = cv2.getStructuringElement(shape=cv2.MORPH_ELLIPSE, ksize=(kernel_size, kernel_size))
    #可以尝试修改kernel的大小和形状
    # close operation fill hole
    closing = cv2.morphologyEx(image, cv2.MORPH_CLOSE, kernel, iterations=1)
    #选择的是闭，去掉图片白色（车道线）中的黑色噪音
    return closing


def _connect_components_analysis(image):
    """
    connect components analysis to remove the small components
    连通域分析以剔除小的连通域
    :param image:
    :return:
    """
    if len(image.shape) == 3:
        gray_image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    else:
        gray_image = image

    return cv2.connectedComponentsWithStats(gray_image, connectivity=8, ltype=cv2.CV_32S)


class _LaneFeat(object):
    """

    """
    def __init__(self, feat, coord, class_id=-1):
        """
        lane feat object
        :param feat: lane embedding feats [feature_1, feature_2, ...]
        :param coord: lane coordinates [x, y]
        :param class_id: lane class id
        """
        self._feat = feat
        self._coord = coord
        self._class_id = class_id

    @property
    def feat(self):
        """
        @与下面的setter一起，将方法变成属性调用，可以对参数进行检查
        :return:
        """
        return self._feat

    @feat.setter
    def feat(self, value):
        """

        :param value:
        :return:
        """
        if not isinstance(value, np.ndarray):
            value = np.array(value, dtype=np.float64)

        if value.dtype != np.float32:
            value = np.array(value, dtype=np.float64)

        self._feat = value

    @property
    def coord(self):
        """

        :return:
        """
        return self._coord

    @coord.setter
    def coord(self, value):
        """

        :param value:
        :return:
        """
        if not isinstance(value, np.ndarray):
            value = np.array(value)

        if value.dtype != np.int32:
            value = np.array(value, dtype=np.int32)

        self._coord = value

    @property
    def class_id(self):
        """

        :return:
        """
        return self._class_id

    @class_id.setter
    def class_id(self, value):
        """

        :param value:
        :return:
        """
        if not isinstance(value, np.int64):
            raise ValueError('Class id must be integer')

        self._class_id = value


class _LaneNetCluster(object):
    """
     Instance segmentation result cluster
     我的理解是，net跑出来的instance segmentation只是哪些像素是车道线，现在需要聚类
     那么，该聚几类呢？车道线的数目不定
    """

    def __init__(self):
        """

        """
        self._color_map = [np.array([255, 0, 0]),
                           np.array([0, 255, 0]),
                           np.array([0, 0, 255]),
                           np.array([125, 125, 0]),
                           np.array([0, 125, 125]),
                           np.array([125, 0, 125]),
                           np.array([50, 100, 50]),
                           np.array([100, 50, 100])]
        #用来画车道线的，所以最多能画八条线

    @staticmethod
    def _embedding_feats_dbscan_cluster(embedding_image_feats):
        """
        dbscan cluster
        :param embedding_image_feats:
        :return:
        """
        db = DBSCAN(eps=CFG.POSTPROCESS.DBSCAN_EPS, min_samples=CFG.POSTPROCESS.DBSCAN_MIN_SAMPLES)
        #DBSCAN聚类，你可以去global_config尝试修改聚类参数
        try:
            features = StandardScaler().fit_transform(embedding_image_feats)
            db.fit(features)
        except Exception as err:
            log.error(err)
            ret = {
                'origin_features': None,
                'cluster_nums': 0,
                'db_labels': None,
                'unique_labels': None,
                'cluster_center': None
            }
            return ret   #return后面的语句不会执行的，这个不要忘了
        db_labels = db.labels_  #db.labels与输入的图像shape相同
        unique_labels = np.unique(db_labels)  #用来获取有几类

        num_clusters = len(unique_labels)
        cluster_centers = db.components_

        ret = {
            'origin_features': features,
            'cluster_nums': num_clusters,
            'db_labels': db_labels,
            'unique_labels': unique_labels,
            'cluster_center': cluster_centers
        }

        return ret

    @staticmethod
    def _get_lane_embedding_feats(binary_seg_ret, instance_seg_ret):
        """
        从instance的结果中选出所有binary判定为车道线的白色像素，获得车道线的像素坐标，但没有聚类
        get lane embedding features according the binary seg result
        :param binary_seg_ret:
        :param instance_seg_ret:
        :return:
        """
        idx = np.where(binary_seg_ret == 255)  #坐标以tuple的形式给出
        lane_embedding_feats = instance_seg_ret[idx]
        # idx_scale = np.vstack((idx[0] / 256.0, idx[1] / 512.0)).transpose()
        # lane_embedding_feats = np.hstack((lane_embedding_feats, idx_scale))
        lane_coordinate = np.vstack((idx[1], idx[0])).transpose() #以垂直方向堆叠，再转置

        assert lane_embedding_feats.shape[0] == lane_coordinate.shape[0]

        ret = {
            'lane_embedding_feats': lane_embedding_feats,
            'lane_coordinates': lane_coordinate
        }

        return ret

    def apply_lane_feats_cluster(self, binary_seg_result, instance_seg_result):
        """
        结合上面的两个函数
        :param binary_seg_result:
        :param instance_seg_result:
        :return:
        """
        # get embedding feats and coords
        get_lane_embedding_feats_result = self._get_lane_embedding_feats(
            binary_seg_ret=binary_seg_result,
            instance_seg_ret=instance_seg_result
        )
        #到这一步，还是选取了binary中的所有白色像素
        # dbscan cluster
        dbscan_cluster_result = self._embedding_feats_dbscan_cluster(
            embedding_image_feats=get_lane_embedding_feats_result['lane_embedding_feats']
        )
        #将所有白色像素对应坐标处的instance_seg_ret进行聚类，并且，聚类的距离应该不是我以为的图片上的距离
        #所以有很多点被归为背景，否则所有的白色像素都留下来，不会聚类不成功
        mask = np.zeros(shape=[binary_seg_result.shape[0], binary_seg_result.shape[1], 3], dtype=np.uint8)
        db_labels = dbscan_cluster_result['db_labels']
        #聚类的要求过高时（eps偏小而num偏大），db_labels全是-1（背景），mask也就全黑了
        #因为binary给出的白色车道线像素不多
        unique_labels = dbscan_cluster_result['unique_labels']
        coord = get_lane_embedding_feats_result['lane_coordinates']

        if db_labels is None:
            return None, None

        lane_coords = []

        for index, label in enumerate(unique_labels.tolist()):
            #unique_labels是一个tuple，先转成list
            if label == -1:
                continue
            idx = np.where(db_labels == label)
            pix_coord_idx = tuple((coord[idx][:, 1], coord[idx][:, 0]))
            mask[pix_coord_idx] = self._color_map[index]
            lane_coords.append(coord[idx])

        return mask, lane_coords
    #上面的这些只产生mask,不产生原始图上的车道线

class LaneNetPostProcessor(object):
    """
    lanenet post process for lane generation
    """
    def __init__(self):
        """

        :param ipm_remap_file_path: ipm generate file path
        """

        self._cluster = _LaneNetCluster()
        self._color_map = [np.array([255, 0, 0]),
                           np.array([0, 255, 0]),
                           np.array([0, 0, 255]),
                           np.array([125, 125, 0]),
                           np.array([0, 125, 125]),
                           np.array([125, 0, 125]),
                           np.array([50, 100, 50]),
                           np.array([100, 50, 100])]

        #这个仿射变换矩阵也是一次计算就行，所以从外部赋值进来
        self.M = np.array([[-2.75395163e-01, -1.75168536e+00, 8.04178548e+02],
                           [2.71544995e-16, -2.22679939e+00, 9.12987751e+02],
                           [3.98836802e-19, -2.76255773e-03, 1.00000000e+00]])

        #在另一个函数中预先算出来的，避免每次都调用最小二乘
        #世界坐标系中距离车5,6,7,8,9,10米，对应的像素坐标的y值
        self.dis_pix_y = np.array([703.9296102, 676.32181888, 648.71402756, 621.10623624,
                                   593.49844492, 565.8906536, 538.28286228, 510.67507096, 483.06727964])


        # lane line fit
        self.lane_coords = []
        self.fit_params_BEV = []
        self.BEV_lane_pts = []
        self.lane_positions = []

        self.pts_num_BEV = []  #还是用BEV中的数据，毕竟是在这里面拟合的
        self.param_2 = 0
        self.param_1 = 0

    def cluster(self, binary_seg_result, instance_seg_result=None,
                    min_area_threshold=CFG.POSTPROCESS.MIN_AREA_THRESHOLD):
        """

        :param binary_seg_result:
        :param instance_seg_result:
        :param min_area_threshold:
        :param source_image:
        :param data_source:
        :return:
        """
        # convert binary_seg_result
        binary_seg_result = np.array(binary_seg_result * 255, dtype=np.uint8)

        # apply image morphology operation to fill in the hold and reduce the small area
        morphological_ret = _morphological_process(binary_seg_result, kernel_size=5)

        #二值图像膨胀，效果不佳
        kernel = np.ones((3, 3), dtype=np.uint8)
        morphological_ret = cv2.dilate(morphological_ret, kernel, 2)  # 1:迭代次数，也就是执行几次膨胀操作

        connect_components_analysis_ret = _connect_components_analysis(image=morphological_ret)

        labels = connect_components_analysis_ret[1]
        stats = connect_components_analysis_ret[2]
        #连通区域分析的返回值，stats是nccomps×5的矩阵 ，表示每个连通区域的外接矩形和面积（pixel）
        for index, stat in enumerate(stats):
            if stat[4] <= min_area_threshold:
                idx = np.where(labels == index)
                morphological_ret[idx] = 0
        #去掉小的连通区域，label的值没必要改，这一步只是为了对binary图像操作
        # apply embedding features cluster

        # plt.figure('morphological_ret')
        # plt.imshow(morphological_ret * 255, cmap='gray')
        # plt.show()

        mask_image, self.lane_coords = self._cluster.apply_lane_feats_cluster(
            binary_seg_result=morphological_ret,
            instance_seg_result=instance_seg_result
        )
        # cv2.imshow('mask_img', mask_image)
        # cv2.waitKey(10000)

        return mask_image



    def postprocess_2(self, source_image=None):
        # 先通过self.lane_coords算出来正视图中的fit_params
        # 且这里的图还是（512, 256）
        for index, coords in enumerate(self.lane_coords):
            tmp_mask = np.zeros(shape=(720, 1280), dtype=np.uint8)

            y = np.array(coords[:, 1])
            y_max = max(y)
            y_min = min(y)
            x = np.array(coords[:, 0])
            fit_param_source = np.polyfit(y, x, 2)

            for add_y in range(y_min, y_max):
                add_x = fit_param_source[0] * add_y ** 2 + fit_param_source[1] * add_y + fit_param_source[2]
                if 0 < add_x < 512:
                    tmp_mask[tuple((np.int_(add_y * 720 / 256), np.int_(add_x * 1280 / 512)))] = 255

            kernel = np.ones((20, 20), dtype=np.uint8)
            tmp_mask = cv2.dilate(tmp_mask, kernel, 2)  # 1:迭代次数，也就是执行几次膨胀操作
            # plt.imshow(tmp_mask)
            # plt.show()

            tmp_mask[tuple((np.int_(coords[:, 1] * 720 / 256), np.int_(coords[:, 0] * 1280 / 512)))] = 255
            # plt.imshow(tmp_mask)
            # plt.show()

        # for lane_index, coords in enumerate(self.lane_coords):
        #     tmp_mask = np.zeros(shape=(720, 1280), dtype=np.uint8)
        #     tmp_mask[tuple((np.int_(coords[:, 1] * 720 / 256), np.int_(coords[:, 0] * 1280 / 512)))] = 255

            # 带有一条线的二值图的鸟瞰图
            tmp_BEV_mask = cv2.warpPerspective(tmp_mask, self.M, tmp_mask.shape[1::-1], flags=cv2.INTER_LINEAR)

            nonzero_y = np.array(tmp_BEV_mask.nonzero()[0])
            nonzero_x = np.array(tmp_BEV_mask.nonzero()[1])
            if not np.any(nonzero_y):
                continue

            fit_param_BEV = np.polyfit(nonzero_y, nonzero_x, 2)
            if abs(fit_param_BEV[0]) > 0.001:
                continue

            self.pts_num_BEV.append(len(nonzero_y))
            self.fit_params_BEV.append(fit_param_BEV)

        # #修正二次项和一次项
        # lane_num = len(self.fit_params_BEV)
        # for i in range(lane_num):
        #     self.param_2 = self.param_2 + self.fit_params_BEV[i][0] * (self.pts_num_BEV[i] / sum(self.pts_num_BEV))
        #     self.param_1 = self.param_1 + self.fit_params_BEV[i][1] * (self.pts_num_BEV[i] / sum(self.pts_num_BEV))
        # print(self.fit_params_BEV[0][:])
        # for param in self.fit_params_BEV:
        #     param[0] = self.param_2
        # for param in self.fit_params_BEV:
        #     param[1] = self.param_1

        #修正零次项,比完之后再删，否则长度就变了
        delete = []
        for i in range(len(self.fit_params_BEV) - 1):
            for j in range(i + 1, len(self.fit_params_BEV)):
                if abs(self.fit_params_BEV[i][2] - self.fit_params_BEV[j][2]) < 200:
                    if self.pts_num_BEV[i] > self.pts_num_BEV[j]:  # 用lane_coord比较也是可以的
                        delete.append(j)
                    else:
                        delete.append(i)
        for i in range(len(delete)-1, -1, -1):
            self.fit_params_BEV.pop(delete[i])
            self.pts_num_BEV.pop(delete[i])
            self.lane_coords.pop(delete[i])

        #输出相对位置，并作图
        #生成原图的鸟瞰图
        BEV_img = cv2.warpPerspective(source_image, self.M, source_image.shape[1::-1], flags=cv2.INTER_LINEAR)

        for fit_param_BEV in self.fit_params_BEV:

            lane_position = cal_position.related_pos(dis_pix_y=self.dis_pix_y, fit_param=fit_param_BEV)
            # 输出该条车道线相对车辆的位置，暂时是输出五个点
            self.lane_positions.append(lane_position)

            [BEV_image_height, BEV_image_width] = list(BEV_img.shape)[:2]

            plot_y = np.linspace(10, BEV_image_height, BEV_image_height - 10)
            fit_x = fit_param_BEV[0] * plot_y ** 2 + fit_param_BEV[1] * plot_y + fit_param_BEV[2]

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

                # 作图范围，即使是鸟瞰图也没必要画出全部
                start_plot_y = 10
                end_plot_y = 720

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
                    cv2.circle(BEV_img, (int(interpolation_src_pt_x), int(interpolation_src_pt_y)), 4, lane_color, -1)

                    # cv2.imshow('BEV_img', BEV_img)
                    # cv2.waitKey(100)


        ret = {
            'source_image': source_image,
            'fit_params': self.fit_params_BEV,
            'BEV_image': BEV_img,
            'position_now': self.lane_positions
        }
        self.lane_coords = []
        self.fit_params_BEV = []
        self.BEV_lane_pts = []
        self.lane_positions = []

        self.pts_num_BEV = []
        self.param_2 = 0
        self.param_1 = 0
        return ret
