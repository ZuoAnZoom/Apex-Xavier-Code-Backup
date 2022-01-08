from numpy import *
import numpy as np
import math
import random
from scipy.spatial import KDTree


class visitlist:
    """
        visitlist类用于记录访问列表
        unvisitedlist记录未访问过的点
        visitedlist记录已访问过的点
        unvisitednum记录访问过的点数量
    """
    def __init__(self, count=0):
        self.unvisitedlist=[i for i in range(count)]
        self.visitedlist=list()
        self.unvisitednum=count

    def visit(self, pointId):
        self.visitedlist.append(pointId)
        self.unvisitedlist.remove(pointId)
        self.unvisitednum -= 1


def dist(a, b):
    """
    @brief      计算a,b两个元组的距离，我想加重x方向的比例
    @param      a，点的坐标(y, x)
    @param      b
    @return     距离
    """
    y1, x1, y2, x2 = a[0], a[1], b[0], b[1]
    return math.sqrt(1.5 * np.power((x1 - x2), 2) + 0.5 * np.power((y1 - y2), 2))


def dbscan_simple(dataSet, eps, minPts):
    """
    @brief      简易DBScan算法
    @param      dataSet  输入数据集，numpy格式
    @param      eps      最短距离
    @param      minPts   最小簇点数
    @return     分类标签
    """
    nPoints = dataSet.shape[0]
    vPoints = visitlist(count=nPoints)
    # 初始化簇标记列表C,簇标记为 k
    k = -1
    C = [-1 for i in range(nPoints)]
    while(vPoints.unvisitednum > 0):
        p = random.choice(vPoints.unvisitedlist)
        vPoints.visit(p)
        N = [i for i in range(nPoints) if dist(dataSet[i], dataSet[p]) <= eps]
        if len(N) >= minPts:
            k += 1
            C[p]=k
            for p1 in N:
                if p1 in vPoints.unvisitedlist:
                    vPoints.visit(p1)
                    M = [i for i in range(nPoints) if dist(dataSet[i], dataSet[p1]) <= eps]
                    if len(M) >= minPts:
                        for i in M:
                            if i not in N:
                                N.append(i)
                    if C[p1] == -1:
                        C[p1]= k
        else:
            C[p] = -1
    return C


def dbscan(dataSet, eps, minPts):
    """
    @brief      基于kd-tree的DBScan算法
    @param      dataSet  输入数据集，numpy格式
    @param      eps      最短距离
    @param      minPts   最小簇点数
    @return     分类标签
    """
    nPoints = dataSet.shape[0]
    vPoints = visitlist(count=nPoints)
    # 初始化簇标记列表C，簇标记为 k
    k = -1
    C = [-1 for i in range(nPoints)]
    # 构建KD-Tree，并生成所有距离<=eps的点集合
    kd = KDTree(dataSet)
    while(vPoints.unvisitednum>0):
        p = random.choice(vPoints.unvisitedlist)
        vPoints.visit(p)
        N = kd.query_ball_point(dataSet[p], eps)
        if len(N) >= minPts:
            k += 1
            C[p] = k
            for p1 in N:
                if p1 in vPoints.unvisitedlist:
                    vPoints.visit(p1)
                    M = kd.query_ball_point(dataSet[p1], eps)
                    if len(M) >= minPts:
                        for i in M:
                            if i not in N:
                                N.append(i)
                    if C[p1] == -1:
                        C[p1] = k
        else:
            C[p] = -1
    return C


def dbscan_lib(dataSet, eps, minPts):
    """
    @brief      利用sklearn包计算DNSCAN
    @param      dataSet  The data set
    @param      eps      The eps
    @param      minPts   The minimum points
    @return     { description_of_the_return_value }
    """
    from sklearn.cluster import DBSCAN
    C = DBSCAN(eps=eps, min_samples= minPts).fit_predict(dataSet)
    return C
