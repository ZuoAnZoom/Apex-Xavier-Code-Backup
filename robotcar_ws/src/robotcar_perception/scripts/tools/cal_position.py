import numpy as np
from numba import jit
from scipy.optimize import leastsq
from tools import IPM_Image_Computation


# def cal_key_params():
#     """
#     计算相机相对车道线的位置
#     """
#
#     # 首先计算纵向上的像素与实际距离的关系
#
#     # 最小二乘
#     # 样本数据(Xi,Yi)，需要转换成数组(列表)形式
#     # 仿真环境
#     X_i = np.array([4.18, 8.97, 13.97, 18.97])
#     Y_i = np.array([699, 566, 430, 290])
#
#     # 需要拟合的函数func：指定函数形状
#     def func(p, x):
#         a, b = p
#         return a * x + b
#
#     # 偏差函数：x,y都是列表，这里的x,y和上面的Xi,Yi是一一对应的
#     def error(p, x, y):
#         return func(p, x) - y
#
#     '''
#       主要部分：附带部分说明
#       1.leastsq函数的返回值tuple，第一个元素是求解结果，第二个是求解的代价值(个人理解)
#       2.官网的原话（第二个值）：Value of the cost function at the solution
#       3.实例：Para=>(array([ 0.61349535, 1.79409255]), 3)
#       4.返回值元组中第一个值的数量跟需要求解的参数的数量一致
#     '''
#     # k,b的初始值，可以任意设定,经过几次试验，发现p0的值会影响cost的值：Para[1]
#     p0 = np.array([1, 20])
#     # 把error函数中除了p0以外的参数打包到args中(使用要求)
#     Para = leastsq(error, p0, args=(X_i, Y_i))
#
#     # 读取结果
#     a, b = Para[0]
#
#     # 然后计算与车辆纵向距离5,6,7,8,9,10米的位置，对应的图像像素
#     # 像素值非整数，但不需要转换成整数
#     dis_real_zong = np.array([5, 6, 7, 8, 9, 10])
#     dis_pix_y = a * dis_real_zong + b
#
#     return dis_pix_y
#
#
# def related_pos(dis_pix_y, fit_param):
#     dst_real = []
#     dis_real_zong = np.array([5, 6, 7, 8, 9, 10, 11])
#
#     # 然后根据拟合结果计算横向像素坐标
#     # 再转换成世界坐标系的横向距离，这里就不需要最小二乘了
#     dis_pix_x = fit_param[0] * dis_pix_y ** 2 + fit_param[1] * dis_pix_y + fit_param[2]
#     # 仿真环境
#     dis_real_heng = (dis_pix_x - 256) * 3 / 120
#     for index in range(len(dis_real_zong)):
#         dst_real.append([dis_real_heng[index], dis_real_zong[index]])
#
#     return dst_real


@jit(nopython=True)
def related_pos(H, center, pixel, f_x, f_y):
    """
    上面是测量几组俯视图中纵向的像素与真实距离的关系，计算相关参数
    在假设相机水平，路面平坦的情况下，这个关系是恒定的且成正比，显然测算比例系数并不准确
    根据成像的相似三角形就能直接计算距离，当然同样假设相机水平，路面平坦
    暂未考虑相机有俯仰角时的成像关系
    :param H: 相机高度
    :param center: 成像中心
    :param pixel: 待求解的像素坐标
    :param f_x: x与y应该是一样的，经过去畸变，比例转换后，f_x与f_y确实相差很大
    :param f_y: 暂时就用两个，之后还得请教
    :return:
    """
    # 先计算深度
    # 应该是D+f，但焦距是毫米，忽略不计
    D = (H / (pixel[1] - center[1])) * f_y

    # 再计算横向距离
    L = (D / f_x) * (pixel[0] - center[0])

    return D, L



if __name__ == '__main__':
    # dis_pix_y = cal_key_params()
    # print(dis_pix_y)
    # 512,256的图上，在一条车道线上找点
    # D, L = related_pos(1.05, (241.27, 106.74), (505, 243), 455.92, 451.04)  # 3.48,2.01
    # D, L = related_pos(1.05, (241.27, 106.74), (428, 208), 455.92, 451.04)  # 4.68,1.92
    # D, L = related_pos(1.05, (241.27, 106.74), (365, 181), 455.92, 451.04)  # 6.38,1.73
    # D, L = related_pos(1.05, (241.27, 106.74), (325, 161), 455.92, 451.04)  # 8.73,1.60

    # 投回到1280,720的图
    # D, L = related_pos(1.05, (603.17, 300.22), (1275, 675), 1139.80, 1268.55)

    # 测量距离，5.20
    # D, L = related_pos(1.05, (241.27, 106.74), (260, 203), 455.92, 451.04)  # 4.92,0.20
    # 5.32
    # D, L = related_pos(1.05, (241.27, 106.74), (260, 200), 455.92, 451.04)  # 5.08,0.21
    # 5.79
    D, L = related_pos(1.05, (241.27, 106.74), (260, 194), 455.92, 451.04)  # 5.43,0.22



    # 仿真环境，计算结果应该是对的，那么关键就在于相机内参
    # D, L = related_pos(1.165, (640.00, 360.00), (1176, 711), 1312.96, 1312.96)

    print(D, L)