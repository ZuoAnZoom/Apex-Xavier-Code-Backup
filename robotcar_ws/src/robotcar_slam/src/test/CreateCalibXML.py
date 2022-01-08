import numpy as np
# from mpl_toolkits import mplot3d
# import matplotlib.pyplot as plt

# esti_solo = np.genfromtxt('esti_result.txt', delimiter=' ')
# esti_none = np.genfromtxt('esti_result2.txt', delimiter=' ')
# truth = np.genfromtxt('truth_result.txt', delimiter=' ')

# esti_solo = esti_solo[:, 1:4]
# esti_none = esti_none[:, 1:4]
# truth = truth[:, 1:4]


# ax = plt.axes(projection='3d')
# ax.plot(esti_solo[:, 0], esti_solo[:, 1], esti_solo[:, 2], 'red')
# ax.plot(esti_none[:, 0], esti_none[:, 1], esti_none[:, 2], 'blue')
# ax.plot(truth[:, 0], truth[:, 1], truth[:, 2], 'black')
# ax.legend(['ON', 'OFF', 'Truth'])
# ax.set_xlabel('X/m')
# ax.set_ylabel('Y/m')
# ax.set_zlabel('Z/m')

# # plt.rcParams['font.sans-serif'] = ['SimHei'] # 指定默认字体
# plt.show()

f = open("test/calib.xml", "w")
f.write("<?xml version=\"1.0\"?>\n")
f.write("<opencv_storage>\n")
f.write("<imagelist>\n")

string = "\"/home/lihua/Document/实测数据集/标定/201215/"
for i in range(0, 438):
    f.write(string+"left/"+str(i)+".png\"\n")
    f.write(string+"right/"+str(i)+".png\"\n")

f.write("</imagelist>\n")
f.write("</opencv_storage>\n")

# f = open("/home/lihua/Document/实测数据集/201214小广场/association/associate.txt", "w")
# for i in range(0, 929):
#     f.write(str(i) + " 1/" + str(i) + ".png" + " 2/" + str(i) + ".png\n")
# f.close()
    