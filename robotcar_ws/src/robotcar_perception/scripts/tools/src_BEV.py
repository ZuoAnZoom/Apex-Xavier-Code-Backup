import cv2
import numpy as np

def get_M_Minv():
    """
    计算仿射变换矩阵
    """
    src = np.float32([[(4, 570), (476, 410), (765, 410), (1256, 570)]])
    dst = np.float32([[(340, 620), (340, 0), (940, 0), (940, 620)]])
    M = cv2.getPerspectiveTransform(src, dst)
    # Minv = cv2.getPerspectiveTransform(dst, src)
    return M

if __name__ == "__main__":
    M = get_M_Minv()
    print(M)