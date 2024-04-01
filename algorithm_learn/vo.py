import cv2
import numpy as np

def compute_motion(matched_points1, matched_points2):
    # 计算基础矩阵
    F, mask = cv2.findFundamentalMat(matched_points1, matched_points2, cv2.FM_LMEDS)

    # 选择内点
    matched_points1 = matched_points1[mask.ravel() == 1]
    matched_points2 = matched_points2[mask.ravel() == 1]

    # 计算本质矩阵
    E = K.T @ F @ K  # 假设相机内参矩阵K已知

    # 从本质矩阵中恢复旋转和平移
    _, R, t, _ = cv2.recoverPose(E, matched_points1, matched_points2, K)

    return R, t

# 假设K是相机内参矩阵，您需要根据您的相机进行调整
K = np.array([[718.8560, 0, 607.1928],
              [0, 718.8560, 185.2157],
              [0, 0, 1]])

# 加载两个连续的图像帧
img1 = cv2.imread('1.png', 0)  # 确保这里的路径是正确的
img2 = cv2.imread('2.png', 0)

# 初始化ORB检测器
orb = cv2.ORB_create()

# 检测关键点和描述符
kp1, des1 = orb.detectAndCompute(img1, None)
kp2, des2 = orb.detectAndCompute(img2, None)

# 使用BF匹配器进行匹配
bf = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=True)
matches = bf.match(des1, des2)

# 根据匹配结果，提取匹配点对的位置
matched_points1 = np.float32([kp1[m.queryIdx].pt for m in matches])
matched_points2 = np.float32([kp2[m.trainIdx].pt for m in matches])

# 计算运动
R, t = compute_motion(matched_points1, matched_points2)

print("Estimated rotation:", R)
print("Estimated translation:", t)

