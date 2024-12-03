

import cv2
import numpy as np

# import fitz  # PyMuPDF
import cv2.aruco as aruco
import rosbag
from cv_bridge import CvBridge
import open3d as o3d
import casadi as ca
import math

def random_orthogonal_matrix():
    Q, _ = np.linalg.qr(np.random.rand(3, 3))  # QR分解生成正交矩阵
    return Q

def rotation_matrices(roll, pitch, yaw):
    # 滚转（Roll）矩阵
    R_x = np.array([[1.0, 0.0, 0.0],
                    [0.0, math.cos(roll), -math.sin(roll)],
                    [0.0, math.sin(roll), math.cos(roll)]], dtype=float)
    
    # 俯仰（Pitch）矩阵
    R_y = np.array([[math.cos(pitch), 0.0, math.sin(pitch)],
                    [0.0, 1.0, 0.0],
                    [-math.sin(pitch), 0.0, math.cos(pitch)]], dtype=float)
    
    # 偏航（Yaw）矩阵
    R_z = np.array([[math.cos(yaw), -math.sin(yaw), 0.0],
                    [math.sin(yaw), math.cos(yaw), 0.0],
                    [0.0, 0.0, 1.0]], dtype=float)

    # 组合旋转矩阵
    Rot = np.dot(R_z, np.dot(R_y, R_x))
    return Rot

def multi_start_optimization(solver, num_trials=10):
    best_solution = None
    best_objective = float('inf')
    
    for _ in range(num_trials):
        # 随机初始化
        initial_guess = np.random.uniform(-1.6, 1.6, 3)  # 这里范围根据问题调整
        
        # 求解问题
        # solution = solver(x0=initial_guess)

        lbx = [-ca.pi, -ca.inf, -ca.inf]  # 对变量的下界，第 2 个变量设置下界为 0，第 3 个为 -5
        ubx = [ca.pi, ca.inf, ca.inf] 
        solution = solver(x0=initial_guess, lbx=lbx, ubx=ubx)
        optimal_d = solution['x']
        optimal_obj = solution['f']
        
        # 记录最优解
        if optimal_obj < best_objective:
            best_solution = optimal_d
            best_objective = optimal_obj
    
    return best_solution, best_objective

# 提取 PDF 页为图像
bag_path = '/home/mike/sick_Ws/src/test/bag/2d_3d_calibration_1.bag'  # 替换为你的rosbag文件路径
topic_name = '/converted_image'  # 替换为你的图像话题名称
# topic_name = '/ifm3d_ros_driver/camera_3d/amplitude'

# 创建CvBridge对象
bridge = CvBridge()

# 打开rosbag文件
with rosbag.Bag(bag_path, 'r') as bag:
    for topic, msg, t in bag.read_messages(topics=[topic_name]):
        # # 将sensor_msgs/Image消息转换为OpenCV格式
        # print(f"Amplitude image dtype: {msg.dtype}")
        # print(f"Amplitude image range: {np.min(msg)}, {np.max(msg)}")

        image = bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        
        # 显示图像或处理图像
        # cv2.imshow('Extracted Image', image)
        # cv2.waitKey(0)  # 按任意键继续
        
        # 如果需要保存图像
        cv2.imwrite('extracted_image.png', image)
        # amplitude_image = bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")

        # # 检查图像信息
        # print(f"Amplitude image dtype: {amplitude_image.dtype}")
        # print(f"Amplitude image shape: {amplitude_image.shape}")

        # normalized_image = cv2.normalize(amplitude_image, None, 0, 255, cv2.NORM_MINMAX)
        # image = np.uint8(normalized_image)

        # cv2.imshow("Gray Image", image)
        # cv2.waitKey(1)
        break  # 如果只想提取第一帧图像，使用break

cv2.destroyAllWindows()

# 加载Aruco字典
aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_6X6_1000)
# parameters = aruco.DetectorParameters()
parameters = aruco.DetectorParameters_create()

# print(ids)

# 相机内参矩阵和畸变系数 (假设已标定)
# fx = 181.14419555 #563.06
# fy = 181.14419555 #563.06
# cx = 111.22945 #654.992
# cy = 89.1921997 #395.85657
# K = np.array([[fx, 0, cx],
#               [0, fy, cy],
#               [0, 0, 1]], dtype=np.float32)  # 确保是 float32 类型
# # distCoeffs = np.array([-0.011449999, 0.0012159999, -0.000514000013936311, 0, 1.4299999], dtype=np.float32)
# distCoeffs = np.array([-0.0591869987, -0.0240529999, 0.0, 0, 0.30400699377], dtype=np.float32)
fx = 563.06
fy = 563.06
cx = 654.992
cy = 395.85657
K = np.array([[fx, 0, cx],
              [0, fy, cy],
              [0, 0, 1]], dtype=np.float32)  # 确保是 float32 类型
distCoeffs = np.array([-0.011449999, 0.0012159999, -0.000514000013936311, 0, 1.4299999], dtype=np.float32)
distCoeffs = np.array([0.0, 0.0, -0.0, 0, 0.0], dtype=np.float32)

# h, w = image.shape[:2]
# new_K, roi = cv2.getOptimalNewCameraMatrix(K, distCoeffs, (w, h), 1, (w, h))

# 使用undistort函数
# image = cv2.undistort(image, K, distCoeffs)

gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
# gray = image

# 读取图像
# image = cv2.imread('aruco_image.jpg')
# gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

# 检测Aruco标记
corners, ids, _ = cv2.aruco.detectMarkers(gray, aruco_dict, parameters=parameters)

pcd = o3d.io.read_point_cloud("/home/mike/sick_Ws/circle_3.pcd")

    # 提取 x, y, z 坐标
points = np.asarray(pcd.points)

object_points = np.mean(points, axis=0)

# print(object_points)
object_points_p = np.asarray(object_points, dtype=np.float32)

projected_points, _ = cv2.projectPoints(object_points_p, np.zeros((3, 1)), np.zeros((3, 1)), K, distCoeffs)
projected_points = np.asarray(projected_points, dtype=np.float32)

# fx = 181.14419555 #563.06
# fy = 181.14419555 #563.06
# cx = 111.22945 #654.992
# cy = 89.1921997 #395.85657
# K = np.array([[fx, 0, cx],
#               [0, fy, cy],
#               [0, 0, 1]], dtype=np.float32)  # 确保是 float32 类型
# # distCoeffs = np.array([-0.011449999, 0.0012159999, -0.000514000013936311, 0, 1.4299999], dtype=np.float32)
# distCoeffs = np.array([-0.0591869987, -0.0240529999, 0.0, 0, 0.30400699377], dtype=np.float32)

fx = 563.06
fy = 563.06
cx = 654.992
cy = 395.85657
K = np.array([[fx, 0, cx],
              [0, fy, cy],
              [0, 0, 1]], dtype=np.float32)  # 确保是 float32 类型
distCoeffs = np.array([-0.011449999, 0.0012159999, -0.000514000013936311, 0, 1.4299999], dtype=np.float32)
distCoeffs = np.array([-0.0591869987, -0.0240529999, 0.0, 0, 0.30400699377], dtype=np.float32)

# object_points[-1] = 0
# print("Number of points:", len(object_points))

# 假设只使用一个Aruco码
if ids is not None and len(ids) == 1:
    # 估计Aruco标记的姿态
    rvec, tvec, _ = cv2.aruco.estimatePoseSingleMarkers(corners, 0.174, K, distCoeffs)
    
    # 在Aruco码平面上定义其他3D点（假设平面上Z=0）
    object_points = np.array([
        [0.205, 0.0, 0.0]
        # [0.0, 0.0, 0],  # Aruco码平面上的其他点
        # [0.0, -0.5, 0],
        # [0.2, 0.0, 0],
    ], dtype=np.float32)
    
    # 将这些点投影到图像平面
    image_points, _ = cv2.projectPoints(object_points, rvec[0], tvec[0], K, distCoeffs)
    image_points = np.asarray(image_points,dtype=np.float32)
else:
    print("未检测到单个Aruco标记")

for point in image_points:
    x, y = point.ravel()
    cv2.circle(image, (int(x), int(y)), 5, (0, 0, 255), -1)
    # cv2.circle(image, (int(x), int(y)), 5, 200, -1)

for point in projected_points:
    x, y = point.ravel()
    cv2.circle(image, (int(x), int(y)), 5, (255, 0, 0), -1)
    # cv2.circle(image, (int(x), int(y)), 5, 100, -1)
    
    # 显示结果
cv2.imshow('Projected Points', image)
cv2.waitKey(0)
cv2.destroyAllWindows()

R = ca.MX.sym('R', 3)
# R_x = ca.vertcat(
#     ca.horzcat(1, 0, 0),
#     ca.horzcat(0, ca.cos(R[0]), -ca.sin(R[0])),
#     ca.horzcat(0, ca.sin(R[0]), ca.cos(R[0]))
# )

# # 绕Y轴的旋转矩阵（俯仰）
# R_y = ca.vertcat(
#     ca.horzcat(ca.cos(R[1]), 0, ca.sin(R[1])),
#     ca.horzcat(0, 1, 0),
#     ca.horzcat(-ca.sin(R[1]), 0, ca.cos(R[1]))
# )

# # 绕Z轴的旋转矩阵（偏航）
# R_z = ca.vertcat(
#     ca.horzcat(ca.cos(R[2]), -ca.sin(R[2]), 0),
#     ca.horzcat(ca.sin(R[2]), ca.cos(R[2]), 0),
#     ca.horzcat(0, 0, 1)
# )

Rot = ca.vertcat(
    ca.horzcat(ca.cos(R[0]), -ca.sin(R[0])),
    ca.horzcat(ca.sin(R[0]), ca.cos(R[0]))
)
# Rot = R_z @ R_y @ R_x

objective = 0

for i in range(projected_points.shape[0]):
    # print(image_points[i, :])
    objective += ca.norm_2(Rot @ image_points[i, :].T + R[1:] - projected_points[i,:].T)

nlp = {'x': ca.vec(R), 'f': objective}#, 'g': ca.vertcat(*constraints)}

solver = ca.nlpsol('solver', 'ipopt', nlp)

R_opt, optimized_objective_value = multi_start_optimization(solver)

print(R_opt)

print("优化后的目标值:", optimized_objective_value)

R_z = np.array([[math.cos(R_opt[0]), -math.sin(R_opt[0])],
                [math.sin(R_opt[0]), math.cos(R_opt[0])]], dtype=float)

dist = np.linalg.norm(R_z @ image_points[i, :].T + R_opt[1:] - projected_points[i,:].T)
print(dist)

print(projected_points)
print(image_points)
