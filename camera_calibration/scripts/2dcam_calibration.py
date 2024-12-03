

import cv2
import numpy as np

import fitz  # PyMuPDF
import cv2.aruco as aruco
import rosbag
from cv_bridge import CvBridge

# 提取 PDF 页为图像
bag_path = '/home/mike/sick_Ws/src/test/bag/2d_3d_calibration_2.bag'  # 替换为你的rosbag文件路径
# topic_name = '/converted_image'  # 替换为你的图像话题名称
topic_name = '/ifm3d_ros_driver/camera_3d/amplitude'

# 创建CvBridge对象
bridge = CvBridge()

# 打开rosbag文件
with rosbag.Bag(bag_path, 'r') as bag:
    for topic, msg, t in bag.read_messages(topics=[topic_name]):
        # 将sensor_msgs/Image消息转换为OpenCV格式
        image = bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        
        # 显示图像或处理图像
        cv2.imshow('Extracted Image', image)
        cv2.waitKey(0)  # 按任意键继续
        
        # 如果需要保存图像
        cv2.imwrite('extracted_image.png', image)
        
        break  # 如果只想提取第一帧图像，使用break

cv2.destroyAllWindows()

# 加载Aruco字典
aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_6X6_1000)
parameters = aruco.DetectorParameters()

gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

# 读取图像
# image = cv2.imread('aruco_image.jpg')
# gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

# 检测Aruco标记
corners, ids, _ = cv2.aruco.detectMarkers(gray, aruco_dict, parameters=parameters)
print(ids)

# 相机内参矩阵和畸变系数 (假设已标定)
fx = 1000.0
fy = 1000.0
cx = 320.0
cy = 240.0
K = np.array([[fx, 0, cx],
              [0, fy, cy],
              [0, 0, 1]], dtype=np.float32)  # 确保是 float32 类型
distCoeffs = np.array([0, 0, 0, 0, 0], dtype=np.float32)

# 假设只使用一个Aruco码
if ids is not None and len(ids) == 1:
    # 估计Aruco标记的姿态
    rvec, tvec, _ = cv2.aruco.estimatePoseSingleMarkers(corners, 0.1, K, distCoeffs)
    
    # 在Aruco码平面上定义其他3D点（假设平面上Z=0）
    object_points = np.array([
        [0.0, 0.0, 0],  # Aruco码平面上的其他点
        [0.0, -0.2, 0],
        [0.1, 0.0, 0],
    ], dtype=np.float32)
    
    # 将这些点投影到图像平面
    image_points, _ = cv2.projectPoints(object_points, rvec[0], tvec[0], K, distCoeffs)
    
    # 绘制投影点
    for point in image_points:
        x, y = point.ravel()
        cv2.circle(image, (int(x), int(y)), 5, (0, 0, 255), -1)
    
    # 显示结果
    cv2.imshow('Projected Points', image)
    cv2.waitKey(0)
    cv2.destroyAllWindows()
else:
    print("未检测到单个Aruco标记")

