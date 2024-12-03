#!/usr/bin/env python
import rospy
from sensor_msgs.msg import CompressedImage, Image
from cv_bridge import CvBridge

def callback(compressed_msg):
    # 使用cv_bridge将CompressedImage转换为Image
    bridge = CvBridge()
    image_msg = bridge.compressed_imgmsg_to_cv2(compressed_msg, desired_encoding="bgr8")
    image_pub.publish(bridge.cv2_to_imgmsg(image_msg, "bgr8"))

rospy.init_node('compressed_image_converter', anonymous=True)
image_pub = rospy.Publisher('/converted_image', Image, queue_size=1)
rospy.Subscriber('/ifm3d_ros_driver/camera_2d/rgb_image/compressed', CompressedImage, callback)
rospy.spin()
