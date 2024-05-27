#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image
from vision.msg import box_data
import cv2
from cv_bridge import CvBridge, CvBridgeError
import numpy as np

bridge = CvBridge()

# 扩大棕色的HSV范围
lower_brown = np.array([5, 50, 50])
upper_brown = np.array([25, 255, 255])

# 假设的面积阈值
area_threshold = 1000


def image_callback(data):
    try:
        raw_image = bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
        rospy.logerr(e)
        return

    cv_image = raw_image.copy()

    # 转换为HSV色彩空间
    hsv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

    # 创建棕色掩膜
    mask = cv2.inRange(hsv_image, lower_brown, upper_brown)

    # 找到轮廓
    contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

    img_center_x = cv_image.shape[1] // 2
    img_center_y = cv_image.shape[0] // 2

    closest_contour = None
    min_distance = float("inf")

    for contour in contours:
        M = cv2.moments(contour)
        if M["m00"] == 0:
            continue
        cX = int(M["m10"] / M["m00"])
        cY = int(M["m01"] / M["m00"])
        distance = ((cX - img_center_x) ** 2 + (cY - img_center_y) ** 2) ** 0.5

        if (
            min_distance == float("inf") or cv2.contourArea(contour) > 10000
        ) and distance < min_distance:
            min_distance = distance
            closest_contour = contour

    box_msg = box_data()

    if closest_contour is not None:
        M = cv2.moments(closest_contour)
        cX = int(M["m10"] / M["m00"])
        cY = int(M["m01"] / M["m00"])
        area = cv2.contourArea(closest_contour)
        if area > 30000:
            box_msg.class_id = 0  # 面积大于阈值
        elif area > 10000:
            box_msg.class_id = 1  # 面积小于阈值
        else:
            box_msg.class_id = -1
            box_msg.x = 0
            box_msg.y = 0
        if area > 10000:
            box_msg.x = int(cX - img_center_x)
            box_msg.y = int(cY - img_center_y)
        cv2.drawContours(
            raw_image,
            [closest_contour],
            -1,
            (0, 255, 0),
            2,
        )
        cv2.putText(
            raw_image,
            f"box_{box_msg.class_id} {area}",
            (cX, cY),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.9,
            (36, 255, 12),
            2,
        )
    else:
        box_msg.class_id = -1
        box_msg.x = 0
        box_msg.y = 0

    box_pub.publish(box_msg)

    try:
        img_msg = bridge.cv2_to_imgmsg(raw_image, "bgr8")
        image_pub.publish(img_msg)
    except CvBridgeError as e:
        rospy.logerr(e)


rospy.init_node("box_detect", anonymous=True)
image_pub = rospy.Publisher("/yolov5/img", Image, queue_size=10)
box_pub = rospy.Publisher("/yolov5/box_detect", box_data, queue_size=10)
image_sub = rospy.Subscriber("/camera/ground", Image, image_callback)
rospy.spin()
