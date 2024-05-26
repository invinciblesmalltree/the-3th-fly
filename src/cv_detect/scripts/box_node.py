#!/usr/bin/env python
#-*-coding:utf-8-*-
import rospy
import cv2
import numpy as np
import time
from cv_bridge import CvBridge
from cv_detect.msg import BoxMsg
from sensor_msgs.msg import Image

def detect_box(image, width, height):
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

    lower_brown = np.array([20, 30, 50])  # 较深的棕色
    upper_brown = np.array([40, 80, 200])  # 较浅的棕色

    mask = cv2.inRange(hsv, lower_brown, upper_brown)

    contours, _ = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    for contour in contours:
        area = cv2.contourArea(contour)
        approx = cv2.approxPolyDP(contour, 0.04 * cv2.arcLength(contour, True), True)

        if len(approx) == 4 and area > 10000:
            # 找到图形轮廓中心坐标
            M = cv2.moments(contour)
            delta_x = int(M['m10'] / M['m00']-width/2)
            delta_y = -int(M['m01'] / M['m00']-height/2)
            x, y, w, h = cv2.boundingRect(approx)
            cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 3)

            return delta_x, delta_y

    return None

global frame
frame = None
def callback(data, bridge=CvBridge()):
    global frame
    if data is not None:
        frame = bridge.imgmsg_to_cv2(data, "bgr8")

def main():
    # 初始化节点
    rospy.init_node('box_node', anonymous=True)

    camera_sub = rospy.Subscriber('/camera/ground', Image, callback)
    pub = rospy.Publisher('box_msg', BoxMsg, queue_size=10)
    rate = rospy.Rate(20)

    while(1):
        if frame is not None:
            frame = cv2.copyMakeBorder(frame, 10, 10, 10, 10, cv2.BORDER_CONSTANT, value=[255,255,255])
            height, width = frame.shape[:2]
            box_msg = BoxMsg()
            delta = detect_box(frame, width, height)

            if delta is None:
                box_msg.value = False
            else:
                box_msg.value = True
                box_msg.delta_x, box_msg.delta_y= delta

            pub.publish(box_msg)

        rate.sleep()

def callback(data):
    global frame
    if data is not None:
        frame = bridge.imgmsg_to_cv2(data, "bgr8")

if __name__ == '__main__':
    main()