#!/usr/bin/env python
#-*-coding:utf-8-*-
import rospy
import cv2
import numpy as np
import time
from cv_detect.msg import BoxMsg

def detect_box(image, width, height):
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

    lower_brown = np.array([10, 100, 20])  # 深棕色范围
    upper_brown = np.array([20, 255, 200])  # 深棕色范围

    mask = cv2.inRange(hsv, lower_brown, upper_brown)

    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    image_copy = image.copy()
    for contour in contours:
        area = cv2.contourArea(contour)
        approx = cv2.approxPolyDP(contour, 0.04 * cv2.arcLength(contour, True), True)

        if (len(approx) == 4 or len(approx) == 5) and area > 7000:
            M = cv2.moments(contour)
            delta_x = int(M['m10'] / M['m00']-width/2)
            delta_y = -int(M['m01'] / M['m00']-height/2)
            x, y, w, h = cv2.boundingRect(approx)
            # cv2.rectangle(image_copy, (x, y), (x + w, y + h), (0, 255, 0), 3)

            return (delta_x, delta_y), image_copy

    return None, None

def main():
    rospy.init_node('box_node', anonymous=True)
    pub = rospy.Publisher('box_msg', BoxMsg, queue_size=10)
    rate = rospy.Rate(20)

    capture = cv2.VideoCapture("/dev/ground")
    capture.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    capture.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

    while not rospy.is_shutdown():
        if capture.isOpened():
            while True:
                ret, frame = capture.read()
                if ret:
                    x1,x2=160,480
                    y1,y2=120,360
                    frame=frame[y1:y2,x1:x2]
                    frame = cv2.copyMakeBorder(frame, 10, 10, 10, 10, cv2.BORDER_CONSTANT, value=[255, 255, 255])
                    height, width = frame.shape[:2]
                    box_msg = BoxMsg()
                    delta, _ = detect_box(frame, width, height)

                    if delta is None:
                        box_msg.value = False
                        box_msg.delta_x, box_msg.delta_y= 0,0
                    else:
                        box_msg.value = True
                        box_msg.delta_x, box_msg.delta_y= delta

                    pub.publish(box_msg)

                rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass