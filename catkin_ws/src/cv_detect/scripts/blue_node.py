#!/usr/bin/env python
#-*-coding:utf-8-*-
import rospy
import cv2
import numpy as np
import Jetson.GPIO as GPIO
import time
from cv_detect.msg import LedMsg

def detect_blue_objects(image):
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

    # 定义蓝色的HSV范围
    lower_blue = np.array([100, 43, 60])
    upper_blue = np.array([124, 255, 255])

    mask1 = cv2.inRange(hsv, lower_blue, upper_blue)

    # 提取蓝色区域（在RGB空间中）
    lower_blue = np.array([100, 0, 0])  # 更低的红色和绿色，更高的蓝色分量
    upper_blue = np.array([255, 80, 80])

    # 提取蓝色区域
    mask2 = cv2.inRange(image, lower_blue, upper_blue)
    mask = cv2.bitwise_and(mask1, mask2)

    contours, _ = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    for contour in contours:
        area = cv2.contourArea(contour)
        approx = cv2.approxPolyDP(contour, 0.04 * cv2.arcLength(contour, True), True)

        if len(approx) == 4 and area > 100:
            # 找到图形轮廓中心坐标
            M = cv2.moments(contour)
            delta_x = int(M['m10'] / M['m00']-width/2)
            delta_y = -int(M['m01'] / M['m00']-height/2)
            x, y, w, h = cv2.boundingRect(approx)
            cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 3)

            return delta_x, delta_y

    return None

def normal_blink(times):
    GPIO.setmode(GPIO.BOARD)  

    LED_PIN = 11
    GPIO.setup(LED_PIN, GPIO.OUT)

    for _ in range(times):
        GPIO.output(LED_PIN, GPIO.HIGH)
        time.sleep(0.5)
        GPIO.output(LED_PIN, GPIO.LOW)
        time.sleep(0.5)

# 初始化节点
rospy.init_node('blue_node', anonymous=True)

pub = rospy.Publisher('blue_msg', LedMsg, queue_size=10)
rate = rospy.Rate(20)

# cv识别程序主体
capture = cv2.VideoCapture('/dev/ground')

while(1):
    if capture.isOpened():
        open, frame = capture.read()
        height, width = frame.shape[:2]
        frame = frame[height // 2: height, 0: width]

        width = int(capture.get(cv2.CAP_PROP_FRAME_WIDTH))
        height = int(capture.get(cv2.CAP_PROP_FRAME_HEIGHT))

        led_msg = LedMsg()

        delta = detect_blue_objects(frame)

        cv2.imshow('frame', frame)
        cv2.waitKey(1)

        if delta is None:
            led_msg.value = False
        else:
            led_msg.value = True
            led_msg.delta_x, led_msg.delta_y= delta

        pub.publish(led_msg)

    rate.sleep()

capture.release()
GPIO.cleanup()