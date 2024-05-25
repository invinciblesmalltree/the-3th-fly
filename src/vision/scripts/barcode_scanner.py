#!/usr/bin/env python

import rospy
from pyzbar.pyzbar import decode
from sensor_msgs.msg import Image
from std_msgs.msg import Int32
from cv_bridge import CvBridge, CvBridgeError
import cv2 as cv
import numpy as np


def increase_brightness(image, value=30):
    hsv = cv.cvtColor(image, cv.COLOR_BGR2HSV)
    h, s, v = cv.split(hsv)
    v = cv.add(v, value)
    v = np.clip(v, 0, 255)
    final_hsv = cv.merge((h, s, v))
    brightened_image = cv.cvtColor(final_hsv, cv.COLOR_HSV2BGR)
    return brightened_image


def replace_brown_with_white(image):
    hsv = cv.cvtColor(image, cv.COLOR_BGR2HSV)
    lower_brown = np.array([10, 100, 20])
    upper_brown = np.array([20, 255, 200])
    mask = cv.inRange(hsv, lower_brown, upper_brown)
    image[mask > 0] = (255, 255, 255)
    return image


def binarize_image(image, threshold_value=127):
    _, binary_image = cv.threshold(image, threshold_value, 255, cv.THRESH_BINARY)
    return binary_image


def callback(data):
    global frame
    try:
        frame = bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
        rospy.logerr(e)
    frame = increase_brightness(frame, 100)
    frame = replace_brown_with_white(frame)
    frame = binarize_image(frame)
    barcodes = decode(frame)
    if barcodes:
        barcode_data = barcodes[0].data.decode("utf-8")
        if not barcode_data.isdigit():
            return
        barcode_data = int(barcode_data)
        if 1 <= barcode_data <= 9:
            pub.publish(barcode_data)
            return
    pub.publish(-1)


rospy.init_node("barcode", anonymous=True)
rgb_sub = rospy.Subscriber("/d435/rgb", Image, callback)
pub = rospy.Publisher("barcode_data", Int32, queue_size=10)

bridge = CvBridge()
frame = None
rospy.spin()
