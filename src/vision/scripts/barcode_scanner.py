#!/usr/bin/env python

import rospy
from pyzbar.pyzbar import decode
from sensor_msgs.msg import Image
from std_msgs.msg import Int32
from cv_bridge import CvBridge, CvBridgeError
import cv2 as cv
import numpy as np


def replace_brown_with_white(image):
    hsv = cv.cvtColor(image, cv.COLOR_BGR2HSV)
    lower_brown = np.array([5, 30, 80])
    upper_brown = np.array([25, 255, 200])
    mask = cv.inRange(hsv, lower_brown, upper_brown)
    image[mask > 0] = (255, 255, 255)
    return image


def binarize_image(image, threshold_value=127):
    gray_image = cv.cvtColor(image, cv.COLOR_BGR2GRAY)
    _, binary_image = cv.threshold(gray_image, threshold_value, 255, cv.THRESH_BINARY)
    return binary_image


def callback(data):
    global frame
    try:
        frame = bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
        rospy.logerr(e)
    frame = replace_brown_with_white(frame)
    frame = binarize_image(frame)
    try:
        barcodes = decode(frame)
    except Exception as e:
        rospy.logerr(f"Barcode decoding failed: {e}")
        pub.publish(-1)
        return
    # cv.imshow("Barcode", frame)
    # cv.waitKey(1)
    if barcodes:
        barcode_data = barcodes[0].data.decode("utf-8")
        if not barcode_data.isdigit():
            pub.publish(-1)
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
