#!/usr/bin/env python
#-*-coding:utf-8-*-
import rospy
import cv2
import numpy as np
from pyzbar.pyzbar import decode
from PIL import Image
import Jetson.GPIO as GPIO
import time
from cv_detect.msg import BarMsg

def decode_barcode(image):
    barcodes = decode(image)
    if len(barcodes) == 0:
        return None
    barcode_data = barcodes[0].data.decode('utf-8')
    return int(barcode_data)

global frame
def callback(frame):
    frame = Image

# 初始化节点
rospy.init_node('barcode_node', anonymous=True)
rgb_sub = rospy.Subscriber('/d435/rgb', Image, callback)
pub = rospy.Publisher('barcode_msg', BarMsg, queue_size=10)
rate = rospy.Rate(20)

# cv识别程序主体
led_open = False # led连闪开关

while(1):
    if frame is not None:
        cv2.imshow('frame', frame)
        cv2.waitKey(1)

        height, width = frame.shape[:2]

        bar_msg = BarMsg()
        ret = decode_barcode(frame)
        if ret is None:
            bar_msg.n = -1
        else:
            if ret < 1 or ret > 9:
                rate.sleep()
                continue
            led_open = True
            bar_msg.n= ret
            rospy.loginfo('Barcode: %s', bar_msg.n)

        pub.publish(bar_msg)

    rate.sleep()