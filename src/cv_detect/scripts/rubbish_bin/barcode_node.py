#!/usr/bin/env python
#-*-coding:utf-8-*-
import rospy
import cv2
import numpy as np
from pyzbar.pyzbar import decode
import time
from cv_bridge import CvBridge
from cv_detect.msg import BarMsg
from sensor_msgs.msg import Image

def decode_barcode(image):
    barcodes = decode(image)
    if len(barcodes) == 0:
        return None
    barcode_data = barcodes[0].data.decode('utf-8')
    return int(barcode_data)

global frame
frame = None
def callback(data, bridge=CvBridge()):
    global frame
    if data is not None:
        frame = bridge.imgmsg_to_cv2(data, "bgr8")

def main():
    rospy.init_node('barcode_node', anonymous=True)
    rgb_sub = rospy.Subscriber('/d435/rgb', Image, callback)
    pub = rospy.Publisher('barcode_msg', BarMsg, queue_size=10)
    rate = rospy.Rate(20)

    while(1):
        if frame is not None:
            height, width = frame.shape[:2]

            bar_msg = BarMsg()
            ret = decode_barcode(frame)
            if ret is None:
                bar_msg.n = -1
            else:
                if ret < 1 or ret > 9:
                    rate.sleep()
                    continue
                bar_msg.n= ret
                rospy.loginfo('Barcode: %s', bar_msg.n)

            pub.publish(bar_msg)

        rate.sleep()

if __name__ == '__main__':
    main()