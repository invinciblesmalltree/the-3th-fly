#!/usr/bin/env python
#-*-coding:utf-8-*-
import rospy
import time
import cv2
import numpy as np
from pyzbar.pyzbar import decode
import pyrealsense2 as rs
from cv_bridge import CvBridge
from cv_detect.msg import BarMsg
from sensor_msgs.msg import Image

def decode_barcode(image):
    barcodes = decode(image)
    if len(barcodes) == 0:
        return None
    barcode_data = barcodes[0].data.decode('utf-8')
    return int(barcode_data)

def d435_2cv2():
    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_stream(rs.stream.color, 1280, 720, rs.format.bgr8, 30)
    config.enable_stream(rs.stream.depth, 1280, 720, rs.format.z16, 30)
    pipeline.start(config)
    while True:
        frames = pipeline.wait_for_frames()
        color_frame = frames.get_color_frame()
        depth_frame = frames.get_depth_frame()

        if color_frame is not None and depth_frame is not None:
            color_image = np.asanyarray(color_frame.get_data())
            depth_image = np.asanyarray(depth_frame.get_data())

            return color_image, depth_image
        
        rospy.sleep(0.1)

def main():
    rospy.init_node('barcode_node', anonymous=True)
    pub = rospy.Publisher('barcode_msg', BarMsg, queue_size=10)
    rate = rospy.Rate(20)

    while True:
        frame, depth_image = d435_2cv2()
        if frame is not None:
            height, width = frame.shape[:2]

            bar_msg = BarMsg()
            ret = decode_barcode(frame)
            if ret is None or ret < 1 or ret > 9:
                bar_msg.n = -1
            else:
                bar_msg.n= ret
                rospy.loginfo('Barcode: %s', bar_msg.n)

            pub.publish(bar_msg)

        rate.sleep()

if __name__ == '__main__':
    main()