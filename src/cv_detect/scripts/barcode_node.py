#!/usr/bin/env python
#-*-coding:utf-8-*-
import rospy
import cv2
import numpy as np
from pyzbar.pyzbar import decode
import pyrealsense2 as rs
from cv_detect.msg import BarMsg

def decode_barcode(image):
    barcodes = decode(image)
    if len(barcodes) == 0:
        return None
    barcode_data = barcodes[0].data.decode('utf-8')
    return int(barcode_data)

def main():
    rospy.init_node('barcode_node', anonymous=True)
    pub = rospy.Publisher('barcode_msg', BarMsg, queue_size=10)
    rate = rospy.Rate(20)

    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_stream(rs.stream.color, 1280, 720, rs.format.bgr8, 30)
    config.enable_stream(rs.stream.depth, 1280, 720, rs.format.z16, 30)
    pipeline.start(config)

    try:
        while not rospy.is_shutdown():
            frames = pipeline.wait_for_frames()
            color_frame = frames.get_color_frame()
            depth_frame = frames.get_depth_frame()

            if color_frame is not None and depth_frame is not None:
                color_image = np.asanyarray(color_frame.get_data())
                depth_image = np.asanyarray(depth_frame.get_data())
            
                frame = color_image
                bar_msg = BarMsg()
                ret = decode_barcode(frame)
                if ret is None:
                    bar_msg.n = -1
                else:
                    bar_msg.n= ret
                    rospy.loginfo('Barcode: %s', bar_msg.n)

                pub.publish(bar_msg)
            rate.sleep()
            
    except Exception as e:
        rospy.logerr('Error: %s', e)

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass