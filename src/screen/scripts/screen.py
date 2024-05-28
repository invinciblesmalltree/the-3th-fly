#!/usr/bin/env python3

import rospy
import serial
from std_msgs.msg import Int32

# 配置串口
ser = serial.Serial("/dev/ttyTHS0", baudrate=9600, timeout=1)

# 发布者
pub = rospy.Publisher("/offboard_order", Int32, queue_size=10)


def screen_data_callback(msg):
    command = f"box{msg.data}.pic=1".encode("utf-8") + b"\xff\xff\xff"
    ser.write(command)
    rospy.loginfo(f"Sent region {msg}")


rospy.init_node("screen", anonymous=True)

# 订阅者
rospy.Subscriber("/screen_data", Int32, screen_data_callback)

ser.write(b"rest\xff\xff\xff")

while not rospy.is_shutdown():
    if ser.in_waiting > 0:
        try:
            line = ser.readline().decode("utf-8").strip()
            if line == "offboard":
                rospy.loginfo("Received offboard from serial, start offboarding")
                pub.publish(Int32(1))
        except Exception as e:
            pass

    rospy.sleep(0.1)
