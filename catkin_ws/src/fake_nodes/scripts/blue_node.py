#!/usr/bin/env python
import rospy
from cv_detect.msg import LedMsg
from lidar_data.msg import LidarPose

def callback(lidar_msg):
    global pub, led_msg
    # led_msg.value = lidar_msg.x > 2 and (lidar_msg.yaw < 0.1 or lidar_msg.yaw > 6.27)
    led_msg.value = lidar_msg.x < 2 and abs(lidar_msg.yaw - 3.14) < 0.1   # 当 x 大于 1 时设置 value 为 True

def blue_publisher():
    global pub, led_msg
    rospy.init_node('blue_node', anonymous=True)
    pub = rospy.Publisher('blue_msg', LedMsg, queue_size=10)
    rospy.Subscriber('lidar_data', LidarPose, callback)
    rate = rospy.Rate(20)  # 设置发布频率为20 Hz

    led_msg = LedMsg()
    led_msg.delta_x = 0
    led_msg.delta_y = 0

    while not rospy.is_shutdown():
        pub.publish(led_msg)
        rate.sleep()

if __name__ == '__main__':
    try:
        blue_publisher()
    except rospy.ROSInterruptException:
        pass
