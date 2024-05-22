#!/usr/bin/env python
import rospy
from cv_detect.msg import BarMsg
from lidar_data.msg import LidarPose

def callback(lidar_msg):
    global pub, bar_msg
    bar_msg.delta_x = int((-1.0 + (0.2) - lidar_msg.y) * 1500)

def barcode_publisher():
    global pub, bar_msg
    rospy.init_node('barcode_node', anonymous=True)
    pub = rospy.Publisher('barcode_msg', BarMsg, queue_size=10)
    rospy.Subscriber('lidar_data', LidarPose, callback)
    rate = rospy.Rate(20)  # 设置发布频率为20 Hz

    bar_msg = BarMsg()
    bar_msg.n = 5
    bar_msg.delta_x = 0

    while not rospy.is_shutdown():
        pub.publish(bar_msg)
        rate.sleep()

if __name__ == '__main__':
    try:
        barcode_publisher()
    except rospy.ROSInterruptException:
        pass
