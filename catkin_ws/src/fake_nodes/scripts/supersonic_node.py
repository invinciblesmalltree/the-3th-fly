#!/usr/bin/env python
import rospy
from std_msgs.msg import Int32
from lidar_data.msg import LidarPose

def callback(lidar_msg):
    global pub, msg
    msg.data = int((lidar_msg.x - 0.225 - (0.2)) * 100)
    

def supersonic_publisher():
    global pub, msg
    rospy.init_node('supersonic_node', anonymous=True)
    pub = rospy.Publisher('supersonic_data', Int32, queue_size=10)
    rospy.Subscriber('lidar_data', LidarPose, callback)
    rate = rospy.Rate(20)  # 设置发布频率为20 Hz

    msg = Int32()
    msg.data = 0

    while not rospy.is_shutdown():
        pub.publish(msg)
        rate.sleep()

if __name__ == '__main__':
    try:
        supersonic_publisher()
    except rospy.ROSInterruptException:
        pass
