#!/usr/bin/env python
#-*-coding:utf-8-*-
import rospy
import Jetson.GPIO as GPIO
from std_msgs.msg import Int32
import time

def get_distance():  
    GPIO.output(TRIG_PIN, GPIO.HIGH)  
    time.sleep(0.00001)  
    GPIO.output(TRIG_PIN, GPIO.LOW)  
  
    start_time = time.time()  
  
    while GPIO.input(ECHO_PIN) == 0:  
        pass
    while GPIO.input(ECHO_PIN) == 1:  
        pass

    end_time = time.time()  

    # 计算时间差
    pulse_duration = end_time - start_time

    # 计算距离（假设声速为340m/s）
    distance = (pulse_duration * 34000) / 2-38 # 偏差量38cm
    if distance is not None:
        return int(distance)
    else:
        return None

GPIO.setmode(GPIO.BOARD)  
TRIG_PIN = 13
ECHO_PIN = 15
GPIO.setup(TRIG_PIN, GPIO.OUT)
GPIO.setup(ECHO_PIN, GPIO.IN)  

rospy.init_node('supersonic_node', anonymous=True)  
pub = rospy.Publisher('supersonic_data', Int32, queue_size=10)  
rate = rospy.Rate(20)

while True:
    distance = get_distance()
    if distance is not None:
        pub.publish(distance)
        rate.sleep()
GPIO.cleanup()