#!/usr/bin/env python

import rospy
import Jetson.GPIO as GPIO
import time
from std_msgs.msg import Int32


def callback(msg):
    pin = pins.get(msg.data)
    for _ in range(3):
        GPIO.output(pin, GPIO.HIGH)
        time.sleep(0.5)
        GPIO.output(pin, GPIO.LOW)
        time.sleep(0.5)


rospy.init_node("led", anonymous=True)
sub = rospy.Subscriber("led_data", Int32, callback)

pins = {
    0: 19,  # 红色LED
    1: 23,  # 绿色LED
}

GPIO.setmode(GPIO.BOARD)
for pin in pins.values():
    GPIO.setup(pin, GPIO.OUT)
    GPIO.output(pin, GPIO.LOW)

rospy.spin()
GPIO.cleanup()
