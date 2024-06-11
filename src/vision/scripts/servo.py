#!/usr/bin/env python

import rospy
from std_msgs.msg import Int32
import RPi.GPIO as GPIO
import time

# 定义舵机控制引脚
servo_pin = 33

def control_servo(angle):
    if angle == 90:
        duty_cycle = 3.5  # 90度位置的占空比
    else:
        duty_cycle = 8.5  # 0度位置的占空比
    p.ChangeDutyCycle(duty_cycle)

def callback(data):
    if data.data == 1:
        rospy.loginfo("Received 1, turning to 90 degrees")
        control_servo(90)
        time.sleep(3)
        control_servo(0)
    else:
        control_servo(0)

def servo_controller():
    rospy.init_node('servo', anonymous=True)
    rospy.Subscriber("/servo", Int32, callback)
    
    # 初始化 GPIO
    GPIO.setmode(GPIO.BOARD)
    GPIO.setup(servo_pin, GPIO.OUT, initial=GPIO.HIGH)

    # 创建 PWM 对象，设置频率为50Hz
    global p
    p = GPIO.PWM(servo_pin, 50)
    p.start(0)
    
    # 保持舵机在0度
    control_servo(0)
    
    rospy.spin()

    # 清理 GPIO
    p.stop()
    GPIO.cleanup()

if __name__ == '__main__':
    try:
        servo_controller()
    except rospy.ROSInterruptException:
        pass
