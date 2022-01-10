#!/usr/bin/env python

import rospy
import RPi.GPIO as GPIO
import time
from std_msgs.msg import Int16


def callback(data):
  angle = data.data

  angle = 180 if angle > 180 else angle
  angle = 0 if angle < 0 else angle
  rospy.loginfo(f'Setting tilt angle to {angle}' + u'\N{DEGREE SIGN}')

  # Set GPIO stuff
  GPIO.setmode(GPIO.BOARD)
  GPIO.setup(18,GPIO.OUT)
  servo1 = GPIO.PWM(18,50) # pin 18 for servo1, pulse 50Hz
  servo1.start(0)

  # Tilt
  servo1.ChangeDutyCycle(2+(angle/18))
  time.sleep(0.5)
  servo1.ChangeDutyCycle(0)

  # Clean up
  servo1.stop()
  GPIO.cleanup()


def listener():
  rospy.init_node('tilt_servo', anonymous=True)
  rospy.Subscriber("tilt_servo_angle", Int16, callback)
  rospy.spin()


if __name__ == '__main__':
  listener()