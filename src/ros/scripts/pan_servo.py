#!/usr/bin/env python

import rospy
import RPi.GPIO as GPIO
import time
from std_msgs.msg import Int16


def callback(data):
  angle = data.data

  angle = 180 if angle > 180 else angle
  angle = 0 if angle < 0 else angle
  rospy.loginfo(f'Setting pan angle to {angle}' + u'\N{DEGREE SIGN}')

  # Set GPIO stuff
  GPIO.setmode(GPIO.BOARD)
  GPIO.setup(22,GPIO.OUT)
  servo2 = GPIO.PWM(22,50) # pin 22 for servo2, pulse 50Hz
  servo2.start(0)

  # Tilt
  servo2.ChangeDutyCycle(2+(angle/18))
  time.sleep(0.5)
  servo2.ChangeDutyCycle(0)

  # Clean up
  servo2.stop()
  GPIO.cleanup()


def listener():
  rospy.init_node('pan_servo', anonymous=True)
  rospy.Subscriber("pan_servo_angle", Int16, callback)
  rospy.spin()


if __name__ == '__main__':
  listener() 