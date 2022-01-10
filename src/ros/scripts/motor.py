#!/usr/bin/env python

import rospy
import RPi.GPIO as GPIO
import time
from awesom_o_robot.msg import MotorValues

class Motor:

    # Pins 24, 26 Right Motor
    # Pins 19, 21 Left Motor
    R1 = 24
    R2 = 26
    L1 = 19
    L2 = 21

    def __init__(self):
        GPIO.setmode(GPIO.BOARD)

        #use pwm on inputs so motors don't go too fast
        GPIO.setup(Motor.L1, GPIO.OUT)
        self.left_motor_forward = GPIO.PWM(Motor.L1, 20)
        self.left_motor_forward.start(0)

        GPIO.setup(Motor.L2, GPIO.OUT)
        self.left_motor_backward = GPIO.PWM(Motor.L2, 20)
        self.left_motor_backward.start(0)

        GPIO.setup(Motor.R1, GPIO.OUT)
        self.right_motor_forward = GPIO.PWM(Motor.R1, 20)
        self.right_motor_forward.start(0)

        GPIO.setup(Motor.R2, GPIO.OUT)
        self.right_motor_backward = GPIO.PWM(Motor.R2, 20)
        self.right_motor_backward.start(0)

    def callback(self, data):
        speed_left = data.speedLeft
        speed_right = data.speedRight

        speed_left = 100 if speed_left > 100 else speed_left
        speed_left = -100 if speed_left < -100 else speed_left

        speed_right = 100 if speed_right > 100 else speed_right
        speed_right = -100 if speed_right < -100 else speed_right

        rospy.loginfo(f'Setting left motor speed to {speed_left}\nSetting right motor speed to {speed_right}')

        # Set speed
        if speed_left < 0:
            self.left_motor_backward.ChangeDutyCycle(abs(speed_left))
            self.left_motor_forward.ChangeDutyCycle(0)
        else:
            self.left_motor_forward.ChangeDutyCycle(speed_left)
            self.left_motor_backward.ChangeDutyCycle(0)

        if speed_right < 0:
            self.right_motor_backward.ChangeDutyCycle(abs(speed_right))
            self.right_motor_forward.ChangeDutyCycle(0)
        else:
            self.right_motor_forward.ChangeDutyCycle(speed_right)
            self.right_motor_backward.ChangeDutyCycle(0)

        self.left_motor_forward.ChangeFrequency(abs(speed_left) + 5)
        self.right_motor_forward.ChangeFrequency(abs(speed_right) + 5)


def listener():
  rospy.init_node('motor', anonymous=True)
  motor = Motor()
  rospy.Subscriber("motor_values", MotorValues, motor.callback)
  rospy.spin()


if __name__ == '__main__':
  listener()
