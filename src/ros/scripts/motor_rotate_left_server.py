#!/usr/bin/env python

from awesom_o_robot.srv import MotorServiceValues, MotorServiceValuesResponse
from awesom_o_robot.msg import MotorValues
from time import time

import rospy


pub = rospy.Publisher('motor_values', MotorValues, queue_size = 10)

def handle_motor_rotate_left(req):
    print(f'Rotating left for {req.duration} seconds with speed {req.speed}')

    motorValues = MotorValues()
    motorValues.speedLeft = - req.speed
    motorValues.speedRight = req.speed

    t_end = time() + req.duration

    pub.publish(motorValues)

    stopped = False
    while not stopped:
        if time() >= t_end:
            motorValues.speedLeft = 0
            motorValues.speedRight = 0
            pub.publish(motorValues)
            stopped = True
            return MotorServiceValuesResponse("success")

    return MotorServiceValuesResponse("fail")


def motor_rotate_left_server():
    rospy.init_node('motor_rotate_left_server')
    s = rospy.Service('motor_rotate_left', MotorServiceValues, handle_motor_rotate_left)
    print("Ready to rotate left.")
    rospy.spin()

if __name__ == "__main__":
    motor_rotate_left_server()
