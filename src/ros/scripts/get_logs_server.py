#!/usr/bin/env python

from awesom_o_robot.srv import MotorServiceValues, MotorServiceValuesResponse
from awesom_o_robot.msg import MotorValues
from time import time
from datetime import datetime, timedelta

import rospy


pub = rospy.Publisher('motor_values', MotorValues, queue_size = 10)

def handle_motor_drive_backward(req):
    print(f'Driving backward for {abs(req.duration)} seconds with speed {abs(req.speed)}')

    motorValues = MotorValues()
    motorValues.speedLeft = -abs(req.speed)
    motorValues.speedRight = -abs(req.speed)

    t_end = datetime.now() + timedelta(milliseconds=abs(req.duration))

    pub.publish(motorValues)

    stopped = False
    while not stopped:
        if datetime.now() >= t_end:
            motorValues.speedLeft = 0
            motorValues.speedRight = 0
            pub.publish(motorValues)
            stopped = True
            return MotorServiceValuesResponse("success")

    return MotorServiceValuesResponse("fail")


def motor_drive_backward_server():
    rospy.init_node('motor_drive_backward_server')
    s = rospy.Service('motor_drive_backward', MotorServiceValues, handle_motor_drive_backward)
    print("Ready to drive backward.")
    rospy.spin()

if __name__ == "__main__":
    motor_drive_backward_server()
