#!/usr/bin/env python

import sys
from awesom_o_robot.srv import MotorServiceValues, MotorServiceValuesResponse
import rospy

def motor_drive_backward_client(speed, duration):
    rospy.wait_for_service('motor_drive_backward')
    try:
        motor_drive_backward = rospy.ServiceProxy('motor_drive_backward', MotorServiceValues)
        resp1 = motor_drive_backward(speed, duration)
        return resp1.result
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)

def usage():
    return "%s [speed duration]"%sys.argv[0]

if __name__ == "__main__":
    if len(sys.argv) == 3:
        speed = int(sys.argv[1])
        duration = int(sys.argv[2])
    else:
        print(usage())
        sys.exit(1)
    motor_drive_backward_client(speed, duration)
