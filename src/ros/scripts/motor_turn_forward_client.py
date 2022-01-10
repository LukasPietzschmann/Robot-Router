#!/usr/bin/env python

import sys
from awesom_o_robot.srv import MotorCurveServiceValues, MotorCurveServiceValuesResponse
import rospy

def motor_turn_forward_client(speed, duration):
    rospy.wait_for_service('motor_turn_forward')
    try:
        motor_turn_forward = rospy.ServiceProxy('motor_turn_forward', MotorCurveServiceValues)
        resp1 = motor_turn_forward(speed, duration)
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
    motor_turn_forward_client(speed, duration)
