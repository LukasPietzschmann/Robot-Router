#!/usr/bin/env python

from awesom_o_robot.srv import MotorCurveServiceValues, MotorCurveServiceValuesResponse
from awesom_o_robot.msg import MotorValues
from datetime import datetime, timedelta
import rospy


pub = rospy.Publisher('motor_values', MotorValues, queue_size = 10)

def handle_motor_turn_forward(req):
    print(f'Driving forward turn for {abs(req.duration)} seconds with left speed {abs(req.speedLeft)} and right speed {abs(req.speedRight)}')

    motorValues = MotorValues()
    motorValues.speedLeft = abs(req.speedLeft)
    motorValues.speedRight = abs(req.speedRight)

    t_end = datetime.now() + timedelta(milliseconds=abs(req.duration))

    pub.publish(motorValues)

    stopped = False
    while not stopped:
        if datetime.now() >= t_end:
            motorValues.speedLeft = 0
            motorValues.speedRight = 0
            pub.publish(motorValues)
            stopped = True
            return MotorCurveServiceValuesResponse("success")

    return MotorCurveServiceValuesResponse("fail")


def motor_turn_forward_server():
    rospy.init_node('motor_turn_forward_server')
    s = rospy.Service('motor_turn_forward', MotorCurveServiceValues, handle_motor_turn_forward)
    print("Ready to turn forward.")
    rospy.spin()

if __name__ == "__main__":
    motor_turn_forward_server()
