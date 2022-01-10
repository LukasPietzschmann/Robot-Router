#!/usr/bin/env python
import rospy
import RPi.GPIO as GPIO, time
from awesom_o_robot.msg import LineValues

class LineSensors():

    lineRight = 13
    lineLeft = 12

    def __init__(self):
        GPIO.setmode(GPIO.BOARD)

        #set up digital line detectors as inputs
        GPIO.setup(self.lineRight, GPIO.IN) # Right line sensor
        GPIO.setup(self.lineLeft, GPIO.IN) # Left line sensor

        self.line_values = LineValues()

    def irLeftLane(self):
        if GPIO.input(self.lineLeft) == 0:
            return True
        else:
            return False

    def irRightLane(self):
        if GPIO.input(self.lineRight) == 0:
            return True
        else:
            return False

    def callback(self):
        pub = rospy.Publisher('ir_lines', LineValues, queue_size=10)
        rospy.init_node('line_sensors', anonymous=True)
        rate = rospy.Rate(10) # 10hz

        while not rospy.is_shutdown():
            #print(self.irLeftLane())
            self.line_values.leftLaneDetected = self.irLeftLane()
            print(self.irRightLane())
            self.line_values.rightLaneDetected = self.irRightLane()
            pub.publish(self.line_values)
            rate.sleep()

if __name__ == '__main__':
    try:
        line_sensors = LineSensors()
        line_sensors.callback()
    except rospy.ROSInterruptException:
        pass
