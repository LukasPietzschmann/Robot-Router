#!/usr/bin/env python
import rospy
from threading import Timer, Thread
import time
import sys
import RPi.GPIO as GPIO, time
from awesom_o_robot.msg import Speed


class ResetTimer(object):
    def __init__(self, time, function, daemon=None):
        self.__time = time
        self.__function = function
        self.__set()
        self.__running = False
        self.__killed = False
        Thread.__init__(self)
        self.__daemon = daemon

    def __set(self):
        self.__timer = Timer(self.__time, self.__function)

    def stop(self):
        self.__daemon = True

    def run(self):
        self.__running = True
        self.__timer.start()
        if self.__daemon:
            sys.exit(0)

    def cancel(self):
        self.__running = False
        self.__timer.cancel()

    def reset(self, start=False):
        if self.__running:
            self.__timer.cancel()
        self.__set()
        if self.__running or start:
            self.start()

        
class SpeedDetector():

    def __init__(self):
        self.speed = Speed()
        self.rpm = 0
        self.mph = 0
        self.kmh = 0.0
        self.total_driven_centimeters = 0.0

        self.counter = 0
        self.pin = 15
        self.interval = 1.0
        self.calc = 60 / int(self.interval)
        self.wheel = 20
        self.wheel_circumference_in_cm = 16

        GPIO.setmode(GPIO.BOARD)
        GPIO.setwarnings(False)
        GPIO.setup(self.pin, GPIO.IN)

        self.timer = ResetTimer(self.interval, self.output)

        GPIO.add_event_detect(self.pin, GPIO.RISING, callback=self.count)
        # Timer startet
        self.timer.run()

    def count(self, arg):
        self.counter = self.counter + 1
        self.total_driven_centimeters += 0.8

    def output(self):
        self.timer.cancel()

        self.rpm = int(((self.counter / 2) * self.calc) / self.wheel)
        self.mph = int(self.rpm * self.wheel_circumference_in_cm * 60 / 100)
        self.kmh = self.mph / 1000

        self.counter = 0
        self.timer.reset()
        self.timer.run()

    def callback(self):
        pub = rospy.Publisher('speed', Speed, queue_size=10)
        rospy.init_node('speed_detector', anonymous=True)
        rate = rospy.Rate(5) # 5hz

        while not rospy.is_shutdown():
            # self.calculate_speeds()
            self.speed.rpm = self.rpm
            self.speed.mph = self.mph
            self.speed.kmh = self.kmh
            self.speed.total_driven_centimeters = self.total_driven_centimeters
            pub.publish(self.speed)
            rate.sleep()

if __name__ == '__main__':
    try:
        speed_detector = SpeedDetector()
        speed_detector.callback()
    except rospy.ROSInterruptException:
        pass
