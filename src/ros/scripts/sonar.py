#!/usr/bin/env python
import rospy
import RPi.GPIO as GPIO, time
from sensor_msgs.msg import Range

class Sonar():

    def __init__(self):
        self.range = Range()
        self.range.header.stamp = None
        self.range.header.frame_id = "/base_link"
        self.range.radiation_type = 0 # 0: Ultrasound, 1: Infrared
        self.range.field_of_view = 0.26
        self.range.min_range = 0.0
        self.range.max_range = 200.0

    def get_current_range(self):
        sonar_pin = 8
        GPIO.setmode(GPIO.BOARD)
        GPIO.setwarnings(False)
        GPIO.setup(sonar_pin, GPIO.OUT)
        # Send 10us pulse to trigger
        GPIO.output(sonar_pin, True)
        time.sleep(0.00001)
        GPIO.output(sonar_pin, False)
        start = time.time()
        count=time.time()
        GPIO.setup(sonar_pin,GPIO.IN)
        while GPIO.input(sonar_pin)==0 and time.time()-count<0.1:
            start = time.time()
        count=time.time()
        stop=count
        while GPIO.input(sonar_pin)==1 and time.time()-count<0.1:
            stop = time.time()
        # Calculate pulse length
        elapsed = stop-start
        # Distance pulse travelled in that time is time
        # multiplied by the speed of sound (cm/s)
        distance = elapsed * 34000
        # That was the distance there and back so halve the value
        distance = distance / 2
        return distance

    def callback(self):
        pub = rospy.Publisher('sonar_range', Range, queue_size=10)
        rospy.init_node('sonar', anonymous=True)
        rate = rospy.Rate(5) # 5hz

        while not rospy.is_shutdown():
            self.range.range = self.get_current_range()
            self.range.header.stamp = rospy.Time.now()
            #rospy.loginfo(self.range)
            pub.publish(self.range)
            rate.sleep()

if __name__ == '__main__':
    try:
        sonar = Sonar()
        sonar.callback()
    except rospy.ROSInterruptException:
        pass
