#!/usr/bin/env python

#for closed loop experiments - adjusts the position of the led according to the fly's position. 

from __future__ import print_function

import Queue
import rospy
import numpy as np

from  basic_led_strip_proxy import BasicLedStripProxy

from magnotether.msg import MsgAngleData

class AngleToLedNode(object):

    def __init__(self):

        rospy.init_node('angle_to_led', anonymous=True)

        self.angle_sub = rospy.Subscriber('/angle_data', MsgAngleData, self.on_angle_data) 
        self.data_queue = Queue.Queue()

        self.num_led = 144
        self.led_strip = BasicLedStripProxy(use_thread=True)

    def on_angle_data(self, angle_data):
        self.data_queue.put(angle_data)

    def run(self):

        while not rospy.is_shutdown():
            while self.data_queue.qsize() > 0:
                angle_data = self.data_queue.get()
                led = self.angle_to_led(angle_data.angle)
                self.led_strip.set_led(led,(0,0,0)) #original (100,0,0)
                print('angle: {0:1.2f}, led: {1}'.format(angle_data.angle, led))

    def angle_to_led(self,angle):
        led = int((angle+180)*self.num_led/360.0)
        led = min(led,self.num_led)
        led = max(0,led)
        return led


    



# ----------------------------------------------------------------------------------------

if __name__ == '__main__':

    node = AngleToLedNode()
    node.run()





	

