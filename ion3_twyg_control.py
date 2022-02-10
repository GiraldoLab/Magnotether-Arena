#!/usr/bin/env python
#modified from f-ponce magnotether_wind.py
#by Ysabel Giraldo, Tim Warren 1.23.20

from __future__ import print_function
import datetime
import roslib
import sys
import rospy
import cv2
import csv
#from std_msgs.msg import Header
#from std_msgs.msg import String
#from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import os
import os.path
import numpy as np
import matplotlib.pyplot as plt
import matplotlib
import Queue
import time
import pytz 
#ADDED below 12/6
#from basic_led_strip_ros.msg import StripLEDInfo
#from basic_led_strip_proxy import BasicLedStripProxy #I don't think we need this if we subscribe to shuffle_sun_node
#END
#added 1/19
from basic_led_strip_ros.msg import SunInfo 

from std_msgs.msg import Float64 

#from magnotether.msg import MsgAngleData
from magno_test.msg import MsgAngleData

class ImageConverter:  

    def __init__(self):
        self.timezone=pytz.timezone('US/Pacific')

        rospy.init_node('image_converter', anonymous=True)
        self.bridge = CvBridge()
        rospy.on_shutdown(self.clean_up)

        # Receives image data from the magno_test_node.py file
        self.angle_sub = rospy.Subscriber("/angle_data",MsgAngleData,self.callback)
        self.sun_sub = rospy.Subscriber("sun_position",SunInfo,self.callback)
        self.queue = Queue.Queue()

        # Most likely garbage code
        # self.rotated_image_pub = rospy.Publisher('/rotated_image', Image, queue_size=10)
        # self.contour_image_pub = rospy.Publisher('/contour_image', Image, queue_size=10)

        #DATA ADD PD added 12/6 
        timestr = time.strftime("magnotether_%Y%m%d_%H%M%S", time.localtime())
        self.current_time = datetime.datetime.now() #added 2/8


        #Saving data
        self.directory = '/home/giraldolab/catkin_ws/src/magno-test/nodes/data'
        self.angle_filename = os.path.join(self.directory,'angle_data_%s.csv'%timestr)
        self.angle_fid = open(self.angle_filename,'w') #changed from 'w+' 12/13
        self.angle_writer = csv.writer(self.angle_fid, delimiter = ",")
        #END 

        self.angle_data = None
        #BEGIN PD added 12/6 from magnotether_node_2.py in FP's github
        self.angle_list = []
        self.frame_list = []
        self.frame_num =0
        self.angle_data_list = []
        self.display_window = 500
        self.sun_position = 0
        self.sun_time = 0 #added 2/8


        plt.ion()
        self.fig = plt.figure(1)
        self.ax = plt.subplot(1,1,1)
        self.line, = plt.plot([0,1],[0,1], 'b')
        plt.grid(True)
        plt.xlabel('frame (#)')
        plt.ylabel('angle (deg)')
        plt.title('Angles vs Frame')
        self.line.set_xdata([])
        self.line.set_ydata([])
        self.ax.set_ylim(-180,180)
        self.ax.set_xlim(0,self.display_window)
        #self.fig.canvas.draw()
        self.fig.canvas.flush_events()
        # #END PD additions 12/6

        cv2.namedWindow('raw image')
        cv2.namedWindow('contour image')
        cv2.namedWindow('rotated image')

        cv2.moveWindow('raw image', 100, 100)
        cv2.moveWindow('contour image', 700, 110)
        cv2.moveWindow('rotated image', 1200, 120)
    print('finish')

    def clean_up(self):
        print('cleaning up')
        self.angle_fid.close()
        cv2.destroyAllWindows()

    def callback(self,data): 
        self.queue.put(data)


    def run(self): 
        while not rospy.is_shutdown():
            self.angle_data = None
            while self.queue.qsize() > 0:
                data = self.queue.get()
                if isinstance(data,SunInfo):
                    self.sun_position = data.sun_position
                    utc_time = pytz.utc.localize(datetime.datetime.utcfromtimestamp(float(data.header.stamp.to_time())))
                    self.sun_time = utc_time.astimezone(self.timezone)
                    print(data.sun_position)
                    continue
                else:
                    self.angle_data = data

                # Grabs raw image from the camera publisher node
                self.frame_num+=1
                self.angle_list.append(self.angle_data.angle) #original self.angle_data.angle 
                self.frame_list.append(self.frame_num)

                if self.angle_data is not None:
                    # Displays images
                    cv2.imshow('raw image', self.bridge.imgmsg_to_cv2(self.angle_data.raw_image, desired_encoding="passthrough"))
                    cv2.imshow('contour image',self.bridge.imgmsg_to_cv2(self.angle_data.contour_image, desired_encoding="passthrough"))
                    cv2.imshow('rotated image', self.bridge.imgmsg_to_cv2(self.angle_data.rotated_image, desired_encoding="passthrough"))
                    cv2.waitKey(1)
                    # #Displays the graph of fly angles during experiment.timeObj.strftime("%H:%M:%S.%f")k this should be included.
                    self.line.set_xdata(range(len(self.angle_list)))
                    self.line.set_ydata(self.angle_list)
                    if self.frame_list:
                        self.ax.set_xlim(self.frame_list[0], max(self.display_window,self.frame_list[-1]))
                        #self.fig.canvas.draw()
                        self.fig.canvas.flush_events()
                    rospy.sleep(0.0001) # YG added 12/15

                self.angle_writer.writerow([datetime.datetime.now(),self.frame_num, self.angle_list[-1],self.sun_position,self.sun_time])

                    #self.angle_writer.writerow([ str(self.current_time), str(self.frame_num), str(self.angle_data), str(self.sun_position), str(self.sun_time)])
                                                                                        #original self.angle_data.angle



def main(args):
    ic = ImageConverter()
    ic.run()


# ---------------------------------------------------------------------------------------
if __name__ == '__main__': 
    main(sys.argv)
