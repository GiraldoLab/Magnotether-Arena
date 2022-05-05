#!/usr/bin/env python
#modified from f-ponce magnotether_wind.py
#by Ysabel Giraldo, Tim Warren 1.23.20
#Control code that receives images from MagnoTestNode, displays windows for raw_image, contour image and rotated image
#Saves timestamp, frame number, angle data,sun position and timestamp of sun position change to a csv file.

from __future__ import print_function
import datetime
import roslib
import sys
import rospy
import cv2
import csv
from cv_bridge import CvBridge, CvBridgeError
import os
import os.path
import numpy as np
import matplotlib.pyplot as plt
import matplotlib
import Queue
import time
import pytz 
from basic_led_strip_ros.msg import SunInfo 
from std_msgs.msg import Float64 
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


        #DATA ADD PD added 12/6 
        timestr = time.strftime("magnotether_%Y%m%d_%H%M%S", time.localtime())


        #Saving data
        self.directory = '/home/giraldolab/catkin_ws/src/magno-test/nodes/data'
        self.angle_filename = os.path.join(self.directory,'angle_data_%s.csv'%timestr)
        self.angle_fid = open(self.angle_filename,'w') #changed from 'w+' 12/13
        self.angle_writer = csv.writer(self.angle_fid, delimiter = ",")
        self.angle_writer.writerow(["Image Time","Frame","Heading Angle","Sun Position","Sun Time"])
        #END 

        self.angle_data = None
        #BEGIN PD added 12/6 from magnotether_node_2.py in FP's github
        self.angle_list = []
        self.frame_list = []
        self.frame_num =0
        self.angle_data_list = []
        self.display_window = 500
        self.sun_position = 0
        self.sun_position_list = []
        self.sun_time = 0 #added 2/9


        plt.ion()
        self.fig = plt.figure(1)
        self.ax = plt.subplot(1,1,1)
        self.fly_angle_plot, = plt.plot([0,1],[0,1], 'b')
        self.sun_angle_plot, = plt.plot([0,1],[0,1], 'g')
        plt.grid(True)
        plt.xlabel('frame (#)')
        plt.ylabel('angle (deg)')
        plt.title('Angles vs Frame')
        self.fly_angle_plot.set_xdata([])
        self.fly_angle_plot.set_ydata([])
        self.sun_angle_plot.set_xdata([])
        self.sun_angle_plot.set_ydata([])
        self.ax.set_ylim(-180,180)
        self.ax.set_xlim(0,self.display_window)
        self.fig.canvas.flush_events()
        # #END PD additions 12/6

        cv2.namedWindow('raw image')
        cv2.namedWindow('contour image')
        cv2.namedWindow('rotated image')

        cv2.moveWindow('raw image', 800, 100)
        cv2.moveWindow('contour image', 1300, 1000)
        cv2.moveWindow('rotated image', 2350, 100)



    print('finish')

    def clean_up(self):
        print('cleaning up')
        self.angle_fid.close()
        cv2.destroyAllWindows()


    def callback(self,data): 
        self.queue.put(data)

    def calc_sun_angle_from_led(self,led_position):
        return (-np.pi*led_position/72.0 +25*np.pi/24.0)/np.pi*180


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
                self.sun_position_list.append(self.calc_sun_angle_from_led(self.sun_position))
                

                if self.angle_data is not None and self.frame_list[-1]%5 == 0:
                    # Displays images
                    cv2.imshow('raw image', self.bridge.imgmsg_to_cv2(self.angle_data.raw_image, desired_encoding="passthrough"))
                    cv2.imshow('contour image',self.bridge.imgmsg_to_cv2(self.angle_data.contour_image, desired_encoding="passthrough"))
                    cv2.imshow('rotated image', self.bridge.imgmsg_to_cv2(self.angle_data.rotated_image, desired_encoding="passthrough"))
                    cv2.waitKey(1)
                    #Displays the graph of fly angles during experiment.timeObj.strftime("%H:%M:%S.%f")k this should be included.
                    self.fly_angle_plot.set_xdata(range(len(self.angle_list)))
                    self.fly_angle_plot.set_ydata(self.angle_list)
                    self.sun_angle_plot.set_xdata(range(len(self.angle_list)))
                    self.sun_angle_plot.set_ydata(self.sun_position_list)
                    if self.frame_list:
                        self.ax.set_xlim(self.frame_list[0], max(self.display_window,self.frame_list[-1]))
                        self.fig.canvas.flush_events()
                    rospy.sleep(0.0001) # YG added 12/15
                img_time = pytz.utc.localize(datetime.datetime.utcfromtimestamp(float(self.angle_data.header.stamp.to_time()))) 
                img_time = img_time.astimezone(self.timezone)

                self.angle_writer.writerow([img_time,self.frame_num, self.angle_list[-1],self.sun_position,self.sun_time])

        


def main(args):
    ic = ImageConverter()
    ic.run()


# ---------------------------------------------------------------------------------------
if __name__ == '__main__': 
    main(sys.argv)
