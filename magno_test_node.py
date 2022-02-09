#!/usr/bin/env python
from __future__ import print_function

import Queue
import rospy
import cv2
import numpy as np

from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from find_fly_angle import find_fly_angle
from magnotether.msg import MsgAngleData

class MagnoTestNode(object):

    def __init__(self):

        self.cv_bridge = CvBridge()

        rospy.init_node('magno_test', anonymous=True)
        self.img_sub = rospy.Subscriber("/camera/image_raw", Image, self.on_img_callback) #Giraldolab camera feed: /camera/image_raw

        self.angle_pub = rospy.Publisher('/angle_data', MsgAngleData, queue_size=10)
        self.img_queue = Queue.Queue()
        self.threshold = 50 # these 4 lines are parameters for find_fly_angle
        self.mask_scale = 0.9
        self.frame_count = 0


    def on_img_callback(self, ros_img):
        self.img_queue.put(ros_img)

    def run(self):

        while not rospy.is_shutdown():

            # Try to get any new images from queue
            ros_img = None
            while self.img_queue.qsize() > 0:
                ros_img = self.img_queue.get()
            if ros_img is None:
                # There are no new image yet - go back to top of loop
                continue

            # We have a new image process it
            self.frame_count+=1
            try:
                cv_img = self.cv_bridge.imgmsg_to_cv2(ros_img, desired_encoding="passthrough")
            except CvBridgeError as e:
                rospy.logwarn(str(e))

            # Create color version of image
            #cv_img_bgr = cv2.cvtColor(cv_img,cv2.COLOR_GRAY2BGR)


            angle_data = self.calc_fly_angle(cv_img)
            #print (angle_data.keys())

            
            rotated_ros_img = self.cv_bridge.cv2_to_imgmsg(angle_data['rotated_image'])
            contour_ros_img = self.cv_bridge.cv2_to_imgmsg(angle_data['contour_image'])
            raw_ros_img     = self.cv_bridge.cv2_to_imgmsg(cv_img)

            self.publish_img_msg(ros_img, rotated_ros_img, contour_ros_img,raw_ros_img)

            # cv2.imshow('cv_img', cv_img)
            # cv2.imshow('contour', angle_data['contour_image'])
            # cv2.waitKey(1)



    def calc_fly_angle(self, cv_img):
        angle_rad, angle_data = find_fly_angle(cv_img, self.threshold, self.mask_scale)

        self.angle_deg = np.rad2deg(angle_rad)
        return angle_data

    def publish_img_msg(self, ros_img, rotated_ros_image, contour_ros_image, raw_ros_image):
        msg_angle_data = MsgAngleData()
        msg_angle_data.header.stamp = ros_img.header.stamp
        msg_angle_data.frame = self.frame_count
        msg_angle_data.angle = self.angle_deg
        msg_angle_data.rotated_image = rotated_ros_image
        msg_angle_data.raw_image = raw_ros_image
        msg_angle_data.contour_image = contour_ros_image
        self.angle_pub.publish(msg_angle_data) 









            

# ----------------------------------------------------------------------------------------

if __name__ == '__main__':

    node = MagnoTestNode()
    node.run()





	

