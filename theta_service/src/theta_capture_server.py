#!/usr/bin/env python

__author__ = 'matsunolab'

import rospy
from sensor_msgs.msg import CompressedImage
from theta_service.srv import *
from theta.theta import Theta


class Theta360Server:
    RESIZED_IMAGE_FLAG = True
    GRAB_CURRENT_IMAGE = 1
    FRAME_ID = 'theta_camera'  # TODO: make it rosparam

    def __init__(self):
        self.theta = Theta()

    def initialize(self):
        if self.theta.open():
            self.theta.set_init_settings()
            return True
        else:
            return False

    def generate_imgmsg(self, image):
        jpeg_img = CompressedImage()
        # data set
        jpeg_img.header.stamp = rospy.Time.now()
        jpeg_img.header.frame_id = self.FRAME_ID
        jpeg_img.format = "jpeg"
        jpeg_img.data = image
        return jpeg_img

    def handle_theta_capture(self, req):
        if req.capture_mode == self.GRAB_CURRENT_IMAGE:
            self.theta.shutter()
            image = self.theta.grab_currentest_image(self.RESIZED_IMAGE_FLAG)
            print 'grab image!'
            return ThetaCaptureServiceResponse(self.generate_imgmsg(image))
        else:
            print 'grab image!'
            image = self.theta.grab_currentest_image(self.RESIZED_IMAGE_FLAG)
            return ThetaCaptureServiceResponse(self.generate_imgmsg(image))

    def main(self):
        rospy.init_node('theta_capture_server')
        service = rospy.Service('theta_capture', ThetaCaptureService, self.handle_theta_capture)
        print 'READY...'
        rospy.spin()

# -------------------------------------------------------------------------
if __name__ == "__main__":
    theta_server = Theta360Server()
    if theta_server.initialize():
        theta_server.main()
    else:
        print 'fail to initialize theta'