#!/usr/bin/env python

__author__ = 'RyosukeMurata'

import rospy
import time
from theta_service.srv import *

GRAB_CURRENT_IMAGE = 1
FILENAME = 'test.jpg'


# -------------------------------------------------------------------------
if __name__ == "__main__":
    rospy.wait_for_service('theta_capture')
    try:
        theta_capture = rospy.ServiceProxy('theta_capture', ThetaCaptureService)
        time.sleep(1)
        print 'request!'
        res = theta_capture(GRAB_CURRENT_IMAGE)
        print 'grabbed!'
        f = open(FILENAME, 'wb')
        f.write(res.image_msg.data)
        f.close()
    except rospy.ServiceException, e:
        print "Service call failed: %s" % e

