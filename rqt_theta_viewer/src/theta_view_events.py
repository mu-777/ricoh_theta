#!/usr/bin/env python

import sys, os
import rospy
from theta_service.srv import *

from python_qt_binding.QtGui import QFileDialog, QImage

GRAB_CURRENT_IMAGE = 1

class ThetaViewEvents:
    def __init__(self, _parent):
        self.parent = _parent

        rospy.wait_for_service('theta_capture')


    # http://retrofocus28.blogspot.jp/2013/09/pyqt9.html
    def open_FileDialog(self):
        filename = QFileDialog.getOpenFileName(self.parent, 'Open file', os.path.expanduser('~'))
        self.parent.filename = filename[0]
        self.parent.qimage = QImage(self.parent.filename, 'JPEG')

    def save_FileDialog(self):
        filename = QFileDialog.getSaveFileName(self.parent, 'Save file', os.path.expanduser('~'))
        self.parent.filename = filename[0]
        if self.parent.jpeg_data is None:
            print 'no image data'
        else:
            f = open(self.parent.filename.encode(), 'wb')
            f.write(self.parent.jpeg_data)
            f.close()

    def shutter_clicked_event(self):
        try:
            theta_capture = rospy.ServiceProxy('theta_capture', ThetaCaptureService)
            print 'request!'
            res = theta_capture(GRAB_CURRENT_IMAGE)
            print 'grabbed!'
            self.parent.jpeg_data = res.image_msg.data
            f = open('.temp', 'wb')
            f.write(self.parent.jpeg_data)
            f.close()
            self.parent.qimage = QImage('.temp', 'JPEG')
        except rospy.ServiceException, e:
            print "Service call failed: %s" % e

