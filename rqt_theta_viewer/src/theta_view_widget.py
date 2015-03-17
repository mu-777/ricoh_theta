from __future__ import division
import os
import numpy as np
import rospkg

from python_qt_binding import loadUi
from python_qt_binding.QtCore import Qt, QTimer, qWarning, Slot
from python_qt_binding.QtGui import QAction, QMenu, QWidget, QImage

import rospy
from rostopic import get_topic_class
from tf.transformations import quaternion_about_axis

from OpenGL.GL import *
from OpenGL.GLUT import *

from gl_widget import GLWidget as MyGLWidget
from theta_view_events import ThetaViewEvents
import gl_painter as gl_painter

## main class inherits from the ui window class
## hint: http://vivi.dyndns.org/tech/Qt/openGL.html


class ThetaViewWidget(QWidget):
    def __init__(self, plugin):
        super(ThetaViewWidget, self).__init__()
        rp = rospkg.RosPack()
        loadUi(os.path.join(rp.get_path('rqt_theta_viewer'), 'resource', 'ThetaViewerWidget.ui'), self)
        self.plugin = plugin
        self.events = ThetaViewEvents(self)

        self._pushButton_open.clicked.connect(self.events.open_FileDialog)
        self._pushButton_save.clicked.connect(self.events.save_FileDialog)
        self._pushButton_shutter.clicked.connect(self.events.shutter_clicked_event)

        self.initialize_vals()
        self.initialize_glview()
        self.initialize_timer()

    # ==============================================
    # rqt requires
    def save_settings(self, plugin_settings, instance_settings):
        def save_setting(_save_instance, _instance_setting_value_str):
            instance_settings.set_value(_instance_setting_value_str, repr(_save_instance))

        save_setting(self._glview.get_view_matrix(), 'view_matrix')

    def restore_settings(self, plugin_settings, instance_settings):
        def restore_setting(_instance_setting_value_str):
            try:
                data = eval(instance_settings.value(_instance_setting_value_str))
            except Exception:
                data = None
            return data

        view_matrix = restore_setting('view_matrix')
        if view_matrix is not None:
            self._glview.set_view_matrix(view_matrix)
        else:
            self.set_default_view()


    def shutdown_plugin(self):
        self.unregister_topic()

    # ==============================================
    # QGLWidget requires
    def set_default_view(self):
        self._glview.makeCurrent()
        self._glview.reset_view()
        self._glview.rotate((0, 0, 1), 45)
        self._glview.rotate((1, 0, 0), -45)
        self._glview.translate((0, 0, -200))

    def update_timeout(self):
        self._glview.makeCurrent()
        self._glview.updateGL()
        glRotated(45, 0, 0, 1)

    def glview_paintGL(self):
        self._glview.paintGL_original()

        gl_painter.draw_basic_objects()

        self.texture = self._glview.get_texture(self.qimage)
        gl_painter.map_texture_on_sphere(self.texture, 1500, 30, 30)


    def glview_mouseReleaseEvent(self, event):
        if event.button() == Qt.RightButton:
            menu = QMenu(self._glview)
            action = QAction(self._glview.tr("Reset view"), self._glview)
            menu.addAction(action)
            action.triggered.connect(self.set_default_view)
            menu.exec_(self._glview.mapToGlobal(event.pos()))


    # ==============================================
    # ROS requires
    def message_callback(self, message):
        self.position = (message.position.x, message.position.y, message.position.z)
        self.orientation = (message.orientation.x, message.orientation.y, message.orientation.z, message.orientation.w)

    def unregister_topic(self):
        if self.subscriber:
            self.subscriber.unregister()

    def subscribe_topic(self, topicName):
        msgClass, self.topicName, _ = get_topic_class(topicName)
        self.subscriber = rospy.Subscriber(self.topicName, msgClass, self.message_callback)


    # ==============================================
    # initialize
    def initialize_vals(self):
        self.position = (0.0, 0.0, 0.0)
        self.orientation = quaternion_about_axis(0.0, (1.0, 0.0, 0.0))
        self.topicName = None
        self.subscriber = None

        self.qimage = None
        self.texture = None

        self.filename = None
        self.jpeg_data = None

    def initialize_glview(self):
        # create GL view
        self._glview = MyGLWidget()
        self._glview.setAcceptDrops(True)

        # backup and replace original paint method
        # self.glView.paintGL is callbacked from QGLWidget
        self._glview.paintGL_original = self._glview.paintGL
        self._glview.paintGL = self.glview_paintGL

        # backup and replace original mouse release method
        self._glview.mouseReleaseEvent_original = self._glview.mouseReleaseEvent
        self._glview.mouseReleaseEvent = self.glview_mouseReleaseEvent

        # add GL view to widget layout
        # http://doc.qt.io/qt-4.8/qgridlayout.html
        self.layout().addWidget(self._glview, 1, 0, 1, 4)

        self.qimage = QImage(self.filename, 'JPEG')  # GL_TEXTURE_2D
        # self.qimage = QImage('/home/matsunolab/Pictures/testimage_big.jpg', 'JPEG')  # GL_TEXTURE_2D

    def initialize_timer(self):
        # updateTimeout is called with interval time
        self.update_timer = QTimer(self)
        self.update_timer.timeout.connect(self.update_timeout)
        # init and start update timer with 40ms (25fps)
        self.update_timer.start(40)















