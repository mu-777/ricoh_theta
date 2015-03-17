# -*- coding: utf-8 -*-
from ptpipForTheta import PtpipForTheta


DEBUG = False
# randomly decided
gGloballyUniqueIdentifier = '8a7ab04f-ebda-4f33-8649-8bf8c1cdc838'

'''
The basic idea:
Any images and folders in the theta360 are "object"

The flow:
1. Get the idx of the object that you want to deal with
2. Access the data using the idx to get the image, info and anything
'''

class Theta(PtpipForTheta):
    # -------------------------------------------------------------------------
    def __init__(self):
        '''Initialize'''
        PtpipForTheta.__init__(self, '192.168.1.1', 'THETA', gGloballyUniqueIdentifier)

    # -------------------------------------------------------------------------
    def open(self):
        if self.OpenConnection() == 0:
            # Failed to open connection
            return False
        if self.OpenSession() == 0:
            # Failed to open session
            return False
        return True

    # -------------------------------------------------------------------------
    def close(self):
        self.CloseSession()
        self.CloseConnection()

    # -------------------------------------------------------------------------
    def shutter(self):
        return self.SingleShotCapture()

    # -------------------------------------------------------------------------
    # interval_msec: 5000(minimum),6000,7000,8000, ...
    def start_interval_shot(self, interval_msec=-1, upper_limit_num_of_images=-1):
        if interval_msec < 0:
            interval_msec = 5000
        if upper_limit_num_of_images < 0:
            upper_limit_num_of_images = 0

        return self.IntervalShotCapture(interval_msec, upper_limit_num_of_images)

    # -------------------------------------------------------------------------
    def stop_interval_shot(self):
        print 'stop!!!'
        return self.TerminateIntervalShotCapture()

    # -------------------------------------------------------------------------
    def grab_currentest_image(self, resize_flag=False):
        obj_idx = self.get_num_of_objs() - 1
        if resize_flag:
            return self.get_resized_object(obj_idx)
        else:
            return self.get_object(obj_idx)

    # -------------------------------------------------------------------------
    def get_num_of_files(self):
        ids = self.GetStorageIDs()
        if len(ids) == 0:
            return 0
        return self.GetNumObjects(ids[0])

    # -------------------------------------------------------------------------
    def get_num_of_objs(self):
        self.ids = self.GetStorageIDs()
        if len(self.ids) == 0:
            return 0
        self.handles = self.GetObjectHandles(self.ids[0])
        return len(self.handles)

    # -------------------------------------------------------------------------
    def get_info(self, idx, debug_flag=DEBUG):
        info = self.GetObjectInfo(self.handles[idx])
        if debug_flag:
            print 'filename: %s' % info['Filename']
            print 'object format: 0x%04X' % info['ObjectFormat']
            print 'object size: %d' % info['ObjectCompressedSize']
            print 'thumbnail size: %d' % info['ThumbCompressedSize']
            print 'seq. no.: %d' % info['SequenceNumber']
            print 'capture date: %s' % info['CaputureDate']
        return info

    # -------------------------------------------------------------------------
    def get_thumbnail(self, idx):
        return self.GetThumb(self.handles[idx])

    # -------------------------------------------------------------------------
    def get_object(self, idx):
        return self.GetObject(self.handles[idx])

    # -------------------------------------------------------------------------
    def get_resized_object(self, idx):
        return self.GetObject(self.handles[idx], True)

    # -------------------------------------------------------------------------
    def write_local(self, filename, image):
        f = open(filename, 'wb')
        f.write(image)
        f.close()

    # -------------------------------------------------------------------------
    def set_init_settings(self):
        self.SetWirelessLANChannel(1)
        self.SetSleepDelay(0)
        self.SetAutoPowerOffDelay(0)
        self.SetShotSoundVolume(100)



