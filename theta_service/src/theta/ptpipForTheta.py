# -*- coding: utf-8 -*-

import socket
import struct
from ptpipConnectionUtils import PtpipConnectionUtils

DEBUG = False

# https://gist.github.com/tako2/7734472
# https://developers.theta360.com/ja/docs/ptpip_reference/
# http://godoc.org/github.com/atotto/ptpip/ptp

# Operation Codes
PTP_OC_GetDeviceInfo = 0x1001
PTP_OC_OpenSession = 0x1002
PTP_OC_CloseSession = 0x1003
PTP_OC_GetStorageIDs = 0x1004
PTP_OC_GetStorageInfo = 0x1005
PTP_OC_GetNumObjects = 0x1006
PTP_OC_GetObjectHandles = 0x1007
PTP_OC_GetObjectInfo = 0x1008
PTP_OC_GetObject = 0x1009
PTP_OC_GetThumb = 0x100A
PTP_OC_DeleteObject = 0x100B
PTP_OC_InitiateCapture = 0x100E
PTP_OC_GetDevicePropDesc = 0x1014
PTP_OC_GetDevicePropValue = 0x1015
PTP_OC_SetDevicePropValue = 0x1016
PTP_OC_TerminateOpenCapture = 0x1018
PTP_OC_InitiateOpenCapture = 0x101C
PTP_OC_GetResizedImageObject = 0x1022

# Response Codes
PTP_RC_Undefined = 0x2000
PTP_RC_OK = 0x2001
PTP_RC_InvalidTransactionID = 0x2004
PTP_RC_CaptureAlreadyTerminated = 0x2018
PTP_RC_DeviceBusy = 0x2019
PTP_RC_InvalidDevicePropValue = 0x201c
PTP_RC_InvalidParameter = 0x201D

# Object Format Codes
PTP_OFC_Undefined = 0x3000
PTP_OFC_Association = 0x3001
PTP_OFC_EXIF_JPEG = 0x3801
PTP_OFC_JFIF = 0x3808

# Event Codes
PTP_EC_ObjectAdded = 0x4002
PTP_EC_DevicePropChanged = 0x4006
PTP_EC_StoreFull = 0x400a
PTP_EC_CaptureComplete = 0x400d

# Device Properties Codes
PTP_DPC_BatteryLevel = 0x5001
PTP_DPC_WhiteBalance = 0x5005
PTP_DPC_ExposureIndex = 0x500F
PTP_DPC_ExposureBiasCompensation = 0x5010
PTP_DPC_DateTime = 0x5011
PTP_DPC_StillCaptureMode = 0x5013
PTP_DPC_TimelapseNumber = 0x501A
PTP_DPC_TimelapseInterval = 0x501B
PTP_DPC_AudioVolume = 0x502C
PTP_DPC_ErrorInfo = 0xD006
PTP_DPC_ShutterSpeed = 0xD00F
PTP_DPC_GpsInfo = 0xD801
PTP_DPC_AutoPowerOffDelay = 0xD802
PTP_DPC_SleepDelay = 0xD803
PTP_DPC_ChannelNumber = 0xD807
PTP_DPC_CaptureStatus = 0xD808
PTP_DPC_RecordingTime = 0xD809  # m15 only
PTP_DPC_RemainingRecordingTime = 0xD80A  # m15 only

# Error codes
THETA_ERR_InsufficientMemorySpace = 0x00000001
THETA_ERR_FileNumberExceeded = 0x00000004
THETA_ERR_ClockNotSet = 0x00000008
THETA_ERR_ElectromagneticCompassError = 0x00000010
THETA_ERR_InsufficientBatteryForFirmwareUpdate = 0x00000020
THETA_ERR_NoFirmwareForFirmwareUpdate = 0x00000040
THETA_ERR_InvalidFirmware = 0x00000080
THETA_ERR_TemperatureError = 0x80000000
THETA_ERR_RechargingError = 0x40000000
THETA_ERR_OtherErrors = 0x20000000
THETA_ERR_SDAccessError = 0x10000000
THETA_ERR_InternalMemoryAccessError = 0x08000000
THETA_ERR_SDFormatError = 0x04000000
THETA_ERR_InternalMemoryFormatError = 0x02000000
THETA_ERR_SDFault = 0x01000000
THETA_ERR_FirmwareUpdateFailed = 0x00800000
THETA_ERR_ShootingHardwareError = 0x00400000


class PtpipForTheta(PtpipConnectionUtils):
    # -------------------------------------------------------------------------
    def __init__(self, host, name, GUID):
        '''Initialize'''
        self.intervalShotIDList = []
        self.shotModeVal = -1
        PtpipConnectionUtils.__init__(self, host, name, GUID)

    # -------------------------------------------------------------------------
    def OpenSession(self):
        self.Send_PTPCommandRequest('', PTP_OC_OpenSession, self.session_id)
        result, args, payload = self.Wait_PTPCommandResponse()
        if result != PTP_RC_OK:
            print 'Failed'
            print hex(result)
            return 0
        return 1

    # -------------------------------------------------------------------------
    def CloseSession(self):
        self.Send_PTPCommandRequest('', PTP_OC_CloseSession)
        result, args, payload = self.Wait_PTPCommandResponse()
        if result != PTP_RC_OK:
            print 'Failed'
            print hex(result)

    # -------------------------------------------------------------------------
    def GetDeviceInfo(self):
        self.Send_PTPCommandRequest('', PTP_OC_GetDeviceInfo)

        result, args, payload = self.Wait_PTPCommandResponse()
        if result != PTP_RC_OK:
            print 'Failed'
            print hex(result)

    # -------------------------------------------------------------------------
    def GetStorageIDs(self):
        self.Send_PTPCommandRequest('', PTP_OC_GetStorageIDs)
        result, args, payload = self.Wait_PTPCommandResponse()
        if result != PTP_RC_OK:
            print 'Failed'
            print hex(result)
            return []
        return self.unpackInt32Array(payload)

    # -------------------------------------------------------------------------
    def GetStorageInfo(self, storage_id):
        self.Send_PTPCommandRequest('', PTP_OC_GetStorageInfo, storage_id)
        result, args, payload = self.Wait_PTPCommandResponse()
        if result != PTP_RC_OK:
            print 'Failed'
            print hex(result)
            return []
        return payload

    # -------------------------------------------------------------------------
    def GetNumObjects(self, storage_id, obj_format=0, parent_obj=0):
        self.Send_PTPCommandRequest('', PTP_OC_GetNumObjects, storage_id, obj_format, parent_obj)
        result, args, payload = self.Wait_PTPCommandResponse()
        if result != PTP_RC_OK:
            print 'Failed'
            print hex(result)
            return 0
        return args[0]

    # -------------------------------------------------------------------------
    def GetObjectHandles(self, storage_id, obj_format=0, parent_obj=0):
        self.Send_PTPCommandRequest('', PTP_OC_GetObjectHandles, storage_id, obj_format, parent_obj)
        result, args, payload = self.Wait_PTPCommandResponse()
        if result != PTP_RC_OK:
            print 'Failed'
            print hex(result)
            return []
        return self.unpackInt32Array(payload)

    # -------------------------------------------------------------------------
    def GetObjectInfo(self, obj_handle):
        self.Send_PTPCommandRequest('', PTP_OC_GetObjectInfo, obj_handle)
        result, args, payload = self.Wait_PTPCommandResponse()
        if result != PTP_RC_OK:
            print 'Failed'
            print hex(result)
            return []
        # return payload
        return self.unpackObjectInfo(payload)

    # -------------------------------------------------------------------------
    def GetObject(self, obj_handle, resizedFlag=False):
        if resizedFlag:
            self.Send_PTPCommandRequest('', PTP_OC_GetResizedImageObject, obj_handle, 2048, 1024, 0, 0)
        else:
            self.Send_PTPCommandRequest('', PTP_OC_GetObject, obj_handle)

        result, args, payload = self.Wait_PTPCommandResponse()
        if result != PTP_RC_OK:
            print 'Failed'
            print hex(result)
            return []
        return payload

    # -------------------------------------------------------------------------
    def GetThumb(self, obj_handle):
        self.Send_PTPCommandRequest('', PTP_OC_GetThumb, obj_handle)

        result, args, payload = self.Wait_PTPCommandResponse()
        if result != PTP_RC_OK:
            print 'Failed'
            print hex(result)
            return []
        return payload


    # -------------------------------------------------------------------------
    def SetDevicePropValue(self, prop_id, payload):

        self.Send_PTPCommandRequest(payload, PTP_OC_SetDevicePropValue, prop_id)

        result, args, payload = self.Wait_PTPCommandResponse()

        if result != PTP_RC_OK:
            print 'Failed'
            print hex(result)
            return 0
        return 1

    # -------------------------------------------------------------------------
    def GetDevicePropValue(self, prop_id, *args, **kwargs):
        self.Send_PTPCommandRequest('', PTP_OC_GetDevicePropValue, prop_id, *args, **kwargs)

        result, args, payload = self.Wait_PTPCommandResponse()

        if result != PTP_RC_OK:
            print 'Failed'
            print hex(result)
            return -1

        return payload


    ## -------------------------------------------------------------------------
    ## -------------------------------------------------------------------------
    def SingleShotCapture(self):
        if self.shotModeVal != 1:
            self.ChangeSingleShotMode()
            self.shotModeVal = self.unpackInts(self.GetDevicePropValue(PTP_DPC_StillCaptureMode))

        self.Send_PTPCommandRequest('', PTP_OC_InitiateCapture, 0, 0)

        result, args, payload = self.Wait_PTPCommandResponse()
        if result == 0:
            print 'Failed'
            return 0

        handle = 0
        for loop in range(0, 20):
            ptp_event, args = self.Wait_PTPEvent()
            if ptp_event == PTP_EC_CaptureComplete:
                break
            elif ptp_event == PTP_EC_ObjectAdded:
                handle = args[0]
        return handle

    # -------------------------------------------------------------------------
    # interval_msec: 5000(min),6000,7000,8000, ...
    def IntervalShotCapture(self, interval_msec, upper_limit_num):
        if self.shotModeVal != 3:
            self.ChangeIntervalShotMode(interval_msec, upper_limit_num)
            self.shotModeVal = self.unpackInts(self.GetDevicePropValue(PTP_DPC_StillCaptureMode))

        transID = self.Send_PTPCommandRequest('', PTP_OC_InitiateOpenCapture, 0, 0)
        self.intervalShotIDList.append(transID)

        result, args, payload = self.Wait_PTPCommandResponse()
        if result == 0:
            print 'Failed'
            return 0
        return 1

    # -------------------------------------------------------------------------
    def TerminateIntervalShotCapture(self):
        self.Send_PTPCommandRequest('', PTP_OC_TerminateOpenCapture, self.intervalShotIDList[-1])
        # self.Send_PTPCommandRequest('', PTP_OC_TerminateOpenCapture, 0xFFFFFFFF)

        result, args, payload = self.Wait_PTPCommandResponse()
        if result != PTP_RC_OK:
            print 'Failed'
            print hex(result)
            return 0
        return 1


    ## -------------------------------------------------------------------------
    ## -------------------------------------------------------------------------
    # set EV shift(露出補正)
    # EV shift: 2000,1700,1300,1000,700,300,0,-300,-700,-1000,-1300,-1700,-2000
    def SetEVShift(self, ev_shift):
        return self.SetDevicePropValue(PTP_DPC_ExposureBiasCompensation, self.packInt16(ev_shift))

    # -------------------------------------------------------------------------
    # After changing to IntervalShotMode,
    # TimelapseInterval and TimelapseNumber are not allowed to change
    def ChangeIntervalShotMode(self, interval_msec, upper_limit_num):

        # Once change to SingleShotMode to change interval and number
        self.ChangeSingleShotMode()

        self.SetDevicePropValue(PTP_DPC_TimelapseInterval, self.packInt32(interval_msec))
        self.SetDevicePropValue(PTP_DPC_TimelapseNumber, self.packInt16(upper_limit_num))

        return self.SetDevicePropValue(PTP_DPC_StillCaptureMode, self.packInt16(3))

    # -------------------------------------------------------------------------
    def ChangeSingleShotMode(self):
        return self.SetDevicePropValue(PTP_DPC_StillCaptureMode, self.packInt16(1));

    # -------------------------------------------------------------------------
    def SetSleepDelay(self, delay_sec):
        if 0 < delay_sec < 60:
            delay_sec = 60

        return self.SetDevicePropValue(PTP_DPC_SleepDelay, self.packInt16(delay_sec))

    # -------------------------------------------------------------------------
    def SetAutoPowerOffDelay(self, delay_min):
        return self.SetDevicePropValue(PTP_DPC_AutoPowerOffDelay, self.packInt8(delay_min))

    # -------------------------------------------------------------------------
    def SetShotSoundVolume(self, percent):
        return self.SetDevicePropValue(PTP_DPC_AudioVolume, self.packInt32(percent))

    # -------------------------------------------------------------------------
    # 0(random), 1, 6 or 11
    def SetWirelessLANChannel(self, channel):
        if 1 <= channel < 6:
            return self.SetDevicePropValue(PTP_DPC_ChannelNumber, self.packInt8(1))
        elif 6 <= channel < 11:
            return self.SetDevicePropValue(PTP_DPC_ChannelNumber, self.packInt8(6))
        elif 11 <= channel:
            return self.SetDevicePropValue(PTP_DPC_ChannelNumber, self.packInt8(11))
        else:
            return self.SetDevicePropValue(PTP_DPC_ChannelNumber, self.packInt8(0))

    # -------------------------------------------------------------------------
    # 0, 33, 67 or 100 (just 4 steps)
    # https://developers.theta360.com/ja/docs/ptpip_reference/property/battery_level.html
    def GetBatteryLevel(self):
        payload = self.GetDevicePropValue(PTP_DPC_BatteryLevel)

        if len(payload) is 1:
            return self.unpackInt8(payload)
        elif len(payload) is 2:
            return self.unpackInt16(payload)
        elif len(payload) is 4:
            return self.unpackInt32(payload)


    # -------------------------------------------------------------------------
    def GetErrorInfo(self):
        payload = self.GetDevicePropValue(PTP_DPC_ErrorInfo)
        if payload is THETA_ERR_InsufficientMemorySpace:
            return -1
        elif payload is THETA_ERR_FileNumberExceeded:
            return -2
        elif payload is THETA_ERR_ClockNotSet:
            return -3
        elif payload is THETA_ERR_ElectromagneticCompassError:
            return -4
        elif payload is THETA_ERR_InsufficientBatteryForFirmwareUpdate:
            return -5
        elif payload is THETA_ERR_NoFirmwareForFirmwareUpdate:
            return -6
        elif payload is THETA_ERR_InvalidFirmware:
            return -7
        elif payload is THETA_ERR_TemperatureError:
            return -8
        elif payload is THETA_ERR_RechargingError:
            return -9
        elif payload is THETA_ERR_OtherErrors:
            return -10
        elif payload is THETA_ERR_SDAccessError:
            return -11
        elif payload is THETA_ERR_InternalMemoryAccessError:
            return -12
        elif payload is THETA_ERR_SDFormatError:
            return -13
        elif payload is THETA_ERR_InternalMemoryFormatError:
            return -14
        elif payload is THETA_ERR_SDFault:
            return -15
        elif payload is THETA_ERR_FirmwareUpdateFailed:
            return -16
        elif payload is THETA_ERR_ShootingHardwareError:
            return -17
        else:
            return 1


    def testFunc(self):
        for id in self.GetStorageIDs():
            print hex(id)

# payload = self.GetDevicePropValue(PTP_DPC_TimelapseInterval)
#		if len(payload) is 1:
#			print 'int8'
#			print self.unpackInt8(payload)
#		elif len(payload) is 2:
#			print 'int16'
#			print self.unpackInt16(payload)
#		elif len(payload) is 4:
#			print 'int32'
#			print self.unpackInt32(payload)
#		else:
#			print len(payload)
#			print str(payload)


















