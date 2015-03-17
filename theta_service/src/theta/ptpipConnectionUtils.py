# -*- coding: utf-8 -*-
import socket
import struct

DEBUG = False


class PtpipConnectionUtils:
    # -------------------------------------------------------------------------
    def __init__(self, host, name, GUID):
        '''Initialize'''
        self.host = host
        self.port = 15740
        self.name = name
        self.GUID = GUID
        self.command_sock = None
        self.event_sock = None

    # -------------------------------------------------------------------------
    def OpenConnection(self):
        # Init_Command
        self.command_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        try:
            self.command_sock.connect((self.host, self.port))
        except:
            print 'Connection Failed'
            return 0

        self.Send_InitCommandRequest()
        result, self.session_id = self.Wait_InitCommandAck()
        if result == 0:
            print 'InitCommandRequest failed'
            return 0
        if DEBUG:
            print '(session_id = %d)' % self.session_id

        # Init_Event
        self.event_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        try:
            self.event_sock.connect((self.host, self.port))
        except:
            print 'Connection Failed'
            return 0

        self.Send_InitEventRequest(self.session_id)
        result = self.Wait_InitEventAck()
        if result == 0:
            print 'InitEventRequest failed'
            return 0

        self.transaction_id = 0

        return self.session_id

    # -------------------------------------------------------------------------
    def CloseConnection(self):
        if self.command_sock is not None:
            self.command_sock.close()
            self.command_sock = None

        if self.event_sock is not None:
            self.event_sock.close()
            self.event_sock = None

    # -------------------------------------------------------------------------
    def Send_InitCommandRequest(self):
        print 'Send InitCommandRequest'
        payload = ''
        payload += self.packGUID()
        payload += self.packString(self.name)
        payload += self.packInt32(1)

        self.sendCommand(self.command_sock, 1, payload)

    # -------------------------------------------------------------------------
    def Wait_InitCommandAck(self):
        print 'Wait InitCommandAck'
        cmd_id, payload = self.recvResponse(self.command_sock)
        if cmd_id != 2:
            print 'failed'
            return 0, 0

        session_id = self.unpackInt32(payload[0:4])
        target_GUID = self.unpackGUID(payload[4:20])
        target_name = self.unpackString(payload[20:-4])
        # and unknown 4 bytes
        if DEBUG:
            print 'Target GUID : %s' % target_GUID
            print 'Target Name : %s' % target_name

        return 1, session_id

    # -------------------------------------------------------------------------
    def Send_InitEventRequest(self, session_id):
        print 'Send InitEventRequest'
        payload = ''
        payload += self.packInt32(session_id)

        self.sendCommand(self.event_sock, 3, payload)

    # -------------------------------------------------------------------------
    def Wait_InitEventAck(self):
        print 'Wait InitEventAck'
        # sock.settimeout(10)
        cmd_id, payload = self.recvResponse(self.event_sock)
        if cmd_id != 4:
            print 'failed'
            return 0
        return 1

    # -------------------------------------------------------------------------
    def Send_PTPCommandRequest(self, ptp_payload, ptp_cmd, *args, **kwargs):

        self.transaction_id += 1
        # Cmd_Request
        payload = ''
        payload += self.packInt32(1)
        payload += self.packInt16(ptp_cmd)
        payload += self.packInt32(self.transaction_id)
        for arg in args:
            payload += self.packInt32(arg)
        self.sendCommand(self.command_sock, 6, payload)

        if ptp_payload == '':
            return self.transaction_id

        # Start_Data_Packet
        payload = ''
        payload += self.packInt32(self.transaction_id)
        payload += self.packInt32(len(ptp_payload))
        payload += self.packInt32(0)
        self.sendCommand(self.command_sock, 9, payload)

        idx = 0
        next_idx = idx + 200
        while idx < len(ptp_payload):
            payload = ''
            payload += self.packInt32(self.transaction_id)
            payload += ptp_payload[idx:next_idx]
            if next_idx < len(ptp_payload):
                # Data_Packet
                self.sendCommand(self.command_sock, 10, payload)
            else:
                # End_Data_Packet
                self.sendCommand(self.command_sock, 12, payload)
            idx = next_idx
            next_idx += 200

        return self.transaction_id

    # -------------------------------------------------------------------------
    def Wait_PTPCommandResponse(self):
        cmd_id, payload = self.recvResponse(self.command_sock)
        ptp_payload = ''
        if cmd_id == 9:
            # Start_Data_Packet
            transaction_id = self.unpackInt32(payload[0:4])
            ptp_payload_len = self.unpackInt32(payload[4:8])
            while True:
                # Data_Packet or End_Data_Packet
                cmd_id, payload = self.recvResponse(self.command_sock)
                if cmd_id != 10 and cmd_id != 12:
                    return 0, None, None
                temp_id = self.unpackInt32(payload[0:4])
                if temp_id != transaction_id:
                    return 0, None, None
                ptp_payload += payload[4:]
                if len(ptp_payload) >= ptp_payload_len or cmd_id == 12:
                    break
                if DEBUG:
                    print '.'
            # Cmd_Response
            cmd_id, payload = self.recvResponse(self.command_sock)

        if cmd_id != 7:
            return 0, None, None
        ptp_res = self.unpackInt16(payload[0:2])
        transaction_id = self.unpackInt32(payload[2:6])
        ptp_args = []
        idx = 6
        while idx < len(payload):
            ptp_args.append(self.unpackInt32(payload[idx:idx + 4]))
            idx += 4

        if DEBUG:
            print 'PTP Response: 0x%04X' % ptp_res
            self.printArgs(ptp_args)
            print '[Payl]',
            self.printPacket(ptp_payload)

        return ptp_res, ptp_args, ptp_payload

    # -------------------------------------------------------------------------
    def Wait_PTPEvent(self):
        self.event_sock.settimeout(0.5)
        cmd_id, payload = self.recvResponse(self.event_sock)
        if cmd_id != 8:
            return 0, None
        # Event
        ptp_event = self.unpackInt16(payload[0:2])
        transaction_id = self.unpackInt32(payload[2:6])
        ptp_args = []
        idx = 6
        while idx < len(payload):
            ptp_args.append(self.unpackInt32(payload[idx:idx + 4]))
            idx += 4

        return ptp_event, ptp_args

    # -------------------------------------------------------------------------
    def sendCommand(self, sock, cmd_id, payload):
        packet = ''
        packet += self.packInt32(len(payload) + 8)
        packet += self.packInt32(cmd_id)
        packet += payload

        if DEBUG:
            print '[SEND]',
            self.printPacket(packet)

        sock.send(packet)

    # -------------------------------------------------------------------------
    def recvResponse(self, sock):
        packet = ''
        # packet length
        try:
            recv_data = sock.recv(4)
        except:
            if DEBUG:
                print '.'  # recv timeout
            return -1, None
        if recv_data is None or len(recv_data) != 4:
            return 0, None
        packet_len = self.unpackInt32(recv_data)
        if DEBUG:
            print 'recv packet len = %d' % packet_len
        if packet_len < 8:
            return 0, None
        packet += recv_data

        # command
        try:
            recv_data = sock.recv(4)
        except:
            if DEBUG:
                print 'recv timeout, len=%d' % packet_len
            return -1, None
        if recv_data is None or len(recv_data) != 4:
            return 0, None
        cmd_id = self.unpackInt32(recv_data)
        if DEBUG:
            print 'recv cmd id = %d' % cmd_id
        packet += recv_data

        # payload
        packet_len -= 8
        if packet_len == 0:
            recv_data = None
        else:
            try:
                recv_data = sock.recv(packet_len)
            except:
                if DEBUG:
                    print 'recv timeout, len=%d, cmd=%d' % (packet_len + 8,
                                                            cmd_id)
                    return -1, None
            if recv_data is None or len(recv_data) != packet_len:
                return 0, None
            packet += recv_data

        if DEBUG:
            print '[RECV]',
            self.printPacket(packet)

        return cmd_id, recv_data


    # -------------------------------------------------------------------------
    def printPacket(self, packet):
        tab_idx = 1
        for ch in packet:
            print '%02X' % ord(ch),
            if (tab_idx % 8) == 0:
                print '\n	  ',
            tab_idx += 1
        print ''

    # -------------------------------------------------------------------------
    def printArgs(self, args):
        print '%d ARGS' % len(args)
        idx = 0
        for arg in args:
            print '[ARGS %d] 0x%08X' % (idx, arg)
            idx += 1

    # -------------------------------------------------------------------------
    def packGUID(self):
        data = ''
        for val in self.GUID.split('-'):
            idx = 0
            while idx < len(val):
                data += chr(int(val[idx:idx + 2], 16))
                idx += 2
        return data

    # -------------------------------------------------------------------------
    def unpackGUID(self, packet):
        guid = ''
        idx = 0
        for ch in packet:
            if idx == 4 or idx == 6 or idx == 8 or idx == 10:
                guid += '-'
            guid += '%02x' % ord(ch)
            idx += 1
        return guid

    # -------------------------------------------------------------------------
    def packString(self, str):
        data = ''
        for ch in str:
            data += ch
            data += '\x00'
        data += '\x00'
        data += '\x00'
        return data

    # -------------------------------------------------------------------------
    def unpackString(self, packet):
        str = ''
        idx = 0
        for ch in packet:
            if (idx & 1) == 0:
                str += ch
            idx += 1
        return str

    # -------------------------------------------------------------------------
    def unpackInt32(self, payload):
        return struct.unpack('<I', payload)[0]

    # -------------------------------------------------------------------------
    def packInt32(self, val):
        return struct.pack('<I', val)

    # -------------------------------------------------------------------------
    def unpackInt16(self, payload):
        return struct.unpack('<H', payload)[0]

    # -------------------------------------------------------------------------
    def packInt16(self, val):
        if val < 0:
            val = 0x10000 + val
        return struct.pack('<H', val)

    # -------------------------------------------------------------------------
    def unpackInt8(self, payload):
        return struct.unpack('<B', payload)[0]

    # -------------------------------------------------------------------------
    def packInt8(self, val):
        return struct.pack('<B', val)

    # -------------------------------------------------------------------------
    def unpackInt32Array(self, payload):
        num_items = self.unpackInt32(payload[0:4])
        if num_items == 0 or (num_items * 4) > (len(payload) - 4):
            return []
        items = []
        idx = 4
        while idx < len(payload):
            items.append(self.unpackInt32(payload[idx:idx + 4]))
            idx += 4
        return items

    # -------------------------------------------------------------------------
    def unpackInts(self, payload):

        if isinstance(payload, int):
            return payload
        elif len(payload) is 1:
            return self.unpackInt8(payload)
        elif len(payload) is 2:
            return self.unpackInt16(payload)
        elif len(payload) is 4:
            return self.unpackInt32(payload)
        else:
            return str(payload)


    # -------------------------------------------------------------------------
    def unpackPTPString(self, payload):
        len = ord(payload[0])
        if len == 0:
            return ''
        end = (len * 2 - 1)
        return self.unpackString(payload[1:end])

    # -------------------------------------------------------------------------
    def unpackObjectInfo(self, payload):
        info = {}
        info['StorageID'] = self.unpackInt32(payload[0:4])
        info['ObjectFormat'] = self.unpackInt16(payload[4:6])
        info['ProtectionStatus'] = self.unpackInt16(payload[6:8])
        info['ObjectCompressedSize'] = self.unpackInt32(payload[8:12])
        info['ThumbFormat'] = self.unpackInt16(payload[12:14])
        info['ThumbCompressedSize'] = self.unpackInt32(payload[14:18])
        info['ThumbPixWidth'] = self.unpackInt32(payload[18:22])
        info['ThumbPixHeight'] = self.unpackInt32(payload[22:26])
        info['ImagePixWidth'] = self.unpackInt32(payload[26:30])
        info['ImagePixHeight'] = self.unpackInt32(payload[30:34])
        info['ImageBitDepth'] = self.unpackInt32(payload[34:38])
        info['ParentObject'] = self.unpackInt32(payload[38:42])
        info['AssociationType'] = self.unpackInt16(payload[42:44])
        info['AssociationDesc'] = self.unpackInt32(payload[44:48])
        info['SequenceNumber'] = self.unpackInt32(payload[48:52])
        idx = 52
        info['Filename'] = self.unpackPTPString(payload[idx:])
        idx += ord(payload[idx]) * 2 + 1
        info['CaputureDate'] = self.unpackPTPString(payload[idx:])
        idx += ord(payload[idx]) * 2 + 1
        info['ModificationDate'] = self.unpackPTPString(payload[idx:])
        idx += ord(payload[idx]) * 2 + 1
        info['Keywords'] = self.unpackPTPString(payload[idx:])
        return info













































