# -*- coding: cp1252 -*-
'''=============================================================================
Module:     TransferFunc_KVW.py

Created on 17.11.2014

@brief:     All functions who needed or used for testing 

@author:    Kai Maulwurf-Just
============================================================================='''

'''=============================================================================
=imports
============================================================================='''
import time
from CRC48 import CRC48

'''=============================================================================
=Constants
============================================================================='''
DEBUG_PRINT = False
CRC_ENABLE  = False
MAX_TRYS    = 50

class TransFunc():
    def __init__(self):
        # create instance of CRC48 class        
        self.crc48 = CRC48()

    def MakeWord(self, lo, hi):
        ret = ((hi & 0xff) << 8) | (lo & 0xff)
        return ret
    
    def MakeLong(self, lo, hi):
        ret = ((hi & 0xffff) << 16) | (lo & 0xffff)
        return ret
    
    def LoByte(self, v):
        ret = v & 0xff
        return ret
    
    def HiByte(self, v):
        ret = (v & 0xff00) >> 8
        return ret
    
    def LoWord(self, v):
        ret = (v & 0xffff)
        return ret
    
    def HiWord(self, v):
        ret = (v & 0xffff0000) >> 16
        return ret
    
    def NetToHost16(self, src):
        val = self.MakeWord(src[0], src[1])
        return val
    
    def HostToNet16(self, src):
        dest = [0]*2
        dest[0] = self.LoByte(src)
        dest[1] = self.HiByte(src)
        return dest
            
    def NetToHost32(self, src):
        val = self.MakeLong(self.MakeWord(src[0], src[1]), self.MakeWord(src[2], src[3]))
        return val
    
    def HostToNet32(self, src):
        dest = [0]*4
        dest[0] = self.LoByte(self.LoWord(src))
        dest[1] = self.HiByte(self.LoWord(src))
        dest[2] = self.LoByte(self.HiWord(src))
        dest[3] = self.HiByte(self.HiWord(src))
        return dest
    
    def DataFromPayload(self, src, lengthEach):
        ptrInd = 0
        ret = []
        for l in lengthEach:
            absL = abs(l)
            if absL == 2:
                ret.append(self.NetToHost16(src[ptrInd:ptrInd+2]))
            elif absL == 4:
                ret.append(self.NetToHost32(src[ptrInd:ptrInd+4]))
            
            if l < 0:
                ret[-1] = self.getSigntVal(v=ret[-1], bits=abs(l)*8)
            ptrInd += absL
        return ret

    '''====================================================================
        @brief:   write int values to Msg-Id-Hex-Value
        
        @param canIds:    Can-Ids instance values inside
        
        @return:          CAN-MsgId as integer
    ===================================================================='''    
    def setBitsToMsgId(self, canIds):
        y = 0x0 # 29 bit long
        
        y += canIds.reservedBit28 << 28
        y += canIds.msgType << 25
        y += canIds.direction << 24
        y += canIds.firstBit << 23
        y += canIds.finalBit << 22
        y += canIds.seqNr << 18
        y += canIds.protMsg << 17
        y += canIds.nodeId << 11
        y += canIds.reserved << 8
        y += canIds.appMsgId << 0

        if DEBUG_PRINT:
            print bin(y)

        return y

    '''====================================================================
        @brief:   get all values from CAN identifier m_dwID
        
        @param m_dwID: message identifier ID
        
        @return:       tuple (CAN-ID's as an instance, sum as hex)
    ===================================================================='''  
    def getCanIds(self, m_dwID):
        canID = CanIds()
        Sum, canID.reservedBit28 = self.getBits(m_dwID, 28, 28, "reservedBit28")
        Sum, canID.msgType       = self.getBits(m_dwID, 25, 27, "msgType", Sum)
        Sum, canID.direction     = self.getBits(m_dwID, 24, 24, "direction", Sum)
        Sum, canID.firstBit      = self.getBits(m_dwID, 23, 23, "firstBit", Sum)
        Sum, canID.finalBit      = self.getBits(m_dwID, 22, 22, "finalBit", Sum)
        Sum, canID.seqNr         = self.getBits(m_dwID, 18, 21, "seqNr", Sum)
        Sum, canID.protMsg       = self.getBits(m_dwID, 17, 17, "protMsg", Sum)
        Sum, canID.nodeId        = self.getBits(m_dwID, 11, 16, "nodeId", Sum)
        Sum, canID.reserved      = self.getBits(m_dwID,  8, 10, "reserved", Sum)
        Sum, canID.appMsgId      = self.getBits(m_dwID,  0,  7, "appMsgId", Sum)
        
        if DEBUG_PRINT:
            print "CAN-ID = %d \t AppMsg-ID = 0x%x"%(int(canID.seqNr, 2), int(canID.appMsgId, 2))
            print "m_dwID: Int = %d"%sum, "Binary = %s"%bin(sum), "Hex = 0x%x"%sum 
            if sum == m_dwID:
                print "Identisch!!!\n"
            else:
                print "Achtung, verschieden -> Fehler!!!\n"
        return canID
    
    '''====================================================================
        @brief:   get bit values in defined range
        
        @param intVal:     intager value where to look in
        @param startBit:   the start bit
        @param stopBit:    the stop bit
        @param label:      label to print as string
        @param preVal:     optional. Range value will be added 
        
        @return:           tuple (preVal + range value, bits from range as string)
    ===================================================================='''    
    def getBits(self, intVal, startBit, stopBit, label, preVal = None):
        y = 0x0
        for i in range(startBit,stopBit+1):
            v = 0
            if (intVal & (1<<i)):
                v = 1
            y |= (v<<i-startBit)
        
        if preVal == None: preVal = y << startBit
        else: preVal += y << startBit
        
        
        bits = bin(y)[2:].zfill(i+1-startBit)
        
        if DEBUG_PRINT:
            print "%s:\tBits %d-%d"%(label, startBit, stopBit), "Len = %d"%(i+1-startBit), "Value = 0x%x\tbits = %s"%(y, bits)
        
        return preVal, bits
    
    
    '''====================================================================
        @brief:   get the signed value from it
        
        @param v:     the value (integer)
        @param bits:  bit length (integer)
        
        @return:      signed value as integer
    ===================================================================='''  
    def getSigntVal(self, v, bits):
        if v >= (1<<(bits-1)):
            v = v - (1<<bits)
        return v

    '''====================================================================
    @brief:   main data transfer function
    ===================================================================='''
    def doTransfer(self, myCan, idObj, SndLenE, SndpL, RecLenE_fix, RecLenE_nFlex, delay = 0.01):

        # remember start time
        t = time.time()

        #======================================================================
        # write data to CAN bus
        load = []
        for i in range(len(SndLenE)):
            if SndLenE[i] == 2:
                load += self.HostToNet16(SndpL[i])
            elif SndLenE[i] == 4:
                load += self.HostToNet32(SndpL[i])

        if CRC_ENABLE:
            # calculate CRC48 checksum of the payload
            self.crc48.reset()
            for byte in load:
                self.crc48.process_byte(byte)

            # append CRC48 checksum to payload
            load += self.crc48.get_crc_bytes()

        # calculate number of data segments (8 bytes each)
        SndMsgN = (len(load) + 7) / 8
        # at least one segment must be transferred
        if SndMsgN == 0:
            SndMsgN = 1

        # add zeros to fill last sequence
        load += (SndMsgN*8 - len(load))*[0x00]
        if DEBUG_PRINT:
            print "PayLoad:", load

        # write data sequences to CAN bus            
        Id = idObj
        for i in range(SndMsgN):
            # prepare sequence ID        
            Id.seqNr = i
            Id.firstBit = int(i == 0)
            Id.finalBit = int(i == SndMsgN-1)
            hexId = self.setBitsToMsgId(Id)
            if DEBUG_PRINT:
                print "HexId-bin:", bin(hexId)

            myCan.WriteCanMsg(hexId, load[i*8 : (i+1)*8])

        time.sleep(delay)

        #======================================================================
        # read data from CAN bus
        m_bData = []        # raw-data container
        m_bDLC = 0          # size of all data
        while 1:                
            a = myCan.ReadCanMsg(MAX_TRYS)
            if a == None:
                raise Exception("Couldn't receive response")

            m_bData += a.m_bData    # array
            m_bDLC += a.m_bDLC      # length of whole message (number)                

            if DEBUG_PRINT:
                self.DebugPrint(a, t, a.m_bData)

            if a.m_dwID & 1<<22 :   # final bit found => exit loop                        
                break

        # check transmitted message ID            
        if idObj.appMsgId != int(self.getCanIds(a.m_dwID).appMsgId, 2):
            raise Exception("Sent msg ID:", idObj.appMsgId, "Received msg ID", int(self.getCanIds(a.m_dwID).appMsgId, 2))

        # calculate expected data size
        absSumRecLenE_fix = sum([abs(i) for i in RecLenE_fix])      # abs() because of signed values
        absSumRecLenE_nFlex = sum([abs(i) for i in RecLenE_nFlex])

        if CRC_ENABLE:

            nFixed = absSumRecLenE_fix + 4*2    # counter + CRC48 code = 4 words
            if nFixed > m_bDLC:
                raise Exception("Expected %d bytes, received %d bytes"%(nFixed, m_bDLC))

            try:
                nFlex = (m_bDLC - nFixed) / absSumRecLenE_nFlex
            except:
                nFlex = 0

            # calculate CRC48 checksum
            self.crc48.reset()
            for byte in m_bData:
                self.crc48.process_byte(byte)
            
            if self.crc48.result():
                raise Exception("CRC48 checksum error")                
            
        else:   # CRC_ENABLE == False
            
            if absSumRecLenE_fix > m_bDLC:
                raise Exception("Expected %d bytes, received %d bytes"%(absSumRecLenE_fix, m_bDLC))

            try:
                nFlex = (m_bDLC - absSumRecLenE_fix) / absSumRecLenE_nFlex
            except:
                nFlex = 0

        # extract data from stream
        RecLenE = RecLenE_fix + nFlex * RecLenE_nFlex
        pLoadRec = self.DataFromPayload(m_bData, RecLenE)

        return pLoadRec

    '''====================================================================
    @brief:   print debug information
    ===================================================================='''
    def DebugPrint(self, a, t, pLoad):
        idVal = self.getCanIds(a.m_dwID)
        print "ID-Hex: 0x%x"%(a.m_dwID), "Prozesszeit: %d ms"%((time.time()-t)*1000.0)
        print "Frame-Format", a.m_bFF, "Data-Length", a.m_bDLC, "Time\t%d ms"%a.m_dwTime
        print "FinalBit:", idVal.finalBit
        print "Value:", pLoad

'''====================================================================
    @brief:   Class instance for CAN-ID's
===================================================================='''
class CanIds():
    def __init__(self):
        self.reservedBit28 = 0   # bit 28:       Reserved (1), always "1", erstmal
        self.msgType = 0         # bit 25-27:    Msg-Type: SYNC, REQ, RSP, IND, ACK,...
        self.direction = 0       # bit 24:       Direction-Bit : 1 Host -> Sensor, 0: Sensor -> Host
        self.firstBit = 0        # bit 23:       first Bit
        self.finalBit = 0        # bit 22:       final Bit
        self.seqNr = 0           # bit 18-21:    Sequence-Nr.: [0 - 15]
        self.protMsg = 0         # bit 17:       ProtoMsg/AppMsg Bit:    SYNC/ACK -> 1, REQ/RSP/IND -> 0
        self.nodeId = 0          # bit 11-16:    Node-ID (0 = BROADCAST)
        self.reserved = 0        # bit 8-10:     Reserved
        self.appMsgId = 0        # bit 0-7:      AppMsgID: PING = 0

'''====================================================================
    @brief:    Class to handle Nodes
===================================================================='''        
class CanNodes():
    def __init__(self, usedNodes):    
        self.usedNodes = usedNodes
        # Actual node is first element in list 
        self.node = self.usedNodes[0]
        
    def getUsedNodes(self):
        return self.usedNodes
    
    def getNode(self, countUp = True, nodNr = None):
        if nodNr == None:
            nodNr = self.getUsedNodes()
        if countUp:
            i = nodNr.index(self.node)
            if i < len(nodNr)-1:
                self.node = nodNr[i+1]
            else:
                self.node = nodNr[0]
        return self.node  
