# -*- coding: cp1252 -*-

import time
from Interfaces import ConversionFuncs as conv
from Interfaces.Commands import Commands
from Interfaces.Commands import CommandError

'''==============================================================================
    @brief:     Class for read write operations on a FT2232-Device using its
                MPSSE-functionality to support an SPI interface (new version)
    
    @author:    IMST GmbH
    @date:      30.09.2015
=============================================================================='''        
class SPI_Commands(Commands):
    '''
    classdocs
    '''
    '--------------------------------------------------------------------------'        
    def __init__(self, port, main_win):
        #invoke base class constructor
        self.super = super(self.__class__,self)
        self.super.__init__(False)

        self.main_win = main_win
        # generate an interface to SPI port        
        self.port = port
        self.transmitOk = True
        self.delay_set_sys_params = 0.01

        # data and data-transport classes
        self.sysParams = self.main_win.sysParams
        self.hwParams = self.main_win.hwParams
        self.TD_Data = self.main_win.TD_Data
        self.FD_Data = self.main_win.FD_Data
        self.AT_Params = self.main_win.AT_Params
        self.AT_Targets = self.main_win.AT_Targets
        self.AT_TL = self.main_win.AT_TL

    '''====================================================================
        @brief: split a string message into a set of sub-messages according 
                to the given structure
                
        @param message: text message
        
        @param struct: byte structure of the message, e.g. [2,2,4,8]
        
        @return: list of sub-messages
    ===================================================================='''  
    def split_message(self, message, struct):
        sub_msg = []
        for n in map(abs,map(int,struct)):
            sub_msg.append(message[:n])
            message = message[n:]
    
        return sub_msg

    '--------------------------------------------------------------------------'        
    def waitDly(self, time_ref, delay):
        
        diff_time = time.time() - time_ref     # [s] 
        if diff_time < delay:
            time.sleep(delay-diff_time)
    
        return time.time()
    
    '''=============================================================================
    @brief:         Transfer Preprocessing
    @note:          Check connection and send handshake
    ============================================================================='''
    def doTransfer_start(self, cmd_code):

        if self.port.is_connected() == False:
            raise SpiError("Connection Error")

        self.transmitOk = True

        if not self.port.start_command(cmd_code):
            self.transmitOk = False

    '''=============================================================================
    @brief:         Transfer Postprocessing
    @note:          Receive handshake except after CPU reset
    ============================================================================='''
    def doTransfer_end(self, cmd_code):

        if not self.port.exit_command(cmd_code):
            self.transmitOk = False

        if not self.transmitOk:
            raise SpiError(("Synchronization Error in Command: %s")%hex(cmd_code))

    '''=============================================================================
    @brief:         Receive Preprocessing
    @note:          Wait for sync word before reception
    ============================================================================='''
    def doTransfer_rxstart(self):
        if not self.port.read_sync_word():
            raise SpiError("Error in reading synch_word")

    '''=============================================================================
        @brief: interface specific data transfer function

        @param: SndLenE: List with length of items to be sent
        @param: SndpL: List items to be sent
        @param: RecLenE_fix: List with length of items to be received once
        @param: RecLenE_nFlex: List with length of items to be received repeatedly
        @param: nFlex: Number of repeatedly received items
        @param: delay: Delay time between transmission and reception in ms

        @note:  The type of the length element determines the way of conversion
                before transmission or after reception.
                - numerical string: no conversion 
                - negative number: conversion from or into a signed integer
                - positive number: conversion from or into an unsigned integer
                The value of integer elements may be 1,2,4,8 or their negatives.
    ============================================================================='''
    def doTransfer(self, SndLenE = [], SndpL = [], RecLenE_fix = [], RecLenE_nFlex = [], nFlex = 0, delay = 0):

        # single-byte transmission is not possible
        # so replace 1 by 2 in byte-length lists,

        SndLenE = [2 if x==1 else x for x in SndLenE]
        RecLenE_fix = [2 if x==1 else x for x in RecLenE_fix]
        RecLenE_nFlex = [2 if x==1 else x for x in RecLenE_nFlex]
        SndLenE = [-2 if x==-1 else x for x in SndLenE]
        RecLenE_fix = [-2 if x==-1 else x for x in RecLenE_fix]
        RecLenE_nFlex = [-2 if x==-1 else x for x in RecLenE_nFlex]
        
        # send message
        SndMsgLen = sum(map(abs,map(int,SndLenE)))
        if SndMsgLen > 0:

            # build payload
            msg = ""
            for (item, item_len) in zip(SndpL, SndLenE):
                if type(item_len) is str:
                    msg+=str(item)
                elif item_len == 1:
                    msg+=conv.u8_to_string(item)
                elif item_len == 2:
                    msg+=conv.u16_to_string(item)
                elif item_len == 4:
                    msg+=conv.u32_to_string(item)
                elif item_len == 8:
                    msg+=conv.u64_to_string(item)
                elif item_len == -1:
                    msg+=conv.int8_to_string(item)
                elif item_len == -2:
                    msg+=conv.int16_to_string(item)
                elif item_len == -4:
                    msg+=conv.int32_to_string(item)
                elif item_len == -8:
                    msg+=conv.int64_to_string(item)

            # send payload
            if self.port.transmit(msg) == False:
                raise SpiError("Transmission Error")

        # wait before reception
        if delay > 0:
            time.sleep(delay) 

        # items received once
        RecMsgLen = sum(map(abs,map(int,RecLenE_fix)))
        RecpL = []
        if RecMsgLen > 0:

            # receive payload
            msg = self.port.receive(RecMsgLen)
            if len(msg) != RecMsgLen:
                raise SpiError("Wrong message length")

            # split payload according to the structure
            sub_msg = self.split_message(msg, RecLenE_fix)

            # convert elements of sub_msg and store in payload
            for item_len in RecLenE_fix:
                if type(item_len) is str:
                    RecpL.append(sub_msg.pop(0))
                elif item_len == 1:
                    RecpL.append(conv.string_to_u8(sub_msg.pop(0)))
                elif item_len == 2:
                    RecpL.append(conv.string_to_u16(sub_msg.pop(0)))
                elif item_len == 4:
                    RecpL.append(conv.string_to_u32(sub_msg.pop(0)))
                elif item_len == 8:
                    RecpL.append(conv.string_to_u64(sub_msg.pop(0)))
                elif item_len == -1:
                    RecpL.append(conv.string_to_int8(sub_msg.pop(0)))
                elif item_len == -2:
                    RecpL.append(conv.string_to_int16(sub_msg.pop(0)))
                elif item_len == -4:
                    RecpL.append(conv.string_to_int32(sub_msg.pop(0)))
                elif item_len == -8:
                    RecpL.append(conv.string_to_int64(sub_msg.pop(0)))
        
        # repeatedly received items
        RecMsgLen = nFlex * sum(map(abs,map(int,RecLenE_nFlex)))
        if RecMsgLen > 0:

            # receive payload
            msg = self.port.receive(RecMsgLen)
            if len(msg) != RecMsgLen:
                raise SpiError("Wrong message length")

            # split payload according to the structure
            sub_msg = self.split_message(msg, RecLenE_nFlex*nFlex)

            # convert elements of sub_msg and store in payload
            for _ in range(nFlex):
                for item_len in RecLenE_nFlex:
                    if type(item_len) is str:
                        RecpL.append(sub_msg)
                    elif item_len == 1:
                        RecpL.append(conv.string_to_u8(sub_msg.pop(0)))
                    elif item_len == 2:
                        RecpL.append(conv.string_to_u16(sub_msg.pop(0)))
                    elif item_len == 4:
                        RecpL.append(conv.string_to_u32(sub_msg.pop(0)))
                    elif item_len == 8:
                        RecpL.append(conv.string_to_u64(sub_msg.pop(0)))
                    elif item_len == -1:
                        RecpL.append(conv.string_to_int8(sub_msg.pop(0)))
                    elif item_len == -2:
                        RecpL.append(conv.string_to_int16(sub_msg.pop(0)))
                    elif item_len == -4:
                        RecpL.append(conv.string_to_int32(sub_msg.pop(0)))
                    elif item_len == -8:
                        RecpL.append(conv.string_to_int64(sub_msg.pop(0)))

        return RecpL

    '''=============================================================================
        @brief: Receive array of signed 32-bit integer values

        @param: nItems: Number of values

        @note:  The type of the length element determines the way of conversion
                before transmission or after reception.
                - numerical string: no conversion 
                - negative number: conversion from or into a signed integer
                - positive number: conversion from or into an unsigned integer
                The value of integer elements may be 1,2,4,8 or their negatives.
    ============================================================================='''
    def doReceive_int32(self, nItems = 0):
        # Receive data
        msg_len = nItems * 4
        msg = self.port.receive(msg_len)
        if len(msg) != msg_len:
            raise SpiError("Wrong message length (2)")
        # convert data to int32
        sub_msg = self.split_message(msg, [4]*nItems)
        return map(conv.string_to_int32, sub_msg)

    '''=========================================================================='''
    def cmd_get_ifc_params(self, cmd_code):
        pass

    '''=========================================================================='''
    def cmd_set_ifc_params(self, cmd_code):
        pass

    '''=========================================================================='''
    def cmd_activate_ifc_params(self, cmd_code):
        pass

    '''=========================================================================='''
    def cmd_restart_system(self):
        self.doTransfer()
        time.sleep(2)
    
'''============================================================================'''
class SpiError(CommandError):
    def __init__(self, value):
        self.value = value
    def __str__(self):
        return repr(self.value)
