# -*- coding: cp1252 -*-
'''
Created on 01.07.2015

@author: rainer.jetten
'''
import dataTransfer as dT
from Interfaces.Commands import Commands

'''=============================================================================
@brief:         Class for command execution via CAN bus
============================================================================='''
class CanCommands(Commands):
    '''
    classdocs
    '''
    '--------------------------------------------------------------------------'        
    def __init__(self, myCan, main_win):
        #invoke base class constructor
        self.super = super(self.__class__,self)
        self.super.__init__(True)

        self.main_win = main_win

        # data and data-transport classes
        self.sysParams = self.main_win.sysParams
        self.hwParams = self.main_win.hwParams
        self.TD_Data = self.main_win.TD_Data
        self.FD_Data = self.main_win.FD_Data
        self.AT_Params = self.main_win.AT_Params
        self.AT_Targets = self.main_win.AT_Targets
        self.AT_TL = self.main_win.AT_TL
        
        # Initialize communication classes
        self.canNodes = dT.CanNodes(self.canParams.Node)        # Get Nodes Object
        self.idObj = dT.CanIds()                                # Get Can ID Object
        self.updateNodes()
        self.myCan = myCan
        self.tfunc = dT.TransFunc()

    '''==========================================================================''' 
    def updateNodes(self, Nodes=None):
        if Nodes:
            self.canNodes = dT.CanNodes(Nodes)
        else:        
            self.canNodes = dT.CanNodes(self.canParams.Node)
        self.idObj.nodeId = self.canNodes.getNode()

    '''=============================================================================
    @brief:         Transfer Preprocessing
    ============================================================================='''
    def doTransfer_start(self, cmd_code):
        # hex-id start values
        self.idObj.reservedBit28 = 1   # bit 28:       Reserved (1), always "1"
        self.idObj.msgType = 1         # bit 25-27:    Message-Type: SYNC, REQ, RSP, IND, ACK,...
        self.idObj.direction = 1       # bit 24:       Direction-Bit : 1 Host -> Sensor, 0: Sensor -> Host
        self.idObj.appMsgId = cmd_code # bit 0-7:      Application Message ID

    '''====================================================================
    @brief:   main data transfer function
    ===================================================================='''
    def doTransfer(self, SndLenE = [], SndpL = [], RecLenE_fix = [], RecLenE_nFlex = [], nFlex = 0, delay = 0, rx_cont = False):

        # single-byte transmission is not possible
        # so replace 1 by 2 in byte-length lists,

        SndLenE = [2 if x==1 else x for x in SndLenE]
        RecLenE_fix = [2 if x==1 else x for x in RecLenE_fix]
        RecLenE_nFlex = [2 if x==1 else x for x in RecLenE_nFlex]
        SndLenE = [-2 if x==-1 else x for x in SndLenE]
        RecLenE_fix = [-2 if x==-1 else x for x in RecLenE_fix]
        RecLenE_nFlex = [-2 if x==-1 else x for x in RecLenE_nFlex]

        # send command to Radar-Network
        return self.tfunc.doTransfer(self.myCan, self.idObj, SndLenE, SndpL, RecLenE_fix, RecLenE_nFlex, delay)

    '''=============================================================================
                            Interface Specific Commands
    ============================================================================='''
    def cmd_get_ifc_params(self):

        try:
            RecpL = self.doTransfer(RecLenE_fix = [2,2])
            node = RecpL[0]
            baud = RecpL[1]
        except:
            node = None
            baud = None

        return (node, baud)

    '''=========================================================================='''
    def cmd_set_ifc_params(self):

        node = self.canParams.Node[0]
        baud = self.canParams.Baud
        
        self.doTransfer(SndLenE = [2, 2], SndpL = [node, baud])
        
        return (node, baud)

    '''=========================================================================='''
    def cmd_activate_ifc_params(self):

        self.doTransfer()

    '''=========================================================================='''
    def cmd_restart_system(self):

        self.doTransfer()

