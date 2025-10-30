# -*- coding: cp1252 -*-
'''

Interface-independent base class for commands

Created on 19.08.2021

@author: MH @ IMST
'''
from math import pi
from Interfaces import ConversionFuncs as conv
from time import time as currenttime

'''===============================================================================
=Constants
==============================================================================='''

# requested type of FD data
DT_MAGN = 0
DT_MAGN_PHASE = 1
DT_REAL_IMAG = 2
DT_MAGN_ANGLE = 3

MAX_CHANNELS = 4
TD_SAMPLES = 1024
FD_SAMPLES = 513

# command delays
SYS_PARAMS_DELAY = 0.01  # [s]

'''=============================================================================
@brief:         Class with command list for every supported command of the radar
                module.
@note:          Functions can be entered by an execute function. The command names
                are the same as in the firmware.
============================================================================='''


class Commands(object):
    '''====================================================================
        @brief: Constructor for Command Base Class
                
        @param single_transfer:
                    True:   The entire command has to be transferred in
                            one packet
                    False:  The command transfer can be split into several
                            packets
    ===================================================================='''

    def __init__(self, single_transfer):

        self.single_transfer = single_transfer
        self.delay_set_sys_params = 0

        # dictionary with command IDs, command codes and command functions
        self.cmd_list = {}
        # System Commands
        self.cmd_list["CMDID_PING"] = (0x0000, self.cmd_ping)
        self.cmd_list["CMDID_GETTIME"] = (0x0001, self.cmd_getTime)
        self.cmd_list["CMDID_SETTIME"] = (0x0002, self.cmd_setTime)
        self.cmd_list["CMDID_SEND_INFO"] = (0x0012, self.cmd_get_sys_info)
        self.cmd_list["CMDID_RST_PARAMS"] = (0x0027, self.cmd_rst_sys_params)
        self.cmd_list["CMDID_SETUP"] = (0x0028, self.cmd_set_sys_params)
        self.cmd_list["CMDID_SEND_PARAMS"] = (0x0029, self.cmd_get_sys_params)
        self.cmd_list["CMDID_SEND_SENSORDATA"] = (0x002B, self.cmd_get_temp_power)
        self.cmd_list["CMDID_RESTART_CPU"] = (0x002C, self.cmd_restart_system)

        # Raw Data Commands
        self.cmd_list["CMDID_UP_RMP"] = (0x0040, self.cmd_up_ramp)
        self.cmd_list["CMDID_DN_RMP"] = (0x0041, self.cmd_down_ramp)
        self.cmd_list["CMDID_GAP"] = (0x0042, self.cmd_gap)
        self.cmd_list["CMDID_TDATA"] = (0x0043, self.cmd_get_td_data)
        self.cmd_list["CMDID_FDATA"] = (0x0044, self.cmd_get_fd_data)
        self.cmd_list["CMDID_UP_RMP_TD"] = (0x0045, self.cmd_get_td_data)
        self.cmd_list["CMDID_UP_RMP_FD"] = (0x0046, self.cmd_get_fd_data)
        self.cmd_list["CMDID_DN_RMP_TD"] = (0x0047, self.cmd_get_td_data)
        self.cmd_list["CMDID_DN_RMP_FD"] = (0x0048, self.cmd_get_fd_data)
        self.cmd_list["CMDID_GP_TD"] = (0x0049, self.cmd_get_td_data)
        self.cmd_list["CMDID_GP_FD"] = (0x004A, self.cmd_get_fd_data)

        # Air Tracker Commands
        self.cmd_list["CMDID_AT_SET_PAR"] = (0x0070, self.cmd_set_AT_params)
        self.cmd_list["CMDID_AT_MEASURE"] = (0x0071, self.cmd_do_AT_meas)
        self.cmd_list["CMDID_AT_SEND_RES"] = (0x0072, self.cmd_get_AT_data)
        self.cmd_list["CMDID_AT_SEND_REF"] = (0x0073, self.cmd_get_AT_thresh)
        self.cmd_list["CMDID_AT_SEND_PAR"] = (0x0074, self.cmd_get_AT_params)
        self.cmd_list["CMDID_AT_START"] = (0x0075, self.cmd_AT_auto_start)
        self.cmd_list["CMDID_AT_STOP"] = (0x0076, self.cmd_AT_auto_stop)
        self.cmd_list["CMDID_AT_TL"] = (0x0077, self.cmd_get_AT_target_list)

    '''=========================================================================='''

    def getSupportedCmds(self):
        # print "\nList of all commands which are supported by this Radar-Module:"
        for cmd_id in self.cmd_list:
            if self.cmd_list[cmd_id][1]:
                print(cmd_id)

    '''=========================================================================='''

    # Function to execute the commands of cmd_list with optional input parameters
    # and optional output parameters
    def execute_cmd(self, cmdID, *opt):

        if not cmdID in self.cmd_list:
            raise CommandError(("Invalid Command ID: %s") % cmdID)

        print
        "--> Execute Command ID: " + cmdID

        cmd_params = self.cmd_list[cmdID]  # Get the right command if supported
        cmd_code = cmd_params[0]  # Get command code which starts the function in the radar module
        cmd_func = cmd_params[1]  # Get the function which starts processing

        # pre-processing
        self.doTransfer_start(cmd_code)

        # invoke command function
        return_val = cmd_func(*opt)

        # post-processing
        self.doTransfer_end(cmd_code)

        # Return result if any
        return return_val

    '''=============================================================================
    @brief:         Transfer Preprocessing
    ============================================================================='''

    def doTransfer_start(self, cmd_code):
        pass

    '''=============================================================================
    @brief:         Transfer Postprocessing
    ============================================================================='''

    def doTransfer_end(self, cmd_code):
        pass

    '''=============================================================================
    @brief:         Receive Preprocessing
    ============================================================================='''

    def doTransfer_rxstart(self):
        pass

    '''=========================================================================='''

    def cmd_dummy(self):
        # do nothing
        pass

    '''=============================================================================
                                    System Functions
    ============================================================================='''

    def cmd_ping(self):

        return True

    '''=========================================================================='''

    def cmd_getTime(self):

        RecpL = self.doTransfer(RecLenE_fix=[2, 2, 2, 2])

        # convert time to Unix timestamp
        return conv.TimeStamp_NetToHost(RecpL)

    '''=========================================================================='''

    def cmd_setTime(self):

        # get current time as Unix timestamp and convert
        # it into a payload of appropriate format
        secs = currenttime()

        self.doTransfer(SndLenE=[2, 2, 2, 2], SndpL=conv.TimeStamp_HostToNet(secs))

        return secs

    '''=========================================================================='''

    def cmd_get_sys_info(self):

        RecpL = self.doTransfer(RecLenE_fix=[4, 4, 4, 4, 4, 2, 2, 2, 4, 4, -4, -4])

        self.hwParams.fwVersion = RecpL.pop(0)
        self.hwParams.fwRevision = RecpL.pop(0)
        self.hwParams.sntID = RecpL.pop(0)
        self.hwParams.basebandID = RecpL.pop(0)
        self.hwParams.frontendID = RecpL.pop(0)
        self.hwParams.availChannels = RecpL.pop(0)
        self.hwParams.availAlgos = RecpL.pop(0)
        self.hwParams.usedHardware = RecpL.pop(0)
        self.hwParams.radarNumber = RecpL.pop(0)
        self.hwParams.flashDate = RecpL.pop(0)
        self.hwParams.phaseOffsetAzi = RecpL.pop(0)
        self.hwParams.phaseOffsetEle = RecpL.pop(0)

    '''=========================================================================='''

    def cmd_rst_sys_params(self):

        self.doTransfer()

    '''=========================================================================='''

    def cmd_set_sys_params(self):

        SndLenE = 7 * [1] + [2, 1, 2, 2, 2, 1]

        SndpL = []
        SndpL.append(self.sysParams.diffMode)
        SndpL.append(self.sysParams.t_ramp)
        SndpL.append(self.sysParams.zero_pad)
        SndpL.append(self.sysParams.FFT_data_type)
        SndpL.append(self.sysParams.frontendEn)
        SndpL.append(self.sysParams.powerSaveEn)
        SndpL.append(self.sysParams.norm)
        SndpL.append(self.sysParams.active_RX_ch)
        SndpL.append(self.sysParams.advanced)
        SndpL.append(self.sysParams.freq_points)
        SndpL.append(self.sysParams.minFreq)
        SndpL.append(self.sysParams.manualBW)
        SndpL.append(self.sysParams.atten)

        self.doTransfer(SndLenE=SndLenE, SndpL=SndpL, delay=SYS_PARAMS_DELAY)

    '''=========================================================================='''

    def cmd_get_sys_params(self):

        RecLenE_fix = 7 * [1] + [2, 1, 2, 2, 2, 1] + [4, 4, 4]

        RecpL = self.doTransfer(RecLenE_fix=RecLenE_fix)

        self.sysParams.diffMode = RecpL.pop(0)
        self.sysParams.t_ramp = RecpL.pop(0)
        self.sysParams.zero_pad = RecpL.pop(0)
        self.sysParams.FFT_data_type = RecpL.pop(0)
        self.sysParams.frontendEn = RecpL.pop(0)
        self.sysParams.powerSaveEn = RecpL.pop(0)
        self.sysParams.norm = RecpL.pop(0)
        self.sysParams.active_RX_ch = RecpL.pop(0)
        self.sysParams.advanced = RecpL.pop(0)
        self.sysParams.freq_points = RecpL.pop(0)
        self.sysParams.minFreq = RecpL.pop(0)
        self.sysParams.manualBW = RecpL.pop(0)
        self.sysParams.atten = RecpL.pop(0)

        self.sysParams.tic = RecpL.pop(0)
        self.sysParams.doppler = RecpL.pop(0)
        self.sysParams.freq_bin = RecpL.pop(0)

    '''=========================================================================='''

    def cmd_get_temp_power(self):

        RecLenE_fix = [-4, -4, -4]

        RecpL = self.doTransfer(RecLenE_fix=RecLenE_fix)

        temp = RecpL[0]
        power = float(RecpL[1]) / 10
        temp2 = RecpL[2]

        return temp, power, temp2

    '''=============================================================================
                                    Raw Data Functions
    ============================================================================='''

    def cmd_up_ramp(self):

        self.doTransfer(delay=self.sysParams.t_ramp / 1000.)  # ramp time in [s]

    '''=========================================================================='''

    def cmd_down_ramp(self):

        self.doTransfer(delay=self.sysParams.t_ramp / 1000.)  # ramp time in [s]

    '''=========================================================================='''

    def cmd_gap(self):

        self.doTransfer(delay=self.sysParams.t_ramp / 1000.)  # ramp time in [s]

    '''=========================================================================='''

    def cmd_get_td_data(self):

        self.TD_Data.data = []  # empty data container

        if self.single_transfer:
            RecpL = self.doTransfer(RecLenE_fix=[2], RecLenE_nFlex=[-4])
        else:
            RecpL = self.doTransfer(RecLenE_fix=[2])

        act_chan = RecpL[0]
        if act_chan < 0 or act_chan > MAX_CHANNELS:
            raise CommandError("Invalid number of active channels: %s" % act_chan)
        self.TD_Data.nValues = act_chan * TD_SAMPLES

        if self.single_transfer:
            # extract time information
            self.TD_Data.time0 = RecpL[-2] / 100.
            self.TD_Data.time1 = RecpL[-1] / 100.
            del RecpL[-2:]
            # extract time domain data
            self.TD_Data.data = RecpL[1:]
        else:
            # read time domain data
            self.TD_Data.data = self.doReceive_int32(nItems=self.TD_Data.nValues)

            # read time information
            RecpL = self.doTransfer(RecLenE_fix=[4, 4])
            self.TD_Data.time0 = RecpL[0] / 100.
            self.TD_Data.time1 = RecpL[1] / 100.

    '''=========================================================================='''

    def cmd_get_fd_data(self):

        RecLenE_fix = [1, 4, 4] + [-4] * MAX_CHANNELS * 2 + [1, 2, 2]

        if self.single_transfer:
            RecpL = self.doTransfer(RecLenE_fix=RecLenE_fix, RecLenE_nFlex=[-4])
        else:
            RecpL = self.doTransfer(RecLenE_fix=RecLenE_fix)

        self.FD_Data.clear()  # empty data container

        self.FD_Data.datType = RecpL[0]
        self.FD_Data.time0 = RecpL[1] / 100.
        self.FD_Data.time1 = RecpL[2] / 100.
        self.FD_Data.minValues = RecpL[3:7]
        self.FD_Data.maxValues = RecpL[7:11]
        self.FD_Data.overload = RecpL[11]
        act_chan = RecpL[12]
        self.FD_Data.nSamples = RecpL[13]

        if act_chan < 1 or act_chan > MAX_CHANNELS:
            raise CommandError("Invalid number of active channels: %s" % act_chan)
        if self.FD_Data.nSamples < 1 or self.FD_Data.nSamples > FD_SAMPLES:
            raise CommandError("Invalid number of samples: %s" % self.FD_Data.nSamples)

        if self.single_transfer:
            # extract time information
            self.FD_Data.time2 = RecpL[-1] / 100.
            del RecpL[-1]
            # extract FD data
            self.FD_Data.data = RecpL[14:]
        else:
            # read FD data
            if self.FD_Data.datType == DT_MAGN:
                num_smpl = act_chan * self.FD_Data.nSamples
            else:
                num_smpl = act_chan * self.FD_Data.nSamples * 2

            self.FD_Data.data = self.doReceive_int32(nItems=num_smpl)

            # read time information
            RecpL = self.doTransfer(RecLenE_fix=[4])
            self.FD_Data.time2 = RecpL[0] / 100.  # [ms]

    '''=============================================================================
                                Air Tracker Functions
    ============================================================================='''

    def cmd_set_AT_params(self):

        SndpL = []
        SndpL.append(self.AT_Params.nRefPulses)
        SndpL.append(self.AT_Params.nTargets)
        SndpL.append(self.AT_Params.nTracks)
        SndpL.append(self.AT_Params.enableTracker)
        SndpL.append(self.AT_Params.overThreshold)
        SndpL.append(self.AT_Params.threshFilter)
        SndpL.append(self.AT_Params.CFAR)
        SndpL.append(self.AT_Params.minDistance)
        SndpL.append(self.AT_Params.minSeparation)
        SndpL.append(self.AT_Params.maxHorSpeed)
        SndpL.append(self.AT_Params.maxVerSpeed)
        SndpL.append(self.AT_Params.maxAccel)
        SndpL.append(self.AT_Params.maxRangeErr)
        SndpL.append(self.AT_Params.timeOffset)
        SndpL.append(self.AT_Params.timeInterval)
        SndpL.append(self.AT_Params.minConfirmCnt)
        SndpL.append(self.AT_Params.assignLimit)
        SndpL.append(self.AT_Params.mergeLimit)

        self.doTransfer(SndLenE=18 * [2], SndpL=SndpL, delay=self.AT_Params.nRefPulses * 10e-3)

    '''=========================================================================='''

    def cmd_do_AT_meas(self):
        self.doTransfer()

    '''=========================================================================='''

    def cmd_get_AT_data(self):
        RecLenE_fix = 6 * [2]
        RecLenE_nFlex = [2, -2, -2, -2, -2, 2, 2]

        if self.single_transfer:
            RecpL = self.doTransfer(RecLenE_fix=RecLenE_fix, RecLenE_nFlex=RecLenE_nFlex)
        else:
            RecpL = self.doTransfer(RecLenE_fix=RecLenE_fix)

            # get time (4x 2 bytes)
        t = []
        for _ in range(4):
            t.append(RecpL.pop(0))
        self.AT_Targets.measTime = conv.TimeStamp_NetToHost(t)
        self.AT_Targets.Status = RecpL.pop(0)
        self.AT_Targets.nTargets = RecpL.pop(0)

        if not self.single_transfer:
            RecpL = self.doTransfer(RecLenE_nFlex=RecLenE_nFlex, nFlex=self.AT_Targets.nTargets)

        # empty entries
        self.AT_Targets.clear()
        for _ in range(self.AT_Targets.nTargets):
            tmp = RecpL.pop(0)
            self.AT_Targets.ID.append(tmp & 0xFF)
            self.AT_Targets.RCS.append(self.B2SB(tmp >> 8))  # [dBm]
            self.AT_Targets.dist.append(RecpL.pop(0) * 0.1)  # [m]
            self.AT_Targets.speed.append(RecpL.pop(0) * 0.1)  # [m/s]
            self.AT_Targets.ele.append(RecpL.pop(0) * 1.8 / pi)  # [deg]
            self.AT_Targets.azi.append(RecpL.pop(0) * 1.8 / pi)  # [deg]
            tmp = RecpL.pop(0)
            self.AT_Targets.var_dist.append(10 ** (self.B2SB(tmp & 0xFF) / 20.))
            self.AT_Targets.var_speed.append(10 ** (self.B2SB(tmp >> 8) / 20.))
            tmp = RecpL.pop(0)
            self.AT_Targets.var_ele.append(10 ** (self.B2SB(tmp & 0xFF) / 20.))
            self.AT_Targets.var_azi.append(10 ** (self.B2SB(tmp >> 8) / 20.))

        if 0 and len(self.AT_Targets.ID) > 0:
            print
            ""
            print
            "IDs: ", self.AT_Targets.ID
            # print "RCS: ",self.AT_Targets.RCS
            print
            "Dist:", self.AT_Targets.dist
            print
            "Speed: ", self.AT_Targets.speed
            # print "ele: ",self.AT_Targets.ele
            # print "azi: ",self.AT_Targets.azi
            print
            "var dist: ", self.AT_Targets.var_dist
            print
            "var_speed: ", self.AT_Targets.var_speed
            print
            "var ele: ", self.AT_Targets.var_ele
            print
            "var_azi: ", self.AT_Targets.var_azi

    '''=========================================================================='''

    def cmd_get_AT_thresh(self):
        pass

    '''=========================================================================='''

    def cmd_get_AT_params(self):
        RecpL = self.doTransfer(RecLenE_fix=18 * [2])

        self.AT_Params.nRefPulses = RecpL.pop(0)
        self.AT_Params.nTargets = RecpL.pop(0)
        self.AT_Params.nTracks = RecpL.pop(0)
        self.AT_Params.enableTracker = RecpL.pop(0)
        self.AT_Params.overThreshold = RecpL.pop(0)
        self.AT_Params.threshFilter = RecpL.pop(0)
        self.AT_Params.CFAR = RecpL.pop(0)
        self.AT_Params.minDistance = RecpL.pop(0)
        self.AT_Params.minSeparation = RecpL.pop(0)
        self.AT_Params.maxHorSpeed = RecpL.pop(0)
        self.AT_Params.maxVerSpeed = RecpL.pop(0)
        self.AT_Params.maxAccel = RecpL.pop(0)
        self.AT_Params.maxRangeErr = RecpL.pop(0)
        self.AT_Params.timeOffset = RecpL.pop(0)
        self.AT_Params.timeInterval = RecpL.pop(0)
        self.AT_Params.minConfirmCnt = RecpL.pop(0)
        self.AT_Params.assignLimit = RecpL.pop(0)
        self.AT_Params.mergeLimit = RecpL.pop(0)

    '''=========================================================================='''

    def cmd_AT_auto_start(self):
        secs = currenttime()
        self.doTransfer(SndLenE=4 * [2], SndpL=conv.TimeStamp_HostToNet(secs))

    '''=========================================================================='''

    def cmd_AT_auto_stop(self):
        self.doTransfer()

    '''=========================================================================='''

    def cmd_get_AT_target_list(self):
        # get air tracker target list before tracking
        RecLenE_fix = [8, 2]
        RecLenE_nFlex = [4, -4, -4, 4, -4, -4]

        if self.single_transfer:
            RecpL = self.doTransfer(RecLenE_fix=RecLenE_fix, RecLenE_nFlex=RecLenE_nFlex)
        else:
            RecpL = self.doTransfer(RecLenE_fix=RecLenE_fix)

        self.AT_TL.clear()

        self.AT_TL.measTime = RecpL.pop(0)
        self.AT_TL.nTargets = RecpL.pop(0)

        if not self.single_transfer:
            RecpL = self.doTransfer(RecLenE_nFlex=RecLenE_nFlex, nFlex=self.AT_TL.nTargets)

        for _ in range(self.AT_TL.nTargets):
            self.AT_TL.magnitude.append(RecpL.pop(0))
            self.AT_TL.snr_ele.append(RecpL.pop(0))
            self.AT_TL.snr_azi.append(RecpL.pop(0))
            self.AT_TL.distance.append(RecpL.pop(0))
            self.AT_TL.elevation.append(RecpL.pop(0))
            self.AT_TL.azimuth.append(RecpL.pop(0))
            self.AT_TL.tracked.append(0)

        if 0:
            print
            ""
            print
            "nTargets: ", self.AT_TL.nTargets
            print
            "Mag: ", self.AT_TL.magnitude
            print
            "SNR (ele): ", self.AT_TL.snr_ele
            print
            "SNR (azi): ", self.AT_TL.snr_azi
            print
            "Distance: ", self.AT_TL.distance
            print
            "Azimuth: ", self.AT_TL.azimuth
            print
            "Elevation: ", self.AT_TL.elevation

    '''=========================================================================='''

    def B2SB(self, b):  # unsigned to signed byte
        return b - 256 if b > 127 else b


'''============================================================================'''


class CommandError(Exception):
    def __init__(self, value):
        self.value = value

    def __str__(self):
        return repr(self.value)
