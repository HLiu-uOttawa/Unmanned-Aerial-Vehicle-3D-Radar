#!/usr/bin/env python3

from .Interfaces.Ethernet.EthernetConfig import EthernetParams
from .Interfaces.Ethernet.IPConnection import IPConnection
from .Interfaces.Ethernet.Commands import IPCommands

# Parameter and data classes
from . import ConfigClasses as cfgCl

# To create a simple plot
import matplotlib.pyplot as plt

import numpy as np
import datetime, time

class ImstRadarModule():
    def __init__(self):

        # initialize needed parameters classes
        self.hwParams = cfgCl.HwParams()
        self.sysParams = cfgCl.SysParams()
        self.AT_Params = cfgCl.AirTrackerParams()

        # initialize needed data classes
        self.FD_Data = cfgCl.FD_Data()
        self.TD_Data = cfgCl.TD_Data()
        self.AT_Targets = cfgCl.AirTrackerTargets()
        self.AT_TL = cfgCl.AirTrackerTL()

        # load the Ethernet parameters and change them
        self.etherParams = EthernetParams()
        self.etherParams.ip = "192.168.0.2"
        self.etherParams.port = 1024

        self.myInterface = None
        self.interface = None
        self.connected = False
        self.error = False

    '--------------------------------------------------------------------------------------------'

    # function to connect to a specified interface
    def Connect(self, interface=None):
        if interface == None:  # do nothing if no interface is specified
            self.error = True
            return

        # load the specified interface
        if interface == "CAN":
            try:
                self.myInterface = can.Init_Hardware()
            except:
                print("Could not find a USB to CAN converter.")
                self.error = True
                return

            # if the needed hardware was found, initialize a CAN connection
            self.myInterface.InitCan(self.canParams.Baud, self.canParams.ACMSK, self.canParams.ACCODE)

            # load the command class
            self.cmd = CanCommands(myCan=self.myInterface, main_win=self)

        if interface == "SPI":
            try:
                self.myInterface = SPI_Interface()
                self.myInterface.openComPort(ErrorRaise=True)
            except Exception as E:
                print(E)
                self.error = True
                return

            # if the connection has been established, load the command class
            self.cmd = SPI_Commands(port=self.myInterface, main_win=self)

        if interface == "Ethernet":
            self.myInterface = IPConnection(self)
            # try to connect
            if not self.myInterface.connect():
                print("Connection to " + self.etherParams.ip + ":" + str(self.etherParams.port) + " failed.")
                self.error = True
                return

            # if the connection has been established, load the command class
            self.cmd = IPCommands(connection=self.myInterface, main_win=self)

        self.interface = interface
        self.connected = True

    '--------------------------------------------------------------------------------------------'

    # function to get the Radar specific parameters
    def GetHwParams(self):
        # do nothing if not connected or an error has occurred
        if not self.connected or self.error:
            return

        # execute the respective command ID
        try:
            self.cmd.execute_cmd("CMDID_SEND_INFO")
        except:
            print("Error in receiving hardware parameters.")
            self.error = True
            return

        # if no error occurred, the received data can be found in hwParams

    '--------------------------------------------------------------------------------------------'

    # function to get the Radar system parameters
    def GetSysParams(self):
        # do nothing if not connected or an error has occurred
        if not self.connected or self.error:
            return

        # execute the respective command ID
        try:
            self.cmd.execute_cmd("CMDID_SEND_PARAMS")
        except:
            print("Error in receiving system parameters.")
            self.error = True
            return

        # if no error occurred, the received data can be found in sysParams

    '--------------------------------------------------------------------------------------------'

    # function to set the Radar system parameters
    def SetSysParams(self):
        # do nothing if not connected or an error has occurred
        if not self.connected or self.error:
            return

        # execute the respective command ID
        try:
            self.cmd.execute_cmd("CMDID_SETUP")
        except:
            print("Error in setting system parameters.")
            self.error = True
            return

    '--------------------------------------------------------------------------------------------'

    # function to get frequency domain data with or without a previous measurement
    # the parameter 'measurement' specifies the type of measurement    
    # possible values are: "UP-Ramp", "DOWN-Ramp", "CW" or "None"
    def GetFdData(self, measurement="UP-Ramp"):
        # do nothing if not connected or an error has occurred
        if not self.connected or self.error:
            return

        # execute the respective command ID
        try:
            if measurement == "UP-Ramp":
                self.cmd.execute_cmd("CMDID_UP_RMP_FD")
            elif measurement == "DOWN-Ramp":
                self.cmd.execute_cmd("CMDID_DN_RMP_FD")
            elif measurement == "CW":
                self.cmd.execute_cmd("CMDID_GP_FD")
            elif measurement == None or measurement == "None":
                self.cmd.execute_cmd("CMDID_FDATA")
            else:
                print
                "No valid measurement type."
                self.error = True
                return

        except:
            print("Error in receiving frequency domain data.")
            self.error = True
            return

        # the received data can be found in FD_Data

    '--------------------------------------------------------------------------------------------'

    # function to get time domain data with or without a previous measurement
    # the parameter 'measurement' specifies the type of measurement    
    # possible values are: "UP-Ramp", "DOWN-Ramp", "CW" or "None"
    def GetTdData(self, measurement="UP-Ramp"):
        # do nothing if not connected or an error has occurred
        if not self.connected or self.error:
            return

        # execute the respective command ID
        try:
            if measurement == "UP-Ramp":
                self.cmd.execute_cmd("CMDID_UP_RMP_TD")
            elif measurement == "DOWN-Ramp":
                self.cmd.execute_cmd("CMDID_DN_RMP_TD")
            elif measurement == "CW":
                self.cmd.execute_cmd("CMDID_GP_TD")
            elif measurement == None or measurement == "None":
                self.cmd.execute_cmd("CMDID_TDATA")
            else:
                print("No valid measurement type.")
                self.error = True
                return

        except:
            print("Error in receiving time domain data.")
            self.error = True
            return

        # the received data can be found in TD_Data

    '--------------------------------------------------------------------------------------------'

    # function to get the Air Tracker parameters
    def GetAtParams(self):
        # do nothing if not connected or an error has occurred
        if not self.connected or self.error:
            return

        # execute the respective command ID
        try:
            self.cmd.execute_cmd("CMDID_AT_SEND_PAR")
        except:
            print("Error in receiving Air Tracker parameters.")
            self.error = True
            return

        # the received parameters can be found in htParams

    '--------------------------------------------------------------------------------------------'

    # function to set the Air Tracker parameters
    # Note that the Radar Module will perform some initial measurements, so it will be waited some time
    def SetAtParams(self):
        # do nothing if not connected or an error has occurred
        if not self.connected or self.error:
            return

        # execute the respective command ID
        try:
            self.cmd.execute_cmd("CMDID_AT_SET_PAR")
        except:
            print("Error in setting Air Tracker parameters.")
            self.error = True
            return

    '--------------------------------------------------------------------------------------------'

    # function that triggers a Air Tracker measurement
    def AtMeasurement(self):
        # do nothing if not connected or an error has occurred
        if not self.connected or self.error:
            return

        # execute the respective command ID
        try:
            self.cmd.execute_cmd("CMDID_AT_MEASURE")
            self.cmd.execute_cmd("CMDID_AT_SEND_RES")
        except:
            print
            "Error in receiving Air Tracker data."
            self.error = True
            return

        # the received data can be found in AT_Targets

    '--------------------------------------------------------------------------------------------'

    # function to disconnect the connected interface
    def Disconnect(self):
        # do nothing if not connected
        if not self.connected:
            return

        if self.interface == "CAN":
            self.myInterface.DeinitCan()
            self.myInterface.DeinitHwModul()
        if self.interface == "SPI":
            self.myInterface.closeComPort()
        if self.interface == "Ethernet":
            self.myInterface.disconnect()

        self.connected = False

def test_sampling_frequency():
    radar = ImstRadarModule()
    radar.etherParams.ip = "192.168.0.2"
    radar.etherParams.port = 1024
    
    print("test_sampling_frequency")
    # print(radar.Connect("Ethernet"))
    radar.Connect("Ethernet")
    if radar.connected:
        
        # Warm up once (optional)
        radar.GetTdData("UP-Ramp")

        # Measure how many frames we can read in 1 second
        frame_count = 0
        t_start = time.time()
        duration = 1.0  # seconds

        while time.time() - t_start < duration:
            radar.GetTdData("UP-Ramp")
            print(f"TD data time0: {radar.TD_Data.time0}")
            frame_count += 1

        elapsed = time.time() - t_start
        fps = frame_count / elapsed

        print(f"Read {frame_count} TD frames in {elapsed:.3f} seconds.")
        print(f"≈ {fps:.2f} frames per second.")

        radar.Disconnect()

def test_basic_infomation():
    radar = ImstRadarModule()
    radar.etherParams.ip = "192.168.0.2"
    radar.etherParams.port = 1024
    
    print("test_basic_infomation")
    # print(radar.Connect("Ethernet"))
    radar.Connect("Ethernet")
    if radar.connected:
        radar.GetHwParams()

        # Retrieve system parameters
        radar.GetSysParams()
        print("Frequency [MHz]:", radar.sysParams.minFreq)
        print("Bandwidth [MHz]:", radar.sysParams.manualBW)
        print("Ramp time [ms]:", radar.sysParams.t_ramp)

        # Acquire one time-domain frame
        radar.GetTdData("UP-Ramp")
        print(f"TD data length: {len(radar.TD_Data.data)}")
        # The start time of the acquisition window when the radar began sampling the current TD frame.
        print(f"TD data time0: {radar.TD_Data.time0}") 
        # The end time of that same acquisition window when the radar finished sampling the frame.
        print(f"TD data time1: {radar.TD_Data.time1}") 

        print(f"TD data delta t: {radar.TD_Data.time1 - radar.TD_Data.time0}") 

        # Acquire one frequency-domain frame
        radar.GetFdData("UP-Ramp")
        print(f"FD data length: {len(radar.FD_Data.data)}")

        # Disconnect the module
        radar.Disconnect()
        print("Disconnected successfully.")


if __name__ == "__main__":
    test_sampling_frequency()

