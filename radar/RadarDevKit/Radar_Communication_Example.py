'''
Created on 19.08.2021

@author: MH @ IMST
'''

'''===========================================================================
This script shows an example code which explains how to
        - connect a Radar Module with the desired interface
        - read some Radar specific parameters
        - setup the Radar Module
        - perform a frequency domain (FD) measurement and read the data
        - show the read data in a simple plot

In the first part an example class is shown which contains the basic functions
to communicate and measure with the Radar Module.
In the lower part it is shown how to handle this class in some examples. 
==========================================================================='''

'''
Imports
'''
# Main classes for the CAN Bus interface
from Interfaces.CAN.CanConfig import CanParams
# from Interfaces.CAN.USB_CAN import USB_CAN_Fkt as can
# from Interfaces.CAN.Commands import CanCommands

# Main classes for the SPI Bus interface
# from Interfaces.SPI.SPI_Interface import SPI_Interface
from Interfaces.SPI.Commands import SPI_Commands

# Main classes for the Ethernet interface
from Interfaces.Ethernet.EthernetConfig import EthernetParams
from Interfaces.Ethernet.IPConnection import IPConnection
from Interfaces.Ethernet.Commands import IPCommands

# Parameter and data classes
import ConfigClasses as cfgCl

# To create a simple plot
import matplotlib.pyplot as plt

import numpy as np
from time import time, sleep

'''
This example class loads all necessary parameter and data classes. It has functions
to connect a Radar Module via an interface specified by the user. It has also
functions to use the basic functions of the Radar Module.  
'''


class Main():
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

        ### interface specific parameters ###
        # load the CAN parameters and change them
        self.canParams = CanParams()
        self.canParams.Node = 1
        self.canParams.Baud = 1000

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


'=================================================================================================='
'''
/// Main Part ///

In the following it is shown how the Main class is used to connect a Radar
Module and to read/change some parameters.  
Later one of three examples can be chosen by setting the "if 0:" to "if 1:".
These examples are using some of the Main class functions to read measurement 
data and to plot it. 
'''

# Get an instance of the main class
main = Main()

# Specify your interface and try to connect it.
interface = "Ethernet"
main.Connect(interface)

if 1:
    # Read some Radar Module specific parameters
    main.GetHwParams()
    # These parameters can be accessed by e.g. main.hwParams.radarNumber
    print("Number of the connected Radar Module: ", main.hwParams.radarNumber)

    # Read the actual system parameters of the Radar Module
    main.GetSysParams()
    # Print an example
    print("Frequency [MHz]: ", main.sysParams.minFreq)
    print("Bandwidth [MHz]: ", main.sysParams.manualBW)
    print("Ramp-time [ms]: ", main.sysParams.t_ramp)
    print("")

    # Change some of the system parameters, e.g.
    main.sysParams.minFreq = 24000
    main.sysParams.manualBW = 250
    main.sysParams.t_ramp = 1
    main.sysParams.active_RX_ch = 15
    main.sysParams.freq_points = 20

    # Check if the frontend is off
    if main.sysParams.frontendEn == 0:
        main.sysParams.frontendEn = 1  # turns it on

    # Send the changed parameters to the Radar Module
    main.SetSysParams()
    # Always read back the system parameters because some read only parameters changed
    main.GetSysParams()
    # Verify that the parameters have changed
    print("Frequency [MHz]: ", main.sysParams.minFreq)
    print("Bandwidth [MHz]: ", main.sysParams.manualBW)
    print("Ramp-time [ms]: ", main.sysParams.t_ramp)
    print("")

'--------------------------------------------------------------------------------------------'
'''
Time domain example
'''
if 1:
    if main.connected:
        # Specify a measurement type, let the Radar perform it and read the data
        measurement = "UP-Ramp"
        main.GetTdData(measurement)

        # Plot the measured data dependent on the activated channels
        td_data = []
        n = 0
        n_samples = 1024  # always 1024 samples per channel
        for ch in range(4):  # maximum possible channels = 4
            if main.sysParams.active_RX_ch & (1 << ch):
                ind1 = n * n_samples
                ind2 = ind1 + n_samples
                n += 1
                td_data.append(main.TD_Data.data[ind1:ind2])
            else:
                td_data.append([0] * n_samples)

        # Generate x-axis data, here the measurement time in [ms]
        step = float(main.sysParams.t_ramp) / n_samples
        x_data = [n * step for n in range(n_samples)]

        # Add the data as lines to the plot
        plt.plot(x_data, td_data[0], 'b',
                 x_data, td_data[1], 'g',
                 x_data, td_data[2], 'c',
                 x_data, td_data[3], 'm')

        plt.grid(True)
        plt.title("Time domain data plot")
        plt.xlabel("Time [m]")
        plt.ylabel("Signal Amplitude")

        # Show the plot
        plt.show()

'--------------------------------------------------------------------------------------------'
'''
Frequency domain example
'''
if 1:
    if main.connected:
        # Specify a measurement type, let the Radar perform it and read the data
        measurement = "UP-Ramp"
        main.GetFdData(measurement)

        # Plot the measured data dependent on the activated channels
        # Only magnitude data will be plotted
        mag_data = []

        if main.sysParams.FFT_data_type == 0:  # only magnitudes were transmitted
            mag_data = main.FD_Data.data

        if main.sysParams.FFT_data_type == 2:  # real/imaginary
            comp_data = [complex(float(main.FD_Data.data[n]), float(main.FD_Data.data[n + 1])) for n in
                         range(0, len(main.FD_Data.data), 2)]
            mag_data = np.abs(comp_data)

        if main.sysParams.FFT_data_type == 1 or main.sysParams.FFT_data_type == 3:  # magnitudes/phase or magnitudes/object angle
            mag_data = [main.FD_Data.data[n] for n in range(0, len(main.FD_Data.data), 2)]

        # Convert to dBm
        min_dbm = -60  # [dBm]
        for n in range(len(mag_data)):
            try:
                mag_data[n] = 20 * np.log10(mag_data[n] / 2. ** 21)
            except:
                mag_data[n] = min_dbm

        # Sort for active channels
        fd_data = []
        n = 0
        for ch in range(4):  # maximum possible channels = 4
            if main.sysParams.active_RX_ch & (1 << ch):
                ind1 = n * main.FD_Data.nSamples
                ind2 = ind1 + main.FD_Data.nSamples
                n += 1
                fd_data.append(mag_data[ind1:ind2])
            else:
                fd_data.append([0] * main.FD_Data.nSamples)

                # Generate x-axis data, here the distance in [m]
        x_data = [main.sysParams.tic * n / main.sysParams.zero_pad / 1.0e6 for n in range(main.FD_Data.nSamples)]

        # Add the data as lines to the plot
        plt.plot(x_data, fd_data[0], 'b',
                 x_data, fd_data[1], 'g',
                 x_data, fd_data[2], 'c',
                 x_data, fd_data[3], 'm')

        plt.grid(True)
        plt.title("Frequency domain data plot")
        plt.xlabel("Distance [m]")
        plt.ylabel("Magnitude [dBm]")

        # Show the plot
        plt.show()

'--------------------------------------------------------------------------------------------'
'''
Air Tracker example
'''
if 1:
    # Get the actual Air Tracker parameters
    main.GetAtParams()

    # Change some of the Air Tracker parameters
    main.AT_Params.minDistance = 0
    main.AT_Params.nRefPulses = 50
    # ...

    # Send the changed parameters to the Radar Module
    print("Radar Module is performing %d initial measurements." % main.AT_Params.nRefPulses)
    print("Please wait for %.2f seconds.\n" % (main.AT_Params.timeInterval * main.AT_Params.nRefPulses / 1000.))
    main.SetAtParams()

    time_interval = 100  # [ms]
    n_measurements = 10  # Number of measurements

    # Perform a number of measurements specified by n_measurements e.g. in a specified time interval 
    for n in range(n_measurements):
        t_start = time()

        main.AtMeasurement()

        dt = time() - t_start

        # Measurement was faster than the specified time interval? -> wait to fulfill it
        if dt < time_interval:
            sleep((time_interval - dt) / 1000.)

        # Print some of the measured results
        print("Finished Air Tracker measurement No.: %d" % (n + 1))
        print("Number of targets found: %d" % main.AT_Targets.nTargets)
        for m in range(main.AT_Targets.nTargets):
            print("-------------------")
            print("Target ID: %d" % main.AT_Targets.ID[m])
            print("Target RCS: %.2f dBm" % main.AT_Targets.RCS[m])
            print("Distance: %.3f m" % main.AT_Targets.dist[m])
            print("Speed: %.3f m/s" % main.AT_Targets.speed[m])
            print("Ele. Angle: %.2f deg" % main.AT_Targets.ele[m])
            print("Azi. Angle: %.2f deg" % main.AT_Targets.azi[m])
        print("==========================================")

# At the end disconnect the interface
main.Disconnect()
print("Disconnected.")
print("End.")
# End
