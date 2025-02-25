'''
Created on 18.01.2011
Details:        
@author:        Eugen Knop, Kai Maulwurf-Just
@Date:          
'''
#####################################
# Imports                           #
#####################################

import ctypes as c
import sys
import time
import os
from Interfaces import ConversionFuncs as conv

# from GUI.CustomEvent import EVT_CUSTOM

find_dll = "USBCAN32.dll"
found_dll = None
my_path = os.path.abspath(".")

for root, _dirs, files in os.walk(my_path):
    for f in files:
        if find_dll in f:
            found_dll = os.path.join(root, f)

if sys.platform == "win32":
    if found_dll != None:
        lib = c.WinDLL(found_dll)
    else:
        raise Exception("USBCAN32.dll not found!")  # USBCAN32 Bib. einbinden, falls Windows- Sytem vorliegt
else:
    lib = c.CDLL(my_path)  # USBCAN32 Bib. einbinden falls Linux als Betriebssystem verwendet wird

USBCAN_BAUD_10kBit = 0x672f  # CAN baud rate 10 kBit/sec
USBCAN_BAUD_20kBit = 0x532f  # CAN baud rate 20 kBit/sec
USBCAN_BAUD_50kBit = 0x472f  # CAN baud rate 50 kBit/sec
USBCAN_BAUD_100kBit = 0x432f  # CAN baud rate 100 kBit/sec
USBCAN_BAUD_125kBit = 0x031c  # CAN baud rate 125 kBit/sec
USBCAN_BAUD_250kBit = 0x011c  # CAN baud rate 250 kBit/sec
USBCAN_BAUD_500kBit = 0x001c  # CAN baud rate 500 kBit/sec
USBCAN_BAUD_800kBit = 0x0016  # CAN baud rate 800 kBit/sec
USBCAN_BAUD_1MBit = 0x0014  # CAN baud rate 1 MBit/sec

# Table for assignment of Baudrate (in kbps) to Baudregister pair
BREG_table = {10: conv.split_u16(USBCAN_BAUD_10kBit)
    , 20: conv.split_u16(USBCAN_BAUD_20kBit)
    , 50: conv.split_u16(USBCAN_BAUD_50kBit)
    , 100: conv.split_u16(USBCAN_BAUD_100kBit)
    , 125: conv.split_u16(USBCAN_BAUD_125kBit)
    , 250: conv.split_u16(USBCAN_BAUD_250kBit)
    , 500: conv.split_u16(USBCAN_BAUD_500kBit)
    , 800: conv.split_u16(USBCAN_BAUD_800kBit)
    , 1000: conv.split_u16(USBCAN_BAUD_1MBit)
              }


# List of implemented Baudrates
def Baudrate_list():
    return sorted(BREG_table.keys())


# Strip unit from Baudrate and convert to integer
def Baudrate_integer(s):
    return int(filter(str.isdigit, str(s)))


# Append unit to Baudrate
def Baudrate_unit(baud):
    return str(baud) + ' kBit/s'


# Baudrate list with unit
def Baudrate_list_unit():
    return [Baudrate_unit(x) for x in Baudrate_list()]


'''================================================
Constants:
================================================'''

SERIAL_DEBUG = 0
PACKSIZE_CAN = 8

USBCAN_CHANNEL_CH0 = 0  # first CAN channel
USBCAN_CHANNEL_CH1 = 1  # second CAN channel
USBCAN_CHANNEL_ANY = 255  # Any CAN channel
USBCAN_CHANNEL_CAN1 = 0  # first CAN channel
USBCAN_CHANNEL_CAN2 = 1  # second CAN channel

CAN_MODULE_ON = 0
FLAG_RX_FW = 0x00000004  # Prueft die Anzahl der Nachrichten im Empfangspuffer der Firmware


# Possible parameters for Software Reset (UcanResetCanEx)

class reset_values:
    USBCAN_RESET_ALL = 0x0000  # Reset all components. However, the firmware is not reset completely.
    USBCAN_RESET_FIRMWARE = 0xFFFFFFFF  # Complete reset of the device firmware. 

    # Values for exclusion of single components at reset (to be OR-ed, if several values are combined)

    USBCAN_RESET_NO_STATUS = 0x0001  # Skip reset of the CAN error status.
    USBCAN_RESET_NO_CANCTRL = 0x0002  # Skip reset of the CAN controller.
    USBCAN_RESET_NO_TXCOUNTER = 0x0004  # Skip reset of the transmit message counter.
    USBCAN_RESET_NO_RXCOUNTER = 0x0008  # Skip reset of the receive message counter.
    USBCAN_RESET_NO_TXBUFFER_CH = 0x0010  # Skip reset of the transmit buffers of a specific CAN-channel.
    USBCAN_RESET_NO_TXBUFFER_DLL = 0x0020  # Skip reset of the transmit buffer for both CAN-channels within the DLL.
    USBCAN_RESET_NO_TXBUFFER_FW = 0x0080  # Skip reset of the transmit buffers of both CAN-channels within the device firmware.
    USBCAN_RESET_NO_RXBUFFER_CH = 0x0100  # Skip reset of the receive buffers of a specific CAN-channel.
    USBCAN_RESET_NO_RXBUFFER_DLL = 0x0200  # Skip reset of both receive message counters within the DLL
    USBCAN_RESET_NO_RXBUFFER_SYS = 0x0400  # Skip reset of the receive message counter of both CAN-channels within the Kernel-Mode driver.
    USBCAN_RESET_NO_RXBUFFER_FW = 0x0800  # Skip reset of receive message counters of both CAN-channels within the device firmware.

    # Values for inclusion of single components at reset (to be AND-ed, if several values are combined)

    USBCAN_RESET_ONLY_STATUS = 0xFFFE  # Reset of the CAN error status only.
    USBCAN_RESET_ONLY_CANCTRL = 0xFFFD  # Only resets the CAN controller of the USB-CANmodul.
    # This has to be done after each busoff state because the
    # CAN controller cannot leave this state automatically.
    USBCAN_RESET_ONLY_RXBUFFER_FW = 0xF7FF  # Only resets the receive buffer.
    USBCAN_RESET_ONLY_TXBUFFER_FW = 0xFF7F  # Only resets the transmit buffer.
    USBCAN_RESET_ONLY_RXCHANNEL_BUFF = 0xFEFF  # Reset of the receive buffer of only one CAN-channel.
    USBCAN_RESET_ONLY_TXCHANNEL_BUFF = 0xFFEF  # Reset of the transmit buffer of only one CAN-cannel.
    USBCAN_RESET_ONLY_RX_BUFF = 0xF0F7  # Reset of the receive buffers
    # and reset of the receive message counter.
    USBCAN_RESET_ONLY_TX_BUFF = 0xFF0B  # Reset of the transmit buffers
    # and reset of the transmit message counter.
    USBCAN_RESET_ONLY_ALL_BUFF = 0xF003  # Reset off all message buffers (receive and transmit)
    # and reset of the reception and transmit message counters.
    USBCAN_RESET_ONLY_ALL_COUNTER = 0xFFF3  # Reset of all reception and transmit counters.


'''================================================
Structures:
================================================'''

USBCAN_EVENT_INITHW = 0x00  # The USB-CANmodul is initialized successfully.
USBCAN_EVENT_INITCAN = 0x01  # The CAN interface is initialized successfully
USBCAN_EVENT_RECEIVE = 0x02  # A CAN message is received.
USBCAN_EVENT_STATUS = 0x03  # The error status at the USB-CANmodul has changed.
USBCAN_EVENT_DEINITCAN = 0x04  # The CAN interface is shut down.
USBCAN_EVENT_DEINITHW = 0x05  # The USB-CANmodul is shut down.
USBCAN_EVENT_CONNECT = 0x06  # A new USB-CANmodul is connected.
USBCAN_EVENT_DISCONNECT = 0x07  # A USB-CANmodul is disconnected.
USBCAN_EVENT_FATALDISCON = 0x08  # A USB-CANmodul in either HW_INIT or CAN_INIT state is disconnected.

# Function return codes 

errorcodes = \
    {0x00: "USBCAN_SUCCESSFUL"
     # Error Codes
        , 0x01: "USBCAN_ERR_RESOURCE"
        , 0x02: "USBCAN_ERR_MAXMODULES"
        , 0x03: "USBCAN_ERR_HWINUSE"
        , 0x04: "USBCAN_ERR_ILLVERSION"
        , 0x05: "USBCAN_ERR_ILLHW"
        , 0x06: "USBCAN_ERR_ILLHANDLE"
        , 0x07: "USBCAN_ERR_ILLPARAM"
        , 0x08: "USBCAN_ERR_BUSY"
        , 0x09: "USBCAN_ERR_TIMEOUT"
        , 0x0A: "USBCAN_ERR_IOFAILED"
        , 0x0B: "USBCAN_ERR_DLL_TXFULL"
        , 0x0C: "USBCAN_ERR_MAXINSTANCES"
        , 0x0D: "USBCAN_ERR_CANNOTINIT"
        , 0x0E: "USBCAN_ERR_DISCONNECT"
        , 0x0F: "USBCAN_ERR_NOHWCLASS"
        , 0x10: "USBCAN_ERR_ILLCHANNEL"
        , 0x12: "USBCAN_ERR_ILLHWTYPE"
        , 0x40: "USBCAN_ERRCMD_NOTEQU"
        , 0x41: "USBCAN_ERRCMD_REGTST"
        , 0x42: "USBCAN_ERRCMD_ILLCMD"
        , 0x43: "USBCAN_ERRCMD_EEPROM"
        , 0x47: "USBCAN_ERRCMD_ILLBDR"
        , 0x48: "USBCAN_ERRCMD_NOTINIT"
        , 0x49: "USBCAN_ERRCMD_ALREADYINIT"
        , 0x4A: "USBCAN_ERRCMD_ILLSUBCMD"
        , 0x4B: "USBCAN_ERRCMD_ILLIDX"
        , 0x4C: "USBCAN_ERRCMD_RUNNING"
     # Warnings
        , 0x80: "USBCAN_WARN_NODATA"
        , 0x81: "USBCAN_WARN_SYS_RXOVERRUN"
        , 0x82: "USBCAN_WARN_DLL_RXOVERRUN"
        , 0x85: "USBCAN_WARN_FW_TXOVERRUN"
        , 0x86: "USBCAN_WARN_FW_RXOVERRUN"
        , 0x90: "USBCAN_WARN_NULL_PTR"
        , 0x91: "USBCAN_WARN_TXLIMIT"
     }

# Dummy initializations to avoid error marks in python 

USBCAN_SUCCESSFUL = int()
USBCAN_ERR_RESOURCE = int()
USBCAN_ERR_MAXMODULES = int()
USBCAN_ERR_HWINUSE = int()
USBCAN_ERR_ILLVERSION = int()
USBCAN_ERR_ILLHW = int()
USBCAN_ERR_ILLHANDLE = int()
USBCAN_ERR_ILLPARAM = int()
USBCAN_ERR_BUSY = int()
USBCAN_ERR_TIMEOUT = int()
USBCAN_ERR_IOFAILED = int()
USBCAN_ERR_DLL_TXFULL = int()
USBCAN_ERR_MAXINSTANCES = int()
USBCAN_ERR_CANNOTINIT = int()
USBCAN_ERR_DISCONNECT = int()
USBCAN_ERR_NOHWCLASS = int()
USBCAN_ERR_ILLCHANNEL = int()
USBCAN_ERR_ILLHWTYPE = int()
USBCAN_ERRCMD_NOTEQU = int()
USBCAN_ERRCMD_REGTST = int()
USBCAN_ERRCMD_ILLCMD = int()
USBCAN_ERRCMD_EEPROM = int()
USBCAN_ERRCMD_ILLBDR = int()
USBCAN_ERRCMD_NOTINIT = int()
USBCAN_ERRCMD_ALREADYINIT = int()
USBCAN_ERRCMD_ILLSUBCMD = int()
USBCAN_ERRCMD_ILLIDX = int()
USBCAN_ERRCMD_RUNNING = int()
USBCAN_WARN_NODATA = int()
USBCAN_WARN_SYS_RXOVERRUN = int()
USBCAN_WARN_DLL_RXOVERRUN = int()
USBCAN_WARN_FW_TXOVERRUN = int()
USBCAN_WARN_FW_RXOVERRUN = int()
USBCAN_WARN_NULL_PTR = int()
USBCAN_WARN_TXLIMIT = int()


# Module initialization function
def module_init():
    # Make error codes global within module
    module = sys.modules[__name__]
    for name, val in errorcodes.items():
        setattr(module, val, name)
    return None


dummy = module_init()

'''================================================
Class for BitModes:
================================================'''


class BitModes:
    # Subset of error codes for this class

    ft_values = [USBCAN_SUCCESSFUL,
                 USBCAN_ERR_RESOURCE,
                 USBCAN_ERR_MAXMODULES,
                 USBCAN_ERR_HWINUSE,
                 USBCAN_ERR_ILLVERSION,
                 USBCAN_ERR_ILLHW,
                 USBCAN_ERR_ILLHANDLE,
                 USBCAN_ERR_ILLPARAM,
                 USBCAN_ERR_BUSY,
                 USBCAN_ERR_TIMEOUT,
                 USBCAN_ERR_IOFAILED,
                 USBCAN_ERR_DLL_TXFULL,
                 USBCAN_ERR_MAXINSTANCES,
                 USBCAN_ERR_CANNOTINIT,
                 USBCAN_ERR_DISCONNECT,
                 USBCAN_ERR_NOHWCLASS,
                 USBCAN_ERR_ILLCHANNEL,
                 USBCAN_ERR_ILLHWTYPE]

    # Lists with names of the subset sorted by value

    ft_lst = sorted([(val, name) for val, name in errorcodes.items() if val in ft_values])
    ft_messages = [(name, val) for val, name in ft_lst]


########################################
# Class for invocation of
# "UcanGetStatus"
########################################

# Error bits for tStatusStruct.m_wUsbStatus

USBCAN_CANERR_OK = 0x0000  # No error
USBCAN_CANERR_XMTFULL = 0x0001  # Transmit buffer in CAN controller is overrun 
USBCAN_CANERR_OVERRUN = 0x0002  # Receive buffer in CAN controller is overrun 
USBCAN_CANERR_BUSLIGHT = 0x0004  # Error limit 1 in CAN controller exceeded, CAN controller is in state "Warning limit" now
USBCAN_CANERR_BUSHEAVY = 0x0008  # Error limit 2 in CAN controller exceeded, CAN controller is in state "Error Passive" now
USBCAN_CANERR_BUSOFF = 0x0010  # CAN controller is in BUSOFF state
USBCAN_CANERR_QOVERRUN = 0x0040  # Receive buffer in module is overrun
USBCAN_CANERR_QXMTFULL = 0x0080  # Transmit buffer in module is overrun
USBCAN_CANERR_REGTEST = 0x0100  # CAN controller not found (hardware error) 
USBCAN_CANERR_TXMSGLOST = 0x0400  # A transmit CAN message was deleted automatically by the firmware because transmission timeout run over


class tStatusStruct(c.Structure):
    _fields_ = [("m_wCanStatus", c.c_uint16)  # present CAN status
        , ("m_wUsbStatus", c.c_uint16)  # present USB status (obsolete)
                ]


########################################
# Class for CAN message structure
#
# For invocation of
# "UcanReadCanMsg"
# "UcanReadCanMsgEx"
# "UcanWriteCanMsg"
# "UcanWriteCanMsgEx"
# "UcanDefineCyclicCanMsg"
# "UcanReadCyclicCanMsg"
# "ReadCanMsg" and "WriteCanMsg"
# "DefineCyclicCanMsg"
# "ReadCyclicCanMsg"
########################################

class tCanMsgStruct(c.Structure):
    _fields_ = [
        ('m_dwID', c.c_uint32),  # message CAN identifier
        ('m_bFF', c.c_byte),  # message CAN frame format
        ('m_bDLC', c.c_byte),  # message CAN data length code
        ('m_bData', c.ARRAY(c.c_ubyte, 8)),  # message CAN data
        ('m_dwTime', c.c_uint32),  # message Receipt time in ms
    ]


########################################
# Class for invocation of
# "UcanGetMsgCountInfo" and
# "UcanGetMsgCountInfoEx"
########################################

class tUcanMsgCountInfo(c.Structure):
    _fields_ = [('SendMsg', c.c_uint16),
                ('RecMsg', c.c_uint16)
                ]


################################################
# Start of pythonic functions for specific     #
# functionality around FTDI API                #
################################################

def Init_Hardware(CanPort=USBCAN_CHANNEL_ANY):
    handle = c.c_int()
    CanPortNr = c.c_int(CanPort)
    ret = lib.UcanInitHardware(c.byref(handle), CanPortNr, None)
    if ret != USBCAN_SUCCESSFUL:
        # raise StandardError, "Hw Error: " + errorcodes[ret]
        raise Exception("Hw Error: " + errorcodes[ret])
    return CAN_Usb_Fkt(handle.value)


##########################################
##     USBCAN32 ctypes DLL wrapper      ##
##########################################

class CAN_Usb_Fkt(object):
    '''class that implements a ctype interface to the USBCAN driver'''

    def __init__(self, handle):
        self.handle = handle
        self.counter = 0
        self.baud = None

    def GetStatus(self):
        pStatus = tStatusStruct()
        lib.UcanGetStatus(self.handle, repr(pStatus))
        return pStatus.m_wCanStatus

    '''==============================================================================================
      Init_Can: Initialisiert die Can Schnittstelle eines USB- CAN Moduls. Die Software wechselt dabei
      in den Zustand CAN_INIT. Danach ist es moglich CAN- Nachrichten zu versenden und zu empfangen.
      
      -    Akzeptanzfiltermaske: bestimmt, welche Bits innerhalb des Identifiers fuer die Filterung
           relevant sind(1=relevant,0=nicht relevant).
      -    Bsp.: Can-Objekte empfangen, bei den die untersten Bits den Wert 4 besitzen:
           Die untersten Bits sind somit relevant:                MASK= 0000 0000 1111 = 0x00F
           Diese relevaten Bits muessen den Wert 5 besitzen:    CODE= 0000 0000 1001 = 0x005
           Der Can Controller empfaengt, nur noch Nachrichten mit einer 5 am ende: 0x005,0x015,0x225
    =============================================================================================='''

    def InitCan(self, Baud=1000, Akzeptanzfiltermaske=0x00000001, Akzeptanzfiltercode=0x00000001):
        self.FlushCan()
        BREG = BREG_table[Baud]
        lib.UcanInitCan(self.handle, c.c_byte(BREG[0]), c.c_byte(BREG[1]), c.c_uint32(Akzeptanzfiltermaske),
                        c.c_uint32(Akzeptanzfiltercode))
        self.baud = Baud
        return None

    def SetBaudRate(self, Baud):
        self.FlushCan()
        BREG = BREG_table[Baud]
        lib.UcanSetBaudrate(self.handle, c.c_byte(BREG[0]), c.c_byte(BREG[1]))
        self.baud = Baud
        return None

    """======================================================
    Deinitialisiert den CAN Port
    ======================================================"""

    def SetAcceptance(self, Akzeptanzfiltermaske, Akzeptanzfiltercode):
        lib.UcanSetAcceptance(self.handle, c.c_ulong(Akzeptanzfiltermaske), c.c_ulong(Akzeptanzfiltercode))
        return None

    """======================================================
    Prueft die Softwareversion der CAN Library
    ======================================================"""

    def GetVersion(self):
        Version = lib.UcanGetVersion()

        return Version.value

    """======================================================
    Prueft die Softwareversion des CAN Moduls
    ======================================================"""

    def GetFwVersion(self):
        Version = lib.UcanGetFwVersion(self.handle)

        return Version

    """======================================================
    Deinitialisiert den CAN Port
    ======================================================"""

    def DeinitCan(self):
        lib.UcanDeinitCan(self.handle)
        return None

    """======================================================
    Gibt die Hardware-Information des CAN_Moduls aus
    ======================================================"""

    def GetHwInfo(self):
        lib.UcanGetHardwareInfo(self.handle)
        return None

    """======================================================
    Deinitialisiert das Hardwaremodul
    ======================================================"""

    def DeinitHwModul(self):
        lib.UcanDeinitHardware(self.handle)
        return None

    """======================================================
    Gibt die Anzahl der gesendeten und empfangenen Nachrichten 
    wieder
    ======================================================"""

    def GetCountInfo(self):
        count = tUcanMsgCountInfo()
        lib.UcanGetMsgCountInfo(self.handle, c.byref(count))

        return "Received Msg:", count.RecMsg, "Send Msg:", count.SendMsg

    """======================================================
    Prueft ob Fehler in der Empfangs- oder im Sendevorgang 
    aufgetreten sind.
    ======================================================"""

    def GetErrorCount(self):
        rx_count = c.c_uint32()
        tx_count = c.c_uint32()

        lib.UcanGetCanErrorCounter(self.handle, 0, c.byref(tx_count), c.byref(rx_count))

        return ("Rx_Error:", rx_count.value, "Tx_Count:", tx_count.value)

    """======================================================
    Prueft die Anzahl der Messages im Hardwarepuffer
    ======================================================"""

    def GetMsgPending(self, USBCAN_CHANNEL=USBCAN_CHANNEL_CH0, FLAG_RX_FW=0x00000001):

        count = c.c_uint32()
        flag = c.c_uint16(FLAG_RX_FW)
        channel = c.c_int(USBCAN_CHANNEL)

        lib.UcanGetMsgPending(self.handle, channel, flag, c.byref(count))

        return count.value

    #####################################################################
    #             Funktionen fuer Byteweise Uebertragung                #
    #####################################################################

    """======================================================
    Stand 29.04.11
    Liest eine CAN Nachricht vom CAN Bus (Byteweise) 
    ===> Bessere funktion<===
    ======================================================"""
    #    def ReadCanMsg(self):
    #        mStr = tCanMsgStruct()
    #        byte = c.c_buffer(10)
    #
    #        while self.GetMsgPending() == 0:
    #            continue
    #        lib.UcanReadCanMsg(self.handle, c.byref(mStr))
    #        byte[0] = chr(mStr.m_bData[0])
    #        return byte[0]

    """======================================================
    Schreibt eine nachricht auf den CAN Bus mit 1 Byte.
    Als Standard ID ist 0x0004 eingestellt
    ======================================================"""

    #    def WriteCanMsg(self,data):
    #        DLC = c.c_byte(1)
    #        FF = c.c_byte(0x00)
    #        ID = c.c_uint32(0x0004)
    #
    #        mStr = tCanMsgStruct()                  #Nachrichtenstruktur
    #
    #        mStr.m_dwID = ID.value
    #        mStr.m_bFF = FF.value
    #        mStr.m_bDLC = DLC.value
    #
    #        if type(data)== str:                    # String eingegeben?? >> Umwandeln
    #            dataC = c.c_byte(ord(data))
    #        else:
    #            dataC = c.c_byte(data)
    #
    #        mStr.m_bData[0] = dataC.value
    #
    #        lib.UcanWriteCanMsg(self.handle,c.byref(mStr))

    #####################################################################
    #             Funktionen fuer 8 Byte Uebertragung                   #
    #####################################################################

    """======================================================
    Schreibt eine nachricht auf den CAN Bus mit 8 Byte.
    Als Standard ID ist 0x07FF eingestellt 
    @param data: String to transmit to Radar.
    ===> diese Funktion wird als Standard 8 Byte Version verwendet.<===
    ======================================================"""

    def WriteCanMsg(self, cmd_id, data):

        mStr = tCanMsgStruct()  # CAN Nachrichten Struktur

        if len(data) > 8:
            DLC_in = 8
        else:
            DLC_in = 8

        DLC = c.c_byte(DLC_in)
        FF = c.c_byte(0x80)
        ID = c.c_uint32(cmd_id)

        mStr.m_dwID = ID.value
        mStr.m_bFF = FF.value
        mStr.m_bDLC = DLC.value

        if len(data) > 1:  # mehr als ein Zeichen zu versenden?
            if len(data) > 8:
                self.counter = 0
                lauf_var = 0
                for i in range(len(data)):

                    if type(data[i]) == str:  # String eingegeben?? >> Umwandeln
                        dataC = c.c_ubyte(ord(data[i]))
                    else:
                        dataC = c.c_ubyte(data[i])

                    mStr.m_bData[self.counter] = dataC.value  # Write CType in message struct

                    if (len(data) - 1) == lauf_var:  # Alle Nutzdaten versendet?
                        pad = PACKSIZE_CAN - (self.counter + 1)

                        if pad:  # Paket Auffuellen auf acht zeichen
                            while pad != 0:
                                mStr.m_bData[self.counter + 1] = 78
                                self.counter += 1
                                pad -= 1

                    if self.counter == 7:
                        lib.UcanWriteCanMsg(self.handle, c.byref(mStr))
                        self.counter = 0
                    else:
                        self.counter += 1
                    lauf_var += 1




            else:
                self.counter = len(data)
                pad = PACKSIZE_CAN - self.counter
                counter2 = 0
                for i in range(self.counter):

                    if type(data[counter2]) == str:  # String eingegeben?? >> Umwandeln
                        dataC = c.c_ubyte(ord(data[counter2]))
                    else:
                        dataC = c.c_ubyte(data[counter2])

                    mStr.m_bData[counter2] = dataC  # Write CType in message struct
                    counter2 += 1

                if pad:
                    a = 7
                    while pad > 0:
                        mStr.m_bData[a] = 78
                        a -= 1
                        pad -= 1

                lib.UcanWriteCanMsg(self.handle, c.byref(mStr))
        else:
            if type(data) == str:  # String eingegeben?? >> Umwandeln
                dataC = c.c_byte(ord(data))
            else:
                dataC = c.c_byte(data)

            mStr.m_bData[0] = dataC.value
            i = 1
            while i < 8:
                mStr.m_bData[i] = 0
                i += 1
            lib.UcanWriteCanMsg(self.handle, c.byref(mStr))

    def ReadCanMsg(self, max_trys):

        mStr = tCanMsgStruct()

        counter = 0
        while self.GetMsgPending() == 0:
            # wait 10ms and try again
            time.sleep(0.01)
            counter += 1
            if counter <= max_trys:
                continue
            else:
                return None

        lib.UcanReadCanMsg(self.handle, c.byref(mStr))
        # Notify Connection Status
        # EVT_CUSTOM.ProcessEvent("ifcConnected", mStr != None)

        return mStr

    '''====================================================================
        @brief:   Flush buffers of CAN module
    ===================================================================='''

    def FlushCan(self):

        flag = c.c_uint16(reset_values.USBCAN_RESET_ONLY_ALL_BUFF)
        channel = c.c_int(USBCAN_CHANNEL_ANY)
        lib.UcanResetCanEx(self.handle, channel, flag)
