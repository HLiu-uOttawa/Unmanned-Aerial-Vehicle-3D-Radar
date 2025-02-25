# -*- coding: cp1252 -*-

'''=============================================================================
Module:     SPI_Interface.py
            derived from dataTransfer.py

Created on 30.09.2015

@brief:     Classes and Functions to transfer Data between Radar-Module 
            and PC via a serial-port.

@author:   IMST GmbH
============================================================================='''


'''=============================================================================
=imports
============================================================================='''
import string
import time
import d2xx as FTDI
from Interfaces import ConversionFuncs as conv

'''===============================================================================
=Constants
==============================================================================='''
PACKSIZE = 32               #Packet-Size of data 
SYNC_WORD = 0xAA55

#SPI Constants
SPI_LATENCY   = 1
SPI_CLK_DIV_H = 0        #TCK = 60MHz /((1 + [(0xValueH*256) OR 0xValueL])*2) 
SPI_CLK_DIV_L = 9

assert(0<=SPI_CLK_DIV_H<=0xFF)
assert(0<=SPI_CLK_DIV_L<=0xFF)

# requested type of FD data
DT_MAGN = 0
DT_MAGN_PHASE = 1
DT_REAL_IMAG = 2
DT_MAGN_ANGLE = 3

MAX_CHANNELS = 4
TD_SAMPLES = 1024
FD_SAMPLES = 513
 



'''==============================================================================
    @brief:     User defined Exceptions
=============================================================================='''
# class Timeout(Exception):       #Exceptions for read errors
#     def __init__(self, cls, msg):
#         self.cls = cls
#         self.msg = msg
# 
# class WriteError(Exception):       #Exceptions for write errors
#     def __init__(self, cls, msg):
#         self.cls = cls
#         self.msg = msg
#         
# class DataError(Exception):     #exceptions for wrong Data
#     def __init__(self, cls, msg):
#         self.cls = cls
#         self.msg = msg

'''==============================================================================
    @brief:     Class for read write operations on a FT2232-Device using its
                MPSSE-functionality to support an SPI interface
    
    @author:    IMST GmbH
    @date:      30.09.2015
=============================================================================='''        
class SPI_Interface():
    
    '''====================================================================
       @brief: constructor
    ====================================================================''' 
    def __init__(self):
        
        self.availComs = None
        self.availModules = []
        self.initialization()
     
    '''====================================================================
       @brief: comport initialization
    ====================================================================''' 
    def initialization(self, doScan = 1):
        
        self.comport = 0            # instance of the comport
        self.mytimeout = 10          # org. 10
        self.pos = 0   
        
        if doScan == 1:
            self.availComs = None   # List of available Comports
            self.availModules = []
            
        self.comIndex  = None 
        self.rtscts    = 0
        
        self.getAvailCOMs(doScan)
        self.getIndexAvailModul(doScan)
        
    '''====================================================================
       @brief: destructor
    ====================================================================''' 
    def __del__(self):

        self.closeComPort()
            
    '''====================================================================
        @brief: Looks for all available MPSSE-SPI Devices on the System and,
                lists them.
                
        @param  doScan:    0: returns the actual List of MPSSE-Devices,
                           1: do Scan before
        
        @return:           List(tuple) of available ComPorts (Description, No.)
    ====================================================================''' 
    def getAvailCOMs(self, doScan = 0):
           
        if doScan:
            nDev = FTDI.createDeviceInfoList()  #List all FTDI-Devices found
            
            self.availComs = []
            for i in range(nDev):    #check which devices are available
                Info = []            #List element for availComs
                try:
                    devInfo = FTDI.getDeviceInfoDetail(i)
                    dSplit = string.split(devInfo.get("description"))
                    found = 0
                    
                    if devInfo.get("type") == FTDI.DEVICE_2232H and dSplit[len(dSplit)-1] == "A":
                        found = 1
                    elif devInfo.get("type") == 3 and devInfo.get("description") == "C232HM-DDHSL-0":
                        found = 1
                    elif devInfo.get("type") == 5 and devInfo.get("description") == "TTL232R-3V3":
                        #found = 1
                        print "Device Type %s with description %s found -> Old Module"%(devInfo.get("type"), devInfo.get("description"))
                    else:
                        found = 1
                    
                    if found:   
                        Info.append(i+1)                                 #comNumber                                   
                        Info.append(devInfo.get("description"))          #device description
                        Info.append(devInfo.get("serial"))               #serial number
                        self.availComs.append(Info)                      #append device Info to availComs
                
                except:
                    pass
    
        return self.availComs    
    
    '''====================================================================
       @brief: opens an MPSSE-Devise listed in self.availComs in SPI-Mode.
       
       @param comPort:    index of available ComPort in self.availComs
    ====================================================================''' 
    def openComPort(self, comPort = 0, ErrorRaise = False):
        
        #1.----Open the selected FTDI-Channel----
        try:    
            self.comIndex = self.availComs[comPort][0] - 1  #safe the index in availComs of the opened comPort
            self.comport  = FTDI.open(self.comIndex)        #Open the MPSSE-Device at index "comPort" in self.availComs,#safe handle to opened MPSSE-Device in self.comport 

        except:
            self.closeComPort()                             #close the port if any error occurs
            self.comIndex = None
            self.initialization(doScan = 0)
        
        if not self.comport:                                #Error, when no FTDI-Device is connected
            if ErrorRaise:                 
                raise StandardError,"No USB Module found"
            else:
                print("No USB Module found")
                return False
        
        #2.----Configure Port for MPSSE use----
        try:
            self.comport.resetDevice()                      #Reset the USB-Device
            #Purge USB receive buffer first by reading out all old data from FT2232H receive buffer
            nRxBuf = self.comport.getQueueStatus()
            if nRxBuf:
                self.comport.read(nRxBuf)
           
            self.comport.setUSBParameters(65536,65536)   # USB Transfersize is 64k; IN, OUT
            self.comport.setChars("\x00", 0, "\x00", 0)  # No special Characters to indicate events, errors
            self.comport.setTimeouts(0, 5000)            # read/write timeouts
            self.comport.setLatencyTimer(SPI_LATENCY)              # Latencytimer to 1ms (default is 16ms)
            self.comport.setFlowControl (FTDI.FLOW_RTS_CTS, 0x00, 0x00) #Turn on flow control to synchronize IN requests 
            self.comport.setBitMode(0x0, 0x00)           #Reset Controller
            self.comport.setBitMode(0x0, 0x02)           #Enable MPSSE Mode   
        except:
            if ErrorRaise:
                raise StandardError, "Error during MPSSE configuration"
            else:
                print("Error during MPSSE configuration")
                return False
        
        #check Rx-Buffer it should be empty
        nRxBuf = self.comport.getQueueStatus()
        if nRxBuf:
            if ErrorRaise:
                raise StandardError, "Error - MPSSE receive buffer should be empty"
            else:
                print("Error - MPSSE receive buffer should be empty")
                return False
        
        #sync MPSSE device by sending a bad command
        self.comport.write("\xAB")
        time.sleep(0.05)
        nPurgeRX = self.comport.getQueueStatus()
        while nPurgeRX:
            read = self.comport.read(nPurgeRX)
            nPurgeRX = self.comport.getQueueStatus()
         
        if not (read[0] == "\xFA" and read[1] == "\xAB"):
            if ErrorRaise:
                raise StandardError, "Error in synchronizing the MPSSE"
            else:
                print("Error in synchronizing the MPSSE")
                return False
        
        #Set up the Hi-Speed specific commands for the FTx232H 
        buf = "\x8A"                #Use 60MHz master clock (disbale divide by 5) 
        buf+= "\x97"                #Turn off adaptive Clocking
        buf+= "\x8D"                #Disable 3-Phase clocking
        self.comport.write(buf)    #send out HS-specific commands
        
        # Set TCK frequency  
        # TCK = 60MHz /((1 + [(1 +0xValueH*256) OR 0xValueL])*2) 
        buf = "\x86"                #Command to set Clock divider
        buf+= chr(SPI_CLK_DIV_L)    #Set low value of clock divisor
        buf+= chr(SPI_CLK_DIV_H)    #Set high value of clock divisor
        self.comport.write(buf)    #send out clock divisor data
                       
        # Set initial states of the MPSSE interface  
        #  - low byte, both pin directions and output values 
        #  Pin name  Signal  Direction  Config  Initial State  Config 
        #  ADBUS0    TCK/SK  output    1  low    0  (LOW-HIGH-LOW)
        #  ADBUS1    TDI/DO  output    1  low    0 
        #  ADBUS2    TDO/DI  input     0         0 
        #  ADBUS3    TMS/CS  output    1  high   1 
        #  ADBUS4    GPIOL0  output    1  low    0 
        #  ADBUS5    GPIOL1  output    1  high   1 
        #  ADBUS6    GPIOL2  output    1  high   1 
        #  ADBUS7    GPIOL3  output    1  high   1 
        
        #Note: Data will be clocked out on neg clk-edges therefor TCK is low initialy (LOW-HIGH-LOW transients)
        buf = "\x80"     #Configure data bits low-byte of MPSSE port
        buf+= "\xE8"     #initial state of output-pins as described above
        buf+= "\xFB"     #direction of pins as described above
        self.comport.write(buf) #Send out GPIO Config commands   
           
        #open was successful
        return True
    
    '''====================================================================
        @brief: close opened MPSSE-Device
    ====================================================================''' 
    def closeComPort(self):
        
        if self.comport:                #close port
            try:
                self.comport.setBitMode(0x0, 0x00)      #disable MPSSE before closing          
            except:
                pass
            
            self.comport.close()
            self.comport = 0;

    '''====================================================================
        @brief: get status of comport
        
        @return: True: connection  established, False: connection closed     
    ====================================================================''' 
    def is_connected(self):
        
        return (self.comport != 0)

    '''====================================================================
        @brief: transmit data to Radar-Module with opened MPSSE-device
        
        @param msg:    string-data to write. must be at a length of 2n,
                       because SPI ist configured to a character length 
                       of 16bits
        
        @return:       True if no error occured, otherwise False
    ====================================================================''' 
    def transmit(self, msg):
             
        try:
            if 0:   # write blockwise    
                lenH = (len(msg)-1)/256
                lenL = (len(msg)-1)%256  
                self.comport.write("\x80\xE0\xFB\x11"+chr(lenL)+chr(lenH)+msg +"\x80\xE8\xFB")  #data is clocked out on falling edge,
                                                                                                #set CS active before starting transmission, an release it afterwards
                                                                                                
            else:   # write blockwise with reassertion of CS signal
                bf = ""
                for i in range(len(msg)/2):
                    bf += "\x80\xE0\xFB\x11"+chr(1)+chr(0)+msg[2*i:2*i+2] +"\x80\xE8\xFB"
                self.comport.write(bf)
                
            return True     # success
        
        except:
            return False    # command failed

    '''====================================================================
        @brief: receive data from the Radar-Module's SPI interface with the 
                opened MPSSE Device
        
        @param lex:    number of bytes to be read from serial-port
                       should be a multiple of 2 as SPI is configured 
                       to a word length of 16bits
        
        @return:       read data (even if not complete)
    ===================================================================='''        
    def receive(self, lex):
        
        msg = ""
        n = 0
        
        for _i_ in xrange(self.mytimeout):      #try to read complete data for several times   
            try:
                if 0:   # read blockwise
                    lenH = ((lex-n)-1)/256
                    lenL = ((lex-n)-1)%256
                    self.comport.write("\x80\xE1\xFB\x20" + chr(lenL) + chr(lenH) + "\x80\xE9\xFB") #clock data in from Radar-Module, data is latched on rising edge 
                    msg_block = self.comport.read(lex-n)               #read data clocked in from Radar-Module
                
                else:   # read blockwise with reassertion of CS signal
                    bf = ""
                    for _i_ in range(lex/2-n):
                        bf += "\x80\xE1\xFB\x20" + chr(1) + chr(0) + "\x80\xE9\xFB"
                    self.comport.write(bf)
                    msg_block = self.comport.read(lex-n)
            except:
                # reading from port failed => terminate loop
                break
                
            msg += msg_block    # add data block to message 
            n = len(msg)        # number of received bytes

        return msg

    '''====================================================================
        @brief: Looks for all available Radar-Modules at "self.availComs"
                
        @return:    Array-Index of functional modules at "self.availComs"
    ====================================================================''' 
    def getIndexAvailModul(self, doScan = 0, PortIsOpened = True):
        
        if doScan:
            comIndex = None
            if not self.comIndex == None and PortIsOpened:
                comIndex = self.comIndex                #safe index of actual opened comPort 
                self.closeComPort()                     #close opened comPort
                
            self.availModules = []                      #List all indexes which are functional
            for i in range(len(self.availComs)):        #check which Modules are available
                time.sleep(0.1)
                try:
                    # open port, send CMDID_PING to radar and receive the (second) magic word
                    if self.openComPort(i) and self.start_command(0x0000) and self.exit_command(0x0000):
                        self.availModules.append(i)
                        
                except: 
                    pass
                
                self.closeComPort()                     # close comPort in any case            
                    
            if not comIndex == None:                    #restore comPort that was opened before scanning
                self.openComPort(comIndex)
                
        return self.availModules     

    '''====================================================================
        @brief: start command by transmitting sync. pattern and 
                command code to radar module
                    
        @param cmd: command code
        
        @return:    True if no error occured, otherwise False
    ====================================================================''' 
    def start_command(self, cmd):

        return self.transmit(conv.u16_to_string(SYNC_WORD) + conv.u16_to_string(cmd))
        
    '''====================================================================
        @brief: exit command by receiving sync. pattern and
                command code from radar module
                
        @param cmd: command code
        
        @return:    True if no error occured, otherwise False
    ====================================================================''' 
    def exit_command(self, cmd):
        
        if not self.read_sync_word():
            return False
        
        return (self.read_word() == cmd)

    '''====================================================================
        @brief:    read expected sync. pattern 0xAA55 from radar
                   for synchronization purposes.
        
        @timeout:  terminate read process after this time [s]  
        
        @return:   True if no error occured, otherwise False
    ===================================================================='''  
    def read_sync_word(self, timeout=0.1):

        t0 = time.time()
        while (time.time()-t0 < timeout):    #try to receive sync. pattern        

            if conv.string_to_u16(self.receive(2)) == SYNC_WORD:
                return True
            #else:
                #print '>> invalid sync. word'
        
        return False

    '''====================================================================
        @brief:        convert 1 read byte to an integer
    ===================================================================='''  
    def read_byte(self, signed=False):
        
        buf = self.receive(2)
        if len(buf) == 2:
            if signed:
                return conv.string_to_int8(buf[0])   # use lower byte only
            else:
                return conv.string_to_u8(buf[0])     # use lower byte only
        else:
            return []

    '''====================================================================
        @brief:        convert 2 read bytes to an integer
    ===================================================================='''  
    def read_word(self, signed=False):
        
        buf = self.receive(2)
        if len(buf) == 2:
            if signed:
                return conv.string_to_int16(buf)
            else:
                return conv.string_to_u16(buf)
        else:
            return []

    '''====================================================================
        @brief:        convert 4 read bytes to an integer
    ===================================================================='''  
    def read_long(self, signed=False):
        
        buf = self.receive(4)
        if len(buf) == 4:
            if signed:
                return conv.string_to_int32(buf)
            else:
                return conv.string_to_u32(buf)
        else:
            return []

    '''====================================================================
        @brief:        convert 8 read bytes to an integer
    ===================================================================='''  
    def read_longlong(self, signed=False):
        
        buf = self.receive(8)
        if len(buf) == 8:
            if signed:
                return conv.string_to_int64(buf)
            else:
                return conv.string_to_u64(buf)
        else:
            return []

    '''====================================================================
        @brief:        write a 1-byte value to port
    ===================================================================='''  
    def write_byte(self, val, signed=False):
        try:
            if signed:
                self.transmit(conv.int8_to_string(val)+chr(0))   # add second byte
            else:
                self.transmit(conv.u8_to_string(val)+chr(0))     # add second byte
            return True  
        except:
            return False  

    '''====================================================================
        @brief:        write a 2-byte value to port
    ===================================================================='''  
    def write_word(self, val, signed=False):
        try:
            if signed:
                self.transmit(conv.int16_to_string(val))
            else:
                self.transmit(conv.u16_to_string(val))
            return True  
        except:
            return False  

    '''====================================================================
        @brief:        write a 4-byte value to port
    ===================================================================='''  
    def write_long(self, val, signed=False):
        try:
            if signed:
                self.transmit(conv.int32_to_string(val))
            else:
                self.transmit(conv.u32_to_string(val))  
            return True  
        except:
            return False  

    '''====================================================================
        @brief:        write a 8-byte value to port
    ===================================================================='''  
    def write_longlong(self, val, signed=False):
        try:
            if signed:
                self.transmit(conv.int64_to_string(val))
            else:
                self.transmit(conv.u64_to_string(val))  
            return True  
        except:
            return False  

