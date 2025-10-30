'''
Created on 22.09.2015

@author: IMST GmbH
'''

'''=============================================================================
@brief:        Class for the radar System Parameters
============================================================================='''
class SysParams():
    def __init__(self):
        self.diffMode =         0x0     # differential measurement mode ON (1), OFF (0)
        self.t_ramp =           0x1     # ramp time of FMCW chirp
        self.zero_pad =         0x1     # zero-pad factor for smoothing the fft-graph (1, 2, 4, 8)
        self.FFT_data_type =    0x0     # output data type of the fft: 0 -> only magnitudes, 1 -> mag + phase angle, 2 -> real + imag, 3 -> mag + object angle 
        self.frontendEn =       0x1     # enable (1) or disable (0) the frontend power 
        self.powerSaveEn =      0x1     # enable (1) or disable (0) the power switching
        self.norm =             0x0     # distance compensation ON (1) or OFF (0)
        self.active_RX_ch =     0xF     # active Rx channels: e.g. 0000 0000 0000 1011 means Rx1, Rx2 and Rx4 are enabled
        self.advanced =         0x0     # enables/disables the advanced mode, typical not available and also no command ids are available
        self.freq_points =      17      # number of fft bins (1-513)
        self.tic =              600000  # only received, distance bin size [um], defined init value for 250 MHz
        self.doppler =          100000  # only received, doppler bin size [um/s]
        self.freq_bin =         0       # only received, frequency bin [Hz]
        
        # new system parameters used for SC radar
        self.minFreq =          24000   # [MHz]
        self.manualBW =         250     # [MHz]        
        self.SC_Enabled =       True    # Single Chip Radar

        self.atten =            7       # Tx attenuation of the radar chip (7 = 9dB)

'''=============================================================================
@brief:        Class for the radar Hardware Parameters
============================================================================='''          
class HwParams(object):
    def __init__(self):        
        self.flashDate =        0      # default
        self.fwVersion =        0
        self.fwRevision =       0
        self.usedHardware =     0
        self.availChannels =    0
        self.availAlgos =       0
        self.radarNumber =      0
        self.sntID =            0
        self.basebandID =       0
        self.frontendID =       0
        self.phaseOffsetAzi =   0
        self.phaseOffsetEle =   0
        
'''=============================================================================
@brief:        Class for the Air Tracker Parameters
============================================================================='''
class AirTrackerParams():
    def __init__(self):
        self.nRefPulses = 50        # number of reference measurements for noise estimation
        self.nTargets = 5           # maximum number of detected targets (1..10)
        self.nTracks = 5            # maximum number of target tracks (1..10)
        self.enableTracker = 1      # enables tracking filter (0: off, 1: on)
        self.overThreshold = 30     # factor by which the signal must exceed the detection threshold
        self.threshFilter = 60      # length of threshold filter
        self.CFAR = 3               # number of CFAR algorithm
        self.minDistance = 0        # minimum target distance in [m]
        self.minSeparation = 2      # minimum separation between two consecutive targets [bins]
        self.maxHorSpeed = 100      # maximum horizontal speed in [m/s]
        self.maxVerSpeed = 10       # maximum vertical speed in [m/s]
        self.maxAccel = 10          # maximum target acceleration in [m/s^2]
        self.maxRangeErr = 2        # maximum allowed range error in [m] = 3x std(R)
        self.timeOffset = 0         # start time offset in [ms]
        self.timeInterval = 100     # time interval between 2 measurements in [ms]
        self.minConfirmCnt = 2      # minimum number of confirmations
        self.assignLimit = 10       # normalized squared distance for target assignment
        self.mergeLimit = 10        # normalized squared distance for track merging
                
'''=============================================================================
                        Several data transport classes
============================================================================='''
class TD_Data():
    def __init__(self):
        self.nValues        = None
        self.time0          = None
        self.time1          = None
        self.data           = []
        self.stored_data    = self.data
        
    def store_data(self):
        self.stored_data = self.data
        
'''-------------------------------------------------------------------------'''
class FD_Data():
    def __init__(self):
        self.datType            = None
        self.time0              = None
        self.time1              = None
        self.time2              = None
        self.overload           = None
        self.maxRxChannels      = 4
        self.nSamples           = None
        self.maxValues          = []
        self.minValues          = []
        self.data               = []
        self.stored_data        = self.data
        
    def clear(self):
        self.datType            = None
        self.maxValues          = []
        self.minValues          = []
        self.data               = []
        
    def store_data(self):
        self.stored_data = self.data
        
'''-------------------------------------------------------------------------'''
class AirTrackerTargets():
    def __init__(self):
        self.measTime = None
        self.Status = None
        self.nTargets = None
        self.ID = []
        self.RCS = []
        self.dist = []
        self.speed = []
        self.ele = []
        self.azi = []
        self.var_dist = []
        self.var_speed = []
        self.var_ele = []
        self.var_azi = []
        
    def clear(self):
        self.ID = []
        self.RCS = []
        self.dist = []
        self.speed = []
        self.ele = []
        self.azi = []
        self.var_dist = []
        self.var_speed = []
        self.var_ele = []
        self.var_azi = []

'''-------------------------------------------------------------------------'''
class AirTrackerTL():
    def __init__(self):
        self.measTime = None
        self.nTargets = 0
        self.magnitude = []
        self.snr_ele = []
        self.snr_azi = []
        self.distance = []
        self.elevation = []
        self.azimuth = []
        self.tracked = []
    
    def clear(self):
        self.__init__()
        

        