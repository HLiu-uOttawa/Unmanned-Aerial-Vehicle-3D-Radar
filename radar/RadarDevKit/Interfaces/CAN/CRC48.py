'''
Created on 12.11.2015

@author: manfred.haegelen
'''

CRC48_MASK = 0x0000D28DB3FA4AADL
MASK_48BIT = 0x0000FFFFFFFFFFFFL

class CRC48():
    '''
    classdocs
    '''

    '''---------------------------------------------------------------------'''
    def __init__(self):
        '''
        Constructor
        '''
        self.reset()
        
    '''---------------------------------------------------------------------'''
    def reset(self):
        self.crc_reg = 0L
        
    '''---------------------------------------------------------------------'''
    def process_byte(self, val):
        
        for n in range(7,-1,-1):   # 7..0
            
            # extract a single bit from data (start with MSB)
            bit = (val >> n) & 0x1
        
            # shift register 1 bit to the left
            self.crc_reg <<= 1
            
            # check highest bit
            if ((self.crc_reg >> 48) & 0x1) != bit:
                # apply CRC48 polynomial mask
                self.crc_reg ^= CRC48_MASK
                
        # remove 16 highest bits from register
        self.crc_reg &= MASK_48BIT
        
    '''---------------------------------------------------------------------'''
    def process(self, val, nbytes):

        for n in range(nbytes):
            self.process_byte((val >> n*8) & 0xFF)  # start with LSB
            
    '''---------------------------------------------------------------------'''
    def get_crc_bytes(self):
        
        ret = []
        for n in range(5,-1,-1):                            # 5,4,3,2,1,0
            ret.append(int((self.crc_reg >> n*8) & 0xFF))   # start with MSB
        
        return ret

    '''---------------------------------------------------------------------'''
    def get_crc_words(self):
        
        ret = []
        for n in range(2,-1,-1):                                # 2,1,0
            ret.append(int((self.crc_reg >> n*16) & 0xFFFF))    # start with MSB

        return ret

    '''---------------------------------------------------------------------'''
    def result(self):
        
        return self.crc_reg
        
