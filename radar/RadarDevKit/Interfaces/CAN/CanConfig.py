'''
Created on 23.09.2015

@author: IMST GmbH
'''

'''=============================================================================
@brief:        Class for the CAN parameters
============================================================================='''

class CanParams():

    Node_broadcast = 63
    Node_dflt = 4
    Baud_dflt = 1000
    ACMSK =0xFFFFFFFF
    ACCODE=0x00000000

    def __init__(self):

        self.Node = [CanParams.Node_dflt]
        self.Baud = CanParams.Baud_dflt