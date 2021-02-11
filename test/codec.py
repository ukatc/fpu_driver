#!/usr/bin/python

import array

STX=0x02 # start transmission
ETX=0x03 # stop transmission
DLE=0x10 # data link escape

class E_STATE:
    UNSYNC = 1
    SYNC_START = 2
    IN_FRAME = 3
    DLE_ESCAPE = 4
    
    

class Decoder:
    def __init__(self, args):
        self.state = E_STATE.UNSYNC
        self.bytelist = []
        self.args = args
        self.verbose=args.debug


    def add_byte(self, b, callback, socket):
        if b == DLE:
            if self.state == E_STATE.UNSYNC:
                self.state = E_STATE.SYNC_START
            elif self.state == E_STATE.IN_FRAME:
                self.state = E_STATE.DLE_ESCAPE
            elif self.state == E_STATE.DLE_ESCAPE:
                self.bytelist.append(DLE)
                self.state = E_STATE.IN_FRAME
                
        elif b == ETX:
            if self.state == E_STATE.DLE_ESCAPE:
                self.state = E_STATE.UNSYNC
                callback(array.array('B', self.bytelist), socket,
                         self.args)
            elif self.state == E_STATE.IN_FRAME:
                self.bytelist.append(b)
            else:
                pass # ignore
        elif b == STX:
            if self.state == E_STATE.SYNC_START:
                self.bytelist = []
                self.state = E_STATE.IN_FRAME
            elif self.state == E_STATE.IN_FRAME:
                self.bytelist.append(b)
            else:
                pass # ignore
        elif self.state == E_STATE.IN_FRAME:
            self.bytelist.append(b)
        else:
            pass # data byte without sync

        
    def decode(self, command, callback, socket):
        command_bytes = array.array('B', command)
        if self.verbose:
            print("command bytes (undecoded)= ", command_bytes)

        for b in command_bytes:
            self.add_byte(b, callback, socket)
                
            
def encode(bytevals,verbose=0):
    if verbose:
        print("response bytevals=", bytevals)
    outbytes = [DLE, STX]
    for b in bytevals:
        outbytes.append(b)
        if b == DLE:
            outbytes.append(DLE)
    outbytes.append(DLE)
    outbytes.append(ETX)
            
    return array.array('B', outbytes).tostring()


            
        
    
