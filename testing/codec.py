#!/usr/bin/python

STX=0x02 # start transmission
ETX=0x03 # stop transmission
DLE=0x10 # data link escape

class E_STATE:
    UNSYNC = 1
    SYNC_START = 2
    IN_FRAME = 3
    DLE_ESCAPE = 4
    
    

class Decoder:
    def __init__(self):
        self.state = E_STATE.UNSYNC
        self.bytes = []


    def add_byte(self, b, callback):
        if b == DLE:
            if self.state == E_STATE.UNSYNC:
                self.state = E_STARTE.SYNC_START
            elif self.state == IN_FRAME:
                self.state = E_STATE.DLE_ESCAPE
            elif self.state == E_STATE.DLE_ESCAPE:
                self.bytes.append(DLE)
                
        elif b == ETX:
            if self.state == E_STATE.DLE_ESCAPE:
                self.state = E_STATE.UNSYNC
                self.callback(bytes)
            elif self.state == E_STATE.IN_FRAME:
                self.bytes.append(b)
            else:
                pass # ignore
        elif b == STX:
            if self.state == E_STARTE.SYNC_START:
                self.bytes = []
                self.state = E_STATE.IN_FRAME
            elif self.state == E_STATE.IN_FRAME:
                self.bytes.append(b)
            else:
                pass # ignore
        elif self.state == E_STATE.IN_FRAME:
            self.bytes.append(b)
        else:
            pass # data byte without sync

        
    def decode(self, bytes, callback):
        for b in bytes:
            self.add_byte(b, callback)
                
            
def encode(bytes):
    outbytes = []
    for b in bytes:
        outbytes.append(b)
        if b == DLE:
            outbytes.append(DLE)
            
    return outbytes


            
        
    
