#!/usr/bin/python
from __future__ import print_function, division

import os
from sys import argv
import argparse

import lmdb
from interval import Interval

from fpu_constants import *

import FpuGridDriver

DATABASE_FILE_NAME = os.environ.get("FPU_DATABASE")

env = lmdb.open(DATABASE_FILE_NAME, max_dbs=10)

fpudb = env.open_db("fpu")


def flash_FPU(fpu_id, serial_number,mockup=True):
    
    gd = FpuGridDriver.UnprotectedGridDriver(fpu_id+1)

    if mockup:
        gateway_address = [ FpuGridDriver.GatewayAddress("127.0.0.1", p)
                            for p in [4700, 4701, 4702] ]
    else:
        gateway_address = [ FpuGridDriver.GatewayAddress(args.gateway_address, args.gateway_port) ]

    print("connecting grid:", gd.connect(address_list=gateway_address))

    grid_state = gd.getGridState()
    gd.pingFPUs(grid_state)
    gd.readSerialNumbers(grid_state)
    print("flashing FPU #%i with serial number %r" % (fpu_id, serial_number))
    rval = gd.writeSerialNumber(fpu_id, serial_number, grid_state)
    print(rval)


def putInterval(txn, key, interval, offset=0):
    """In theory, it is cleaner to only store
    the relative values. But we want the DB content
    to be human-readable, and easy to interpret,
    and a uniform angle interpretation, 
    so it is better to store posiitional values always 
    along with the offset they refer to."""
    val = repr([Interval(alpha_pos,alpha_pos), offset])
    txn.put(key, val)

            
def getInterval(txn, key, offset=0):
    val = repr([Interval(alpha_pos,alpha_pos), offset])
    val = txn.get(key)
    ivlist, stored_offset = literal_eval(val)
    return Interval(ivlist) + (offset- stored_offset) 



if __name__ == '__main__' :
    print(repr(argv))
    if len(argv) < 2:
        print("""usage:
        init <serial_number> <alpha_pos> <beta_pos> [<adatum_offset>]
        list
        list1 <serial_number>
        alimits <serial_number> <alpha_min> <alpha_max> [<adatum_offset>]
        blimits <serial_number> <beta_min> <beta_max>
        bretries <serial_number> <freebetatries>

        Default alpha datum offset: %f 
        """ % ALPHA_DATUM_OFFSET)
              
    command = argv[1]

    if command == "init":
        if not (len(argv) in [5,6]):
            print("usage: init <serial_number> <alpha_pos> <beta_pos> [<adatum_offset>]")
            exit(1)
            
        serial_number = argv[2]
        alpha_pos = float(argv[3])
        beta_pos = float(argv[4])
        if len(argv) == 6:
            alpha_offset = float(argv[5])
        else:
            alpha_offset = ALPHA_DATUM_OFFSET

        max_waveform = 0
        min_waveform = 0
        waveform_reversed = False

        with env.begin(write=True,db=fpudb) as txn:
            key = str( (serial_number, "apos"))
            val = Interval(alpha_pos,alpha_pos)
            putInterval(txn, key, val, alpha_offset)
            key = str( (serial_number, "bpos"))
            val = Interval(beta_pos,beta_pos)
            putInterval(txn, key, val, BETA_DATUM_OFFSET)
            key = str((serial_number, "wtab"))
            val = repr(Interval(min_waveform, max_waveform))
            txn.put(key,val)
            key = str((serial_number, "wf_reversed"))
            val = str(waveform_reversed)
            txn.put(key,val)
            key = str((serial_number, "alimits"))
            val = Interval(ALPHA_MIN_DEGREE, ALPHA_MAX_DEGREE)
            putInterval(txn, key, val, alpha_offset)
            key = str((serial_number, "blimits"))
            val = Interval(BETA_MIN_DEGREE, BETA_MAX_DEGREE)
            putInterval(txn, key,val, BETA_DATUM_OFFSET)
            key = str((serial_number, "bretries"))
            val = str(DEFAULT_FREE_BETA_RETRIES)
            txn.put(key,val)

    if command == "flash":
        if len(argv) != 4:
            print("usage: init <serial_number> <fpu_id>")
            exit(1)
        serial_number = argv[2]
        assert(len(serial_number) <= 5)
        fpu_id = int(argv[3])
        
        flash_FPU(fpu_id, serial_number)

            
    if command == "alimits":
        if not (len(argv) in [5,6]) :
            print("usage: alimits <amin> <amax> [<alpha_datum_offset>]")
            exit(1)
            
        serial_number = argv[2]
        alpha_min = float(argv[3])
        alpha_max = float(argv[4])
        if len(argv) == 6:
            alpha_offset = float(argv[5])
        else:
            alpha_offset = ALPHA_DATUM_OFFSET
            
        with env.begin(write=True,db=fpudb) as txn:
            key = str((serial_number, "alimits"))
            val = Interval(alpha_min, alpha_max)
            putInterval(txn, key, val, alpha_offset)
            
    elif command == "blimits":
        if len(argv) != 5:
            print("usage: blimits <bmin> <bmax>")
            exit(1)
            
        serial_number = argv[2]
        beta_min = float(argv[3])
        beta_max = float(argv[4])        


        with env.begin(write=True,db=fpudb) as txn:
            key = str((serial_number, "blimits"))
            val = Interval(beta_min, beta_max)
            putInterval(txn, key, val, BETA_DATUM_OFFSET)
              
              
    elif command == "bretries":
        if len(argv) != 5:
            print("usage: bretries <nretries>")
            exit(1)
            
        serial_number = argv[2]
        bretries = int(argv[3])
        

        with env.begin(write=True,db=fpudb) as txn:
            key = str((serial_number, "retries"))
            val = str(bretries)
            txn.put(key,val)
              
              

    elif command == "list":
        with env.begin(db=fpudb) as txn:
            for key, val in txn.cursor():
                print(key, val)

    if command == "list1":
        if len(argv) != 3:
            print("usage: list <serial_number>")
            exit(1)
        serial_number = argv[2]
        with env.begin(db=fpudb) as txn:
            for subkey in ["apos", "bpos", "wtab", "alimits", "blimits", "bretries"]:
                key = str((serial_number, subkey))
                val = txn.get(key)
                          
                print(key,":", val)
                
                

                      
            

