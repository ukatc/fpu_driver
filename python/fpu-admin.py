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






if __name__ == '__main__' :
    print(repr(argv))
    if len(argv) < 2:
        print("""usage:
        init <serial_number> <alpha_pos> <beta_pos>
        list
        list1 <serial_number>
        alimits <serial_number> <alpha_min> <alpha_max>
        blimits <serial_number> <beta_min> <beta_max>
        bretries <serial_number> <freebetatries>
        """)
              
    command = argv[1]

    if command == "init":
        if len(argv) != 5:
            print("usage: init <serial_number> <alpha_pos> <beta_pos>")
            exit(1)
            
        serial_number = argv[2]
        alpha_pos = float(argv[3])
        beta_pos = float(argv[4])

        max_waveform = 0
        min_waveform = 0
        waveform_reversed = False

        with env.begin(write=True,db=fpudb) as txn:
            key = str( (serial_number, "apos"))
            val = str(Interval(alpha_pos,alpha_pos))
            txn.put(key, val)
            key = str( (serial_number, "bpos"))
            val = str(Interval(beta_pos,beta_pos))
            txn.put(key, val)
            key = str((serial_number, "wtab"))
            val = str(Interval(min_waveform, max_waveform))
            txn.put(key,val)
            key = str((serial_number, "wf_reversed"))
            val = str(waveform_reversed)
            txn.put(key,val)
            key = str((serial_number, "alimits"))
            val = str(Interval(ALPHA_MIN_DEGREE, ALPHA_MAX_DEGREE))
            txn.put(key,val)
            key = str((serial_number, "blimits"))
            val = str(Interval(BETA_MIN_DEGREE, BETA_MAX_DEGREE))
            txn.put(key,val)
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
        if len(argv) != 5 :
            print("usage: alimits <amin> <amax>")
            exit(1)
            
        serial_number = argv[2]
        alpha_min = float(argv[3])
        alpha_max = float(argv[4])
        with env.begin(write=True,db=fpudb) as txn:
            key = str((serial_number, "alimits"))
            val = str(Interval(alpha_min, alpha_max))
            txn.put(key,val)
            
    elif command == "blimits":
        if len(argv) != 5:
            print("usage: blimits <bmin> <bmax>")
            exit(1)
            
        serial_number = argv[2]
        beta_min = float(argv[3])
        beta_max = float(argv[4])        


        with env.begin(write=True,db=fpudb) as txn:
            key = str((serial_number, "blimits"))
            val = str(Interval(beta_min, beta_max))
            txn.put(key,val)
              
              
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
                
                

                      
            

