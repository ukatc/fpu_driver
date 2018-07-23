#!/usr/bin/python
from __future__ import print_function, division

import os
from sys import argv, exit
import argparse

import lmdb
from interval import Interval
from protectiondb import ProtectionDB as pdb

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
        flash <serial_number> <fpu_id>
        init <serial_number> <alpha_pos> <beta_pos> [<adatum_offset>]
        list
        list1 <serial_number>
        alimits <serial_number> <alpha_min> <alpha_max> [<adatum_offset>]
        blimits <serial_number> <beta_min> <beta_max>
        bretries <serial_number> <freebetatries>

        Default alpha datum offset: %f 
        """ % ALPHA_DATUM_OFFSET)
        exit(1)
              
    command = argv[1]

    if command == "init":
        if not (len(argv) in [5,6]):
            print("usage: init <serial_number> <alpha_pos> <beta_pos> [<adatum_offset>]")
            exit(1)
            
        sn = serial_number = argv[2]
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

            pdb.putInterval(txn, sn, pdb.alpha_positions, Interval(alpha_pos,alpha_pos), alpha_offset)
            pdb.putInterval(txn, sn, pdb.beta_positions, Interval(beta_pos,beta_pos), BETA_DATUM_OFFSET)
            pdb.putField(txn, sn, pdb.waveform_table, [])
            pdb.putField(txn, sn, pdb.waveform_reversed, waveform_reversed)
            pdb.putInterval(txn, sn, pdb.alpha_limits, Interval(ALPHA_MIN_DEGREE, ALPHA_MAX_DEGREE), alpha_offset)
            pdb.putInterval(txn, sn, pdb.beta_limits, Interval(BETA_MIN_DEGREE, BETA_MAX_DEGREE), BETA_DATUM_OFFSET)
            pdb.putField(txn, sn, pdb.free_beta_retries, str(DEFAULT_FREE_BETA_RETRIES))
            pdb.putField(txn, sn, pdb.beta_retry_count_cw, str(0))
            pdb.putField(txn, sn, pdb.beta_retry_count_acw, str(0))

    if command == "flash":
        if len(argv) != 4:
            print("usage: flash <serial_number> <fpu_id>")
            exit(1)
        serial_number = argv[2]
        assert(len(serial_number) <= 5)
        fpu_id = int(argv[3])
        
        flash_FPU(fpu_id, serial_number)

            
    if command == "alimits":
        if not (len(argv) in [5,6]) :
            print("usage: alimits <serial_number> <amin> <amax> [<alpha_datum_offset>]")
            exit(1)
            
        serial_number = argv[2]
        alpha_min = float(argv[3])
        alpha_max = float(argv[4])
        if len(argv) == 6:
            alpha_offset = float(argv[5])
        else:
            alpha_offset = ALPHA_DATUM_OFFSET
            
        with env.begin(write=True,db=fpudb) as txn:
            val = Interval(alpha_min, alpha_max)
            pdb.putInterval(txn, serial_number, pdb.alpha_limits, val, alpha_offset)
            
    elif command == "blimits":
        if len(argv) != 5:
            print("usage: blimits <serial_number> <bmin> <bmax>")
            exit(1)
            
        serial_number = argv[2]
        beta_min = float(argv[3])
        beta_max = float(argv[4])        


        with env.begin(write=True,db=fpudb) as txn:
            val = Interval(beta_min, beta_max)
            pdb.putInterval(txn, serial_number, pdb.beta_limits, val, BETA_DATUM_OFFSET)
              
              
    elif command == "bretries":
        if len(argv) != 4:
            print("usage: bretries <serial_number> <nretries>")
            exit(1)
            
        serial_number = argv[2]
        bretries = int(argv[3])
        

        with env.begin(write=True,db=fpudb) as txn:
            val = str(bretries)
            pdb.putField(txn, serial_number, pdb.free_beta_retries, val)
              
              

    elif command == "list":
        with env.begin(db=fpudb) as txn:
            for key, val in txn.cursor():
                print(key, val)

    elif command == "list1":
        if len(argv) != 3:
            print("usage: list1 <serial_number>")
            exit(1)
        serial_number = argv[2]
        with env.begin(db=fpudb) as txn:
            for subkey in [ pdb.alpha_positions,
                            pdb.beta_positions ,
                            # pdb.waveform_table ,
                            pdb.waveform_reversed, 
                            pdb.alpha_limits ,
                            pdb.beta_limits ,
                            pdb.free_beta_retries,
                            pdb.beta_retry_count_cw,
                            pdb.beta_retry_count_acw]:
                
                key = str((serial_number, subkey))
                val = pdb.getRawField(txn,serial_number,subkey)
                          
                print(key,":", val)
                
                

                      
            

