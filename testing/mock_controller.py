#!/usr/bin/python

"""
This is a mock FPU grid controller which decodes and
synthesizes a response.

"""

from __future__ import print_function
import codec

#  number of buses on one gateway
BUSES_PER_GATEWAY =  5
# number of FPUs on one CAN bus
FPUS_PER_BUS = 67


def command_handler(cmd, socket):
    print("command decoded bytes are:", cmd)
    gateway_id = gateway_map[socket.getsockname()]
    busid = cmd[0]
    canid = cmd[1] + (cmd[2] << 8)
    fpu_busid = canid & 0x7f
    priority = (canid >> 7)
    command_id = cmd[3]
    busnum = busid + gateway_id * BUSES_PER_GATEWAY
    fpu_id = fpu_busid + busnum * FPUS_PER_BUS
    print("CAN command to gw %i, bus %i, fpu # %i (priority %i), command id=%i"
          % (gateway_id, busid, fpu_busid, priority, command_id))
    print("CAN command #%i to FPU %i" % (command_id, fpu_id))

    tx_busid = busid
    tx_prio = 0x02
    tx_canid = (tx_prio << 7) | fpu_busid
    tx0_fpu_busid = fpu_busid
    tx1_cmdid = command_id
    tx2_status = 0
    tx3_errcode = 0
    if command_id == 4:
        pos = 10000 + fpu_busid
    else:
        pos = 10000 + fpu_busid * 100
        
    tx4_count0 = pos & 0xff
    tx5_count1 = (pos >> 8) & 0xff
    tx6_count2 = (pos >> 16) & 0xff
    
    
    resp = [ tx_busid,
             (tx_canid & 0xff),
             ((tx_canid >> 8) & 0xff),
             tx0_fpu_busid,
             tx1_cmdid,
             tx2_status,
             tx3_errcode,
             tx4_count0,
             tx5_count1,
             tx6_count2]
    
    response = codec.encode(resp)
    socket.sendall(response)
    #print("echoed %r" % response)
