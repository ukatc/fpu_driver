#!/usr/bin/python

"""
This is a mock FPU grid controller which decodes and
synthesizes a response.

"""

from __future__ import print_function
import codec


def command_handler(cmd, socket):
    print("command decoded bytes are:", cmd)
    gateway_id = gateway_map[socket.getsockname()]
    busid = cmd[0]
    canid = cmd[1] + (cmd[2] << 8)
    fpuid = canid & 0x7f
    priority = (canid >> 7)
    command_id = cmd[3]
    print("CAN command to gw %i, bus %i, fpu # %i (priority %i), command id=%i"
          % (gateway_id, busid, fpuid, priority, command_id))

    tx_busid = busid
    tx_prio = 0x02
    tx_canid = (tx_prio << 7) | fpuid
    tx0_fpuid = fpuid
    tx1_cmdid = command_id
    tx2_status = 0
    tx3_errcode = 0
    if command_id == 4:
        pos = 10000 + fpuid
    else:
        pos = 10000 + fpuid * 100
        
    tx4_count0 = pos & 0xff
    tx5_count1 = (pos >> 8) & 0xff
    tx6_count2 = (pos >> 16) & 0xff
    
    
    resp = [ tx_busid,
             (tx_canid & 0xff),
             ((tx_canid >> 8) & 0xff),
             tx0_fpuid,
             tx1_cmdid,
             tx2_status,
             tx3_errcode,
             tx4_count0,
             tx5_count1,
             tx6_count2]
    
    response = codec.encode(resp)
    socket.sendall(response)
    #print("echoed %r" % response)
