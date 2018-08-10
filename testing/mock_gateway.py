#!/usr/bin/env python
"""Simple server that listens on ports 4700,4701, and 4702, prints input and  and echoes it back to the client.

Terminate using Ctrl-C.

This test code uses the gevent library for asynchronous I/O in python.
The reason for chosing gevent is that it is widely used and available
on Python 2.7 as well as Python 3.4.  Future versions should probably
use the curio library for more complex tasks, which uses Python3.5's
coroutines.

"""
from __future__ import print_function

import argparse

from gevent.server import StreamServer
from gevent.pool import Pool
from gevent.monkey import patch_all
from socket import IPPROTO_TCP, TCP_NODELAY
import os
import array

import codec

import fpu_sim
import mock_controller as mock_controller
from mock_controller import FPUS_PER_BUS, BUSES_PER_GATEWAY

from mock_controller import command_handler

DEFAULT_PORTS = [ 4700, 4701, 4702]

DEBUG=int(os.environ.get("DEBUG","0"))

if os.environ.has_key("NUM_FPUS"):
    DEFAULT_NUM_FPUS=int(os.environ.get("NUM_FPUS"))
else:
    DEFAULT_NUM_FPUS = len(DEFAULT_PORTS) * FPUS_PER_BUS * BUSES_PER_GATEWAY

# this handler will be run for each incoming connection in a dedicated greenlet
def gateway(socket, address, args):
    print('New connection from %s:%s' % address)
    # using a makefile because we want to use readline()
    msg_len = 10
    prot = codec.Decoder(args)

    socket.setsockopt(IPPROTO_TCP, TCP_NODELAY, 1)

    while True:
        command = socket.recv(msg_len)
        if not command:
            print("client disconnected")
            break
        prot.decode(command, command_handler, socket)
        

def parse_args():
    parser = argparse.ArgumentParser(description='Start EtherCAN gateway simulation')
    parser.add_argument('ports', metavar='p', type=int, nargs='*',
                        default = DEFAULT_PORTS,
                        help='ports which will listen to a connection')
    
    parser.add_argument('-d', '--debug', dest='debug',  action='store_true',
                        default=DEBUG,
                        help='print received binary commands and responses')
    
    parser.add_argument('-v', '--verbosity', dest='verbosity',  
                        default=1,
                        help='verbosity: 0 - no extra output ... 5 - print extensive debug output')

    parser.add_argument('-V', '--protocol_version',  dest='protocol_version',
                        default="1.4.3",
                        help='CAN protocol version')

    parser.add_argument('-t', '--datum_alpha_timeout_steps',  dest='datum_alpha_timeout_steps',
                        default=500,
                        help='timeout limit for alpha arm, in steps')
    
    parser.add_argument('-b', '--datum_beta_timeout_steps',  dest='datum_beta_timeout_steps',
                        default=125 * 20,
                        help='timeout limit for beta arm, in steps')

    parser.add_argument('-D', '--firmware_date',  dest='firmware_date',
                        default="18-01-01",
                        help='ISO timestamp with firmware date (format yy-mm-dd as "18-12-31")')
    
    parser.add_argument('-N', '--NUM_FPUS',  type=int, dest='NUM_FPUS',
                        default=int(DEFAULT_NUM_FPUS),
                        help='number of simulated FPUs')

    parser.add_argument('-O', '--alpha-datum-offset',  type=float, dest='alpha_datum_offset',
                        default=-180.0,
                        help=("""Conventional angle of datum position."""))
    
    parser.add_argument('-A', '--alpha-start',  type=float, dest='alpha_start',
                        default=0.0,
                        help=("""simulated offset of alpha arm at start, 
                        when the step count is 0.
                        This can be used to simulate conditions like a power failure"""))
    
    parser.add_argument('-B', '--beta-start',  type=float, dest='beta_start',
                        default=0.0,
                        help='simulated offset of beta arm at start')
    
    args = parser.parse_args()
    
    version_tuple = map(int, args.protocol_version.split("."))
    
    while len(version_tuple) < 3:
        version_tuple = version_tuple + [0]
        print("firmware version=", version_tuple)
    args.fw_version = tuple(version_tuple)
    
    del args.protocol_version # delete for safety

    args.fw_date = map(int, args.firmware_date.split("-"))

    return args
     

        
if __name__ == '__main__':
    ip = '127.0.0.1'
    
    args = parse_args()
    
    
    print("protocol_version:", args.fw_version)
    print("listening to ports:", args.ports)
    print("listening to ports:", args.ports)
    print("number of FPUs    :", args.NUM_FPUS)
    
    fpu_sim.init_FPUGrid(args, args.NUM_FPUS)
    mock_controller.gateway_map = { (ip, args.ports[i]) : i for i in range(len(args.ports))  }
    print("gateway map:", mock_controller.gateway_map)
    pool = Pool(10000)

    start_gateway = lambda socket, address: gateway(socket, address, args)
    
    servers = [ StreamServer((ip, p), start_gateway, spawn=pool) for p in args.ports]
    # to start the servers asynchronously, we use its start() method;
    # we use blocking serve_forever() for the third and last connection.
    print('Starting mock gateway on ports %s' % args.ports)
    ##server.serve_forever()
    for s in servers[:-1]:
        s.start()
    servers[2].serve_forever()
