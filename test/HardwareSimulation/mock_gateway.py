#!/usr/bin/env python
"""

Python script which starts a simple server that listens on TCP ports
4700, 4701 and 4702 (see DEFAULT_PORTS) and simulates an EtherCAN
device responding to the MOONS fibre positioner communication protocol.

Example usage (for 3 FPUs and verbose mode):

    python mock_gateway.py -N 3 [-v 3]

Terminate using Ctrl-C.

This test code uses the gevent library for asynchronous I/O in python.
The reason for chosing gevent is that it is widely used and available
on Python 2.7 as well as Python 3.4.  Future versions could use the
curio library for more complex tasks, which uses Python3.5's coroutines.

See http://www.gevent.org/contents.html

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

# Default TCP ports devoted to mock controller.
DEFAULT_PORTS = [ 4700, 4701, 4702]

# Default debugging mode from environment variable DEBUG
DEBUG=int(os.environ.get("DEBUG","0"))

# Default number of FPUs, either from NUM_FPUS environment variable
# or from the maximum number each gateway can support.
if "NUM_FPUS" in os.environ:
    DEFAULT_NUM_FPUS=int(os.environ.get("NUM_FPUS"))
else:
    DEFAULT_NUM_FPUS = len(DEFAULT_PORTS) * FPUS_PER_BUS * BUSES_PER_GATEWAY


# This handler will be run for each incoming connection in a dedicated greenlet
def gateway(socket, address, args):
    print('New connection from %s:%s' % address)
    # using a makefile because we want to use readline()
    msg_len = 10
    prot = codec.Decoder(args)

    socket.setsockopt(IPPROTO_TCP, TCP_NODELAY, 1)

    # Append to list of sockets, for handling SYNC commands
    sockname = socket.getsockname()
    print("Appending socket '%r' for server address %s to socket list" % (socket, sockname))
    mock_controller.gateway_socket_map[sockname] = socket

    while True:
        command = socket.recv(msg_len)
        if not command:
            print("Client disconnected")
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

    parser.add_argument('-v', '--verbosity', dest='verbosity',  type=int,
                        default=1,
                        help='verbosity: 0=silent; 1=minimal; 2=info; 3=verbose; 4=debug; 5=trace')

    parser.add_argument('-V', '--protocol_version',  dest='protocol_version',
                        default="2.0.0",
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

    parser.add_argument('-fda', '--fail-datum-alpha', type=int, action='append', default=[],
                        help="list of units which simulate a failure of the alpha datum operation, sending a time-out response")

    parser.add_argument('-fdb', '--fail-datum-beta', type=int, action='append', default=[],
                        help="list of units which simulate a failure of the beta datum operation, sending a time-out response")

    args = parser.parse_args()
    version_tuple = tuple(map(int, args.protocol_version.split(".")))

    while len(version_tuple) < 3:
        version_tuple = version_tuple + [0]
        #print("Firmware version=", version_tuple)
    args.fw_version = version_tuple
    #print("Firmware version :", args.fw_version)

    del args.protocol_version # delete for safety

    args.fw_date = tuple(map(int, args.firmware_date.split("-")))

    return args


if __name__ == '__main__':
    ip = '127.0.0.1'

    args = parse_args()

    print("Protocol firmware version:", args.fw_version)
    print("Listening to ports:", args.ports)
    print("Number of FPUs    :", args.NUM_FPUS)

    fpu_sim.init_FPUGrid(args, args.NUM_FPUS)
    mock_controller.gateway_map = { (ip, args.ports[i]) : i for i in range(len(args.ports))  }
    print("Gateway map:", mock_controller.gateway_map)
    pool = Pool(10000)

    start_gateway = lambda socket, address: gateway(socket, address, args)

    servers = [ StreamServer((ip, p), start_gateway, spawn=pool) for p in args.ports]

    # To start each server asynchronously, we use its start() method;
    # we use blocking serve_forever() for the third and last connection.
    print('Starting mock gateway on ports %s' % args.ports)

    for s in servers[:-1]:
        s.start()
    servers[-1].serve_forever()
