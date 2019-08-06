#!/usr/bin/python
from __future__ import print_function, division

import os
import sys
import time
import argparse

import FpuGridDriver
from FpuGridDriver import  DASEL_BOTH, DASEL_ALPHA, DASEL_BETA, \
    SEARCH_CLOCKWISE, SEARCH_ANTI_CLOCKWISE, SEARCH_AUTO, SKIP_FPU, \
    REQD_CLOCKWISE, REQD_ANTI_CLOCKWISE, DATUM_TIMEOUT_DISABLE

from fpu_commands import *

# query environment for default number of FPUs.
NUM_FPUS = int(os.environ.get("NUM_FPUS","4"))


def parse_args():
    parser = argparse.ArgumentParser(description='Send a number of CAN commands to an FPU controller, to test connectivity.')

    parser.add_argument('--mockup',   default=False, action='store_true',
                        help="set gateway address to use mock-up gateway and FPU "
                             "(you need to run the mock-up gateway simulation in another terminal for this to work)")

    parser.add_argument('--resetFPU',   default=False, action='store_true',
                        help='reset FPU controller so that earlier aborts / collisions are ignored')

    parser.add_argument('--gateway-port', metavar='GATEWAY_PORT', type=int, default=4700,
                        help='EtherCAN gateway port number (default: %(default)s)')

    parser.add_argument('--gateway-address', metavar='GATEWAY_ADDRESS', type=str, default="192.168.0.11",
                        help='EtherCAN gateway IP address or hostname (default: %(default)r)')

    parser.add_argument('-D', '--bus-repeat-dummy-delay',  metavar='BUS_REPEAT_DUMMY_DELAY',
                        type=int, default=0,
                        help='Dummy delay inserted before writing to the same bus (default: %(default)s).')

    parser.add_argument('-N', '--NUM-FPUS',  metavar='NUM_FPUS', dest='N', type=int, default=NUM_FPUS,
                        help='Number of adressed FPUs (default: %(default)s).')

    parser.add_argument('-K', '--COUNT-COMMANDS',  metavar='COUNT_COMMANDS', dest='K', type=int, default=6000000,
                        help='COUNT of high-level commands (default: %(default)s).')

    parser.add_argument('-t', '--trace-messages',   default=False, action='store_true',
                        help="enable log level LOG_TRACE_CAN_MESSAGES which will log "
                        "all sent and received CAN messages (this requires some amount of disk space).")

    parser.add_argument('--command', metavar='COMMAND', type=str, default="configMotion",
                        help='CAN command which is sent in test. COMMAND can be either '
                        '"ping" or "configMotion" (default: %(default)r)')

    args = parser.parse_args()
    return args


def initialize_FPUs(args):

    # We initialize a driver instance without position tracking and
    # software protection. This allows to skip to install and
    # initialize the LMDB position database, but also disables any safety
    # checks on valid movement ranges.
    #
    # Warning: This driver class should *not* be used in combination
    # with actual hardware, which can be damaged by out-of-range
    # movements.

    if args.trace_messages:
        loglevel = FpuGridDriver.LOG_TRACE_CAN_MESSAGES
    else:
        loglevel = FpuGridDriver.LOG_INFO

    gd = FpuGridDriver.UnprotectedGridDriver(args.N, # number of FPUs
                                             # dummy delay before sending to the same bus
                                             min_bus_repeat_delay_ms=args.bus_repeat_dummy_delay,
                                             # dummy delay before sending to the same FPU
                                             min_fpu_repeat_delay_ms=0,
                                             # disable re-sending some CAN command messages in case of time-outs
                                             # (directly return an error instead)
                                             configmotion_max_retry_count=0,
                                             configmotion_max_resend_count=0,
                                             # set log level to low value
                                             # (try FpuGridDriver.LOG_TRACE_CAN_MESSAGES to have
                                             # every CAN message logged)
                                             logLevel=loglevel,
                                             log_dir="./_logs")
    if args.mockup:
        # test against fast hardware simulation
        gateway_address = [ FpuGridDriver.GatewayAddress("127.0.0.1", p)
                            for p in [4700, 4701, 4702] ]
    else:
        gateway_address = [ FpuGridDriver.GatewayAddress(args.gateway_address, args.gateway_port) ]

    print("connecting grid:", gd.connect(address_list=gateway_address))


    # We monitor the FPU grid by a variable which is
    # called grid_state, and reflects the state of
    # all FPUs when each command returns.
    grid_state = gd.getGridState()

    if args.resetFPU:
        print("resetting FPU controller")
        gd.resetFPUs(grid_state)
        print("OK")

    # warn about unprotected, low-level access (this driver class does
    # not use LMDB, and therefore cannot protect precious FPU hardware
    # against potentially destructive movements)
    if gd is not FpuGridDriver.GridDriver:
        print("Warning: This driver class should *not* be used in combination"
              " with real mechanical FPU hardware, which can be damaged by out-of-range"
              " movements.")


    return gd, grid_state


if __name__ == '__main__':

    print("module version is:", FpuGridDriver.__version__, ", CAN PROTOCOL version:", FpuGridDriver.CAN_PROTOCOL_VERSION)

    args = parse_args()

    gd, grid_state = initialize_FPUs(args)

    print("retrieving firmware versions...")
    gd.printFirmwareVersion(grid_state)


    print("testGateway: issuing %i %s commands with dummyDelay=%i to %i FPUs:" %
          (args.K, args.command, args.bus_repeat_dummy_delay, args.N))

    if args.command == "configMotion":
        # pre-generate a waveform table with about the maximum number of entries
        # (each entry becomes an individual message)
        waveform = gen_wf([325] * args.N, 5, max_change=1.03)
        messages_per_command = args.N * len(waveform[0])
        report_interval = 2

        # Send an enableMove command to each FPU,
        # to unlock uninitialized FPUs.
        for fpu_id in range(args.N):
            gd.enableMove(fpu_id, grid_state)

    else:
        messages_per_command = args.N
        report_interval = 20

    start_time = time.time()
    nmsgs = 0
    num_commands = (args.K + messages_per_command -1) // messages_per_command
    try:
        for k in range(num_commands):
            if args.command == "ping":
                gd.pingFPUs(grid_state)
            elif args.command == "configMotion":
                # we need to pass 'allow_uninitialized' because the FPUs are
                # not datumed
                gd.configMotion(waveform, grid_state, allow_uninitialized=True)
            else:
                raise ValueError("command type %s not recognized" % args.command)

            nmsgs += messages_per_command
            print(".",end='')
            sys.stdout.flush()
            if ((k +1) % report_interval) == 0:
                elapsed_time_sec =  time.time() - start_time
                print("%i messages in %7.2f seconds = %7.1f messages/sec, count_timeouts = %i"
                      % (nmsgs, elapsed_time_sec, nmsgs / elapsed_time_sec, grid_state.count_timeout))
        print("")
    finally:
        elapsed_time_sec =  time.time() - start_time
        print("%i messages in %7.2f seconds = %7.1f messages/sec"
              % (nmsgs, elapsed_time_sec, nmsgs / elapsed_time_sec))
