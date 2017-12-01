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
from gevent.server import StreamServer
import array

import codec

import mock_controller
from mock_controller import command_handler


# this handler will be run for each incoming connection in a dedicated greenlet
def echo(socket, address):
    print('New connection from %s:%s' % address)
    # using a makefile because we want to use readline()
    msg_len = 10
    prot = codec.Decoder()
    
    while True:
        command = socket.recv(msg_len)
        if not command:
            print("client disconnected")
            break
        prot.decode(command, command_handler, socket)
        

if __name__ == '__main__':
    ports = [ 4700, 4701, 4702]
    ip = '127.0.0.1'
    mock_controller.gateway_map = { (ip, ports[i]) : i for i in range(len(ports))  }
    print("gateway map:", mock_controller.gateway_map)
    servers = [ StreamServer((ip, p), echo, 50) for p in ports]
    # to start the servers asynchronously, we use its start() method;
    # we use blocking serve_forever() for the third and last connection.
    print('Starting mock gateway on ports %s' % ports)
    ##server.serve_forever()
    for s in servers[:-1]:
        s.start()
    servers[2].serve_forever()
