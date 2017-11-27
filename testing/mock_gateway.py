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


# this handler will be run for each incoming connection in a dedicated greenlet
def echo(socket, address):
    print('New connection from %s:%s' % address)
    # using a makefile because we want to use readline()
    msg_len = 10
    while True:
        command = socket.recv(msg_len)
        if not command:
            print("client disconnected")
            break
        socket.sendall(command)
        print("echoed %r" % command)

if __name__ == '__main__':
    # to make the server use SSL, pass certfile and keyfile arguments to the constructor
    ports = [ 4700, 4701, 4702]
    servers = [ StreamServer(('0.0.0.0', p), echo) for p in ports]
    # to start the server asynchronously, use its start() method;
    # we use blocking serve_forever() for the third and last connection.
    print('Starting mock gateway on ports %s' % ports)
    ##server.serve_forever()
    for s in servers[:-1]:
        s.start()
    servers[2].serve_forever()
