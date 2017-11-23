// -*- mode: c++ -*-

////////////////////////////////////////////////////////////////////////////////
// ESO - VLT Project
//
// Copyright 2017 E.S.O,
//
// Who       When        What
// --------  ----------  -------------------------------------------------------
// Pablo Gutierrez 2017-07-22  created CAN client sample
// jnix      2017-10-18  Created driver class using Pablo Guiterrez' CAN client sample
//------------------------------------------------------------------------------

////////////////////////////////////////////////////////////////////////////////
// NAME SBuffer.h
//
// This class implements the low-level CAN driver for the MOONS fiber
// positioner grid
//
////////////////////////////////////////////////////////////////////////////////

#ifndef S_BUFFER_H
#define S_BUFFER_H

#include <string.h>		/// memset()
#include <stdint.h>

#include "CAN_Constants.h"

#include "I_ResponseHandler.h"  // interface for processing received CAN responses

namespace mpifps
{

namespace canlayer
{

/* this class provides a buffer that does the byte
   stuffing of messages which is required before
   sending them to the socket interface, as well
   as the unstuffing needed before interpreting
   return messages as commands. */

class SBuffer
{
public:

    enum E_SocketStatus
    {
        // everything worked
        ST_OK = 0,

        // the connection was lost
        ST_NO_CONNECTION = 1,

        // An assumption about the connection state
        // is not met (probably logical error)

        ST_ASSERTION_FAILED=2,
    };


    SBuffer();

    // encodes a buffer with a CAN message and sends it to
    // the socket identified with sockfd
    // this operation might block!
    E_SocketStatus encode_and_send(int sockfd,
                                   int const input_len,
                                   uint8_t bytes[MAX_CAN_MESSAGE_BYTES]);

    // we send pending data and return the
    // result of the send command.
    // note: positive number - all OK
    // result is zero - means socket was closed
    // result is negative - error in errno
    E_SocketStatus send_pending(int sockfd);

    // number of bytes which still wait for being sent,
    // from the last command.
    int numUnsentBytes();

    // reads data from a socket (which presumable has been
    // indicated to have new data available), unwraps and
    // stores read data bytes in an command buffer,
    // and executes the response handler when a complete
    // response has been received.
    // the return value is either zero, or the
    // errno value when reading from the socket failed
    // (for example because the connection was closed
    // by another thread).
    // The blocking behavior is inherited from the
    // recv() system call - it should be non-blocking
    // because poll() was inquired whether the used
    // socket has data available.
    E_SocketStatus decode_and_process(int sockfd, int gateway_id, I_ResponseHandler* rhandler);

private:


    // The internal buffer needs to have twice the maximum length of a
    // CAN message because four sync bytes are added and each message
    // byte could be encoded as two bytes.
    const static int BUFSIZE = 2 * MAX_CAN_MESSAGE_BYTES;


    // read buffer for data from socket
    uint8_t rbuf[BUFSIZE];
    bool sync;
    bool dle;
    int unsent_len;
    int out_offset;
    // internal buffer for command
    uint8_t command_buf[MAX_CAN_MESSAGE_BYTES];
    // length of command
    int clen;

    uint8_t wbuf[BUFSIZE];

};

}

}
#endif
