// -*- mode: c++ -*-

////////////////////////////////////////////////////////////////////////////////
//
// Copyright 2017 UKRI. See file "LICENSE" for license information.
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
#include "../EtherCANInterfaceConfig.h"

#include "I_ResponseHandler.h"  // interface for processing received CAN responses

namespace mpifps
{

namespace ethercanif
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

    void setConfig(const EtherCANInterfaceConfig &config_vals);

    // encodes a buffer with a CAN message and sends it to
    // the socket identified with sockfd
    // this operation might block!
    E_SocketStatus encode_and_send(int sockfd,
                                   int const input_len,
                                   const uint8_t bytes[MAX_UNENCODED_GATEWAY_MESSAGE_BYTES],
                                   int busid,
                                   int fpu_canid);

    // we send pending data and return the
    // result of the send command.
    // note: positive number - all OK
    // result is zero - means socket was closed
    // result is negative - error in errno
    E_SocketStatus send_pending(int sockfd);

    // number of bytes which still wait for being sent,
    // from the last command.
    int numUnsentBytes() const;

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


    // The internal buffer needs to have more than twice the maximum
    // length of a CAN message because four sync bytes are added and
    // each message byte could be encoded as two bytes.  We have two
    // start bytes, two stop bytes, and any payload byte can be
    // swizzled to two bytes.
    static const int MAX_STUFFED_MESSAGE_LENGTH = (4 + 2 * MAX_UNENCODED_GATEWAY_MESSAGE_BYTES);

    const int max_gw_delay = 0xff;


    // read buffer for data from socket
    uint8_t rbuf[MAX_STUFFED_MESSAGE_LENGTH];
    bool sync;
    bool dle;
    int unsent_len;
    int out_offset;
    // internal buffer for command
    t_CAN_buffer command_buf;
    uint8_t bus_delays[BUSES_PER_GATEWAY];
    uint8_t fpu_delays[BUSES_PER_GATEWAY][FPUS_PER_BUS];

    // length of command
    int clen;

    uint8_t wbuf[2 * MAX_STUFFED_MESSAGE_LENGTH];

    // this isn't declared as const because sbuffer is an array member
    // in use, and C++11 lacks a pratical way to initialize this
    EtherCANInterfaceConfig config;

};

}

}
#endif
