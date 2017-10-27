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
// NAME FPU_CAN_driver.h
//
// This class implements the low-level CAN driver for the MOONS fiber
// positioner grid
//
////////////////////////////////////////////////////////////////////////////////


#include <stdio.h>
#include <string.h>		/// strerror
#include <pthread.h>
#include "stdlib.h"		/// system("/bin/stty raw");
#include "stdbool.h"	/// bool
#include <unistd.h>
#include <stdint.h>
#include <std>

#include "ResponseHandler.h"  // interface for processing received CAN responses

namespace mpifps
{

/* this class provides a buffer that does the byte
   stuffing of messages which is required before
   sending them to the socket interface, as well
   as the unstuffing needed before interpreting
   return messages as commands. */

class SBuffer
{

    sbuffer()
    {
        // initialize state of read and command buffer
        clen = 0;
        sync = false;
        dle = false;
    }

    // encodes a buffer with a CAN message and sends it to
    // the socket identified with sockfd
    // this operation might block!
    int encode_and_send(int sockfd,
                        int const input_len, uint8_t const * const src)
    {
        int out_len = 0;
        int unsent_lern = 0;
        ssize_t retval = 0;

        encode_buffer(input_len, src, out_len, wbuf);
        unsent_len = out_len;
        while ((unsent_len > 0) && (retval >= 0))
        {
            retval = send(sockfd, wbuf, out_len, 0);
            if (retval > 0)
            {
                unsent_len -= retval;
            }
            // we might need to repeat if not all data
            // could be sent (this can happen due to
            // interrupts, full buffers, etc)
        }
        if (retval < 0)
        {
            return errno;
        }
        else
        {
            // returns 0 if socket was closed
            return retval;
        }
    }

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
    int decode_and_process(int sockfd, int bus_id, ResponseHandler rhandler)
    {
        bool frame_complete = false;
        ssize_t rsize = 0;

        rsize = recv(sockfd, rbuf, BUFSIZE, NULL);

        if (rsize < 0)
        {
            return errno;
        }

        for (int i=0; i < rsize; i++)
        {
            frame_complete = decode_and_append_byte(command_buf,  clen, sync, dle,  rbuf[i]);

            if (frame_complete)
            {
                // send the received data to the response handler
                rhandler.handleFrame(bus_id, command_buf, clen);
            }

        }
    }

private:

    const uint8_t STX = 0x02;
    const uint8_t ETX = 0x03;
    const uint8_t DLE = 0x10;

    // The internal buffer needs to have twice the maximum length of a
    // CAN message because four sync bytes are added and each message
    // byte could be encoded as two bytes.
    const int BUFSIZE = 4 + 2 * MAX_CAN_MESSAGE_LENGTH_BYTES;


    inline void byte_stuff(uint8_t * buf, int& out_len, uint8_t const b)
    {
        if (b==DLE)
        {
            buf[outlen++] = b;
        }

        buf[outlen++] = b;
    }

    void encode_buffer(int const input_len, uint8_t const * const src,
                       int& output_len, uint8_t const * dst)
    {
        output_len = 0;
        dst[output_len++] = DLE;
        dst[output_len++] = STX;

        for (int i = 0; i < input_len; i++)
        {
            byte_stuff(dst, output_len, src[i]);
        }

        dst[output_len++] = DLE;
        dst[output_len++] = ETX;
    }

    bool decode_and_append_byte(uint8_t buf*, int& buflen, bool& sync, bool& dle,  uint8_t data)
    {
        bool frame_complete = false;

        if (data == DLE && (!dle))
        {
            dle = true;
            return;
        }

        if (dle)
        {
            dle = false;
            switch (data)
            {
            case STX:
                // start a new frame
                sync = true;
                buflen = 0;
                return;

            case ETX:
                // this marks the end of a frame
                if (sync)
                {
                    sync = false;
                    frame_complete = true;
                }
                return;

            case DLE:
                // interpret this DLE byte as data
                break;

            default:
                sync = false;
                // invalid sequence, skip frame
                return;
            }
        }

        if (sync)
        {
            if (buflen < MAX_CAN_MESSAGE_LENGTH_BYTES)
            {
                buf[buflen++] = data;
            }
            else
            {
                // maximum frame length was exceeded, ignore frame
                sync = false;
            }
        }
        else
        {
            sync = false;
        }
        return;
    }


    // read buffer for data from socket
    uint8_t rbuf[BUFSIZE];
    bool sync;
    bool dle;
    // internal buffer for command
    uint8_t command_buf[MAX_CAN_MESSAGE_LENGTH_BYTES];
    // length of command
    int clen;

    uint8_t wbuf[BUFSIZE];

}

}
