// -*- mode: c++ -*-

////////////////////////////////////////////////////////////////////////////////
//
// Copyright 2017 UKRI. See file "LICENSE" for license information.
//
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

#include <sys/types.h>
#include <sys/socket.h>
#include <errno.h>
#include <cassert>
#include <stdio.h>

#include <algorithm>

#include "ethercan/SBuffer.h"
#include "ethercan/CAN_Command.h"
#include "ethercan/time_utils.h"

// FIXME: reading and writing data is technically unrelated, and
// should probably splitted into two classes.

// An important opportunity for speeding up loading of the waveform
// tables is to aggregate data and to send it in bulk commands.  This
// is more efficient than to send many small packets over the sockets
// in different syscalls. Needs check whether this is relevant for
// performance.

namespace mpifps
{

namespace ethercanif
{

using std::min;
using std::max;

const uint8_t STX = 0x02;
const uint8_t ETX = 0x03;
const uint8_t DLE = 0x10;


inline void byte_stuff(uint8_t * buf, int& out_len, uint8_t const b)
{
    if (b==DLE)
    {
        buf[out_len++] = b;
    }

    buf[out_len++] = b;
}

inline void encode_buffer(int const input_len, uint8_t const * const src,
                          int& output_len, uint8_t * const dst)
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


// returns true when frame is complete
inline bool decode_and_append_byte(uint8_t* buf,
                                   int& buflen,
                                   bool& sync,
                                   bool& dle,
                                   uint8_t data)
{
    bool frame_complete = false;

    if (data == DLE && (!dle))
    {
        dle = true;
        return frame_complete;
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
            return frame_complete;

        case ETX:
            // this marks the end of a frame
            if (sync)
            {
                sync = false;
                frame_complete = true;
            }
            return frame_complete;

        case DLE:
            // interpret this DLE byte as data
            break;

        default:
            sync = false;
            // invalid sequence, skip frame
            return frame_complete;
        }
    }

    if (sync)
    {
        if (buflen < MAX_UNENCODED_GATEWAY_MESSAGE_BYTES)
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
    return frame_complete;
}


SBuffer::SBuffer()
{
    // initialize state of read and command buffer
    clen = 0;
    sync = false;
    dle = false;
    // set unsent length of write buffer to zero
    unsent_len = 0;
    out_offset = 0;

    // zero out buffers - this is defensive
    memset(rbuf, 0, sizeof(rbuf));
    memset(wbuf, 0, sizeof(wbuf));
    memset(command_buf.bytes, 0, sizeof(command_buf.bytes));

    memset(bus_delays, max_gw_delay, sizeof(bus_delays));
    memset(fpu_delays, max_gw_delay, sizeof(fpu_delays));
}

void SBuffer::setConfig(const EtherCANInterfaceConfig &config_vals)
{
    config = config_vals;
    // config must not be changed any more, it is meant to be const

}


SBuffer::E_SocketStatus SBuffer::encode_and_send(int sockfd,
        int const input_len,
        const uint8_t src[MAX_UNENCODED_GATEWAY_MESSAGE_BYTES],
        int busid,
        int fpu_canid)
{
    int out_len = 0;

    int gw_delay = 0;
    const int min_bus_repeat_delay_ms = max(0, min(config.min_bus_repeat_delay_ms, max_gw_delay));
    const int min_fpu_repeat_delay_ms = max(0, min(config.min_fpu_repeat_delay_ms, max_gw_delay));


    if (bus_delays[busid] < min_bus_repeat_delay_ms)
    {
        gw_delay = (min_bus_repeat_delay_ms - static_cast<unsigned int>(bus_delays[busid]));
    }

    int fpu_mindelay = max_gw_delay;
    if (fpu_canid > 0)
    {
        fpu_mindelay = fpu_delays[busid][fpu_canid -1];
    }
    else
    {
        for(int i = 0; i < FPUS_PER_BUS; i++)
        {
            if (fpu_mindelay > fpu_delays[busid][i])
            {
                fpu_mindelay = fpu_delays[busid][i];
            }
        }
    }
    if (gw_delay < min_fpu_repeat_delay_ms - fpu_mindelay)
    {
        gw_delay = min_fpu_repeat_delay_ms - fpu_mindelay;
    }

    {
        if (gw_delay > max_gw_delay)
        {
            gw_delay = max_gw_delay;
        }

        if (gw_delay > 0)
        {
            t_CAN_buffer delay_msg;
            memset(delay_msg.bytes, 0, sizeof(delay_msg.bytes));
            delay_msg.message.busid = MSG_TYPE_DELY; // that's the pseudo bus number which receives a delay message
            delay_msg.message.identifier = 0x777; // that's from Pablo's sample code but probably irrelevant
            delay_msg.message.data[0] = gw_delay & 0xff;
            const int msg_len = 4;

            LOG_TX(LOG_VERBOSE, "%18.6f : TX: encode_and_send(): pre-pending dummy delay = %i\n",
                   ethercanif::get_realtime(),
                   gw_delay);


            {
                const int LINE_LEN=128;
                char log_buffer[LINE_LEN];

                int buf_idx = sprintf(log_buffer, "delay message bytes (len=%i)= [", input_len);

                if (config.logLevel >= LOG_TRACE_CAN_MESSAGES)
                {
                    for(int i=0; i < input_len; i++)
                    {
                        int nchars = sprintf(log_buffer + buf_idx," %02x", delay_msg.bytes[i]);
                        buf_idx += nchars;

                        sprintf(log_buffer + buf_idx,"]\n");
                    }


                    LOG_TX(LOG_TRACE_CAN_MESSAGES, "%18.6f : TX: encode_and_send(): sending %s",
                           ethercanif::get_realtime(),
                           log_buffer);
                }
            }

            // (note: wbuf was increased to make room for added dummy delay message)
            encode_buffer(msg_len, delay_msg.bytes, out_len, wbuf);
        }

        // count up running delay for all buses and FPUs which were not addressed
        for(int b = 0; b <  BUSES_PER_GATEWAY; b++)
        {
            if (b == busid)
            {
                bus_delays[b] = 0;
            }
            else
            {
                bus_delays[b] = min(bus_delays[b] + gw_delay, max_gw_delay);
            }

            for(int i = 0; i <  FPUS_PER_BUS; i++)
            {
                if ((b == busid) && ((fpu_canid == 0) || (i == (fpu_canid -1)) ))
                {
                    // note: CAN broadcast re-sets delays for all FPUs on the same bus
                    fpu_delays[b][i]= 0;
                }
                else
                {
                    fpu_delays[b][i] = min(fpu_delays[b][i] + gw_delay, max_gw_delay);
                }
            }
        }

    }

    {
        const int LINE_LEN=128;
        char log_buffer[LINE_LEN];

        int buf_idx = sprintf(log_buffer, "command bytes (len=%i)= [", input_len);

        if (config.logLevel >= LOG_TRACE_CAN_MESSAGES)
        {
            for(int i=0; i < input_len; i++)
            {
                int nchars = sprintf(log_buffer + buf_idx," %02x", src[i]);
                buf_idx += nchars;

                sprintf(log_buffer + buf_idx,"]\n");
            }


            LOG_TX(LOG_TRACE_CAN_MESSAGES, "%18.6f : TX: encode_and_send(): sending %s",
                   ethercanif::get_realtime(),
                   log_buffer);
        }
    }

    const int out_len1 = out_len;
    encode_buffer(input_len, src, out_len, wbuf + out_len1);
    out_offset = 0;
    unsent_len = out_len + out_len1;

    return send_pending(sockfd);
}


SBuffer::E_SocketStatus SBuffer::send_pending(int sockfd)
{
    // note, we use the DONTWAIT flag here even if
    // we checked writability before using poll()
    // - in some cases, operation can still block,
    // so we double-check.

    bool do_retry = false;
    int retval = 0;
    int out_len = unsent_len;
    do
    {
        do_retry = false;
        retval = send(sockfd, wbuf + out_offset, out_len,
                      MSG_DONTWAIT | MSG_NOSIGNAL);
        if (retval == 0)
        {
            // a return value of zero indicates that
            // the connection was closed
            //
            // This happens if the tcp connection is
            // lost, for example to a physical connection
            // problem. As TCP will normally try and
            // resent packes until connection time-outs are reached,
            // there is not much we can do at that
            // level, so let the driver return an error.
            return ST_NO_CONNECTION;
        }
        if (retval < 0)
        {
            int errcode = errno;
            switch (errcode)
            {
            case EWOULDBLOCK:
            case ECONNRESET:
            case ENOBUFS :
                // sending data would block, and the MSG_DONTWAIT
                // flag was set.  In Linux, this seems possible to
                // happen even if poll indicates that data can be
                // sent.  We just try to send later.
                return ST_OK;
                break;

            case EINTR:
                // a signal happened and interrupted the syscall
                // we just retry the send() call.
                do_retry = true;
                break;

            case ENOTCONN: // socket not connected
            case EPIPE: // connection has been shut down
                return ST_NO_CONNECTION;

            case EINVAL: // invalid argument
            case EBADF: // bad file descriptor
            case ENOTSOCK: // descriptor is not a socket
            case EFAULT: // invalid user space address
            case EMSGSIZE : // message too large
            case ENOMEM: // no memory available
            case EOPNOTSUPP: // unsupported flag
            default:
                // we have a logical error,
                // this should never happen.
                // FIXME: add extended logging later
                return ST_ASSERTION_FAILED;


            }
        }
    }
    while (do_retry);

    if (retval > 0)
    {
        // in this case, retval is the number of sent bytes.
        // decrement number of unsent bytes
        unsent_len -= retval;
        // increment offset into buffer
        out_offset += retval;
    }
    if (unsent_len >0)
    {
        LOG_TX(LOG_TRACE_CAN_MESSAGES, "%18.6f : RX: send_pending(): %i bytes left to send",
               ethercanif::get_realtime(), unsent_len);
    }
    return ST_OK;
}

int SBuffer::numUnsentBytes() const
{
    return unsent_len;
}


SBuffer::E_SocketStatus SBuffer::decode_and_process(int sockfd, int gateway_id, I_ResponseHandler *rhandler)
{
    ssize_t rsize = 0;

    bool do_retry = false;
    do
    {
        do_retry = false;
        rsize = recv(sockfd, rbuf, MAX_STUFFED_MESSAGE_LENGTH, MSG_DONTWAIT | MSG_NOSIGNAL);
        int errcode = errno;

        // check and process errors
        if (rsize == 0)
        {
            // connection was closed because of socket failure
            return ST_NO_CONNECTION;
        }
        else if (rsize < 0)
        {
            switch (errcode)
            {
            case EWOULDBLOCK:
            case ECONNRESET:
            case ENOBUFS :
                // reading data would block, and the MSG_DONTWAIT
                // flag was set.
                return ST_OK;
                break;

            case EINTR:
                // a signal happened and interrupted the syscall
                // we just retry the send() call.
                do_retry = true;
                break;

            case ENOTCONN: // socket not connected
                fflush(stdout);
                return ST_NO_CONNECTION;

            case EINVAL: // invalid argument
            case EBADF: // bad file descriptor
            case ENOTSOCK: // descriptor is not a socket
            case EFAULT: // invalid user space address
            case EMSGSIZE : // message too large
            case ENOMEM: // no memory available
            default:
                // we have a logical error,
                // this should never happen.
                // FIXME: add extended logging later
                return ST_ASSERTION_FAILED;


            }
        }
    }
    while (do_retry);

    for (int i=0; i < rsize; i++)
    {
        bool frame_complete = decode_and_append_byte(command_buf.bytes,  clen, sync, dle,  rbuf[i]);

        if (frame_complete)
        {
            // send the received data to the response handler
            assert(rhandler != nullptr);
            rhandler->handleFrame(gateway_id, command_buf, clen);
        }

    }
    return ST_OK;
}



}

}
