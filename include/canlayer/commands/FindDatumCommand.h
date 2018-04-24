
// -*- mode: c++ -*-
////////////////////////////////////////////////////////////////////////////////
//
// Copyright 2017 UKRI. See file "LICENSE" for license information.
//
//
// Who       When        What
// --------  ----------  -------------------------------------------------------
// jnix      2017-10-18  Created driver class using Pablo Guiterrez' CAN client sample
//------------------------------------------------------------------------------

////////////////////////////////////////////////////////////////////////////////
// NAME MoveDatumOnCommand.h
//
// This class implements the low-level CAN driver for the MOONS fiber
// positioner grid
//
//////////////////////////////////////////////////////////////////////////
//////

#ifndef MOVE_DATUM_COMMAND_H
#define MOVE_DATUM_COMMAND_H

#include <string.h>
#include <cassert>
#include "../I_CAN_Command.h"

namespace mpifps
{

namespace canlayer
{

class FindDatumCommand : public I_CAN_Command
{

public:

    static E_CAN_COMMAND getCommandCode()
    {
        return CCMD_FIND_DATUM;
    };


    FindDatumCommand()
    {
        fpu_id = 0;
        bcast = false;
#if (CAN_PROTOCOL_VERSION > 1)
        _auto_datum = false;
        _clockwise_first = false;
#endif
    };

    ~FindDatumCommand() {};

    E_CAN_COMMAND getInstanceCommandCode()
    {
        return getCommandCode();
    };


#if (CAN_PROTOCOL_VERSION == 1)
    void parametrize(int f_id, bool broadcast, E_DATUM_SELECTION arm_selection)
    {
        fpu_id = f_id;
        bcast = broadcast;
        _arm_selection = arm_selection;
    };
#else
    void parametrize(int f_id, bool broadcast, bool auto_datum, bool clockwise_first, E_DATUM_SELECTION arm_selection)
    {
        fpu_id = f_id;
        bcast = broadcast;
        _auto_datum = auto_datum;
        _clockwise_first = clockwise_first;
        _arm_selection = arm_selection;
    };
#endif


    void SerializeToBuffer(const uint8_t busid,
                           const uint8_t fpu_canid,
                           int& buf_len,
                           t_CAN_buffer& can_buffer)
    {

        // zero buffer to make sure no spurious DLEs are sent
        bzero(&can_buffer.message, sizeof(can_buffer.message));
        // CAN bus id for that gateway to which message should go
        can_buffer.message.busid = busid;

        // we use bit 7 to 10 for the command code,
        // and bit 0 to 6 for the FPU bus id.
        assert(fpu_canid <= FPUS_PER_BUS);
        if (! bcast)
        {
            assert(fpu_canid > 0);
        }


        // the CAN identifier is either all zeros (for a broadcast
        // message) or bits 7 - 10 are the proiority and bits 0 -
        // 6 the CAN id of the FPU.
        const E_CAN_COMMAND cmd_code = getCommandCode();

        uint16_t can_identifier = 0;

        if (! bcast)
        {
            // priority evaluates to zero for protocol version 1
            can_identifier = (getMessagePriority(cmd_code)
                              << 7) | fpu_canid;
        }

        // The protocol uses little-endian encoding here
        // (the byte order used in the CANOpen protocol).
        can_buffer.message.identifier = htole64(can_identifier);
        buf_len = 3;


        // CAN command code
        can_buffer.message.data[0] = cmd_code;
#if (CAN_PROTOCOL_VERSION == 1)
        bool skip_alpha = false;
        bool skip_beta = false;
        switch (_arm_selection)
        {
        case DASEL_BOTH:
            break;
        case DASEL_ALPHA:
            skip_beta = true;
            break;
        case DASEL_BETA:
            skip_alpha = true;
            break;
        case DASEL_NONE:
            skip_alpha = true;
            skip_beta = true;
            break;

        default:
            // Invalid values have been filtered out by
            // AsyncDriver, so this can't happen.
            assert(0);
        }
        // this is defined so that an empty field (all-zero)
        // has the defeault behavoir implemented by the
        // current firmware, which datums both arms.
        //
        // Note that this is not necessarily safe if
        // one of the switches is broken - old firmware ignoring
        // the arm selection can break the FPU then.
        can_buffer.message.data[1] = ( (skip_alpha ? DATUM_SKIP_ALPHA : 0)
                                       | (skip_beta ? DATUM_SKIP_BETA : 0));

#endif

#if (CAN_PROTOCOL_VERSION > 1)
        can_buffer.message.data[2] = ( ( _auto_datum ? 1 : 0)
                                       | ((_clockwise_first ?  1 : 0) << 1));
#endif

        buf_len += 8;

    };


    // FPU id to which message is sent
    int getFPU_ID()
    {
        return fpu_id;
    };

    // boolean value indicating whether
    // the driver should wait for a response
    bool expectsResponse()
    {
        return true;
    };

    // time-out period for a response to the message
    timespec getTimeOut()
    {
        // Largest possible waiting time for a working datum
        // search is 35 seconds.
        const struct timespec  toval =
        {
            /* .tv_sec = */ 60,
            /* .tv_nsec = */ 0
        };

        return toval;
    };

    bool doBroadcast()
    {
        return bcast;
    }

private:
    uint16_t fpu_id;
    E_DATUM_SELECTION _arm_selection;

#if (CAN_PROTOCOL_VERSION > 1)
    bool _auto_datum;
    bool _clockwise_first;
#endif
    bool bcast;


};

}

}
#endif
