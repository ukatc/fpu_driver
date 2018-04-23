
// -*- mode: c++ -*-
////////////////////////////////////////////////////////////////////////////////
//
// Copyright 2017 UKRI. See file "LICENSE" for license information.
//

// --------  ----------  -------------------------------------------------------
// jnix      2017-10-18  Created driver class using Pablo Guiterrez' CAN client sample
//------------------------------------------------------------------------------

////////////////////////////////////////////////////////////////////////////////
// NAME SetUStepLevelCommand.h
//
// This class implements the low-level CAN driver for the MOONS fiber
// positioner grid
//
////////////////////////////////////////////////////////////////////////////////

#ifndef SET_USTEP_LEVEL_COMMAND_H
#define SET_USTEP_LEVEL_COMMAND_H

#include <string.h>
#include <cassert>
#include "../I_CAN_Command.h"
#include "../../DriverConstants.h"

namespace mpifps
{

namespace canlayer
{

class SetUStepLevelCommand : public I_CAN_Command
{

public:

    static E_CAN_COMMAND getCommandCode()
    {
        return CCMD_SET_USTEP_LEVEL;
    };

    SetUStepLevelCommand()
    {
        fpu_id = 0;
        ustep_level=1;
        broadcast = false;
    };

    E_CAN_COMMAND getInstanceCommandCode()
    {
        return getCommandCode();
    };


    void parametrize(int f_id, bool bcast, uint8_t ustep)
    {
        fpu_id = f_id;
        broadcast = bcast;
        switch(ustep)
        {
        case 1:
        case 2:
        case 4:
        case 8:
            break;
        default:
            assert(0);
        }
        ustep_level = ustep;
    };

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


        // the CAN identifier is either all zeros (for a broadcast
        // message) or bits 7 - 10 are the proiority and bits 0 -
        // 6 the CAN id of the FPU.
        const E_CAN_COMMAND cmd_code = getCommandCode();

        uint16_t can_identifier = (getMessagePriority(cmd_code)
                                   << 7) | fpu_canid;


        // The protocol uses little-endian encoding here
        // (the byte order used in the CANOpen protocol).
        can_buffer.message.identifier = htole64(can_identifier);
        buf_len = 3;


        // CAN command code
        can_buffer.message.data[0] = cmd_code;
        can_buffer.message.data[1] = ustep_level;

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
        timespec const toval =
        {
            /* .tv_sec = */ 10,
            /* .tv_nsec = */ 0
        };

        return toval;
    };

    bool doBroadcast()
    {
        return false;
    }

private:
    uint16_t fpu_id;
    bool broadcast;
    uint8_t ustep_level;


};

}

}
#endif
