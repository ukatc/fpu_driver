
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
#include "../../InterfaceConstants.h"

namespace mpifps
{

namespace ethercanif
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
                           t_CAN_buffer& can_buffer,
			   const uint8_t sequence_number)
    {

	set_msg_header(can_buffer, buf_len, busid, fpu_canid, bcast, sequence_number);

        can_buffer.message.data[2] = ustep_level;

        buf_len += 1;

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
