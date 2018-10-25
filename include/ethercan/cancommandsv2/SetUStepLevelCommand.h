
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
#include "../CAN_Command.h"
#include "../../InterfaceConstants.h"

namespace mpifps
{

namespace ethercanif
{

class SetUStepLevelCommand : public CAN_Command
{

public:

    static const E_CAN_COMMAND command_code = CCMD_SET_USTEP_LEVEL;

    static E_CAN_COMMAND getCommandCode()
    {
        return command_code;
    };

    SetUStepLevelCommand() : CAN_Command(command_code)
    {
        ustep_level = 1;
    };


    void parametrize(int f_id, bool _bcast, uint8_t ustep)
    {
        fpu_id = f_id;
        bcast = _bcast;
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


private:
    uint8_t ustep_level;


};

}

}
#endif
