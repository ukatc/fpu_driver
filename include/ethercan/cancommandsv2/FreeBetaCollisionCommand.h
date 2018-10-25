
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
// NAME ExecuteMotionCommand.h
//
// This class implements the low-level CAN driver for the MOONS fiber
// positioner grid
//
////////////////////////////////////////////////////////////////////////////////

#ifndef FREE_BETA_COLLISION_COMMAND_H
#define FREE_BETA_COLLISION_COMMAND_H

#include <string.h>
#include <cassert>
#include "../CAN_Command.h"
#include "../../InterfaceConstants.h"

namespace mpifps
{

namespace ethercanif
{

class FreeBetaCollisionCommand : public CAN_Command
{

public:


    static const E_CAN_COMMAND command_code = CCMD_FREE_BETA_COLLISION;

    static E_CAN_COMMAND getCommandCode()
    {
        return command_code;
    };

    FreeBetaCollisionCommand() : CAN_Command(command_code),
        request_direction(REQD_ANTI_CLOCKWISE)
    {
    };



    void parametrize(int f_id, E_REQUEST_DIRECTION request_dir)
    {
        fpu_id = f_id;
        request_direction = request_dir;
    };

    void SerializeToBuffer(const uint8_t busid,
                           const uint8_t fpu_canid,
                           int& buf_len,
                           t_CAN_buffer& can_buffer,
                           const uint8_t sequence_number)
    {

        set_msg_header(can_buffer, buf_len, busid, fpu_canid, bcast, sequence_number);

        can_buffer.message.data[2] = (request_direction == REQD_CLOCKWISE) ? 1 : 0;

        buf_len += 1;

    };


    // time-out period for a response to the message
    timespec getTimeOut()
    {
        timespec const toval =
        {
            /* .tv_sec = */ 5,
            /* .tv_nsec = */ 0
        };

        return toval;
    };


private:
    E_REQUEST_DIRECTION request_direction;


};

}

}
#endif
