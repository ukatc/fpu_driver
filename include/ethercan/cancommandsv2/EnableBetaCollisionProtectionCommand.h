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

#ifndef ENABLE_BETA_COLLISION_PROTECTION_COMMAND_H
#define ENABLE_BETA_COLLISION_PROTECTION_COMMAND_H
#include <string.h>

#include <cassert>
#include "../I_CAN_Command.h"

namespace mpifps
{

namespace ethercanif
{

class EnableBetaCollisionProtectionCommand : public I_CAN_Command
{

public:

    static E_CAN_COMMAND getCommandCode()
    {
        return CCMD_ENABLE_BETA_COLLISION_PROTECTION;
    };

    EnableBetaCollisionProtectionCommand()
    {
        fpu_id = false;
        bcast = false;
    };

    E_CAN_COMMAND getInstanceCommandCode()
    {
        return getCommandCode();
    };


    void parametrize(int f_id, bool broadcast)
    {
        fpu_id = f_id;
        bcast = broadcast;
    };

    void SerializeToBuffer(const uint8_t busid,
                           const uint8_t fpu_canid,
                           int& buf_len,
                           t_CAN_buffer& can_buffer,
			   const uint8_t sequence_number)
    {

	set_msg_header(can_buffer, buf_len, busid, fpu_canid, bcast, sequence_number);

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
#if (CAN_PROTOCOL_VERSION == 1)
        return false;
#else
        return true;
#endif
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

    bool doBroadcast()
    {
        return bcast;
    }

private:
    uint16_t fpu_id;
    bool bcast;


};

}

}
#endif
