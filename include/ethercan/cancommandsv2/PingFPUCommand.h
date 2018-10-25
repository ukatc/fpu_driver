
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
// NAME PingCommand.h
//
// This class implements the low-level CAN driver for the MOONS fiber
// positioner grid
//
////////////////////////////////////////////////////////////////////////////////

#ifndef PING_FPU_COMMAND_H
#define PING_FPU_COMMAND_H

#include <cassert>
#include "../CAN_Command.h"

namespace mpifps
{

namespace ethercanif
{

class PingFPUCommand : public CAN_Command
{

public:

    static const E_CAN_COMMAND command_code = CCMD_PING_FPU;

    static E_CAN_COMMAND getCommandCode()
    {
        return command_code;
    };

    PingFPUCommand(): CAN_Command(command_code)
    {
    };



    void parametrize(int f_id, bool broadcast)
    {
        fpu_id = f_id;
        bcast = broadcast;
    };

    // time-out period for a response to the message
    timespec getTimeOut()
    {
        timespec const toval =
        {
            /* .tv_sec = */ 1,
            /* .tv_nsec = */ 000000000
        };

        return toval;
    };



};

}

}
#endif
