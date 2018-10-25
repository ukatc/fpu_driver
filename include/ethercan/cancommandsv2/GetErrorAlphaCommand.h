
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

#ifndef GET_ERROR_ALPHA_H
#define GET_ERROR_ALPHA_H

#include <string.h>
#include <cassert>
#include "../CAN_Command.h"

namespace mpifps
{

namespace ethercanif
{

class GetErrorAlphaCommand : public CAN_Command
{

public:

    static const E_CAN_COMMAND command_code = CCMD_GET_COUNTER_DEVIATION;

    static E_CAN_COMMAND getCommandCode()
    {
        return command_code;
    };

    GetErrorAlphaCommand() : CAN_Command(command_code)
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
            /* .tv_nsec = */ 0
        };

        return toval;
    };



};

}

}
#endif
