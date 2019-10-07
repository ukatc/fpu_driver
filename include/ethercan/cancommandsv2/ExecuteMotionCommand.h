
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

#ifndef EXECUTE_MOTION_COMMAND_H
#define EXECUTE_MOTION_COMMAND_H

#include <string.h>
#include <cassert>
#include "../CAN_Command.h"

namespace mpifps
{

namespace ethercanif
{

class ExecuteMotionCommand : public CAN_Command
{

public:


    static const E_CAN_COMMAND command_code = CCMD_EXECUTE_MOTION;

    static const E_SYNC_TYPE sync_code = SYNC_EXECUTE_MOTION;

    static E_CAN_COMMAND getCommandCode()
    {
        return command_code;
    };
    ExecuteMotionCommand() : CAN_Command(command_code)
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
            /* currently, the time-out is for the whole movement, not
               only the confirmation.  That's a bit simple but will
               work mostly.  You'll need to increase this time value
               if you want to configure a longer waveform segment
               duration.*/
            /* .tv_sec = */ 60,
            /* .tv_nsec = */ 0
        };

        return toval;
    };


};

}

}
#endif
