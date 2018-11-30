
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

#ifndef SET_STEPS_PER_SEGMENT_COMMAND_H
#define SET_STEPS_PER_SEGMENT_COMMAND_H

#include <cassert>
#include "../CAN_Command.h"

namespace mpifps
{

namespace ethercanif
{

class SetStepsPerSegmentCommand : public CAN_Command
{

public:

    static const E_CAN_COMMAND command_code = CCMD_SET_STEPS_PER_SEGMENT;

    static E_CAN_COMMAND getCommandCode()
    {
        return command_code;
    };

    SetStepsPerSegmentCommand():
        CAN_Command(command_code),
        min_steps_per_segment(0),
        max_steps_per_segment(0)
    {
    };



    void parametrize(int f_id, unsigned int min_steps_ps, unsigned int max_steps_ps, bool broadcast)
    {
        fpu_id = f_id;
        bcast = broadcast;
        min_steps_per_segment = min_steps_ps;
        max_steps_per_segment = max_steps_ps;
    };

    void SerializeToBuffer(const uint8_t busid,
                           const uint8_t fpu_canid,
                           int& buf_len,
                           t_CAN_buffer& can_buffer,
                           const uint8_t _sequence_number)
    {
        set_msg_header(can_buffer, buf_len, busid, fpu_canid, bcast, _sequence_number);

        can_buffer.message.data[2] = min_steps_per_segment & 0xff;
        can_buffer.message.data[3] = (min_steps_per_segment >> 8) & 0xff;
        can_buffer.message.data[4] = max_steps_per_segment & 0xff;
        can_buffer.message.data[5] = (max_steps_per_segment >> 8) & 0xff;

        buf_len += 4;

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

private:
    unsigned int min_steps_per_segment;
    unsigned int max_steps_per_segment;


};

}

}
#endif
