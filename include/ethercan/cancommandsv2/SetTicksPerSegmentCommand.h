
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

#ifndef SET_TICKS_PER_SEGMENT_COMMAND_H
#define SET_TICKS_PER_SEGMENT_COMMAND_H

#include <cassert>
#include "../CAN_Command.h"

namespace mpifps
{

namespace ethercanif
{

class SetTicksPerSegmentCommand : public CAN_Command
{

public:

    static const E_CAN_COMMAND command_code = CCMD_SET_TICKS_PER_SEGMENT;

    static E_CAN_COMMAND getCommandCode()
    {
        return command_code;
    };

    SetTicksPerSegmentCommand():
	CAN_Command(command_code),
	ticks_per_segment(0)
    {
    };



    void parametrize(int f_id, unsigned int ticks_per_seg, bool broadcast)
    {
        fpu_id = f_id;
        bcast = broadcast;
	// unit is 100 nanosecond clock cycles
	ticks_per_segment = ticks_per_seg;
    };

    void SerializeToBuffer(const uint8_t busid,
			   const uint8_t fpu_canid,
			   int& buf_len,
			   t_CAN_buffer& can_buffer,
			   const uint8_t sequence_number)
    {
        set_msg_header(can_buffer, buf_len, busid, fpu_canid, bcast, sequence_number);

	can_buffer.message.data[2] = ticks_per_segment & 0xff;
	can_buffer.message.data[3] = (ticks_per_segment >> 8) & 0xff;
	can_buffer.message.data[4] = (ticks_per_segment >> 16) & 0xff;

        buf_len += 3;

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
    static_assert(sizeof(unsigned int) >= 3);
    unsigned int ticks_per_segment;


};

}

}
#endif
