
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

#ifndef RESET_STEP_COUNTER_COMMAND_H
#define RESET_STEP_COUNTER_COMMAND_H

#include <cassert>
#include "../CAN_Command.h"

namespace mpifps
{

namespace ethercanif
{

class ResetStepCounterCommand : public CAN_Command
{

public:

    static const E_CAN_COMMAND command_code = CCMD_RESET_STEPCOUNTER;

    static E_CAN_COMMAND getCommandCode()
    {
        return command_code;
    };

    ResetStepCounterCommand(): CAN_Command(command_code), alpha_steps(0), beta_steps(0)
    {
    };



    void parametrize(int f_id, bool broadcast, long _alpha_steps, long _beta_steps)
    {
        fpu_id = f_id;
        bcast = broadcast;
	alpha_steps = _alpha_steps & 0xffffff;
	beta_steps = _beta_steps & 0xffffff;
	    
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

    void SerializeToBuffer(const uint8_t busid,
                           const uint8_t fpu_canid,
                           int& buf_len,
                           t_CAN_buffer& can_buffer,
                           const uint8_t _sequence_number)
    {

        set_msg_header(can_buffer, buf_len, busid, fpu_canid, bcast, _sequence_number);


        can_buffer.message.data[2] = 0xff & alpha_steps;
        can_buffer.message.data[3] = 0xff & (alpha_steps >> 8);
        can_buffer.message.data[4] = 0xff & (alpha_steps >> 16);
        can_buffer.message.data[5] = 0xff & beta_steps;
        can_buffer.message.data[6] = 0xff & (beta_steps >> 8);
        can_buffer.message.data[7] = 0xff & (beta_steps >> 16);

        buf_len += 6;


    };


    private:
    long alpha_steps;
    long beta_steps;

};

}

}
#endif
