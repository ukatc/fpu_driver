
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

#ifndef READ_REGISTER_H
#define READ_REGISTER_H

#include <string.h>
#include <cassert>
#include "../CAN_Command.h"

namespace mpifps
{

namespace ethercanif
{

class ReadRegisterCommand : public CAN_Command
{

public:


    static const E_CAN_COMMAND command_code = CCMD_READ_REGISTER;
    
    static E_CAN_COMMAND getCommandCode()
    {
        return command_code;
    };
    
    ReadRegisterCommand(): CAN_Command(command_code)
    {
        bank = 0;
        address=0;
    };

    void parametrize(int f_id, bool broadcast, uint8_t _bank, uint8_t _address)
    {
        fpu_id = f_id;
        bcast = broadcast;
        bank = _bank;
        address = _address;
    };

    void SerializeToBuffer(const uint8_t busid,
                           const uint8_t fpu_canid,
                           int& buf_len,
                           t_CAN_buffer& can_buffer,
			   const uint8_t sequence_number)
    {

	set_msg_header(can_buffer, buf_len, busid, fpu_canid, bcast, sequence_number);

        can_buffer.message.data[2] = bank;
        can_buffer.message.data[3] = address;

        buf_len += 2;

    };



    // time-out period for a response to the message
    timespec getTimeOut()
    {
        timespec const toval =
        {
            /* .tv_sec = */ 20,
            /* .tv_nsec = */ 500000000
        };

        return toval;
    };


private:
    uint8_t bank;
    uint8_t address;

};

}

}
#endif
