
// -*- mode: c++ -*-
////////////////////////////////////////////////////////////////////////////////
//
// Copyright 2017 UKRI. See file "LICENSE" for license information.
//

// --------  ----------  -------------------------------------------------------
// jnix      2017-10-18  Created driver class using Pablo Guiterrez' CAN client sample
//------------------------------------------------------------------------------

////////////////////////////////////////////////////////////////////////////////
// NAME WriteSerialNumberCommand.h
//
// This class implements the low-level CAN driver for the MOONS fiber
// positioner grid
//
////////////////////////////////////////////////////////////////////////////////

#ifndef WRITE_SERIAL_NUMBER_COMMAND_H
#define WRITE_SERIAL_NUMBER_COMMAND_H

#include <string.h>
#include <cassert>
#include "../CAN_Command.h"
#include "../../InterfaceConstants.h"

namespace mpifps
{

namespace ethercanif
{

class WriteSerialNumberCommand : public CAN_Command
{

public:


    static const E_CAN_COMMAND command_code = CCMD_WRITE_SERIAL_NUMBER;

    static E_CAN_COMMAND getCommandCode()
    {
        return command_code;
    };

    WriteSerialNumberCommand(): CAN_Command(command_code)
    {
        memset(serial_number, 0, sizeof(serial_number));
    };



    void parametrize(int f_id, char const new_serial_number[DIGITS_SERIAL_NUMBER + 1])
    {
        fpu_id = f_id;
        memcpy(serial_number, (char*) new_serial_number, sizeof(serial_number));
    };

    void SerializeToBuffer(const uint8_t busid,
                           const uint8_t fpu_canid,
                           int& buf_len,
                           t_CAN_buffer& can_buffer,
                           const uint8_t sequence_number)
    {
        set_msg_header(can_buffer, buf_len, busid, fpu_canid, bcast, sequence_number);

        for(int i=0; i < DIGITS_SERIAL_NUMBER; i++)
        {
            can_buffer.message.data[i+2] = serial_number[i];
        }

        buf_len += DIGITS_SERIAL_NUMBER;

    };



    // time-out period for a response to the message
    timespec getTimeOut()
    {
        timespec const toval =
        {
            /* .tv_sec = */ 15,
            /* .tv_nsec = */ 0
        };

        return toval;
    };

    bool doBroadcast()
    {
        return false;
    }

private:
    uint8_t serial_number[DIGITS_SERIAL_NUMBER];

};

}

}
#endif
