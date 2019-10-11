
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

#ifndef SYNC_COMMAND_H
#define SYNC_COMMAND_H

#include <string.h>
#include <cassert>
#include "../CAN_Command.h"
#include "../../InterfaceConstants.h"

namespace mpifps
{

namespace ethercanif
{

class SyncCommand : public CAN_Command
{

public:


    static const E_CAN_COMMAND command_code = CCMD_SYNC_COMMAND;

    static E_CAN_COMMAND getCommandCode()
    {
        return command_code;
    };

    SyncCommand() : CAN_Command(command_code),
		    sync_type(SYNC_ABORT_MOTION),
		    can_command_code(CCMD_ABORT_MOTION)
    {
    };



    void parametrize(E_SYNC_TYPE _sync_type)
    {
        fpu_id = 0; // always a broadcast command
        sync_type = _sync_type;
	switch (sync_type){

	case SYNC_ABORT_MOTION:
	    can_command_code = CCMD_ABORT_MOTION;
	    break;

	case SYNC_EXECUTE_MOTION:
	    can_command_code = CCMD_EXECUTE_MOTION;
	    break;

	default:
	    assert(false); break;
	}
    };

    void SerializeToBuffer(const uint8_t busid,
                           const uint8_t fpu_canid,
                           int& buf_len,
                           t_CAN_buffer& can_buffer,
                           const uint8_t _sequence_number)
    {

	// sequelch warnings about unused parameters
	assert(busid | 1);
	assert(fpu_canid | 1);

	sequence_number = _sequence_number;


        bzero(&can_buffer.message, sizeof(can_buffer.message));

	can_buffer.message.busid = GW_MSG_TYPE_SYNC;                // byte 0

	const uint16_t can_identifier = 0; // always canid zero, meaning 'broadcast'
        can_buffer.message.identifier = htole16(can_identifier); // bytes 1 and 2

        can_buffer.message.data[0] = (uint8_t) sync_type;        // byte 3

        buf_len = 4;

    };


    E_CAN_COMMAND getCANCommandCode()
    {
        assert(command_code != CCMD_NO_COMMAND);

	switch (sync_type){

	case SYNC_ABORT_MOTION:
	    return CCMD_ABORT_MOTION;
	    break;

	case SYNC_EXECUTE_MOTION:
	    return CCMD_EXECUTE_MOTION;
	break;

	default: assert(false);
	}
    };

    bool expectsResponse()
    {
        return true;
    };


    // allows to test whether this CAN_Command instance is a SYNC command
    bool doSync()
    {
        return true;
    }

    // Only for this class, the command_code (which identifies the
    // message sent) and the can_command_code (which identifies the
    // message type a response is expected for) are different.  This
    // is because the SYNC command causes the gateway to sent one of
    // two different CAN command to the FPUs. The time-out detection
    // mechanism has to account for those latter commands.



    // time-out period for a response to the message
    timespec getTimeOut()
    {

	if (sync_type == SYNC_EXECUTE_MOTION)
	{
	    timespec const toval =
		{
		    /* .tv_sec = */ 60,
		    /* .tv_nsec = */ 0
		};

	    return toval;
	}
	else
	{
	    timespec const toval =
		{
		    /* .tv_sec = */ 5,
		    /* .tv_nsec = */ 0
		};

	    return toval;
	}
    };


private:
    E_SYNC_TYPE sync_type;
    E_CAN_COMMAND can_command_code;


};

}

}
#endif
