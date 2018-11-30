
// -*- mode: c++ -*-
////////////////////////////////////////////////////////////////////////////////
//
// Copyright 2017 UKRI. See file "LICENSE" for license information.
//
// Who       When        What
// --------  ----------  -------------------------------------------------------
// jnix      2017-10-18  Created driver class using Pablo Guiterrez' CAN client sample
//------------------------------------------------------------------------------

////////////////////////////////////////////////////////////////////////////////
// NAME I_CAN_Command.h
//
// This class implements the low-level CAN driver for the MOONS fiber
// positioner grid
//
////////////////////////////////////////////////////////////////////////////////

#ifndef CAN_COMMAND_H
#define CAN_COMMAND_H

#include <string.h>
#include <endian.h>
#include <time.h>

#ifdef DEBUG
#include <stdio.h>
#endif

#include "CAN_Constants.h"
#include "E_CAN_COMMAND.h"

namespace mpifps
{

namespace ethercanif
{



// buffer which can hold a CAN payload (8 bytes)
typedef uint8_t t_response_buf[MAX_CAN_PAYLOAD_BYTES];

// unstuffed message to gateway which holds address,
// CAN identifier, and message
typedef struct __attribute__((packed)) t_msg
{
    uint8_t busid;
    uint16_t identifier; // little-endian
    t_response_buf data;
} t_msg;

// buffer which holds the unencoded (not byte-stuffed)
// message to the EtherCAN gateway
typedef union   __attribute__((packed))
{
    struct t_msg message;
    uint8_t bytes[MAX_UNENCODED_GATEWAY_MESSAGE_BYTES];
} t_CAN_buffer;

class CAN_Command
{
private:
    const E_CAN_COMMAND command_code = CCMD_NO_COMMAND;

public:

    static const int CMD_CODE_MASK = 0x1F;



    CAN_Command(E_CAN_COMMAND _command_code) : command_code(_command_code), fpu_id(-1), bcast(false), sequence_number(0) {};

    virtual ~CAN_Command() {};



    E_CAN_COMMAND getInstanceCommandCode()
    {
        assert(command_code != CCMD_NO_COMMAND);
        return command_code;
    };


    // method which serializes parameters into
    // byte array which contains CAN message
    virtual void SerializeToBuffer(const uint8_t busid,
                                   const uint8_t fpu_canid,
                                   int& buf_len,
                                   t_CAN_buffer& can_buffer,
                                   const uint8_t _sequence_number)
    {
        set_msg_header(can_buffer, buf_len, busid, fpu_canid, bcast, _sequence_number);
    };



    // FPU id to which message is sent; valid after instance was parametrized
    int getFPU_ID()
    {
        return fpu_id;
    };


    // boolean value indicating whether
    // the driver should wait for a response
    virtual bool expectsResponse()
    {
        return true;
    };

    // time-out period for a response to the message
    virtual timespec getTimeOut() = 0;

    // if this is set, a response will be expected
    // from all FPUs which are not locked.
    virtual bool doBroadcast()
    {
        return bcast;
    }

    uint8_t getSequenceNumber()
    {
        return sequence_number;
    }



    // functions which populates CAN message header, and saves the
    // sequence number in a member variable
    void set_msg_header(t_CAN_buffer& can_buffer, int& buflen,
                        const uint8_t busid, const uint8_t fpu_canid,
                        const bool _bcast, const uint8_t _sequence_number)
    {
        // zero buffer to make sure no spurious DLEs are sent
        bzero(&can_buffer.message, sizeof(can_buffer.message));
        // CAN bus id for that gateway to which message should go
        can_buffer.message.busid = busid;

        // we use bit 7 to 10 for the command code,
        // and bit 0 to 6 for the FPU bus id.
        assert(fpu_canid <= FPUS_PER_BUS);
	bcast = _bcast;
        if (! bcast)
        {
            assert(fpu_canid > 0);
        }


        // the CAN identifier is either all zeros (for a broadcast
        // message) or bits 7 - 10 are the proiority and bits 0 -
        // 6 the CAN id of the FPU.
        const E_CAN_COMMAND cmd_code = getInstanceCommandCode();

        uint16_t can_identifier = 0;

        if (! bcast)
        {
            can_identifier = (getMessagePriority(cmd_code)
                              << 7) | fpu_canid;
        }

        // The protocol uses little-endian encoding here
        // (the byte order used in the CANOpen protocol).
        //can_buffer.message.identifier = htole64(can_identifier);
        can_buffer.message.identifier = htole16(can_identifier);

        sequence_number = _sequence_number;
        can_buffer.message.data[0] = sequence_number;
        // CAN command code
        can_buffer.message.data[1] = cmd_code & CMD_CODE_MASK;

#if 0
	fprintf(stderr, "setting header: busid = %i, canid = %i, priority = %i, bcast=%i, command code = %i, seqno =%i\n",
		busid, fpu_canid, getMessagePriority(cmd_code), bcast ? 1 : 0, cmd_code, sequence_number);
#endif
	
        buflen = 5; // 3 bytes header, 2 bytes payload
    }



protected:

    uint16_t fpu_id;
    bool bcast;
    uint8_t sequence_number;


};

}

} // end of namespace

#endif
