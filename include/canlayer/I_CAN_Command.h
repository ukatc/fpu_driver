
// -*- mode: c++ -*-
////////////////////////////////////////////////////////////////////////////////
// ESO - VLT Project
//
// Copyright 2017 E.S.O,
//
// Who       When        What
// --------  ----------  -------------------------------------------------------
// Pablo Gutierrez 2017-07-22  created CAN client sample
// jnix      2017-10-18  Created driver class using Pablo Guiterrez' CAN client sample
//------------------------------------------------------------------------------

////////////////////////////////////////////////////////////////////////////////
// NAME I_CAN_Command.h
//
// This class implements the low-level CAN driver for the MOONS fiber
// positioner grid
//
////////////////////////////////////////////////////////////////////////////////

#ifndef I_CAN_COMMAND_H
#define I_CAN_COMMAND_H

#include <endian.h>
#include <stdint.h>
#include <time.h>

#include "CAN_Constants.h"
#include "E_CAN_COMMAND.h"

namespace mpifps
{

namespace canlayer
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

class I_CAN_Command
{
public:


    I_CAN_Command() {};
    virtual ~I_CAN_Command() {};


    // method which serializes parameters into
    // CAN message
    virtual void SerializeToBuffer(const uint8_t busid,
                                   const uint8_t fpu_canid,
                                   int& buf_len, t_CAN_buffer& buf) = 0;


    virtual E_CAN_COMMAND getInstanceCommandCode() = 0;

    // FPU id to which message is sent
    virtual int getFPU_ID()=0;

    // boolean value indicating whether
    // the driver should wait for a response
    virtual bool expectsResponse() = 0;

    // time-out period for a response to the message
    virtual timespec getTimeOut() = 0;

    // if this is set, a response will be expected
    // from all FPUs which are not locked.
    virtual bool doBroadcast() = 0;

};

}

} // end of namespace

#endif
