
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
// NAME FPU_CAN_driver.h
// 
// This class implements the low-level CAN driver for the MOONS fiber
// positioner grid
//
////////////////////////////////////////////////////////////////////////////////

#include <endian.h>

namespace mpifps
{

const int MAX_CAN_MESSAGE_LENGTH_BYTES = 16;

typedef union __attribute__((packed))
{
    struct 
    {
        uint8_t busid;
        uint16_t identifier; // little-endian
        uint8_t data[8];
    } msg;    
    uint8_t bytes[11];
} t_CAN_buffer;

class I_CAN_Command
{
public:

    virtual I_Can_Command(){};

    // method which serializes parameters into
    // CAN message
    virtual void SerializeToBuffer(const uint8_t busid,
                                   const uint16_t fpuid,
                                   int& buf_len, t_CAN_buffer& buf);


    // FPU id to which message is sent
    virtual int getFPU_ID();

    // boolean value indicating whether
    // the driver should wait for a response
    virtual bool expectsResponse();

    virtual E_CAN_COMMAND getCommandCode();

    // time-out period for a response to the message
    virtual timespec getTimeOut();
 
}

} // end of namespace
