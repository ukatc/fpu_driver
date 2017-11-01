
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

namespace mpifps {

  const int MAX_CAN_MESSAGE_LENGTH_BYTES = 16;

class I_CAN_Command {
public:


    // method which serializes message for sending
    void SerializeToBuffer(int& buf_len, uint8_t * buf);


    // FPU id to which message is sent
    int getFPU_ID();

    // boolean value indicating whether
    // the driver should wait for a response
    bool expectsResponse();

    E_CAN_COMMAND getCommandCode();

    // time-out period for a response to the message
    timespec getTimeOut();
 
}

} // end of namespace
