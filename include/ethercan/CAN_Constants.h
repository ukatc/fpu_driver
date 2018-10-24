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
// NAME DriverConstants.h
//
// This file defines global constants which reflect some
// properties of the hardware.
//
////////////////////////////////////////////////////////////////////////////////

#ifndef CAN_CONSTANTS_H
#define CAN_CONSTANTS_H

namespace mpifps
{

// this namespace contains definitions
// which are internal to the CAN bus access layer.


namespace ethercanif
{

// number of can buses on one gateway
const int BUSES_PER_GATEWAY =  5;
// number of FPUs on one CAN bus
const int FPUS_PER_BUS = 76;


// maximum number of elementary commands resulting from one
// high-level command
const int MAX_SUB_COMMANDS = 300;

// bytes of CAN payload
const int MAX_CAN_PAYLOAD_BYTES = 8;

// length of unstuffed message to gateway,
// including 11-bit CAN identifier and
// 8-bit bus identifier.
const int MAX_UNENCODED_GATEWAY_MESSAGE_BYTES = 11;

// length of serial number stored in FPU NVRAM
const int DIGITS_SERIAL_NUMBER=6;


}

}
#endif
