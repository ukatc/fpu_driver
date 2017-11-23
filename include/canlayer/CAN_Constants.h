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


namespace canlayer
{

// number of can buses on one gateway
const int BUSES_PER_GATEWAY =  5;
// number of FPUs on one CAN bus
const int FPUS_PER_BUS = 67;


// maximum number of elementary commands resulting from one
// high-level command
const int MAX_SUB_COMMANDS = 300;

// bytes of CAN payload
const int MAX_CAN_PAYLOAD_BYTES = 8;

// length of unstuffed message to gateway,
// including 11-bit CAN identifier and
// 8-bit bus identifier.
const int MAX_UNENCODED_GATEWAY_MESSAGE_BYTES = 11;


}

}
#endif
