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
// NAME CAN_Constants.h
//
// This file defines global constants which reflect some
// properties of the hardware.
//
////////////////////////////////////////////////////////////////////////////////

#ifndef CAN_CONSTANTS_H
#define CAN_CONSTANTS_H

#include <inttypes.h>

namespace mpifps
{

// This namespace contains definitions which are internal to the CAN bus access
// layer.

namespace ethercanif
{

// Number of can buses on one gateway (FPU grid layout)
const int BUSES_PER_GATEWAY = 5;

// Maximum number of can buses on one gateway (ethercan hardware layout)
const int MAX_BUSES_PER_GATEWAY = 6;
// Number of FPUs on one CAN bus
const int FPUS_PER_BUS = 76;

// Maximum number of elementary commands resulting from one high-level command
const int MAX_SUB_COMMANDS = 300;

// Bytes of CAN payload
const int MAX_CAN_PAYLOAD_BYTES = 8;

// Length of unstuffed message to gateway, including 11-bit CAN identifier and
// 8-bit bus identifier
const int MAX_UNENCODED_GATEWAY_MESSAGE_BYTES = 11;

// Length of serial number stored in FPU NVRAM
const int DIGITS_SERIAL_NUMBER = 6;

// Pseudo bus addresses for messages to EtherCAN gateway
const uint8_t GW_MSG_TYPE_DELY = 0x06;   /// Gateway delay message
const uint8_t GW_MSG_TYPE_SYNC = 0x07;   /// Send Sync Message 0 or 1
const uint8_t GW_MSG_TYPE_COB0 = 0x08;   /// Sync Message 0 : CAN-OBJECT
const uint8_t GW_MSG_TYPE_COB1 = 0x09;   /// Sync Message 1 : CAN-OBJECT
const uint8_t GW_MSG_TYPE_MSK0 = 0x0A;   /// Sync Message 0 : CHANNEL MASK
const uint8_t GW_MSG_TYPE_MSK1 = 0x0B;   /// Sync Message 1 : CHANNEL MASK

// SYNC commands always use a sequence number of 1
const uint8_t SYNC_SEQUENCE_NUMBER = 1;
}

}
#endif
