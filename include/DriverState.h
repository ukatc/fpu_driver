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
// NAME DriverState.h
//
// This class implements the low-level CAN driver for the MOONS fiber
// positioner grid.
//
////////////////////////////////////////////////////////////////////////////////

#ifndef DRIVER_STATE_H
#define DRIVER_STATE_H

namespace mpifps
{

enum E_DriverState
{
    // the driver can have the following
    // non-operative states
    DS_UNINITIALIZED = 1,  // not yet initialized, or
    // resource allocation failed,
    // for example because out-of-memory


    DS_UNCONNECTED  = 2,   // the driver is not connected,
    // this is the state before connecting
    // to the gateway or after the TCP
    // connection was lost. The latter would
    // happen if there is a serious extended
    // failure, like a broken cable or
    // a system error with the gateway service.

    DS_CONNECTED  = 3,     // driver is connected to gateways
    // and working

    DS_ASSERTION_FAILED  = 4, // a fatal error occured,
    // such as out-of-memory during ppoll()


} ;


// This enum contains return codes
// which should shed light on the cause
// if something went seriously wrong.
enum E_DriverErrCode
{
    // everything worked
    DE_OK = 0,


    // A command was tried to send, or the
    // driver was instructed to connect, but
    // the driver was not initialized properly.
    // That can happen if the system goes
    // out of memory, or if a logical error
    // affects the initialization.
    DE_DRIVER_NOT_INITIALIZED = 1,


    // driver has already been initialised,
    // and another initialisation was tried.
    DE_DRIVER_ALREADY_INITIALIZED = 2,


    // A command was tried to send to the
    // FPUs but this was not possible
    // because the driver was or became
    // disconnected from a gateway. During operation,
    // this should only happen when the
    // socket connection breaks down for
    // an extended time, as the socket
    // protocol will try hard to do re-sends
    // for several minutes.
    // Before this error happens, one will probably see
    // time-outs on every single FPU command
    // to the corresponding gateways as they
    // all fail to respond.
    DE_NO_CONNECTION = 3,


    // The user tried to send a high-level
    // command while another high-level
    // command was still not finished.
    DE_STILL_BUSY = 4,


    // a collision occured, and the operation was aborted
    DE_NEW_COLLISION = 5,
    
    // the user tried to start a movement
    // command while at least one FPU was in collided
    // or aborted state - the command
    // was rejected because of that
    DE_UNRESOLVED_COLLISION = 6,

    // an FPU has not been initialised, so
    // it cannot be moved accurately and safely.
    DE_FPU_NOT_INITIALIZED = 7,

    // driver is already initialized
    DE_DRIVER_ALREADY_CONNECTED = 8,

    // driver is connected
    DE_DRIVER_STILL_CONNECTED = 9,

    // An assumption about the driver state
    // is not met, the driver is switched off
    // and the problem logged.

    // maximum retry count was exceeded for command
    DE_MAX_RETRIES_EXCEEDED = 10,

    // we tried to move FPUs but some addressed FPUs
    // still have invalid waveforms.
    DE_INVALID_WAVEFORM = 11,

    // a motion command was issued but no FPUs are allowed to move
    DE_NO_MOVABLE_FPUS = 12,

    // The user waited for a command completion using a time-out
    // value, and the state has not been reached yet
    DE_COMMAND_TIMEOUT = 13,

    // movement was aborted
    DE_ABORTED_STATE = 14,

    // some addressed FPUs are locked
    DE_FPUS_LOCKED = 15,

    // operation not implemented for this protocol version
    DE_UNIMPLEMENTED=98,
    
    DE_ASSERTION_FAILED=99,

};

// this is a one-bit parameter to several commands
enum E_REQUEST_DIRECTION
{
    REQD_ANTI_CLOCKWISE = 0,
    REQD_CLOCKWISE      = 1,
};


}
#endif
