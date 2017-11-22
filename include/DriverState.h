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
    DS_UNINITIALISED = 1,  // not yet initialized, or
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
    DE_DRIVER_NOT_INITIALISED = 1,


    DE_DRIVER_ALREADY_INITIALISED = 2,


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

    // the user tried to start a movement
    // command while at least one FPU was in collided
    // or aborted state - the command
    // was rejected because of that

    DE_UNRESOLVED_COLLISION = 5,

    // an FPU has not been initialised, so
    // it cannot be moved accurately and safely.
    DE_NOT_INITIALISED = 6,

    // An assumption about the driver state
    // is not met, the driver is switched off
    // and the problem logged.

    DE_ASSERTION_FAILED=99,

} ;



}
#endif
