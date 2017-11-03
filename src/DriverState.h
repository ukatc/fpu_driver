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

namespace mpifps
{

enum E_DriverState
{
    // the driver can have the following
    // non-operative states
    UNINITIALISED = 1,  // not yet initialized, or
                        // resource allocation failed,
                        // for example because out-of-memory

    
    UNCONNECTED  = 2,   // the driver is not connected,
                        // this is the state before connecting
                        // to the gateway or after the TCP
                        // connection was lost. The latter would
                        // happen if there is a serious extended
                        // failure, like a broken cable or
                        // a system error with the gateway service.

    CONNECTED  = 3,     // driver is connected to gateways
                        // and working
    

} ;


// This enum contains return codes
// which should shed light on the cause
// if something went seriously wrong.
enum E_DriverErrCode
{
    // everything worked
    OK = 0,

    
    // A command was tried to send, or the
    // driver was instructed to connect, but
    // the driver was not initialized properly.
    // That can happen if the system goes
    // out of memory, or if a logical error
    // affects the initialization.
    DRIVER_NOT_INITIALISED = 1,


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
    NO_CONNECTION = 2,


    // The user tried to send a high-level
    // command while another high-level
    // command was still not finished.
    STILL_BUSY = 3,
    
    // It was tried to send more CAN commands
    // at the same time than possible,
    // so that the internal command pool
    // became exhausted.
    // This should only happen with low-level
    // engineering commands.
    COMMAND_POOL_EXHAUSTED = 4,
    

} ;



}
