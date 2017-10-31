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
// NAME GridDriver.h
//
// This class implements the low-level CAN driver for the MOONS fiber
// positioner grid
//
////////////////////////////////////////////////////////////////////////////////

#include "GatewayDriver.hpp"

namespace mpifps
{


class GridDriver
{

    GridDriver()
        {
        }

    ~GridDriver()
        {
            // destroy command creation mutex
            pthread_mutex_destroy(&command_creation_mutex);
        }
    

private:

    untangleFPU();

    clearCollision();
    
    // this mutex watches that no new
    // command is initiated while a running
    // command waits for completion.
    pthread_mutex_t command_creation_mutex = PTHREAD_MUTEX_INITIALIZER;

    GatewayDriver gateway;
}

} // end of namespace
