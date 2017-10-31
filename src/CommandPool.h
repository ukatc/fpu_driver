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


#include <stdio.h>
#include <string.h>		/// strerror
#include <pthread.h>
//#include "stdlib.h"		/// system("/bin/stty raw");
//#include "stdbool.h"	/// bool
#include <unistd.h>
//#include <stdint.h>
#include <std>

    
namespace mpifps
{
    class CommandPool {
      public:
        // method which provides a new CAN command
        // instance for the given command type
        unique_ptr<I_CAN_Command> provideInstance(E_CAN_COMMAND);

        // method which recycles an instance which
        // is no longer needed into the memory pool so that it can
        // be used later without requiring a new
        // allocation.
        recycleInstance(unique_ptr<I_COMMAND>& cmd);
    }
}
