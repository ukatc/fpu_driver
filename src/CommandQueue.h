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
#include <unistd.h>
//#include <stdint.h>
#include <std>


// this interface defines a thread-safe queue
// which holds commands before sending them
// to the different gateways
namespace mpifps
{

    class CommandQueue{
      public:
        CommandQueue();
        ~CommandQueue();

        void enqueueCommand(unique_ptr<Command>);

        unique_ptr<Command> dequeueCommand();

        bool is_empty();                
        
        
        
    }
}
