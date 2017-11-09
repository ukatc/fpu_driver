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
// NAME GatewayDriver.h
//
// This class implements the low-level CAN driver for the MOONS fiber
// positioner grid
//
////////////////////////////////////////////////////////////////////////////////

#ifndef GATEWAY_DRIVER_H
#define GATEWAY_DRIVER_H

#include <stdio.h>
#include <string.h>		/// strerror
#include <pthread.h>
//#include "stdlib.h"		/// system("/bin/stty raw");
//#include "stdbool.h"	/// bool
#include <unistd.h>
//#include <stdint.h>

#include <atomic>

#include "E_GridState.h"
#include "SBuffer.h"          // coding and decoding message frames
#include "I_ResponseHandler.h"  // interface for processing CAN responses

#include "FPUArray.h" // defines thread-safe structure of FPU state info
#include "TimeOutList.h"
#include "CommandQueue.h"
#include "CommandPool.h"

namespace mpifps
{



typedef struct
{
    char * ip;
    uint16_t port;
} t_gateway_address;

void* threadTxFun(void *arg);
void* threadRxFun(void *arg);


class GatewayDriver: private ResponseHandler
{
public:

    // timeout for reading from command FIFO if nothing is
    // pending - 50 ms
    const timespec COMMAND_WAIT_TIME = { .tv_sec = 0,
                                         .tv_nsec = 50000000 };
    // timeout for polling write socket - 5 ms
    const timespec MAX_TX_TIMEOUT = { .tv_sec = 0,
                                      .tv_nsec = 5000000 };

    // default timeout for polling read socket - 5 sec
    // (this max times determines the time it takes to
    // shut down the conenction if nothing happens).
    const timespec MAX_RX_TIMEOUT = FPUArray::MAX_TIMEOUT;

        
    GatewayDriver(int num_fpus);
    ~GatewayDriver();

    E_DriverErrCode initialize();

    E_DriverErrCode connect(const int ngateways, const t_gateway_address gateway_addresses[]);

    // disconnect socket, and re-add any pending commands to
    // the command queue.
    E_DriverErrCode disconnect();

    // get the current state of the FPU grid, which
    // is stored in the reference parameter
    void getGridState(t_grid_state& out_state);

    // get both the summed up state of the FPU grid,
    // and a detailed status for each FPU.
    E_GridState waitForState(E_WaitTarget target, t_grid_state& out_detailed_state);

                     

    // provide a command instance with buffer space for
    // sending CAN parameters. This method is thread-safe
    unique_ptr<I_CAN_Command> provideInstance(E_CAN_COMMAND cmd_type);

    // send a CAN command to the gateway.
    // This method is thread-safe
    E_QUEUE_STATE sendCommand(int gateway_id, unique_ptr<I_CAN_Command> new_command);
    


private:

    // interface method which handles decoded CAN response messages
    virtual void handleFrame(int const gateway_id, uint8_t const * const  command_buffer, int const clen);


    int num_gateways = 0;

    // socket descriptor
    int SocketID[MAX_NUM_GATEWAYS];

    // read buffer (only to be accessed in reading thread)
    // write buffer (only to be accessed in writing thread)

    // two POSIX threads for sending and receiving data
    pthread_t    send_thread;
    pthread_t    receive_thread;
    // this atomic flag serves to signal both threads to exit
    std::atomic<bool> exit_threads = false;




    // this mutex ensures that only one command is sent at the same time
    pthread_mutex_t command_creation_mutex;

    // buffer class for encoded reads and writes to sockets
    SBuffer sbuffer[MAX_NUM_GATEWAYS];

    // mapping of FPU IDs to physical addresses.
    // (can be made configurable if required)
    t_address_map  address_map;

    // reverse map of addresses to FPU id.    
    t_address_map fpu_id_by_adr; // address map from fpu id to can bus addresses
        
    FPUArray fpuArray;        // member which stores the state of the grid
    
    TimeOutList timeOutList; // list of pending time-outs 

    CommandPool command_pool; // memory pool for unused command objects

    CommandQueue commandQueue();


};

} // end of namespace

#endif
