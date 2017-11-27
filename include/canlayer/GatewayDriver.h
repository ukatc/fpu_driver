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



#include <atomic>

#include "../E_GridState.h"
#include "../T_GatewayAddress.h"

#include "SBuffer.h"          // coding and decoding message frames
#include "I_ResponseHandler.h"  // interface for processing CAN responses
#include "FPUArray.h" // defines thread-safe structure of FPU state info
#include "TimeOutList.h"
#include "CommandQueue.h"
#include "CommandPool.h"

namespace mpifps
{

namespace canlayer
{


void* threadTxFun(void *arg);
void* threadRxFun(void *arg);


class GatewayDriver: private I_ResponseHandler
{
public:

    // timeout for reading from command FIFO if nothing is
    // pending - 50 ms
    const struct timespec COMMAND_WAIT_TIME = { /* .tv_sec = */ 0,
                                         /* .tv_nsec = */ 50000000
                                       };
    // timeout for polling write socket - 5 ms
    const struct timespec MAX_TX_TIMEOUT = { /* .tv_sec = */ 0,
                                      /* .tv_nsec = */ 50000000
                                    };

    // default timeout for polling read socket - 0.1 sec
    // (this max times determines the time it takes to
    // shut down the conenction if nothing happens).
    const timespec MAX_RX_TIMEOUT = { /* .tv_sec = */ 0,
                                      /* .tv_nsec = */ 100000000
                                    };


    GatewayDriver(int num_fpus);
    ~GatewayDriver();

    E_DriverErrCode initialize();

    E_DriverErrCode connect(const int ngateways, const t_gateway_address gateway_addresses[]);

    // disconnect socket, and re-add any pending commands to
    // the command queue. (If pending commands should be
    // discarded, the command queue needs to be explicitly
    // flushed).
    E_DriverErrCode disconnect();

    // get the current state of the FPU grid, which
    // is stored in the reference parameter
    E_GridState getGridState(t_grid_state& out_state);

    // get the current state of the driver (this is a convenience
    // function, the state is contained in the grid state).
    E_DriverState getDriverState();

    // get both the summed up state of the FPU grid,
    // and a detailed status for each FPU.
    E_GridState waitForState(E_WaitTarget target, t_grid_state& out_detailed_state);



    // provide a command instance with buffer space for
    // sending CAN parameters. This method is thread-safe

    template <typename T>
    inline unique_ptr<T> provideInstance()
    {
        return command_pool.provideInstance<T>();
    }

    // send a CAN command to the gateway.
    // This method is thread-safe
    CommandQueue::E_QueueState sendCommand(int fpu_id, unique_ptr<I_CAN_Command> new_command);

    CommandQueue::E_QueueState broadcastCommand(const int gateway_id, unique_ptr<I_CAN_Command> new_command);

    // returns gateway ID for an FPU
    int getGatewayIdByFPUID(int fpu_id);

    // returns whether an FPU is currently marked as locked.
    bool isLocked(int fpu_id);

    // get number of unsent commands
    int getNumUnsentCommands();


    // the following two methods are actually internal -
    // they need to be visible in a non-member function.
    void* threadTxFun();
    void* threadRxFun();


private:


    int num_gateways = 0;
    // socket descriptor
    int SocketID[MAX_NUM_GATEWAYS];
    CommandQueue commandQueue;


    // send a buffer (either pending data or new command)
    SBuffer::E_SocketStatus send_buffer(unique_ptr<I_CAN_Command> &active_can_command,
                                        int gateway_id);

    // interface method which handles decoded CAN response messages
    virtual void handleFrame(int const gateway_id, uint8_t const command_buffer[MAX_UNENCODED_GATEWAY_MESSAGE_BYTES], int const clen);


    void updatePendingCommand(int fpu_id,
                              std::unique_ptr<I_CAN_Command>& can_command);



    // read buffer (only to be accessed in reading thread)
    // write buffer (only to be accessed in writing thread)

    // two POSIX threads for sending and receiving data
    pthread_t    tx_thread;
    pthread_t    rx_thread;
    // this atomic flag serves to signal both threads to exit
    std::atomic<bool> exit_threads;




    // this mutex ensures that only one command is sent at the same time
    pthread_mutex_t command_creation_mutex;

    // buffer class for encoded reads and writes to sockets
    SBuffer sbuffer[MAX_NUM_GATEWAYS];

    // mapping of FPU IDs to physical addresses.
    // (can be made configurable if required)
    FPUArray::t_bus_address_map  address_map;

    // reverse map of addresses to FPU id.
    FPUArray::t_address_map fpu_id_by_adr; // address map from fpu id to can bus addresses

    int num_fpus;

    FPUArray fpuArray;        // member which stores the state of the grid

    TimeOutList timeOutList; // list of pending time-outs

    CommandPool command_pool; // memory pool for unused command objects


    std::atomic<int> num_commands_being_sent;
    

};

}

} // end of namespace

#endif
