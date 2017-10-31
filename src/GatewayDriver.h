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

#include "SBuffer.h"          // coding and decoding message frames
#include "ResponseHandler.h"  // interface for processing CAN responses

#include "FPUArrya.h" // defines thread-safe structure of FPU state info
    
namespace mpifps
{


typedef struct
{
    uint8 gateway_id;
    uint8 bus_id;
    uint8 can_id;
} t_bus_address;

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

    // timeout for reading from command FIFO
    const timespec MAX_TX_TIMEOUT = { .tv_sec = 0,


                                       .tv_nsec = 50000000 };

    const timespec MAX_RX_TIMEOUT = FPUArray::MAX_TIMEOUT;
    

    const double poll_timeout_sec = 5e-3;
    
    GatewayDriver(int num_fpus);
    ~GatewayDriver();

    void connect(const int ngateways, const t_gateway_address gateway_addresses[]);

    void disconnect();

    initializeGrid();

    resetFPUs();

    findDatum();

    configMotion();

    executeMotion();

    repeatMotion();

    reverseMotion();

    abortMotion();

    assignPositions();

    lockFPU();

    unlockFPU();


    getCurrentState();

    waitForState();


    // method which handles decoded CAN response messages
    virtual void handleFrame(uint8_t const * const  command_buffer, int const clen);





private:


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
        t_bus_address  address_map[MAX_NUM_POSITIONERS];
        // reverse map of addresses to FPU id.
        uint16 fpu_id_by_adr[MAX_NUM_GATEWAYS][BUSES_PER_GATEWAY][FPUS_PER_BUS];
        
        FPUArray fpuArray;

    CommandPool command_pool; // memory pool for unused command objects


}

} // end of namespace
