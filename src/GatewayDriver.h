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

namespace mpifps
{

// maximum length of CAN message (extended frame format)
// number of FPUs
const int BUSES_PER_GATEWAY =  5;
const int FPUS_PER_BUS = 67;

const int MAX_NUM_GATEWAYS = 3;

const int MAX_NUM_POSITIONERS = (MAX_NUM_GATEWAYS
                                 * BUSES_PER_GATEWAY
                                 * FPUS_PER_BUS);

typedef struct
{
    int gateway_id;
    int bus_id;
    int can_id;
    int alpha_steps;
    int beta_steps;
    bool is_initialized;
    bool on_alpha_datum;
    bool on_beta_datum;
    bool alpha_collision;
    bool at_alpha_limit;
    bool beta_collision;

    E_COMMAND pending_command;


} t_fpu_state;

typedef struct
{
    t_FPU_state FPU_states[MAX_NUM_POSITIONERS];
    int count_collision;
    int count_initialised;
    int count_ready;
    int count_moving;
    int count_error;
} t_grid_state;


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
    struct timeval const command_timeout = { .tv_sec = 0,.tv_usec = 50000 };


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







private:

    untangleFPU();

    clearCollision();

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



    // this mutex protects the FPU state array structure
    pthread_mutex_t grid_state_mutex;

    // this mutex ensures that only one command is sent at the same time
    pthread_mutex_t command_creation_mutex;

    // condition variables which is signaled on state changes
    pthread_cond_t cond_state_change;

    // buffer class for encoded reads and writes to sockets
    SBuffer sbuffer[MAX_NUM_GATEWAYS];

    t_grid_state FPUGridState;


    // method which handles decoded CAN response messages
    virtual void handleFrame(uint8_t const * const  command_buffer, int const clen);

}

} // end of namespace
