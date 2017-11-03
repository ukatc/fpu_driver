////////////////////////////////////////////////////////////////////////////////
// ESO - VLT Project
//
// Copyright 2017 E.S.O,
//
// Who       When        What
// --------  ----------  -------------------------------------------------------
// Pablo Gutierrez 2017-07-22  created CAN client sample
// jnix      2017-10-18  Created driver class using Pablo Guiterrez' CAN client samplepn


//------------------------------------------------------------------------------

////////////////////////////////////////////////////////////////////////////////
// NAME
//
// This class implements the low-level CAN driver for the MOONS fiber
// positioner grid
//
////////////////////////////////////////////////////////////////////////////////

#include <poll.h>
#include <signal.h>
#include "time_utils.h"

namespace mpifps
{


GatewayDriver::GatewayDriver(int num_fpus)
{




    // assing default mapping and reverse mapping to
    // logical FPU ids.
    for (int i=0; i < MAX_NUM_POSITIONERS; i++)
    {

        t_bus_address bus_adr;
        bus_adr.gateway_id = (uint8) (i / (FPUS_PER_BUS * BUSES_PER_GATEWAY));
        bus_adr.bus_id =  (uint8) (i % BUSES_PER_GATEWAY);
        bus_adr.can_id = (uint8)(i % (BUSES_PER_GATEWAY * FPUS_PER_BUS));

        address_map[i] = bus_adr;
        
        fpu_id_by_adr[bus_adr.gateway_id][bus_adr.bus_id][bus_adr.can_id] = (uint16) i;

    }

    memset(ReadBuffer, 0, sizeof(ReadBuffer));
    memset(WriteBuffer, 0, sizeof(WriteBuffer));


};

GatewayDriver::~GatewayDriver()
{


};

int make_socket(const char *ip, uint16_t port)
{
    int sck;
    struct sockaddr_in addr;

    long value = 1;

    addr.sin_family = AF_INET;
    addr.sin_port = htons(port);
    addr.sin_addr.s_addr = inet_addr(ip);

    if (addr.sin_addr.s_addr == INADDR_NONE)
    {
        fprintf(stderr, "invalid ip address\n");
        return -1;
    }

    sck = socket (PF_INET, SOCK_STREAM, 0);

    if (sck < 0)
    {
        perror("socket");
        return -1;
    }

    if (connect(sck, (struct sockaddr *) &addr, sizeof(addr)) < 0)
    {
        perror("connect");
        close(sck);
        return -1;
    }

    // disable Nagle algorithm meaning that
    // segments of any size will be sent
    // without waiting.
    setsockopt(sck, IPPROTO_TCP, TCP_NODELAY, &value,
               sizeof(value));

    return sck;
}


void GatewayDriver::connect(const int ngateways,
                            const t_gateway_address gateway_addresses[])
{

    ASSERT(ngateways <= MAX_NUM_GATEWAYS);
    ASSERT(ngateways >= 0);


    for (i = 0; i < ngateways; i++)
    {
        ip = gateway_addresses[i].ip;
        port = gateway_addresses[i].port;

        SocketID[i] = make_socket(ip, port);

    }

    pthread_attr_t attr;
    /* Initialize and set thread joinable attribute */
    pthread_attr_init(&attr);
    pthread_attr_setdetachstate(&attr, PTHREAD_CREATE_JOINABLE);


    // we create only one thread for reading and writing.
    // FIXME: set error and driver state on success of
    // connection.
    err = pthread_create(&rx_thread, &attr, &threadRxFun,
                         (void *) self);
    if (err != 0)  printf("\ncan't create thread :[%s]",
                              strerror(err));

    err = pthread_create(&tx_thread, &attr, &threadTxFun,
                         (void *) self);
    if (err != 0)  printf("\ncan't create thread :[%s]",
                              strerror(err));

    pthread_attr_destroy(&attr);


    fpuArray.setDriverState(CONNECTED);

    
};

void GatewayDriver::disconnect()
{
    
    E_DRIVER_STATE dstate = fpuArray.getDriverState();
    
    if ( (dstate == UNCONNECTED) || (dstate == UNINITIALISED))
    {
        // nothing to be done
        return;
    }

    bool sockets_closed = false;

    // check whether there was any error (so threads are
    // already terminating)
    if (! exit_threads.load(std::memory_order_acquire))
    {
        // write flag which signals both threads (reading and writing)
        // to exit
        exit_threads.store(true, std::memory_order_release);

        // close socket - this will terminate pending read and
        // write operations
        for (int i = 0; i < num_gateways; i++)
        {
            shutdown(SocketID[i], SHUT_RDWR);
        }
    }
    else
    {
        for (int i = 0; i < num_gateways; i++)
        {
            close(SocketID[i]);
        }
        sockets_closed = true;
    }


    // (both threads have to exit now!)
    // need to be read with
    //b = exit_threads.load(std::memory_order_acquire);

    // we wait for both threads to check
    // the wait flag and terminate in an orderly
    // manner.
    pthread_join(tx_thread, NULL);
    pthread_join(rx_thread, NULL);

    if (!sockets_closed)
    {
        for (int i = 0; i < num_gateways; i++)
        {
            close(SocketID[i]);
        }
    }
    // we update the grid state - importantly,
    // this also signals callers of waitForState()
    // so they don't go into dead-lock.
    fpuArray.setDriverState(UNCONNECTED);

};



void GatewayDriver::updatePendingCommand(I_CAN_Command& can_command)
{
    if (can_command.expectsResponse())
    {

        // we set the time out
        // for this command
        // - get current monotonous time
        timespec send_time;
        get_monotonic_time(send_time);

        timespec wait_period = can_command.getTimeOut();
        timespec deadline = add_time(send_time, wait_period);
        int fpu_id = can_command.getFPU_ID();
        
        fpuArray.setPendingCommand(fpu_id,
                                   can_command.getCommandCode(),
                                   deadline);
        timeOutList.insertTimeOut(fpu_id, deadline);
    }
    else
    {
        fpuArray.setLastCommand(fpu_id,
                                can_command.getCommandCode());
    }
}


void* GatewayDriver::threadTxFun(void *arg)
{

    bool exitFlag = false;

    struct pollfd pfd[NUM_GATEWAYS];

    nfds_t num_fds = num_gateways;

    for (int i=0; i < num_gateways; i++)
    {
        pfd[i].fd = SocketID[i];
        pfd[i].events = POLLOUT;        
    }

    /* Create mask to block SIGPIPE during calls to ppoll()*/
    sigset_t signal_set;
    sigemptyset(&signal_set);
    sigaddset(&signal_set, SIGPIPE);

    unique_ptr<I_CAN_Command> active_can_command[NUM_GATEWAYS];

    while (true)
    {

        // update poll mask so that only sockets
        // for which commands are queued will be
        // polled.
        // get currently pending commands
        t_command_mask cmd_mask = command_FIFO.checkForCommand();
        
        for (int i=0; i < num_gateways; i++)
        {
            if (sbuffer[i].numUnsentBytes() > 0)
            {
                // indicate unsent date
                cmd_mask |= (1 << i);
            }
        }

        if (cmd_mask == 0)
        {
            // no commands pending, wait a bit
            cmd_mask = command_FIFO.waitForCommand(COMMAND_WAIT_TIME);
        }

        // set poll parameters accordingly
        for (int i=0; i < num_gateways; i++)
        {
            if ((cmd_mask >> i) & 1)
            {
                pfd[i].events = POLLOUT;
            }
            else
            {
                pfd[i].events = 0;
            }
        }

        
        // this waits for a short time for sending data
        // (we could shorten the timeout if no command was pending)
        retval =  ppoll(pfd, num_fds, MAX_TX_TIMEOUT, signal_set);
        // TODO: error checking
        ASSERT(retval >= 0);

        // check all file descriptors for readiness
        for (int gateway_id=0; gateway_id < num_gateways; gateway_id++)
        {
            // FIXME: split overly long method into smaller ones
            
            if (pfd[gateway_id]].revent & POLLOUT)
            {

                // because we use nonblocking writes, it is not
                // likely, but entirely possible that some buffered data was
                // not yet completely send. If so, we try to catch up now.
                if (sbuffer[gateway_id].numUnsentBytes() > 0)
                {
                    // send remaining bytes of previous message
                    sbuffer[gateway_id].send_pending();
                }
                else
                {
                    // we can send a new message. Safely pop the
                    // pending command coming from the control thread
                    active_can_command[gateway_id] = command_FIFO.dequeueCommand(gateway_id);

                    if (active_can_command[gateway_id] != null) 
                    {

                        int message_len = 0;
                        t_CAN_buffer can_buffer;
                        ssize_t result;
                        int fpu_id = active_can_command[gateway_id].getFPU_ID();
                        const uint16_t busid = address_map[fpu_id].bus_id;
                        const uint8_t canid = address_map[fpu_id].can_id;
                               
                        // serialize data
                        active_can_command[gateway_id].SerializeToBuffer(busid,
                                                                         canid,
                                                                         message_len,
                                                                         can_buffer);
                        // byte-swizzle and send buffer
                        result = sbuffer[gateway_id].encode_and_send(SockedID[gateway_id],
                                                                     message_len, &(can_buffer.bytes));


                        // FIXME!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
                        // FIXME!!! the error checking here needs to be
                        // unified and combined with the check of the
                        // send_pending result.
                        // FIXME!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
                        
                        if (result == 0)
                        {
                            // this means the socket was closed,
                            // either by shutting down or by a
                            // serious connection error.
                            exitFlag = true;
                            // signal event listeners
                            fpuArray.setDriverState(UNCONNECTED); 
                        }

                        // FIXME: checking errno here is brittle as it could become
                        // overwritten; rather return it in a reference paramater
                        if ((result < 0) && (errno  != EWOULDBLOCK ) && (errno  != EAGAIN))
                        {
                            // an error occured, we need to check errno
                            exitFlag = true;
                            perror("socket error in receive:");
                        }

                        
                    }
                    exitFlag = exitFlag || exit_threads.load(std::memory_order_acquire);
                    if (exitFlag)
                    {
                        break; // terminate thread
                    }
                }
                if ( (sbuffer[gateway_id].numUnsentBytes() > 0)
                     && ( active_can_command[gateway_id] != null))
                {
                    // set send time in fpuArray memory structure
                    updatePendingCommand(active_can_command[gateway_id]);
                    // return CAN command instance to memory pool
                    command_pool.recycleInstance(active_can_command[gateway_id]);
                }
            }
    }
}





void* threadRxFun(void *arg)
{
    int		i;
    int		nread;

    struct pollfd pfd[NUM_GATEWAYS];

    int poll_timeout_ms = int(poll_timeout_sec * 1000);
    ASSERT(poll_timeout_ms > 0);
    
    nfds_t num_fds = num_gateways;
    
    for (int i=0; i < num_gateways; i++)
    {
        pfd[i].fd = SocketID[i];
        pfd[i].events = POLLIN;        
    }

    /* Create mask to block SIGPIPE during calls to ppoll()*/
    sigset_t signal_set;
    sigemptyset(&signal_set);
    sigaddset(&signal_set, SIGPIPE);


    while (true)
    {
        int retval;
        timespec cur_time;


            
        timespec next_timeout = timeOutList.getNextTimeOut(MAX_RX_TIMEOUT);
        get_monotonic_time(cur_time);

        timespec max_wait = time_to_wait(cur_time, next_timeout);

        retval =  ppoll(pfd, num_fds, max_wait, signal_set);
        // TODO: error checking
        ASSERT(retval >= 0);

        if (retval == 0)
        {
            // a time-out was hit - go through the list of FPUs
            // and mark each operation which has timed out.
            get_monotonic_time(cur_time);
            
            fpuArray.processTimeouts(cur_time, timeOutList);
        }
        else
        {
            for (int gateway_id=0; gateway_id  < num_gateways; bus_id++)
            {
                // for receiving, we listen to all descriptors at once
                if (pfd[i].revent | POLLIN)
                {

                    nread = decode_and_process(SocketID[gateway_id], gateway_id, this);

                    if (nread <= 0)
                    {
                        // a error happened when reading the socket,
                        // or connection was closed
                        exitFlag = true;
                        break;
                    }
                }
            }
        }
        // check whether terminating the thread was requested
        exitFlag = exitFlag || exit_threads.load(std::memory_order_acquire);
        if (exitFlag)
        {
            // signal event listeners
            fpuArray.setDriverState(UNCONNECTED); 
            break; // exit outer loop, and terminate thread
        }

    }
};

// This method parses any CAN response, dispatches it and stores
// result in fpu state array. It also clears any time-out flags for
// FPUs which did respond.
void GatewayDriver::handleFrame(int const gateway_id, uint8_t const * const  command_buffer, int const clen)
{
    t_CAN_buffer* message = data;

    if ((message != null) && (clen >= 3))
    {
        uint8_t busid = message->busid;
        uint_t can_identifier = message->identifier;

        fpuArray.dispatchResponse(fpu_id_by_adr, gateway_id, busid,
                                  can_identifier, &(message->data),
                                  clen -3, timeOutList);                   
    }
    // otherwise we have an invalid message
    // FIXME: logging of invalid messages
};




} // end of namespace
