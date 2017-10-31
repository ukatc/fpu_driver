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


};

void GatewayDriver::disconnect()
{
    // write flag which signals both threads (reading and writing)
    // to exit
    exit_threads.store(true, std::memory_order_release);

    // close socket - this will terminate pending read and
    // write operations
    for (int i = 0; i < num_gateways; i++)
    {
        close(SocketID);
    }


    // (both threads have to exit now!)
    // need to be read with
    //b = exit_threads.load(std::memory_order_acquire);

    // wait for both threads to terminate
    // TODO: make sure we can progress here
    pthread_join(tx_thread, NULL);
    pthread_join(rx_thread, NULL);

};





void* threadTxFun(void *arg)
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


    while (true)
    {

        // update poll mask so that only sockets
        // for which commands are queued will be
        // polled.
        for (int i=0; i < num_gateways; i++)
        {
            if ((! command_FIFO[i].is_empty()) || sbuffer[i].numUnsentBytes() > 0)
            {
                pfd[i].events = POLLOUT;
            }
            else
            {
                pfd[i].events = 0;
            }
        }

        retval =  ppoll(pfd, num_fds, MAX_TX_TIMEOUT, signal_set);
        // TODO: error checking
        ASSERT(retval >= 0);

        // check all file descriptors for readiness
        for (int gateway_id=0; gateway_id < num_gateways; gateway_id++)
        {
            
            if (pfd[gateway_id]].revent & POLLOUT)
            {

                // because we use nonblocking writes, it is not
                // likely, but possible that some buffered data was
                // not yet send. We try to catch up now.
                if (sbuffer[gateway_id].numUnsentBytes() > 0)
                {
                    // send remaining bytes of previous message
                    sbuffer[gateway_id].send_pending();
                }
                else
                {
                    // we can send a new message. Safely pop the
                    // pending command coming from the control thread
                    can_command = command_FIFO[i].dequeueCommand();

                    if (can_command != null) // this check is redundant...
                    {
                        gateway_number = can_command.getGatewayID();

                        int message_len = 0;
                        uint8_t   message_buf[MAX_CAN_MESSAGE_LENGTH_BYTES];
                        
                        can_command.SerializeToBuffer(message_len, message_buf);
                        
                        
                        ssize_t result;

                        result = sbuffer[gateway_id].encode_and_send(SockedID[gateway_number], message_len, can_message);

                        if (result > 0)
                        {
                            // now, we set the time out
                            // for this command
                            // - get current monotonous time
                            timespec send_time;
                            get_monotonic_time(send_time);

                            timespec wait_period = can_command.getTimeOut();
                            timespec deadline = add_time(send_time, wait_period);
                            // TODO: This is a bit sloppy because in some circumstances
                            // the sending over the socket might take
                            // longer. Probably the attributes of the
                            // command should be set into the sbuffer instance,
                            // and the time-out stored when the sending is complete.
                            fpuArray.setNextTimeOut(can_command.getFPU_ID(),
                                                    can_command.getCommandCode(),
                                                    deadline);

                            // return CAN command to memory pool
                            command_pool.recycleInstance(can_command);
                        
                        }

                        
                        if (result == 0)
                        {
                            // this means the socket was closed
                            exitFlag = true;
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
        }
    }
}


void process_frame(uint8_t *data, uint8_t len)
{
    // parses CAN reply and stores result in
    // fpu state array
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



        timespec next_timeout = fpuArray.getNextTimeOut(MAX_RX_TIMEOUT);
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
            
            fpuArray.processTimeouts(cur_time);
        }
        else
        {
            for (int bus_id=0; bus_id  < num_gateways; bus_id++)
            {
                // for receiving, we listen to all descriptors at once
                if (pfd[i].revent | POLLIN)
                {

                    nread = decode_and_process(SocketID[socked_id], bus_id, self);

                    if (nread < 0)
                    {
                        // an error happened when reading the socket
                        perror("recv_ck");
                        return -1;
                    }
                    else if (nread == 0)
                    {
                        // connection was closed
                        fprintf(stderr, "client closed connection\n");
                        return -1;
                    }
                }
            }
        }
        // check whether terminating the thread was requested
        exitFlag = exit_threads.load(std::memory_order_acquire);
        if (exitFlag)
        {
            break; // exit outer loop, and terminate thread
        }

    }
};

// this method dispatches handles any response from the CAN bus
void handleFrame(int bus_id, uint8_t const * const  command_buffer, int const clen)
{
};


} // end of namespace
