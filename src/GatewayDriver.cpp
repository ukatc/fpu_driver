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
// NAME GatewayDriver.cpp
//
// This class implements the low-level CAN driver for the MOONS fiber
// positioner grid
//
////////////////////////////////////////////////////////////////////////////////

#include <cassert>
#include <poll.h>
#include <signal.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <unistd.h>

#include <arpa/inet.h>		/// inet_addr //
#include <netinet/tcp.h>	/// TCP_NODELAY
#include <pthread.h>

#include "canlayer/time_utils.h"
#include "canlayer/GatewayDriver.h"


namespace mpifps
{

namespace canlayer
{


GatewayDriver::GatewayDriver(int nfpus)
    : fpuArray(nfpus), command_pool(nfpus)
{

    assert(nfpus <= MAX_NUM_POSITIONERS);
    num_fpus = nfpus;
    
    // number of commands which are being processed
    fpuArray.setDriverState(DS_UNINITIALIZED);


    // assing default mapping and reverse mapping to
    // logical FPU ids.
    for (int fpuid=0; fpuid < MAX_NUM_POSITIONERS; fpuid++)
    {

        FPUArray::t_bus_address bus_adr;
        int busnum = fpuid / FPUS_PER_BUS;
        bus_adr.gateway_id = (uint8_t) (busnum / BUSES_PER_GATEWAY);
        bus_adr.bus_id =  (uint8_t) (busnum % BUSES_PER_GATEWAY);
        bus_adr.can_id = (uint8_t)(fpuid % FPUS_PER_BUS);

        address_map[fpuid] = bus_adr;

        fpu_id_by_adr[bus_adr.gateway_id][bus_adr.bus_id][bus_adr.can_id] = (uint16_t) fpuid;

    }

    exit_threads = false;

}

GatewayDriver::~GatewayDriver()
{


}


E_DriverErrCode GatewayDriver::initialize()
{
    E_DriverErrCode status;
    
    fpuArray.setDriverState(DS_UNINITIALIZED);
    status = command_pool.initialize();

    if (status != DE_OK)
    {
        return status;
    }

    status = commandQueue.initialize();

    if (status != DE_OK)
    {
        return status;
    }

    fpuArray.setDriverState(DS_UNCONNECTED);

    return DE_OK;
    

}


E_DriverErrCode GatewayDriver::deInitialize()
{
    E_DriverErrCode status;

    switch (fpuArray.getDriverState())
    {
    case DS_ASSERTION_FAILED:
    case DS_UNCONNECTED:
        break;

    case DS_CONNECTED:
        return DE_DRIVER_STILL_CONNECTED;
        
    case DS_UNINITIALIZED:
        return DE_DRIVER_NOT_INITIALIZED;
    }
                                        
    
    status = command_pool.deInitialize();

    if (status != DE_OK)
    {
        return status;
    }

    status = commandQueue.deInitialize();

    if (status != DE_OK)
    {
        return status;
    }

    fpuArray.setDriverState(DS_UNINITIALIZED);

    return DE_OK;
    

}


bool GatewayDriver::isLocked(int fpu_id)
{
    return fpuArray.isLocked(fpu_id);
}


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


static void* threadTxEntryFun(void *arg)
{
    GatewayDriver* driver = (GatewayDriver*) arg;
    return driver->threadTxFun();
}

static void* threadRxEntryFun(void *arg)
{
    GatewayDriver* driver = (GatewayDriver*) arg;
    return driver->threadRxFun();
}


E_DriverErrCode GatewayDriver::connect(const int ngateways,
                                       const t_gateway_address gateway_addresses[])
{

    assert(ngateways <= MAX_NUM_GATEWAYS);
    assert(ngateways >= 0);

    // check initialization state
    E_DriverState state = fpuArray.getDriverState();
    switch (state)
    {
    case DS_UNCONNECTED:
        break; // OK

    case DS_UNINITIALIZED:
        return DE_DRIVER_NOT_INITIALIZED;        
        
    case DS_CONNECTED:
        return DE_DRIVER_ALREADY_CONNECTED;
        
    default:
    case DS_ASSERTION_FAILED:
        return DE_ASSERTION_FAILED;
    }
        
    E_DriverErrCode rval = command_pool.initialize();

    if (rval != DE_OK)
    {
        return rval;
    }



    for (int i = 0; i < ngateways; i++)
    {
        const char* ip = gateway_addresses[i].ip;
        uint16_t port = gateway_addresses[i].port;

        SocketID[i] = make_socket(ip, port);

    }

    pthread_attr_t attr;
    /* Initialize and set thread joinable attribute */
    pthread_attr_init(&attr);
    pthread_attr_setdetachstate(&attr, PTHREAD_CREATE_JOINABLE);


    // we create only one thread for reading and writing.
    // FIXME: set error and driver state on success of
    // connection.
    exit_threads = false;
    num_gateways = ngateways;
    

    int err = pthread_create(&rx_thread, &attr, &threadRxEntryFun,
                             (void *) this);
    
    if (err != 0)
    {
        printf("\ncan't create thread :[%s]",
                              strerror(err));
    }

    if (err == 0)
    {

        err = pthread_create(&tx_thread, &attr, &threadTxEntryFun,
                         (void *) this);
        if (err != 0)
        {
            exit_threads.store(true, std::memory_order_release);
            printf("\ncan't create thread :[%s]",
                   strerror(err));
        }
    }

    pthread_attr_destroy(&attr);


    commandQueue.setNumGateways(ngateways);
    fpuArray.setDriverState(DS_CONNECTED);

    return DE_OK;

}

E_DriverErrCode GatewayDriver::disconnect()
{

    E_DriverState dstate = fpuArray.getDriverState();

    if ( (dstate == DS_UNCONNECTED) || (dstate == DS_UNINITIALIZED))
    {
        // nothing to be done
        return DE_NO_CONNECTION;
    }

    // disable retrieval of new commands from command queue
    commandQueue.setNumGateways(0);

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
    fpuArray.setDriverState(DS_UNCONNECTED);

    return DE_OK;

}



void GatewayDriver::updatePendingCommand(int fpu_id,
                                         std::unique_ptr<I_CAN_Command>& can_command)
{

    if (can_command->expectsResponse())
    {

        // we set the time out
        // for this command
        // - get current monotonous time
        timespec send_time;
        get_monotonic_time(send_time);

        timespec wait_period = can_command->getTimeOut();
        timespec deadline = time_add(send_time, wait_period);

        fpuArray.setPendingCommand(fpu_id,
                                   can_command->getInstanceCommandCode(),
                                   deadline);
        timeOutList.insertTimeOut(fpu_id, deadline);
    }
    else
    {
        fpuArray.setLastCommand(fpu_id,
                                can_command->getInstanceCommandCode());
    }
    // update number of commands being sent
    fpuArray.decSending();
}

// This method either fetches and sends a new buffer
// of CAN command data to a gateway, or completes sending of
// a pending buffer, returning the status of the connection.
SBuffer::E_SocketStatus GatewayDriver::send_buffer(unique_ptr<I_CAN_Command> &active_can_command, int gateway_id)
{
    SBuffer::E_SocketStatus status = SBuffer::ST_OK;

    // because we use nonblocking writes, it is not
    // likely, but entirely possible that some buffered data was
    // not yet completely send. If so, we try to catch up now.
    if (sbuffer[gateway_id].numUnsentBytes() > 0)
    {
        // send remaining bytes of previous message
        status = sbuffer[gateway_id].send_pending(SocketID[gateway_id]);
#ifdef DEBUG
        printf("p"); fflush(stdout);
#endif
    }
    else
    {
        // we can send a new message. Safely pop the
        // pending command coming from the control thread
        
        // update number of commands being sent first
        // (this avoids a race condition when querying
        // the number of commands which are not sent).
        fpuArray.incSending();
        active_can_command = commandQueue.dequeue(gateway_id);

        if (active_can_command)
        {

            int message_len = 0;
            t_CAN_buffer can_buffer;
            int fpu_id = active_can_command->getFPU_ID();
            const uint16_t busid = address_map[fpu_id].bus_id;
            const uint8_t fpu_canid = address_map[fpu_id].can_id;
#ifdef DEBUG3
            printf("txFPU#%i[%i] ", fpu_id, active_can_command->getInstanceCommandCode());
            fflush(stdout);
//            printf("command pars: fpu_id=%i, GW = %i, busid = %i, "
//                   " fpu_canid = %i\n", fpu_id, gateway_id, busid, fpu_canid);
#endif
            // serialize data
            active_can_command->SerializeToBuffer(busid,
                                                  fpu_canid,
                                                  message_len,
                                                  can_buffer);
            // byte-swizzle and send buffer
            status  = sbuffer[gateway_id].encode_and_send(SocketID[gateway_id],
                      message_len, can_buffer.bytes);

        }
    }
    return status;
}

// returns number of commands which are not yet sent
// in a thread-safe way. This is needed for waiting
// until all commands are sent.
int GatewayDriver::getNumUnsentCommands()
{
    // TODO: Check for potential race conditions if a
    // command is being sent and is processed very
    // quickly.
    return (fpuArray.countSending()
            + commandQueue.getNumQueuedCommands());
}



void* GatewayDriver::threadTxFun()
{


    bool exitFlag = false;

    struct pollfd pfd[MAX_NUM_GATEWAYS];

    nfds_t num_fds = num_gateways;
#ifdef DEBUG3
    printf("TX : num_gateways = %i\n", num_gateways);
#endif
    for (int gateway_id=0; gateway_id < num_gateways; gateway_id++)
    {
        pfd[gateway_id].fd = SocketID[gateway_id];
        pfd[gateway_id].events = POLLOUT;
    }

    /* Create mask to block SIGPIPE during calls to ppoll()*/
    sigset_t signal_set;
    sigemptyset(&signal_set);
    sigaddset(&signal_set, SIGPIPE);

    unique_ptr<I_CAN_Command> active_can_command[MAX_NUM_GATEWAYS];

    while (true)
    {

        // update poll mask so that only sockets
        // for which commands are queued will be
        // polled.
        // get currently pending commands
        CommandQueue::t_command_mask cmd_mask = commandQueue.checkForCommand();

        for (int gateway_id=0; gateway_id < num_gateways; gateway_id++)
        {
            if (sbuffer[gateway_id].numUnsentBytes() > 0)
            {
                // indicate unsent date
                cmd_mask |= (1 << gateway_id);
            }
        }

        if (cmd_mask == 0)
        {
            // no commands pending, wait a bit
            cmd_mask = commandQueue.waitForCommand(COMMAND_WAIT_TIME);
        }

        // set poll parameters accordingly
        for (int gateway_id=0; gateway_id < num_gateways; gateway_id++)
        {
            if ((cmd_mask >> gateway_id) & 1)
            {
                pfd[gateway_id].events = POLLOUT;
            }
            else
            {
                pfd[gateway_id].events = 0;
            }
        }
#ifdef DEBUG_POLL
        if (cmd_mask == 0)
        {
            printf("Q"); fflush(stdout);
        }
        printf("E%s", exit_threads.load(std::memory_order_acquire)
               ? "1" : "0");
        fflush(stdout);
        
#endif


        // this waits for a short time for sending data
        // (we could shorten the timeout if no command was pending)
        bool retry = false;
        do
        {
            retry = false;
            int retval =  ppoll(pfd, num_fds, &MAX_TX_TIMEOUT, &signal_set);
            //int retval =  ppoll(pfd, num_fds, &MAX_TX_TIMEOUT, NULL);
            if (retval < 0)
            {
                int errcode = errno;
                switch (errcode)
                {
                case EINTR: // an interrupt occured
                    //         this can happen and is fixed by just trying again
                    retry = true;
                    break;
                case EFAULT: // argument not contained in address space, see man page for ppoll()
                case EINVAL: // nfds value too large
                case ENOMEM: // out of memory
#ifdef DEBUG
                    printf("TX error: fatal error returnd from ppoll(), retval = %i\n",
                           retval);
#endif
                    fpuArray.setDriverState(DS_ASSERTION_FAILED);
                    exitFlag = true;
                    break;
                default:
                    // unknown return code
                    assert(false);

                }
            }
#ifdef DEBUG_POLL
            if (retval == 0)
            {
                printf("T"); fflush(stdout);
            }
#endif
        }
        while (retry);


        // check all file descriptors for readiness
        SBuffer::E_SocketStatus status = SBuffer::ST_OK;
        for (int gateway_id=0; gateway_id < num_gateways; gateway_id++)
        {
            // FIXME: split long method into smaller ones

            if (pfd[gateway_id].revents & POLLOUT)
            {

                // gets command and sends buffer
                status = send_buffer(active_can_command[gateway_id],
                                     gateway_id);
                // if finished, set command to pending
                if ( (sbuffer[gateway_id].numUnsentBytes() == 0)
                        && ( active_can_command[gateway_id]))
                {
                    // we completed sending a command.
                    // set send time in fpuArray memory structure
                    if (! active_can_command[gateway_id]->doBroadcast())
                    {
                        updatePendingCommand(active_can_command[gateway_id]->getFPU_ID(),
                                             active_can_command[gateway_id]);
                    }
                    else
                    {
                        // set pending command for all FPUs on the
                        // gateway (will ignore if state is locked).
                        for (int i; i < num_fpus; i++)
                        {
                            if (address_map[i].gateway_id
                                == gateway_id)
                            {
                                updatePendingCommand(i,
                                                     active_can_command[gateway_id]);
                            }
                        }
                    }
                    // return CAN command instance to memory pool
                    command_pool.recycleInstance(active_can_command[gateway_id]);
                }
                // check whether there was any serious error
                // such as a broken connection
                if (status != SBuffer::ST_OK)
                {
                    // this means the socket was closed,
                    // either by shutting down or by a
                    // serious connection error.
                    exitFlag = true;
                    // signal event listeners
                    switch (status)
                    {
                    case SBuffer::ST_NO_CONNECTION:
#ifdef DEBUG
                        printf("TX error: SBuffer::ST_NO_CONNECTION, disconnecting driver\n");
#endif
                        fpuArray.setDriverState(DS_UNCONNECTED);
                        break;

                    case SBuffer::ST_ASSERTION_FAILED:
                    default:
#ifdef DEBUG
                        printf("TX error: SBuffer::ST_ASSERTION_FAILED or unknown state, disconnecting driver\n");
#endif
                        fpuArray.setDriverState(DS_ASSERTION_FAILED);
                        break;
                    }

                }
            }
            exitFlag = exitFlag || exit_threads.load(std::memory_order_acquire);
            if (exitFlag)
            {
                exit_threads.store(true, std::memory_order_release);
                break; // break out of inner loop (iterating gateways)
            }
        }
        // poll the exit flag, it might be set by another thread
        exitFlag = exitFlag || exit_threads.load(std::memory_order_acquire);
        if (exitFlag)
        {
            break; // break main loop and terminate thread
        }

    }
    // clean-up before terminating the thread

    // return pending commands to the *front* of the command queue
    // so that they are not lost, and memory leaks are avoided
    for (int gateway_id=0; gateway_id < num_gateways; gateway_id++)
    {
        unique_ptr<I_CAN_Command> can_cmd = std::move(active_can_command[gateway_id]);
        if (can_cmd != nullptr)
        {
            commandQueue.requeue(gateway_id, std::move(can_cmd));
        }
    }
    return NULL;
}

#ifdef DEBUG
inline void print_time(char* label, struct timespec tm)
{
    printf("%s : %li / %09li\n", label, tm.tv_sec, tm.tv_nsec);
    fflush(stdout);
}

inline void print_curtime(char* label)    
{
    struct timespec tm;
    get_monotonic_time(tm);
    print_time(label, tm);
    fflush(stdout);
}

#endif

void* GatewayDriver::threadRxFun()
{

    struct pollfd pfd[MAX_NUM_GATEWAYS];

    //int poll_timeout_ms = int(poll_timeout_sec * 1000);
    //assert(poll_timeout_ms > 0);

    nfds_t num_fds = num_gateways;
#ifdef DEBUG3
    printf("RX : num_gateways = %i\n", num_gateways);
#endif

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
        bool exitFlag = false;
        int retval;
        timespec cur_time;

        get_monotonic_time(cur_time);
        // get absolute timeout value
        timespec next_rx_tmout = time_add(cur_time, MAX_RX_TIMEOUT);
#ifdef DEBUG_POLL
        print_time("RX next time-out maximum:    ", next_rx_tmout);
#endif


        timespec next_timeout = timeOutList.getNextTimeOut(next_rx_tmout);
#ifdef DEBUG_POLL
        print_time("RX next time-out from list:  ", next_timeout);
#endif

        timespec max_wait = time_to_wait(cur_time, next_timeout);
#ifdef DEBUG_POLL
        print_time("RX max_wait for socket read:", max_wait);
        print_time("RX cur_time before read poll:", cur_time);
#endif

        bool retry = false;
        do
        {
            retry = false;
#ifdef DEBUG_POLL
            print_time("rx timeout = ", max_wait);
#endif
// bug is seemingly NOT caused by signal_set
//            retval =  ppoll(pfd, num_fds, &max_wait, NULL);
            retval =  ppoll(pfd, num_fds, &max_wait, &signal_set);
#ifdef DEBUG_POLL
        print_curtime("cur_time after write poll: ");
#endif
            if (retval < 0)
            {
                int errcode = errno;
                switch (errcode)
                {
                case EINTR: // an interrupt occured.
                    //         this is fixed by just trying again
                    retry = true;
                    break;
                case EFAULT: // argument not contained in address space, see man page for ppoll()
                case EINVAL: // nfds value too large
                case ENOMEM: // out of memory
#ifdef DEBUG
                    printf("RX error: fatal error from ppoll() (errno = %i),"
                           " disconnecting driver\n", errcode);
#endif
                    fpuArray.setDriverState(DS_ASSERTION_FAILED);
                    exitFlag = true;
                    break;
                default:
                    // unknown return code
                    assert(false);

                }
            }
        }
        while (retry);

        if (retval == 0)
        {
            // a time-out was hit - go through the list of FPUs
            // and mark each FPU which has timed out.
#ifdef DEBUG_POLL
            printf("R"); fflush(stdout);
#endif
            get_monotonic_time(cur_time);

            fpuArray.processTimeouts(cur_time, timeOutList);
        }
        else if (retval > 0)
        {
            for (int gateway_id=0; gateway_id  < num_gateways; gateway_id++)
            {
                // for receiving, we listen to all descriptors at once
                if (pfd[gateway_id].revents | POLLIN)
                {

                    SBuffer::E_SocketStatus status = sbuffer[gateway_id].decode_and_process(SocketID[gateway_id], gateway_id, this);

                    if (status != SBuffer::ST_OK)
                    {
                        // a error happened when reading the socket,
                        // or the connection was closed
#ifdef DEBUG
                        printf("RX error: sbuffer socket status = %i, exiting\n",
                              status);
#endif
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
            exit_threads.store(true, std::memory_order_release);
#ifdef DEBUG
            printf("RX error: disconnecting driver\n");
#endif
            fpuArray.setDriverState(DS_UNCONNECTED);
            break; // exit outer loop, and terminate thread
        }
    }
    return NULL;
}

// This method parses any CAN response, dispatches it and stores
// result in fpu state array. It also clears any time-out flags for
// FPUs which did respond.
void GatewayDriver::handleFrame(int const gateway_id, uint8_t const command_buffer[MAX_UNENCODED_GATEWAY_MESSAGE_BYTES], int const clen)
{
    t_CAN_buffer* can_msg = (t_CAN_buffer*) command_buffer;

    if (can_msg == nullptr)
    {
        // FIXME: logging of invalid messages
#ifdef DEBUG
            printf("RX invalid CAN message (empty)- ignoring.\n");
#endif
    }

    if (clen < 3)
    {
        // FIXME: logging of invalid messages
#ifdef DEBUG
        printf("RX invalid CAN message (length is only %i)- ignoring.\n", clen);
#endif
    }
    {
        const uint8_t busid = can_msg->message.busid;
        const uint16_t can_identifier = can_msg->message.identifier;

        fpuArray.dispatchResponse(fpu_id_by_adr,
                                  gateway_id,
                                  busid,
                                  can_identifier,
                                  can_msg->message.data,
                                  clen -3, timeOutList);
    }
}


E_GridState GatewayDriver::getGridState(t_grid_state& out_state)
{
    return fpuArray.getGridState(out_state);
}

// get the current state of the driver
E_DriverState GatewayDriver::getDriverState()
{
    t_grid_state state;
    getGridState(state);
    return state.driver_state;
}


E_GridState GatewayDriver::waitForState(E_WaitTarget target, t_grid_state& out_detailed_state)
{
    return fpuArray.waitForState(target, out_detailed_state);
}


CommandQueue::E_QueueState GatewayDriver::sendCommand(int fpu_id, unique_ptr<I_CAN_Command>& new_command)
{

    assert(fpu_id < num_fpus);
    const int gateway_id = address_map[fpu_id].gateway_id;
    
#ifdef DEBUG
    if (! new_command)
    {
        printf("nullpointer passed!\n");
    }
#endif
    
    return commandQueue.enqueue(gateway_id, new_command);
}

CommandQueue::E_QueueState GatewayDriver::broadcastCommand(const int gateway_id, unique_ptr<I_CAN_Command>& new_command)
{

    assert(gateway_id < MAX_NUM_GATEWAYS);
    return commandQueue.enqueue(gateway_id, new_command);
}


int GatewayDriver::getGatewayIdByFPUID(const int fpu_id)
{
    return address_map[fpu_id].gateway_id;
}

}

} // end of namespace
