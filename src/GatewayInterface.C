////////////////////////////////////////////////////////////////////////////////
//
// Copyright 2017 UKRI. See file "LICENSE" for license information.
//
// Who       When        What
// --------  ----------  -------------------------------------------------------
// Pablo Gutierrez 2017-07-22  created CAN client sample
// jnix      2017-10-18  Created driver class using Pablo Guiterrez' CAN client samplepn


//------------------------------------------------------------------------------

////////////////////////////////////////////////////////////////////////////////
// NAME GatewayInterface.cpp
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
#include <sys/eventfd.h>
#include <sys/mman.h>
#include <unistd.h>
#include <sched.h> // sched_setscheduler()
#include <math.h>


#include <arpa/inet.h>		/// inet_addr //
#include <netinet/tcp.h>	/// TCP_NODELAY
#include <pthread.h>

#include "InterfaceConstants.h"
#include "ethercan/time_utils.h"
#include "ethercan/GatewayInterface.h"
#include "ethercan/cancommands/AbortMotionCommand.h"


namespace mpifps
{

namespace ethercanif
{


GatewayInterface::GatewayInterface(const EtherCANInterfaceConfig &config_vals)
    : commandQueue(config_vals), config(config_vals),
      fpuArray(config_vals),
      command_pool(config_vals)
{

    assert(config.num_fpus <= MAX_NUM_POSITIONERS);

    // pass config parameters to sbuffer instances.  This is a bit
    // ugly as config needs to be kept const, but sbuffer being an
    // array, we unfortunately cannot set it in the initializer list.
    for(int i=0; i < MAX_NUM_GATEWAYS; i++)
    {
        sbuffer[i].setConfig(config_vals);
    }

    // number of commands which are being processed
    fpuArray.setInterfaceState(DS_UNINITIALIZED);

    memset(SocketID, 0, sizeof(SocketID));
    DescriptorCommandEvent = 0;
    DescriptorCloseEvent = 0;

    // initialize address map
    memset(fpu_id_by_adr, 0, sizeof(fpu_id_by_adr));
    // assing default mapping and reverse mapping to
    // logical FPU ids.
    // Important: the reverse mapping needs to be defined
    // for all FPUs which can in theory respond to
    // a broadcast, because the FPUids are registered
    // using reverse lookup.
    for (int fpuid=0; fpuid < MAX_NUM_POSITIONERS; fpuid++)
    {

        FPUArray::t_bus_address bus_adr;
        int busnum = fpuid / FPUS_PER_BUS;
        bus_adr.gateway_id = (uint8_t) (busnum / BUSES_PER_GATEWAY);
        bus_adr.bus_id =  (uint8_t) (busnum % BUSES_PER_GATEWAY);
        bus_adr.can_id = 1 + (uint8_t)(fpuid % FPUS_PER_BUS);

        address_map[fpuid] = bus_adr;

        fpu_id_by_adr[bus_adr.gateway_id][bus_adr.bus_id][bus_adr.can_id] = (uint16_t) fpuid;

    }

    exit_threads = false;
    shutdown_in_progress = false;
}

GatewayInterface::~GatewayInterface()
{
}


E_EtherCANErrCode GatewayInterface::initialize()
{
    E_EtherCANErrCode status;

    fpuArray.setInterfaceState(DS_UNINITIALIZED);
    status = fpuArray.initialize();

    if (status != DE_OK)
    {
        return status;
    }

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

    fpuArray.setInterfaceState(DS_UNCONNECTED);

    return DE_OK;


}


E_EtherCANErrCode GatewayInterface::deInitialize()
{
    E_EtherCANErrCode status;

    switch (fpuArray.getInterfaceState())
    {
    case DS_ASSERTION_FAILED:
    case DS_UNCONNECTED:
        break;

    case DS_CONNECTED:

        LOG_CONTROL(LOG_ERROR, "%18.6f : error: GridDriver::deInitialize() : "
                    "GatewayInterface::deInitialize() - driver is still connected",
                    ethercanif::get_realtime());
        return DE_INTERFACE_STILL_CONNECTED;

    case DS_UNINITIALIZED:
        LOG_CONTROL(LOG_ERROR, "%18.6f : warning: GridDriver::deInitialize() : "
                    "GatewayInterface::deInitialize() - driver is not initialized",
                    ethercanif::get_realtime());
        return DE_INTERFACE_NOT_INITIALIZED;
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

    status = fpuArray.deInitialize();

    if (status != DE_OK)
    {
        return status;
    }

    fpuArray.setInterfaceState(DS_UNINITIALIZED);

    return DE_OK;


}


bool GatewayInterface::isLocked(int fpu_id) const
{
    return fpuArray.isLocked(fpu_id);
}


int make_socket(const EtherCANInterfaceConfig &config, const char *ip, uint16_t port)
{
    int sck = -1;
    int rval = 0;
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

    sck = socket (PF_INET, SOCK_STREAM | SOCK_NONBLOCK, 0);

    if (sck < 0)
    {
        perror("socket");
        return -1;
    }

    rval = connect(sck, (struct sockaddr *) &addr, sizeof(addr));
    if ((rval < 0) && (errno != EINPROGRESS))
    {
        fprintf(stderr, "connect failed: %s\n", strerror(errno));
        close(sck);
        return -1;
    }

    // disable Nagle algorithm meaning that
    // segments of any size will be sent
    // without waiting. This is bad for throughput,
    // but keeps latency down.

    int errstate=0;
    errstate = setsockopt(sck, IPPROTO_TCP, TCP_NODELAY, &value,
                          sizeof(value));

    if (errstate < 0)
    {
        close(sck);
        return -1;
    }

#if 0
    if (config.SocketTimeOutSeconds > 0)
    {

        struct timeval timeout;
        timeout.tv_sec = int(config.SocketTimeOutSeconds);
        timeout.tv_usec = int((config.SocketTimeOutSeconds - timeout.tv_sec) * 1e6);

        errstate = setsockopt(sck, SOL_SOCKET, SO_RCVTIMEO, (char *)&timeout,
                              sizeof(timeout));

        if (errstate < 0)
        {
            close(sck);
            return -1;
        }

        errstate = setsockopt(sck, SOL_SOCKET, SO_SNDTIMEO, (char *)&timeout,
                              sizeof(timeout));

        if (errstate < 0)
        {
            close(sck);
            return -1;
        }
    }
#endif

    if (config.SocketTimeOutSeconds > 0)
    {
        // set TCP keepalive parameters and time-outs

        if (config.TCP_KeepaliveIntervalSeconds > 0)
        {
            /* configure keepalive probing of the connection. After
               idle_time idle seconds, a packet is sent every keep_alive_interval
               seconds. If no response is seen after max_keepalives packets,
               the connection is deemed dead and the driver returns with an error.
               We need to set this option because the default parameters
               on Linux are very long, 7200 seconds.
            */

            int opt_ka = 1;
            errstate = setsockopt(sck, SOL_SOCKET, SO_KEEPALIVE, &opt_ka,
                                  sizeof(opt_ka));

            if (errstate < 0)
            {
                printf("socket error: setting SO_KEEPALIVE failed!\n");
                close(sck);
                return -1;
            }

            int idle_time = std::max(1, config.TCP_IdleSeconds);
            errstate = setsockopt(sck, IPPROTO_TCP, TCP_KEEPIDLE, &idle_time,
                                  sizeof(idle_time));

            if (errstate < 0)
            {
                printf("socket error: setting TCP_KEEPIDLE failed!\n");
                close(sck);
                return -1;
            }

            int keep_alive_interval = std::max(1, config.TCP_KeepaliveIntervalSeconds);
            errstate = setsockopt(sck, IPPROTO_TCP, TCP_KEEPINTVL, &keep_alive_interval,
                                  sizeof(keep_alive_interval));

            if (errstate < 0)
            {
                close(sck);
                return -1;
            }

            const double max_idle_time = config.SocketTimeOutSeconds - config.TCP_IdleSeconds;
            const int max_keepalives = std::max(1, int(ceil(max_idle_time / keep_alive_interval)));
            errstate = setsockopt(sck, IPPROTO_TCP, TCP_KEEPCNT, &max_keepalives,
                                  sizeof(max_keepalives));

            if (errstate < 0)
            {
                printf("socket error: setting TCP_KEEPCNT failed!\n");
                close(sck);
                return -1;
            }
        }

        // This sets an additional time-out for the case that a sent packet is not
        // acknowledged. This is more fine-grained than using keep-alives, and the
        // time used here can be much shorter than one second.
        int user_timeout_ms = int(ceil(config.SocketTimeOutSeconds * 1000));
        errstate = setsockopt(sck, IPPROTO_TCP, TCP_USER_TIMEOUT, &user_timeout_ms,
                              sizeof(user_timeout_ms));

        if (errstate < 0)
        {
            printf("socket error: setting TCP_USER_TIMEOUT failed!\n");
            close(sck);
            return -1;
        }

    }

    return sck;
}


static void* threadTxEntryFun(void *arg)
{
    GatewayInterface* driver = static_cast<GatewayInterface*>(arg);
    return driver->threadTxFun();
}

static void* threadRxEntryFun(void *arg)
{
    GatewayInterface* driver = static_cast<GatewayInterface*>(arg);
    return driver->threadRxFun();
}

void set_rt_priority(const EtherCANInterfaceConfig &config, int prio)
{
    if (USE_REALTIME_SCHEDULING)
    {
        const pid_t pid = 0;
        struct sched_param sparam;
        sparam.sched_priority = prio;

        int rv = sched_setscheduler(pid, SCHED_FIFO, &sparam);
        if (rv == 0)
        {

            // allocate reserve pages and lock memory to avoid paging latency
            const size_t MEM_RESERVE_BYTES = 1024 * 1024 * 5;
            char mem_reserve[MEM_RESERVE_BYTES];
            memset(mem_reserve, 1, sizeof(mem_reserve));
            mlockall(MCL_FUTURE);

            LOG_CONTROL(LOG_DEBUG, "Info: real-time priority successfully set to %i\n", prio);
        }
        else
        {
            int errcode = errno;

            assert(errcode == EPERM);
            LOG_CONTROL(LOG_DEBUG, "Warning: real-time scheduling not active, occasional large latencies are possible.\n");
        }
    }

}

void unset_rt_priority()
{
    if (USE_REALTIME_SCHEDULING)
    {
        const pid_t pid = 0;
        struct sched_param sparam;
        sparam.sched_priority = 0;

        int rv = sched_setscheduler(pid, SCHED_OTHER, &sparam);
        assert(rv == 0);
    }
}

E_EtherCANErrCode GatewayInterface::connect(const int ngateways,
                                       const t_gateway_address gateway_addresses[])
{

    assert(ngateways <= MAX_NUM_GATEWAYS);
    assert(ngateways >= 0);

    // check initialization state
    E_InterfaceState state = fpuArray.getInterfaceState();
    switch (state)
    {
    case DS_UNCONNECTED:
        break; // OK

    case DS_UNINITIALIZED:
        LOG_CONTROL(LOG_ERROR, "%18.6f : error: GridDriver::connect() : "
                    "GatewayInterface::connect() - driver is not initialized",
                    ethercanif::get_realtime());
        return DE_INTERFACE_NOT_INITIALIZED;

    case DS_CONNECTED:
        LOG_CONTROL(LOG_ERROR, "%18.6f : error: GridDriver::connect() : "
                    "GatewayInterface::connect() - driver is already connected",
                    ethercanif::get_realtime());

        return DE_INTERFACE_ALREADY_CONNECTED;

    default:
    case DS_ASSERTION_FAILED:
        LOG_CONTROL(LOG_ERROR, "%18.6f : error: GridDriver::connect() : "
                    "GatewayInterface::connect() - assertion failed in FPUArray.getInterfaceState()",
                    ethercanif::get_realtime());
        return DE_ASSERTION_FAILED;
    }

    E_EtherCANErrCode ecode = DE_OK;
    int num_initialized_sockets= 0; // this is needed for error cleanup

    // create two eventfds to signal changes while waiting for I/O
    DescriptorCommandEvent = eventfd(0, EFD_NONBLOCK);
    if (DescriptorCommandEvent < 0)
    {
        LOG_CONTROL(LOG_ERROR, "%18.6f : error: GridDriver::connect() : "
                    "GatewayInterface::connect() - assertion failed,"
                    " creation of event file descriptor failed",
                    ethercanif::get_realtime());
        ecode = DE_ASSERTION_FAILED;
        // we use goto here to avoid code duplication.
        goto error_exit;
    }

    DescriptorCloseEvent = eventfd(0, EFD_NONBLOCK);

    if (DescriptorCloseEvent < 0)
    {
        LOG_CONTROL(LOG_ERROR, "%18.6f : error: GridDriver::connect() : "
                    "GatewayInterface::connect() - assertion failed,"
                    " creation of event file descriptor failed",
                    ethercanif::get_realtime());

        ecode= DE_ASSERTION_FAILED;
        goto close_CommandEventDescriptor;
    }

    // initialize command pool
    {
        E_EtherCANErrCode rval = command_pool.initialize();

        if (rval != DE_OK)
        {
            ecode = rval;
            goto close_CloseEventDescriptor;
        }
    }


    // open sockets
    for (int i = 0; i < ngateways; i++)
    {
        const char* ip = gateway_addresses[i].ip;
        uint16_t port = gateway_addresses[i].port;

        int sock_fd = make_socket(config, ip, port);
        if (sock_fd < 0)
        {
            LOG_CONTROL(LOG_ERROR, "%18.6f : error: GridDriver::connect() : "
                        "GatewayInterface::connect() - assertion failed,"
                        " creation of socket failed",
                        ethercanif::get_realtime());
            ecode = DE_NO_CONNECTION;
            goto close_sockets;
        }
        SocketID[i] = sock_fd;
        num_initialized_sockets++;
    }

    // If configured, try to set real-time process scheduling policy
    // to keep latency low. This is optional.

    set_rt_priority(config, CONTROL_PRIORITY);


    // new block for locals
    {
        // finally, create threads
        pthread_attr_t attr;
        /* Initialize and set thread joinable attribute */
        pthread_attr_init(&attr);
        pthread_attr_setdetachstate(&attr, PTHREAD_CREATE_JOINABLE);


        // we create one thread for reading and one for writing.
        exit_threads = false; // assure flag is cleared
        shutdown_in_progress = false;

        num_gateways = ngateways; /* this becomes fixed for the threads */

        // At this point, all constant shared data and synchronization
        // objects should be in place.

        int err = pthread_create(&rx_thread, &attr, &threadRxEntryFun,
                                 (void *) this);

        if (err != 0)
        {
            fprintf(stderr, "\ncan't create thread :[%s]", strerror(err));

            LOG_CONTROL(LOG_ERROR, "%18.6f : error: GridDriver::connect() : "
                        "GatewayInterface::connect() - assertion failed,"
                        "RX thread creation failed : %s",
                        ethercanif::get_realtime(),
                        strerror(err));
            ecode = DE_ASSERTION_FAILED;
            // no goto here, this is intentional, we need
            // to deallocate attr.
        }
        else
        {

            err = pthread_create(&tx_thread, &attr, &threadTxEntryFun,
                                 (void *) this);
            if (err != 0)
            {

                fprintf(stderr, "\ncan't create thread :[%s]", strerror(err));

                LOG_CONTROL(LOG_ERROR, "%18.6f : error: GridDriver::connect() : "
                            "GatewayInterface::connect() - assertion failed,"
                            "RX thread creation failed : %s",
                            ethercanif::get_realtime(),
                            strerror(err));

                ecode = DE_ASSERTION_FAILED;

                // set flag to stop first thread
                exit_threads.store(true, std::memory_order_release);
                // also signal termination via eventfd, to inform epoll()
                uint64_t val = 2;
                write(DescriptorCloseEvent, &val, sizeof(val));

                // wait for rx thread to terminate
                pthread_join(rx_thread, NULL);

            }
        }

        pthread_attr_destroy(&attr);

    }

    if (ecode != DE_OK)
    {

        LOG_CONTROL(LOG_DEBUG, "%18.6f : error: GridDriver::connect() : "
                    "GatewayInterface::connect() - error exit, freeing any open resources ",
                    ethercanif::get_realtime());


close_sockets:
        for(int k = (num_initialized_sockets -1); k >= 0; k--)
        {
            shutdown(SocketID[k], SHUT_RDWR);
            close(SocketID[k]);
        }
        command_pool.deInitialize();
close_CloseEventDescriptor:
        close(DescriptorCloseEvent);
close_CommandEventDescriptor:
        close(DescriptorCommandEvent);

error_exit:
        ;/* nothing to be done */
    }
    else
    {
        commandQueue.setNumGateways(ngateways);
        fpuArray.setInterfaceState(DS_CONNECTED);
    }

    unset_rt_priority();


    return ecode;

}

E_EtherCANErrCode GatewayInterface::disconnect()
{

    E_InterfaceState dstate = fpuArray.getInterfaceState();

    if ( (dstate == DS_UNCONNECTED) || (dstate == DS_UNINITIALIZED))
    {
        // nothing to be done
        LOG_CONTROL(LOG_ERROR, "%18.6f : warning: GridDriver::disconnect() : "
                    "GatewayInterface::disconnect() - driver is not connected ",
                    ethercanif::get_realtime());

        return DE_NO_CONNECTION;
    }

    LOG_CONTROL(LOG_DEBUG, "%18.6f : GatewayInterface::disconnect(): disconnecting driver\n",
                get_realtime());

    // disable retrieval of new commands from command queue
    commandQueue.setNumGateways(0);

    // inform threads about shutdown
    shutdown_in_progress.store(true, std::memory_order_release);
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

    // Write to close-event descriptor to make epoll calls return
    // without waiting for time-out.
    uint64_t val = 2;
    write(DescriptorCloseEvent, &val, sizeof(val));


    // (both threads have to exit now!)
    // need to be read with
    //b = exit_threads.load(std::memory_order_acquire);

    // we wait for both threads to check
    // the wait flag and terminate in an orderly
    // manner.
    pthread_join(tx_thread, NULL);
    pthread_join(rx_thread, NULL);

    if (config.fd_txlog >= 0)
    {
        syncfs(config.fd_txlog);
    }
    if (config.fd_rxlog >= 0)
    {
        syncfs(config.fd_rxlog);
    }

    if (!sockets_closed)
    {
        for (int i = 0; i < num_gateways; i++)
        {
            close(SocketID[i]);
        }
    }

    // close eventfds

    assert(close(DescriptorCloseEvent) == 0);
    assert(close(DescriptorCommandEvent) == 0);

    // we update the grid state - importantly,
    // this also signals callers of waitForState()
    // so they don't go into dead-lock.
    fpuArray.setInterfaceState(DS_UNCONNECTED);

    LOG_CONTROL(LOG_GRIDSTATE, "%18.6f : disconnect(): driver is disconnected\n",
                get_realtime());

    return DE_OK;

}



void GatewayInterface::updatePendingCommand(int fpu_id,
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
                                   deadline, timeOutList);
    }
    else
    {
        fpuArray.setLastCommand(fpu_id,
                                can_command->getInstanceCommandCode());
    }
}


// update the pending sets either of one FPU or of all FPUs
// to which a broadcast command is sent.
//
// Note: Getting timing issues is tricky but it is probably best to
// set the pending flags before the command is actually sent.
// Otherwise, it is possible and happens that the response is
// processed before the pending bit is set which is confusing.

void GatewayInterface::updatePendingSets(unique_ptr<I_CAN_Command> &active_can_command,
                                      int gateway_id, int busid)
{
    if (! active_can_command->doBroadcast())
    {
        updatePendingCommand(active_can_command->getFPU_ID(),
                             active_can_command);
    }
    else
    {
        // set pending command for all FPUs on the same
        // (gateway, busid) address (will ignore if state is locked).
        for (int bus_adr=1; bus_adr < (1 + FPUS_PER_BUS); bus_adr++)
        {
            const int fpu_id = fpu_id_by_adr[gateway_id][busid][bus_adr];
            if ((fpu_id < config.num_fpus) && (fpu_id >= 0))
            {
                updatePendingCommand(fpu_id, active_can_command);
            }
        }
    }
}


// This method either fetches and sends a new buffer
// of CAN command data to a gateway, or completes sending of
// a pending buffer, returning the status of the connection.
SBuffer::E_SocketStatus GatewayInterface::send_buffer(unique_ptr<I_CAN_Command> &active_can_command, int gateway_id)
{
    SBuffer::E_SocketStatus status = SBuffer::ST_OK;

    // because we use nonblocking writes, it is not
    // likely, but entirely possible that some buffered data was
    // not yet completely send. If so, we try to catch up now.
    if (sbuffer[gateway_id].numUnsentBytes() > 0)
    {
        // send remaining bytes of previous message
        status = sbuffer[gateway_id].send_pending(SocketID[gateway_id]);
    }
    else
    {
        // we can send a new message. Safely pop the
        // pending command coming from the control thread

        // Update number of commands being sent first
        // (this avoids a race condition when querying
        // the number of commands which are not sent).
        active_can_command = commandQueue.dequeue(gateway_id);

        if (active_can_command)
        {

            int message_len = 0;
            t_CAN_buffer can_buffer;
            int fpu_id = active_can_command->getFPU_ID();
            const uint16_t busid = address_map[fpu_id].bus_id;
            const uint8_t fpu_canid = address_map[fpu_id].can_id;
            // serialize data
            active_can_command->SerializeToBuffer(busid,
                                                  fpu_canid,
                                                  message_len,
                                                  can_buffer);

            updatePendingSets(active_can_command, gateway_id, busid);
            // update number of queued commands
            fpuArray.decSending();

            // byte-swizzle and send buffer
            status  = sbuffer[gateway_id].encode_and_send(SocketID[gateway_id],
                      message_len, can_buffer.bytes);

        }
    }
    return status;
}



void GatewayInterface::incSending()
{
    fpuArray.incSending();
}

void* GatewayInterface::threadTxFun()
{

    LOG_TX(LOG_GRIDSTATE, "%18.6f : starting TX loop\n",
           get_realtime());

    bool exitFlag = false;

    const int NUM_TX_DESCRIPTORS = MAX_NUM_GATEWAYS + 2;

    struct pollfd pfd[NUM_TX_DESCRIPTORS];

    nfds_t num_fds = NUM_TX_DESCRIPTORS;

    for (int gateway_id=0; gateway_id < num_gateways; gateway_id++)
    {
        pfd[gateway_id].fd = SocketID[gateway_id];
        pfd[gateway_id].events = POLLOUT;
    }

    // add eventfd for closing connection
    const int idx_close_event = num_gateways;
    pfd[idx_close_event].fd = DescriptorCloseEvent;
    pfd[idx_close_event].events = POLLIN;

    // add eventfd for new command in queue
    const int idx_cmd_event = num_gateways +1;
    pfd[idx_cmd_event].fd = DescriptorCommandEvent;
    pfd[idx_cmd_event].events = POLLIN;

    commandQueue.setEventDescriptor(DescriptorCommandEvent);

    /* Create mask to block SIGPIPE during calls to ppoll()*/
    sigset_t signal_set;
    sigemptyset(&signal_set);
    sigaddset(&signal_set, SIGPIPE);

    unique_ptr<I_CAN_Command> active_can_command[MAX_NUM_GATEWAYS];


    set_rt_priority(config, WRITER_PRIORITY);

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


        // this waits for a short time for sending data
        // (we could shorten the timeout if no command was pending)
        int retval = 0;
        bool retry = false;
        do
        {
            retry = false;
            retval =  ppoll(pfd, num_fds, &MAX_TX_TIMEOUT, &signal_set);
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
                    LOG_TX(LOG_ERROR, "TX error: fatal error returnd from ppoll(), retval = %i\n",
                           retval);

                    fpuArray.setInterfaceState(DS_ASSERTION_FAILED);
                    exitFlag = true;
                    break;
                default:
                    // unknown return code
                    assert(false);

                }
            }
        }
        while (retry);


        if ((retval > 0 ) && (pfd[idx_cmd_event].revents & POLLIN))
        {
            // we need to read the descriptor to clear the event.
            uint64_t val;
            read(DescriptorCommandEvent, &val, sizeof(val));
        }

        // check all writable file descriptors for readiness
        SBuffer::E_SocketStatus status = SBuffer::ST_OK;
        for (int gateway_id=0; gateway_id < num_gateways; gateway_id++)
        {
            // FIXME: split long method into smaller ones

            if ((retval > 0 ) && (pfd[gateway_id].revents & POLLOUT))
            {

                // gets command and sends buffer
                status = send_buffer(active_can_command[gateway_id],
                                     gateway_id);
                // if finished, set command to pending
                if ( (sbuffer[gateway_id].numUnsentBytes() == 0)
                        && ( active_can_command[gateway_id]))
                {

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
                        LOG_TX(LOG_INFO, "%18.6f : TX: SBuffer::ST_NO_CONNECTION, disconnecting driver\n",
                               get_realtime());

                        fpuArray.setInterfaceState(DS_UNCONNECTED);
                        break;

                    case SBuffer::ST_ASSERTION_FAILED:
                    default:
                        LOG_TX(LOG_ERROR, "TX error: SBuffer::ST_ASSERTION_FAILED or unknown state, disconnecting driver\n");
                        fpuArray.setInterfaceState(DS_ASSERTION_FAILED);
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

    LOG_TX(LOG_GRIDSTATE, "%18.6f : exited TX loop\n",
           get_realtime());
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

    // clear event descriptor on commandQueue
    commandQueue.setEventDescriptor(-1);

    return NULL;
}

#ifdef DEBUG
inline void print_time(char* label, struct timespec tm)
{
    printf("%s : %li / %09li\n", label, tm.tv_sec, tm.tv_nsec);
    fflush(stdout);
}
#endif

#if 0
inline void print_curtime(char* label)
{
    struct timespec tm;
    get_monotonic_time(tm);
    print_time(label, tm);
    fflush(stdout);
}
#endif


void* GatewayInterface::threadRxFun()
{

    struct pollfd pfd[MAX_NUM_GATEWAYS+1];

    //int poll_timeout_ms = int(poll_timeout_sec * 1000);
    //assert(poll_timeout_ms > 0);

    nfds_t num_fds = num_gateways + 1;

    LOG_RX(LOG_GRIDSTATE, "%18.6f : starting RX loop\n",
           get_realtime());


    for (int i=0; i < num_gateways; i++)
    {
        pfd[i].fd = SocketID[i];
        pfd[i].events = POLLIN;
    }


    // add eventfd for closing connection
    pfd[num_gateways].fd = DescriptorCloseEvent;
    pfd[num_gateways].events = POLLIN;

    /* Create mask to block SIGPIPE during calls to ppoll()*/
    sigset_t signal_set;
    sigemptyset(&signal_set);
    sigaddset(&signal_set, SIGPIPE);

    set_rt_priority(config, READER_PRIORITY);


    while (true)
    {
        bool exitFlag = false;
        int retval;
        timespec cur_time;

        get_monotonic_time(cur_time);

        // compute a bounded absolute time


        timespec next_timeout = timeOutList.getNextTimeOut();


        timespec max_rx_tmout = time_add(cur_time, MAX_RX_TIMEOUT);
        if (time_smaller(max_rx_tmout, next_timeout))
        {
            next_timeout = max_rx_tmout;
        }


        timespec max_wait = time_to_wait(cur_time, next_timeout);

        bool retry = false;
        do
        {
            retry = false;
            retval =  ppoll(pfd, num_fds, &max_wait, &signal_set);
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
                    LOG_RX(LOG_ERROR, "RX error: fatal error from ppoll() (errno = %i),"
                           " disconnecting driver\n", errcode);
                    fpuArray.setInterfaceState(DS_ASSERTION_FAILED);
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

                        if (! shutdown_in_progress.load(std::memory_order_acquire))
                        {

                            LOG_RX(LOG_ERROR, "%18.6f : RX: read error from socket, exiting read loop\n",
                                   get_realtime());
                            printf("RX thread fatal error: sbuffer socket status = %i, exiting\n",
                                   status);
                        }
                        else
                        {
                            LOG_RX(LOG_INFO, "%18.6f : RX: shutdown in progress, exiting read loop\n",
                                   get_realtime());
                        }
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
            LOG_RX(LOG_INFO, "%18.6f : RX: loop exit, disconnecting driver\n",
                   get_realtime());

            fprintf(stderr, "RX thread: disconnecting driver\n");

            fpuArray.setInterfaceState(DS_UNCONNECTED);
            break; // exit outer loop, and terminate thread
        }
    }
    return NULL;
}

// This method parses any CAN response, dispatches it and stores
// result in fpu state array. It also clears any time-out flags for
// FPUs which did respond.
void GatewayInterface::handleFrame(int const gateway_id, uint8_t const command_buffer[MAX_UNENCODED_GATEWAY_MESSAGE_BYTES], int const clen)
{
    t_CAN_buffer* can_msg = (t_CAN_buffer*) command_buffer;

    // do basic filtering for correctness, and
    // call fpu-secific handler.
    if (can_msg == nullptr)
    {
        LOG_RX(LOG_ERROR, "RX %18.6f: error:  invalid CAN message (empty)- ignoring.\n",
               ethercanif::get_realtime());
    }
    else if (clen < 3)
    {
        LOG_RX(LOG_ERROR, "RX %18.6f : error: invalid CAN message (length is only %i)- ignoring.\n",
               ethercanif::get_realtime(), clen);
    }
    else
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


E_GridState GatewayInterface::getGridState(t_grid_state& out_state) const
{
    return fpuArray.getGridState(out_state);
}

// get the current state of the driver
E_InterfaceState GatewayInterface::getInterfaceState() const
{
    t_grid_state state;
    getGridState(state);
    return state.interface_state;
}


E_GridState GatewayInterface::waitForState(E_WaitTarget target, t_grid_state& out_detailed_state,
                                        double &max_wait_time, bool &cancelled) const
{
    return fpuArray.waitForState(target, out_detailed_state, max_wait_time, cancelled);
}


CommandQueue::E_QueueState GatewayInterface::sendCommand(const int fpu_id, unique_ptr<I_CAN_Command>& new_command)
{

    assert(fpu_id < config.num_fpus);
    const int gateway_id = address_map[fpu_id].gateway_id;
    assert(gateway_id < MAX_NUM_GATEWAYS);

    if (! new_command)
    {
        printf("nullpointer passed!\n");
        assert(0);
    }

    incSending();
    return commandQueue.enqueue(gateway_id, new_command);
}


#if 0
int GatewayInterface::getGatewayIdByFPUID(const int fpu_id) const
{
    return address_map[fpu_id].gateway_id;
}
#endif


int GatewayInterface::getBroadcastID(const int gateway_id, const int busid)
{
    // get the id of fpu number one for this bus on this gateway.
    return fpu_id_by_adr[gateway_id][busid][1];
}


// This command is implemented on the gateway driver level so that the
// reading thread can call it directly in the case that too many
// collisions have been observed.
//
// This command should always be called from a thread executing with
// real-time priority in order to keep latencies between the different
// gateway messages low.

E_EtherCANErrCode GatewayInterface::abortMotion(t_grid_state& grid_state,
        E_GridState& state_summary)
{
    // first, get current state of the grid
    state_summary = getGridState(grid_state);
    // check driver is connected
    if (grid_state.interface_state != DS_CONNECTED)
    {
        LOG_CONTROL(LOG_ERROR, "%18.6f : FATAL ERROR: GridDriver::abortMotion() : "
                    " - driver is not connected, skipping command ",
                    ethercanif::get_realtime());

        return DE_NO_CONNECTION;
    }


    // Flush all queued commands from queue to command pool,
    // so that abort message is sent without delay.
    commandQueue.flushToPool(command_pool);

    // Send broadcast command to each gateway to abort movement of all
    // FPUs.

    E_EtherCANErrCode ecode =  broadcastMessage<AbortMotionCommand>();

    return ecode;

}


}

} // end of namespace
