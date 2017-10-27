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
// NAME
//
// This class implements the low-level CAN driver for the MOONS fiber
// positioner grid
//
////////////////////////////////////////////////////////////////////////////////

namespace mpifps
{

GatewayDriver::GatewayDriver(int num_fpus)
{

    // initialize condition variable
    pthread_cond_init(&cond_state_change, NULL);

    // initialize mutex which protects FPU grid state
    grid_state_mutex = PTHREAD_MUTEX_INITIALIZER;

    // initialize mutex which ensures that only one
    // command is sent at the same time
    command_creation_mutex = PTHREAD_MUTEX_INITIALIZER;

    FPUGridState.count_collision = 0;
    FPUGridState.count_initialised = 0;
    FPUGridState.count_ready = 0;
    FPUGridState.count_moving = 0;
    FPUGridState.count_error = 0;

    for (int i=0; i < MAX_NUM_POSITIONERS; i++)
    {

        t_fpu_state fpu= FPUGridState[i];

        fpu.gateway_id = i / (FPUS_PER_BUS * BUSES_PER_GATEWAY);
        fpu.bus_id =  i % BUSES_PER_GATEWAY;
        fpu.can_id = i % (BUSES_PER_GATEWAY * FPUS_PER_BUS);

        fpu.is_initialized = false;
        // the values below are invalid, they need
        // initialization from fpu response.
        fpu.alpha_steps = 0;
        fpu.beta_steps = 0;
        fpu.on_alpha_data = false;
        fpu.on_beta_datum = false;
        fpu.alpha_collision = false;
        fpu.at_alpha_limit = false;
        fpu.beta_collision = false;

    }

    memset(ReadBuffer, 0, sizeof(ReadBuffer));
    memset(WriteBuffer, 0, sizeof(WriteBuffer));


};

GatewayDriver::~GatewayDriver()
{

    // destroy condition variable
    pthread_cond_destroy(&cond_state_change);

    // destroy grid state mutex
    pthread_mutex_destroy(&cohort_state_mutex);

    // destroy command creation mutex
    pthread_mutex_destroy(&command_creation_mutex);

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
    // TODO: does it make sense to have separate threads
    // for each socket?
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

    while (true)
    {
        int i = 3; // sample payload

        if (flag_Connect)
        {

            // safely pop the next command coming from the
            // control thread
            // this function waits a while if there is no command
            can_command = command_FIFO.pop(command_timeout);

            if (can_command != null)
            {
                gateway_number = can_command.getGatewayID();

                int message_len = 0;
                uint8_t   message_buf[MAX_CAN_MESSAGE_LENGTH_BYTES];

                can_command.SerializeToBuffer(message_len, message_buf);


                ssize_t result;

                result = encode_and_send(SockedID[gateway_number], message_len, can_message);

                // FIXME: CHECK ERROR CODE in errno if result equals -1!!

            }
            exitFlag = exit_threads.load(std::memory_order_acquire);
            if (exitFlag)
            {
                break; // terminate thread
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
    char	buffer[0x40];
    fd_set	rfds;

    // that should be rewritten using poll()
    struct timeval timeout = { .tv_sec = 0,.tv_usec = 50000 };

    while (true)
    {

        FD_ZERO(&rfds);

        FD_SET(sock, &rfds);

        if (select(sock + 1, &rfds, NULL, NULL, &timeout) < 0)
        {
            perror("select");
            return -1;
        }



        if (FD_ISSET(sock, &rfds))
        {
            // identify socket_id / bus_id
            int bus__id = ?;

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

};

// this method dispatches handles any response from the CAN bus
void handleFrame(int bus_id, uint8_t const * const  command_buffer, int const clen)
{
};


} // end of namespace
