// -*- mode: c++ -*-

////////////////////////////////////////////////////////////////////////////////
//
// Copyright 2017 UKRI. See file "LICENSE" for license information.
//
//
// Who       When        What
// --------  ----------  -------------------------------------------------------
// Pablo Gutierrez 2017-07-22  created CAN client sample
// jnix      2017-10-18  Created driver class using Pablo Guiterrez' CAN client sample
// bwillemse 2021-03-26  Modified for new non-contiguous FPU IDs and CAN mapping.
//------------------------------------------------------------------------------

////////////////////////////////////////////////////////////////////////////////
// NAME GatewayInterface.h
//
// This class implements the low-level CAN driver for the MOONS fiber
// positioner grid
//
////////////////////////////////////////////////////////////////////////////////

#ifndef GATEWAY_INTERFACE_H
#define GATEWAY_INTERFACE_H

#include <atomic>

#include "../E_GridState.h"
#include "../EtherCANInterfaceConfig.h"
#include "../T_GatewayAddress.h"

#include "SBuffer.h"            // Coding and decoding message frames
#include "I_ResponseHandler.h"  // Interface for processing CAN responses
#include "FPUArray.h"           // Defines thread-safe structure of FPU state info
#include "TimeOutList.h"
#include "CommandQueue.h"
#include "CommandPool.h"
#include "ethercan/cancommandsv2/SyncCommand.h"

namespace mpifps
{

namespace ethercanif
{

//==============================================================================

void* threadTxFun(void *arg);
void* threadRxFun(void *arg);

//==============================================================================

#ifdef FLEXIBLE_CAN_MAPPING

using GridCanMap = std::vector<std::pair<int, FPUArray::t_bus_address>>;

#endif // FLEXIBLE_CAN_MAPPING

//==============================================================================
class GatewayInterface: private I_ResponseHandler
{
public:

    // TODO: BW: Noticed that for 2 of the timespecs below, the times mentioned
    // in the comments don't seem to match the actual time values?

    // Timeout for reading from command FIFO if nothing is pending - 500 ms
    const struct timespec COMMAND_WAIT_TIME = 
    {
        0,          // .tv_sec
        10000000    // .tv_nsec
    };

    // timeout for polling write socket - 500 ms
    const struct timespec MAX_TX_TIMEOUT =
    {
        0,          // .tv_sec
        500000000   // .tv_nsec
    };

    // default timeout for polling read socket - 500 msec
    const timespec MAX_RX_TIMEOUT =
    {
        10,         // .tv_sec
        500000000   // .tv_nsec
    };

#ifdef FLEXIBLE_CAN_MAPPING
    explicit GatewayInterface(const EtherCANInterfaceConfig &config_vals,
                              const GridCanMap &grid_can_map);
#else // NOT FLEXIBLE_CAN_MAPPING
    explicit GatewayInterface(const EtherCANInterfaceConfig &config_vals);
#endif // NOT FLEXIBLE_CAN_MAPPING
    ~GatewayInterface();

    E_EtherCANErrCode initialize();

    E_EtherCANErrCode deInitialize();

    static void set_sync_mask_message(t_CAN_buffer& can_buffer, int& buflen,
				                      const uint8_t msgid, uint8_t sync_mask);

    E_EtherCANErrCode configSyncCommands(const int ngateways);

    E_EtherCANErrCode connect(const int ngateways,
                              const t_gateway_address gateway_addresses[]);

    // Disconnects socket, and re-adds any pending commands to the command
    // queue. (If pending commands should be discarded, the command queue
    // needs to be explicitly flushed).
    E_EtherCANErrCode disconnect();

    // Gets the current state of the FPU grid, which is stored in the reference
    // parameter
    E_GridState getGridState(t_grid_state& out_state) const;

    // Gets the current state of the driver (this is a convenience function, the
    // state is contained in the grid state).
    E_InterfaceState getInterfaceState() const;

    // Gets both the summed up state of the FPU grid, and a detailed status for
    // each FPU.
    E_GridState waitForState(E_WaitTarget target, t_grid_state& out_detailed_state,
                             double &max_wait_time, bool &cancelled) const;

    // Provides a command instance with buffer space for sending CAN parameters.
    // This method is thread-safe
    template <typename T>
    inline unique_ptr<T> provideInstance()
    {
        return command_pool.provideInstance<T>();
    }

    void updatePendingSets(unique_ptr<CAN_Command> &active_can_command,
                           int gateway_id, int busid);

    // Sends a CAN command to the gateway. This method is thread-safe
    CommandQueue::E_QueueState sendCommand(const int fpu_id,
                                     unique_ptr<CAN_Command>& new_command);

    // Returns id which needs to be set as fpu id for broadcast command
    int getBroadcastID(const int gateway_id, const int busid);

    // Returns whether an FPU is currently marked as locked.
    bool isLocked(int fpu_id) const;

    // Sends an abortMotion broadcast command to all gateways.
    // (This is implemented at the CAN driver level because we need the Rx
    // thread to be able to trigger an automatic abort if too many collisions
    // happen in a short time span.).
    E_EtherCANErrCode abortMotion(t_grid_state& grid_state,
                                  E_GridState& state_summary,
				                  bool sync_message=true);

    //--------------------------------------------------------------------------
    template<typename T> E_EtherCANErrCode broadcastMessage(bool sync_message=false)
    {
        if (sync_message && (T::sync_code != SYNC_NOSYNC))
        {
            // A SYNC message is only sent once to a single gateway, the 'master'
            // gateway. This gateway will send the message as broadcast messages
            // to all buses, and forward it to the other gateways.
            // Electronics will ensure that all gateways broadcast the message
            // in a synchronized way (thus the name).
            unique_ptr<SyncCommand> sync_command;

            sync_command = provideInstance<SyncCommand>();

            if (sync_command == nullptr)
            {
                return DE_ASSERTION_FAILED;
            }

#ifdef FLEXIBLE_CAN_MAPPING
            // Gateway number zero is SYNC master
            const int broadcast_id = fpu_id_broadcast_base;
#else // NOT FLEXIBLE_CAN_MAPPING
            const int broadcast_id = 0; // gateway number zero is SYNC master
#endif // NOT FLEXIBLE_CAN_MAPPING

            // Broadcast_id is an fpu id which makes sure the message goes to
            // the requested bus.
            sync_command->parametrize(T::sync_code);
            unique_ptr<CAN_Command> cmd(sync_command.release());
            sendCommand(broadcast_id, cmd);
        }
        else
        {
            unique_ptr<T> can_command;

            for (int gateway_id = 0; gateway_id < num_gateways; gateway_id++)
            {
                for (int busid = 0; busid < BUSES_PER_GATEWAY; busid++)
                {
                    const int broadcast_id = getBroadcastID(gateway_id, busid);
#ifndef FLEXIBLE_CAN_MAPPING // NOT FLEXIBLE_CAN_MAPPING
                    if (broadcast_id >= config.num_fpus)
                    {
                        goto Exit;
                    }
#endif // NOT FLEXIBLE_CAN_MAPPING
                    can_command = provideInstance<T>();

                    if (can_command == nullptr)
                    {
                        return DE_ASSERTION_FAILED;
                    }
                    const bool do_broadcast = true;
                    // Broadcast_id is an fpu id which makes sure the message
                    // goes to the requested bus.
                    can_command->parametrize(broadcast_id, do_broadcast);
                    unique_ptr<CAN_Command> cmd(can_command.release());
                    sendCommand(broadcast_id, cmd);
                }
            }
        }
#ifndef FLEXIBLE_CAN_MAPPING // NOT FLEXIBLE_CAN_MAPPING
Exit:
#endif // NOT FLEXIBLE_CAN_MAPPING
        return DE_OK;
    }

    //--------------------------------------------------------------------------

    // The following two methods are actually internal - they need to be
    // visible in a non-member function.
    void* threadTxFun();
    void* threadRxFun();

private:

    void incSending();

    // Gets number of unsent commands
    int getNumUnsentCommands() const;

    // Sends a specific CAN command as SYNC configuration to all gateways.
    E_EtherCANErrCode send_sync_command(CAN_Command &can_command,
                                        const int ngateways,
                                        const int buses_per_gateway,
                                        uint8_t msgid_sync_data,
                                        uint8_t msgid_sync_mask);

    // Sends a single gateway message for SYNC configuration, waiting until
    // sending is finished
    E_EtherCANErrCode send_config(int gatway_id,
				  int buf_len,
				  uint8_t bytes[MAX_UNENCODED_GATEWAY_MESSAGE_BYTES],
				  uint8_t const msgid,
				  uint8_t const can_identifier);

    int num_gateways = 0;
    
    int SocketID[MAX_NUM_GATEWAYS]; // Socket descriptor
    int DescriptorCommandEvent;     // eventfd for new command
    int DescriptorCloseEvent;       // eventfd for closing connection

    CommandQueue commandQueue;

    // Sends a buffer (either pending data or new command)
    SBuffer::E_SocketStatus send_buffer(unique_ptr<CAN_Command> &active_can_command,
                                        int gateway_id);

    // Interface method which handles decoded CAN response messages
    virtual void handleFrame(int const gateway_id,
                             const t_CAN_buffer& command_buffer,
                             int const clen);

    void updatePendingCommand(int fpu_id,
                              std::unique_ptr<CAN_Command>& can_command);

    // Read buffer (only to be accessed in reading thread)
    // Write buffer (only to be accessed in writing thread)

    // Two POSIX threads for sending and receiving data
    pthread_t tx_thread = 0;
    pthread_t rx_thread = 0;
    // This atomic flag serves to signal both threads to exit
    std::atomic<bool> exit_threads;
    // This flag informs that a driver shutdown is in progress
    std::atomic<bool> shutdown_in_progress;

    // Buffer class for encoded reads and writes to sockets
    SBuffer sbuffer[MAX_NUM_GATEWAYS];

    // Mapping of FPU IDs to physical addresses (can be made configurable if
    // required)
    FPUArray::t_bus_address_map  address_map;

    // Reverse map of addresses to FPU id - from can bus addresses to fpu id
    FPUArray::t_address_map fpu_id_by_adr; 

    const EtherCANInterfaceConfig config;

    // Member which stores the state of the grid
    FPUArray fpuArray;

    // List of pending time-outs
    TimeOutList timeOutList;

    // Memory pool for unused command objects
    CommandPool command_pool;

#ifdef FLEXIBLE_CAN_MAPPING
    const int fpu_id_broadcast_base = MAX_NUM_POSITIONERS -
                                      (MAX_NUM_GATEWAYS * BUSES_PER_GATEWAY);
#endif // FLEXIBLE_CAN_MAPPING
};

//==============================================================================

// Function to create sockets
int make_socket(const EtherCANInterfaceConfig &config, const char *ip,
                uint16_t port);

// Functions for enabling / disabling real-time scheduling for time-critical
// broadcast commands.
void set_rt_priority(const EtherCANInterfaceConfig &config, int prio);
void unset_rt_priority();

//==============================================================================

// Real-time priority values for threads
const int CONTROL_PRIORITY = 1;
const int WRITER_PRIORITY = 2;
const int READER_PRIORITY = 3;

//==============================================================================

} // namespace ethercanif

} // namespace mpifps

#endif
