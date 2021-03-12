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

#include "SBuffer.h"          // coding and decoding message frames
#include "I_ResponseHandler.h"  // interface for processing CAN responses
#include "FPUArray.h" // defines thread-safe structure of FPU state info
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

//**********************************
// Macro for enabling the new flexible grid CAN mapping functionality
#define FLEXIBLE_CAN_MAPPING
//**********************************

#ifdef FLEXIBLE_CAN_MAPPING

using GridCanMap = std::vector<std::pair<int, FPUArray::t_bus_address>>;

#endif // FLEXIBLE_CAN_MAPPING

//==============================================================================
class GatewayInterface: private I_ResponseHandler
{
public:

    // timeout for reading from command FIFO if nothing is
    // pending - 500 ms
    const struct timespec COMMAND_WAIT_TIME = { /* .tv_sec = */ 0,
              /* .tv_nsec = */ 10000000
    };
    // timeout for polling write socket - 500 ms
    const struct timespec MAX_TX_TIMEOUT = { /* .tv_sec = */ 0,
              /* .tv_nsec = */ 500000000
    };

    // default timeout for polling read socket - 500 msec
    const timespec MAX_RX_TIMEOUT = { /* .tv_sec = */ 10,
                                      /* .tv_nsec = */ 500000000
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

    E_EtherCANErrCode connect(const int ngateways, const t_gateway_address gateway_addresses[]);

    // disconnect socket, and re-add any pending commands to
    // the command queue. (If pending commands should be
    // discarded, the command queue needs to be explicitly
    // flushed).
    E_EtherCANErrCode disconnect();

    // get the current state of the FPU grid, which
    // is stored in the reference parameter
    E_GridState getGridState(t_grid_state& out_state) const;

    // get the current state of the driver (this is a convenience
    // function, the state is contained in the grid state).
    E_InterfaceState getInterfaceState() const;

    // get both the summed up state of the FPU grid,
    // and a detailed status for each FPU.
    E_GridState waitForState(E_WaitTarget target, t_grid_state& out_detailed_state,
                             double &max_wait_time, bool &cancelled) const;

    // provide a command instance with buffer space for
    // sending CAN parameters. This method is thread-safe

    template <typename T>
    inline unique_ptr<T> provideInstance()
    {
        return command_pool.provideInstance<T>();
    }

    void updatePendingSets(unique_ptr<CAN_Command> &active_can_command,
                           int gateway_id, int busid);

    // send a CAN command to the gateway.
    // This method is thread-safe
    CommandQueue::E_QueueState sendCommand(const int fpu_id, unique_ptr<CAN_Command>& new_command);

    // returns id which needs to be set as fpu id for broadcast command
    int getBroadcastID(const int gateway_id, const int busid);

#if 0
    // returns gateway ID for an FPU
    int getGatewayIdByFPUID(int fpu_id) const;
#endif

    // returns whether an FPU is currently marked as locked.
    bool isLocked(int fpu_id) const;

    // Send an abortMotion broadcast command to all gateways.
    //
    // (This is implemented at the CAN driver level because we need
    // the Rx thread to be able to trigger an automatic abort if too
    // many collisions happen in a short time span.).
    E_EtherCANErrCode abortMotion(t_grid_state& grid_state,
                                  E_GridState& state_summary,
				                  bool sync_message=true);

    template<typename T> E_EtherCANErrCode broadcastMessage(bool sync_message=false)
    {

	if (sync_message && (T::sync_code != SYNC_NOSYNC)){
	    // a SYNC message is only sent once to a single gateway,
	    // the 'master' gateway. This gateway will sent the
	    // message as broadcast messages to all buses, and forward
	    // it to the other gateways.
	    //
	    // Electronics will ensure that all gateways broadcast the
	    // message in a synchronized way (thus the name).
	    unique_ptr<SyncCommand> sync_command;

	    sync_command = provideInstance<SyncCommand>();

	    if (sync_command == nullptr)
	    {
		return DE_ASSERTION_FAILED;
	    }
	    const int broadcast_id = 0; // gateway number zero is SYNC master

	    // broadcast_id is an fpu id which makes sure
	    // the message goes to the requested bus.
	    sync_command->parametrize(T::sync_code);
	    unique_ptr<CAN_Command> cmd(sync_command.release());
	    sendCommand(broadcast_id, cmd);
	}
	else
	{
	    unique_ptr<T> can_command;

	    for (int gateway_id=0; gateway_id < num_gateways; gateway_id++)
	    {
		for(int busid=0; busid < BUSES_PER_GATEWAY; busid++)
		{
		    const int broadcast_id = getBroadcastID(gateway_id, busid);
		    if (broadcast_id >= config.num_fpus)
		    {
			goto Exit;
		    }
 		    can_command = provideInstance<T>();

		    if (can_command == nullptr)
		    {
			return DE_ASSERTION_FAILED;
		    }
		    const bool do_broadcast = true;
		    // broadcast_id is an fpu id which makes sure
		    // the message goes to the requested bus.
		    can_command->parametrize(broadcast_id, do_broadcast);
		    unique_ptr<CAN_Command> cmd(can_command.release());
		    sendCommand(broadcast_id, cmd);
		}
	    }
	}
Exit:
        return DE_OK;
    }

    // the following two methods are actually internal -
    // they need to be visible in a non-member function.
    void* threadTxFun();
    void* threadRxFun();

private:

    void incSending();

    // get number of unsent commands
    int getNumUnsentCommands() const;

    // send a specific CAN command as SYNC configuration
    // to all gateways.
    E_EtherCANErrCode send_sync_command(CAN_Command &can_command,
					const int ngateways,
					const int buses_per_gateway,
					uint8_t msgid_sync_data,
					uint8_t msgid_sync_mask);

    // send a single gateway message for SYNC configuration,
    // waiting until sending is finished
    E_EtherCANErrCode send_config(int gatway_id,
				  int buf_len,
				  uint8_t bytes[MAX_UNENCODED_GATEWAY_MESSAGE_BYTES],
				  uint8_t const msgid,
				  uint8_t const can_identifier);

    int num_gateways = 0;
    // socket descriptor
    int SocketID[MAX_NUM_GATEWAYS];
    int DescriptorCommandEvent; // eventfd for new command
    int DescriptorCloseEvent;  // eventfd for closing connection

    CommandQueue commandQueue;

    // send a buffer (either pending data or new command)
    SBuffer::E_SocketStatus send_buffer(unique_ptr<CAN_Command> &active_can_command,
                                        int gateway_id);

    // interface method which handles decoded CAN response messages
    virtual void handleFrame(int const gateway_id, const t_CAN_buffer& command_buffer, int const clen);

    void updatePendingCommand(int fpu_id,
                              std::unique_ptr<CAN_Command>& can_command);

    // read buffer (only to be accessed in reading thread)
    // write buffer (only to be accessed in writing thread)

    // two POSIX threads for sending and receiving data
    pthread_t    tx_thread;
    pthread_t    rx_thread;
    // this atomic flag serves to signal both threads to exit
    std::atomic<bool> exit_threads;
    // this flag informs that a driver shutdown is in progress
    std::atomic<bool> shutdown_in_progress;

    // buffer class for encoded reads and writes to sockets
    SBuffer sbuffer[MAX_NUM_GATEWAYS];

    // mapping of FPU IDs to physical addresses.
    // (can be made configurable if required)
    FPUArray::t_bus_address_map  address_map;

    // reverse map of addresses to FPU id.
    FPUArray::t_address_map fpu_id_by_adr; // address map from can bus addresses to fpu id

    const EtherCANInterfaceConfig config;

    FPUArray fpuArray;        // member which stores the state of the grid

    TimeOutList timeOutList; // list of pending time-outs

    CommandPool command_pool; // memory pool for unused command objects
};

//==============================================================================

// function to create sockets
int make_socket(const EtherCANInterfaceConfig &config, const char *ip, uint16_t port);

// functions for enabling / disabling real-time scheduling
// for time-critical broadcast commands.
void set_rt_priority(const EtherCANInterfaceConfig &config, int prio);
void unset_rt_priority();

//==============================================================================

// real-time priority values for threads
const int CONTROL_PRIORITY = 1;
const int WRITER_PRIORITY = 2;
const int READER_PRIORITY = 3;

//==============================================================================

} // namespace ethercanif

} // namespace mpifps

#endif
