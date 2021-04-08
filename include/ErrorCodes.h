// -*- mode: c++ -*-

////////////////////////////////////////////////////////////////////////////////
//
// Copyright 2021 UKRI. See file "LICENSE" for license information.
//
//
// Who       When        What
// --------  ----------  -------------------------------------------------------
// bwillemse 2021-02-10  Created.
// sbeard    2021-02-25  Added DE_INVALID_WAVEFORM_REJECTED return code
//------------------------------------------------------------------------------
  
////////////////////////////////////////////////////////////////////////////////
// NAME ErrorCodes.h
//
// Specifies all grid driver error codes and their groupings.
//
////////////////////////////////////////////////////////////////////////////////

#ifndef ERRORCODES_H
#define ERRORCODES_H

// TODO: InterfaceConstants.h is only included here for the FLEXIBLE_CAN_MAPPING
// macro - remove this #include when the FLEXIBLE_CAN_MAPPING macro is
// eventually removed
#include "InterfaceConstants.h"

namespace mpifps
{

//==============================================================================
// This enum contains return codes which should shed light on the cause if
// something went seriously wrong.
enum E_EtherCANErrCode
{
    //..........................................................................
    // Everything worked
    DE_OK = 0,

    //..........................................................................
    // Non-error return codes

    // The user waited for a command completion using a time-out value, and the
    // state has not been reached yet. This is a "user notification", not an
    // error.
    DE_WAIT_TIMEOUT = 1,

    // Firmware does not implement operation for this protocol version - the
    // calling code might need to check and branch according to the used
    // protocol version
    DE_FIRMWARE_UNIMPLEMENTED = 2,

    //..........................................................................
    // Fatal system failure

    // An initialization command ran out of memory, which prevents successful
    // driver start-up.
    DE_OUT_OF_MEMORY = 10,

    // Some resource from the OS is not available, which leads to an
    // unrecoverable situation.
    DE_RESOURCE_ERROR = 11,

    // A necessary assumption or check for correctness of the driver was
    // violated.
    DE_ASSERTION_FAILED = 12,

    //..........................................................................
    // State errors where requested operations do not match the current system
    // state

    // A command was tried to send, or the driver was instructed to connect,
    // but the driver was not initialized properly.  That can happen if the
    // system goes out of memory, or if a logical error affected the
    // initialization.
    DE_INTERFACE_NOT_INITIALIZED = 101,

    // Driver has already been correctly initialised, and another
    // initialisation was tried.
    DE_INTERFACE_ALREADY_INITIALIZED = 102,

    // The user tried to send a high-level command while another high-level
    // command was still not finished and waited for.
    DE_STILL_BUSY = 103,

    // The user tried to start a movement command while at least one FPU was
    // in collided or aborted state - the command was rejected
    // because of that.
    DE_UNRESOLVED_COLLISION = 104,

    // An FPU has not been initialised, so it cannot be moved accurately and
    // safely.
    DE_FPU_NOT_INITIALIZED = 105,

    // Driver is already initialized.
    DE_INTERFACE_ALREADY_CONNECTED = 106,

    // Driver is still connected.
    DE_INTERFACE_STILL_CONNECTED = 107,

    // Waveform is not configured / not ready for movement.
    DE_WAVEFORM_NOT_READY = 108,

    // The addressed FPUs were not yet calibrated by a datum search.
    DE_FPUS_NOT_CALIBRATED = 109,

    // A motion command was issued but no FPUs are allowed to move.
    DE_NO_MOVABLE_FPUS = 110,

    // Command not allowed for present FPU state.
    DE_INVALID_FPU_STATE = 111,

    // The operation can damage hardware and protection is enabled
    DE_PROTECTION_ERROR = 112,

    // The driver state does not allows the operation
    DE_INVALID_INTERFACE_STATE = 113,

    // Some addressed FPUs are locked.
    DE_FPUS_LOCKED = 114,

    // A previous movement was aborted.
    DE_IN_ABORTED_STATE = 115,

    // An alpha arm is on the limit switch, and cannot be datumed.
    DE_ALPHA_ARM_ON_LIMIT_SWITCH = 116,

    //..........................................................................
    // Setup errors

    // Insufficient number of gateways for requested number of FPUs.
    DE_INSUFFICENT_NUM_GATEWAYS = 201,

    // Configuration parameters invalid, see log message.
    DE_INVALID_CONFIG = 202,
    
    // Sending SYNC configuration failed
    DE_SYNC_CONFIG_FAILED = 203,

    // A write to an FPU did not read back the same value
    DE_WRITE_VERIFICATION_FAILED = 204,

#ifdef FLEXIBLE_CAN_MAPPING
    // No FPUs are defined
    DE_NO_FPUS_DEFINED = 205,
#endif // FLEXIBLE_CAN_MAPPING

    //..........................................................................
    // Invalid parameter values

    // An FPU ID is invalid
    DE_INVALID_FPU_ID = 301,

    // Passed parameter value is invalid
    DE_INVALID_PAR_VALUE = 302,

    // Duplicate serial number
    DE_DUPLICATE_SERIAL_NUMBER = 303,

#ifdef FLEXIBLE_CAN_MAPPING
    // Invalid gateway ID
    DE_INVALID_GATEWAY_ID = 304,

    // Invalid CAN bus ID
    DE_INVALID_CAN_BUS_ID = 305,

    // Invalid CAN ID
    DE_INVALID_CAN_ID = 306,

    // Invalid number of parameters
    DE_INVALID_NUM_PARAMS = 307,

    // Duplicate FPU ID
    DE_DUPLICATE_FPU_ID = 308,

    // Duplicate CAN route
    DE_DUPLICATE_CAN_ROUTE = 309,
#endif // FLEXIBLE_CAN_MAPPING

    // No waveform(s) defined for specified FPU(s)
    DE_NO_WAVEFORMS = 310,

    //..........................................................................
    // Connection failures

    // The maximum retry count was exceeded for command.
    DE_MAX_RETRIES_EXCEEDED = 401,

    // A CAN command to an FPU surpassed the maximum waiting time for a response.
    // This can indicate either a connection problem, a failure of the FPU
    // controller, or a failure of the FPU hardware.
    DE_CAN_COMMAND_TIMEOUT_ERROR = 402,

    // A command was tried to send to the FPUs but this was not possible because
    // the driver was or became disconnected from a gateway. During operation,
    // this should only happen when the socket connection breaks down for an
    // extended time, as the socket protocol will try hard to do re-sends for
    // several minutes.  Before this error happens, one will probably see
    // time-outs on every single FPU command to the corresponding gateways as
    // they all fail to respond.
    DE_NO_CONNECTION = 403,

    // A CAN buffer overflow warning was received, meaning that more commands
    // were sent at once than the FPU firmware and CAN implementation were able
    // to process. This is similar to a COMMAND_TIMEOUT_ERROR, except that we
    // know that the last message wasn't processed.
    DE_FIRMWARE_CAN_BUFFER_OVERFLOW = 404,

    //..........................................................................
    // Invalid waveforms

    // General error in waveform definition, see text.
    // Also: We tried to move FPUs but some addressed FPUs still have invalid
    // waveforms.
    DE_INVALID_WAVEFORM = 500,

    // Waveform has to many steps
    DE_INVALID_WAVEFORM_TOO_MANY_SECTIONS = 501,

    // Number of sections different for different FPUS - this isn't allowed to 
    // avoid collisions.
    DE_INVALID_WAVEFORM_RAGGED = 502,

    // Step number in section is too high for current firmware.
    DE_INVALID_WAVEFORM_STEPCOUNT_TOO_LARGE = 503,

    // The change in step count per section is incorrect (e.g. too large)
    DE_INVALID_WAVEFORM_CHANGE = 504,

    // The tail of the waveform is incorrect.
    DE_INVALID_WAVEFORM_TAIL = 505,

    // A waveform is rejected by the interface so loading state not achieved.
    DE_INVALID_WAVEFORM_REJECTED = 506,

    //..........................................................................
    // Errors which terminate movements

    // Collision error
    // A collision occured, and the operation was aborted.
    DE_NEW_COLLISION = 601,

    // Limit breach
    // An alpha limit breach occured, and the operation was aborted.
    DE_NEW_LIMIT_BREACH = 602,

    // At least one FPU ran into a step timing error, which means the FPU's
    // motion controller was not able to compute the required step frequency
    // quick enough for the configured microstepping level.
    DE_STEP_TIMING_ERROR = 603,

    // Abort message - the movement has just been aborted.
    DE_MOVEMENT_ABORTED = 604,

    // Datum rejected: alpha arm on limit switch. The datum command was rejected.
    DE_HW_ALPHA_ARM_ON_LIMIT_SWITCH = 605,

    // Datum time-out - the datum command has timed out on the FPU.
    DE_DATUM_COMMAND_HW_TIMEOUT = 606,

    // The driver received an illegal counter value from an FPU, so that it
    // cannot correctly track the FPUs any more. It is required to measure the
    // position and update the position database.
    DE_INCONSISTENT_STEP_COUNT = 607,

    //..........................................................................
    // Database errors

    // Database opening failures
    DE_DB_ENV_VARIABLE_NOT_FOUND = 701,
    DE_DB_DIR_OR_FILE_NOT_FOUND = 702,
    DE_DB_ACCESS_DENIED = 703,
    DE_DB_OLD_FORMAT = 704,
    DE_DB_OTHER_OPENING_FAILURE = 705,

    // Transaction creation failed
    DE_DB_TRANSACTION_CREATION_FAILED = 706,

    // No database FPU entry corresponding to a physical FPU, or there was a
    // read error of some sort
    DE_DB_MISSING_FPU_ENTRY_OR_READ_FAILED = 707,

    // Writing of a database FPU item failed
    DE_DB_WRITE_FAILED = 708,

    // Database synchronisation operation failed
    DE_DB_SYNC_FAILED = 709,

    //..........................................................................
    // Unknown error - use for e.g. initialising return values at the
    // beginnings of functions to catch if return value is not set
    // properly
    DE_ERROR_UNKNOWN = 9999,

    //..........................................................................
};

// Define error groups enum
enum class EtherCANErrorGroup
{
    InvalidState,
    Protection,
    SystemFailure,
    Setup,
    InvalidParameter,
    ConnectionFailure,
    SocketFailure,
    CommandTimeout,
    CANOverflow,
    InvalidWaveform,
    Collision,
    LimitBreach,
    Timing,
    AbortMotion,
    FirmwareTimeout,
    HardwareProtection,
    Database,
    General
};

//==============================================================================

EtherCANErrorGroup errorGroup(E_EtherCANErrCode ecan_result);

//==============================================================================

} // namespace mpifps

#endif // ERRORCODES_H
