// -*- mode: c++ -*-

////////////////////////////////////////////////////////////////////////////////
//
// Copyright 2021 UKRI. See file "LICENSE" for license information.
//
//
// Who       When        What
// --------  ----------  -------------------------------------------------------
// bwillemse 2021-02-10  Created.
//------------------------------------------------------------------------------

////////////////////////////////////////////////////////////////////////////////
// NAME ErrorCodes.C
//
// Specifies all grid driver error codes and their groupings.
//
////////////////////////////////////////////////////////////////////////////////

#include "ErrorCodes.h"


namespace mpifps
{

//------------------------------------------------------------------------------
EtherCANErrorGroup errorGroup(E_EtherCANErrCode ecan_result)
{
    // Returns the error group for the specified error code.
    // NOTE: These groups do NOT exactly correspond to the E_EtherCANErrCode
    // value ranges starting from 100, 200 etc - this is because they were
    // adapted directly from the original translate_interface_error() function,
    // which did not exactly correspond to those groupings.

    switch (ecan_result)
    {
    case DE_INTERFACE_NOT_INITIALIZED:
    case DE_INTERFACE_ALREADY_INITIALIZED:
    case DE_STILL_BUSY:
    case DE_UNRESOLVED_COLLISION:
    case DE_FPU_NOT_INITIALIZED:
    case DE_INTERFACE_ALREADY_CONNECTED:
    case DE_INTERFACE_STILL_CONNECTED:
    case DE_WAVEFORM_NOT_READY:
    case DE_FPUS_NOT_CALIBRATED:
    case DE_NO_MOVABLE_FPUS:
    case DE_FPUS_LOCKED:
    case DE_INVALID_FPU_STATE:
    case DE_INVALID_INTERFACE_STATE:
    case DE_IN_ABORTED_STATE:
    case DE_ALPHA_ARM_ON_LIMIT_SWITCH:
        return EtherCANErrorGroup::InvalidState;
        break;

    case DE_PROTECTION_ERROR:
        return EtherCANErrorGroup::Protection;
        break;

    case DE_OUT_OF_MEMORY:
    case DE_RESOURCE_ERROR:
    case DE_ASSERTION_FAILED:
    case DE_ERROR_UNKNOWN:
        return EtherCANErrorGroup::SystemFailure;
        break;

    case DE_FIRMWARE_UNIMPLEMENTED:
    case DE_INSUFFICENT_NUM_GATEWAYS:
    case DE_INVALID_CONFIG:
    case DE_SYNC_CONFIG_FAILED:
    case DE_WRITE_VERIFICATION_FAILED:
#ifdef FLEXIBLE_CAN_MAPPING
    case DE_NO_FPUS_DEFINED:
#endif // FLEXIBLE_CAN_MAPPING
        return EtherCANErrorGroup::Setup;
        break;

    case DE_INVALID_FPU_ID:
    case DE_INVALID_PAR_VALUE:
    case DE_DUPLICATE_SERIAL_NUMBER:
#ifdef FLEXIBLE_CAN_MAPPING
    case DE_INVALID_GATEWAY_ID:
    case DE_INVALID_CAN_BUS_ID:
    case DE_INVALID_CAN_ID:
    case DE_INVALID_NUM_PARAMS:
    case DE_DUPLICATE_FPU_ID:
    case DE_DUPLICATE_CAN_ROUTE:
#endif // FLEXIBLE_CAN_MAPPING
        return EtherCANErrorGroup::InvalidParameter;
        break;

    case DE_WAIT_TIMEOUT:
        // This is normally not raised, because not necessarily an error
        return EtherCANErrorGroup::ConnectionFailure;
        break;

    case DE_NO_CONNECTION:
        return EtherCANErrorGroup::SocketFailure;
        break;

    case DE_MAX_RETRIES_EXCEEDED:
    case DE_CAN_COMMAND_TIMEOUT_ERROR:
        return EtherCANErrorGroup::CommandTimeout;
        break;

    case DE_FIRMWARE_CAN_BUFFER_OVERFLOW:
        return EtherCANErrorGroup::CANOverflow;
        break;

    case DE_INVALID_WAVEFORM:
    case DE_INVALID_WAVEFORM_TAIL:
    case DE_INVALID_WAVEFORM_TOO_MANY_SECTIONS:
    case DE_INVALID_WAVEFORM_RAGGED:
    case DE_INVALID_WAVEFORM_STEPCOUNT_TOO_LARGE:
    case DE_INVALID_WAVEFORM_CHANGE:
    case DE_INVALID_WAVEFORM_REJECTED:
        return EtherCANErrorGroup::InvalidWaveform;
        break;

    case DE_NEW_COLLISION:
        return EtherCANErrorGroup::Collision;
        break;

    case DE_NEW_LIMIT_BREACH:
        return EtherCANErrorGroup::LimitBreach;
        break;

    case DE_STEP_TIMING_ERROR:
        return EtherCANErrorGroup::Timing;
        break;

    case DE_MOVEMENT_ABORTED:
        return EtherCANErrorGroup::AbortMotion;
        break;

    case DE_DATUM_COMMAND_HW_TIMEOUT:
        return EtherCANErrorGroup::FirmwareTimeout;
        break;

    case DE_HW_ALPHA_ARM_ON_LIMIT_SWITCH:
    case DE_INCONSISTENT_STEP_COUNT:
        return EtherCANErrorGroup::HardwareProtection;
        break;

    case DE_DB_ENV_VARIABLE_NOT_FOUND:
    case DE_DB_DIR_OR_FILE_NOT_FOUND:
    case DE_DB_ACCESS_DENIED:
    case DE_DB_OLD_FORMAT:
    case DE_DB_OTHER_OPENING_FAILURE:
    case DE_DB_TRANSACTION_CREATION_FAILED:
    case DE_DB_MISSING_FPU_ENTRY_OR_READ_FAILED:
    case DE_DB_WRITE_FAILED:
    case DE_DB_SYNC_FAILED:
        return EtherCANErrorGroup::Database;
        break;

    default:
        return EtherCANErrorGroup::General;
    }
}

//------------------------------------------------------------------------------

} // namespace mpifps

