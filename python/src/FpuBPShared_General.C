// -*- mode: c++ -*-

////////////////////////////////////////////////////////////////////////////////
//
// Copyright 2020 UKRI. See file "LICENSE" for license information.
//
//
// Who       When        What
// --------  ----------  -------------------------------------------------------
// bwillemse 2020-06-25  TODO: Put comment here
//------------------------------------------------------------------------------

////////////////////////////////////////////////////////////////////////////////
// NAME FpuBPShared_General.C
//
// TODO: Put description here
//
////////////////////////////////////////////////////////////////////////////////

#include "FpuBPShared_General.h"


// -----------------------------------------------------------------------------
std::ostringstream& operator<<(std::ostringstream &out, const E_FPU_STATE &s)
{
    switch(s)
    {
    case FPST_UNKNOWN:
        out << "'FPST_UNKNOWN'";
        break;
    case FPST_UNINITIALIZED:
        out << "'FPST_UNINITIALIZED'";
        break;
    case FPST_LOCKED:
        out << "'FPST_LOCKED'";
        break;
    case FPST_DATUM_SEARCH:
        out << "'FPST_DATUM_SEARCH'";
        break;
    case FPST_AT_DATUM:
        out << "'FPST_AT_DATUM'";
        break;
    case FPST_LOADING:
        out << "'FPST_LOADING'";
        break;
    case FPST_READY_FORWARD:
        out << "'FPST_READY_FORWARD'";
        break;
    case FPST_READY_REVERSE:
        out << "'FPST_READY_REVERSE'";
        break;
    case FPST_MOVING:
        out << "'FPST_MOVING'";
        break;
    case FPST_RESTING:
        out << "'FPST_RESTING'";
        break;
    case FPST_ABORTED:
        out << "'FPST_ABORTED'";
        break;
    case FPST_OBSTACLE_ERROR:
        out << "'FPST_OBSTACLE_ERROR'";
        break;
    }
    return out;
}


// -----------------------------------------------------------------------------
std::ostringstream& operator<<(std::ostringstream &out, const E_InterfaceState &s)
{
    switch(s)
    {
    case DS_UNINITIALIZED:
        out << "'DS_UNINITIALIZED'";
        break;
    case DS_UNCONNECTED:
        out << "'DS_UNCONNECTED'";
        break;
    case DS_CONNECTED:
        out << "'DS_CONNECTED'";
        break;
    case DS_ASSERTION_FAILED:
        out << "'DS_ASSERTION_FAILED'";
        break;
    }
    return out;
}

// -----------------------------------------------------------------------------
void checkInterfaceError(E_EtherCANErrCode ecode)
{
    switch(ecode)
    {
    case DE_OK:
        break;

    case DE_INTERFACE_NOT_INITIALIZED:
        throw EtherCANException("DE_INTERFACE_NOT_INITIALIZED: EtherCANInterface was not initialized "
                                "properly, possibly due to system error or out-of-memory condition.",
                                DE_INTERFACE_NOT_INITIALIZED);
        break;

    case DE_INTERFACE_ALREADY_INITIALIZED:
        throw EtherCANException("DE_INTERFACE_ALREADY_INITIALIZED: EtherCANInterface was already initialized properly.",
                                DE_INTERFACE_ALREADY_INITIALIZED);
        break;

    case DE_NO_CONNECTION :
        throw EtherCANException("DE_NO_CONNECTION: The EtherCAN Interface is not connected to a gateway.",
                                DE_NO_CONNECTION);
        break;

    case DE_CAN_COMMAND_TIMEOUT_ERROR:
        throw EtherCANException("DE_CAN_COMMAND_TIMEOUT_ERROR:"
                                " A CAN command to an FPU surpassed the maximum waiting time"
                                " determined by the CAN protocol."
                                " This likely indicates a failure of the controller or a"
                                " serious connection problem.",
                                DE_CAN_COMMAND_TIMEOUT_ERROR);
        break;

    case DE_FIRMWARE_CAN_BUFFER_OVERFLOW:
        throw EtherCANException("DE_FIRMWARE_CAN_BUFFER_OVERFLOW:"
                                " A CAN command to an FPU could not be processed and was lost"
                                " because the FPU firmware buffer was full.",
                                DE_FIRMWARE_CAN_BUFFER_OVERFLOW);
        break;

    case DE_INSUFFICENT_NUM_GATEWAYS:
        throw EtherCANException("DE_INSUFFICENT_NUM_GATEWAYS: The number of EtherCAN gateways"
                                " configured is insufficient for the configured number of FPUs",
                                DE_INSUFFICENT_NUM_GATEWAYS);
        break;

    case DE_STILL_BUSY:
        throw EtherCANException("DE_STILL_BUSY: The EtherCAN interface is still busy "
                                "working on a previosu command",
                                DE_STILL_BUSY);
        break;

    case DE_NEW_COLLISION :
        throw EtherCANException("DE_NEW_COLLISION: A collision was detected,"
                                " movement for this FPU aborted.",
                                DE_NEW_COLLISION);
        break;

    case DE_NEW_LIMIT_BREACH :
        throw EtherCANException("DE_NEW_LIMIT_BREACH: An alpha limit breach was detected,"
                                " movement for this FPU aborted.",
                                DE_NEW_LIMIT_BREACH);
        break;

    case DE_UNRESOLVED_COLLISION :
        throw EtherCANException("DE_UNRESOLVED_COLLISION: A previous collision, limit breach,"
                                " or abort message needs to be resolved first",
                                DE_UNRESOLVED_COLLISION);
        break;

    case DE_FPU_NOT_INITIALIZED:
        throw EtherCANException("DE_FPU_NOT_INITIALIZED: A fibre positioner unit (FPU) was not initialized as"
                                " required, needs to do a datum search first",
                                DE_FPU_NOT_INITIALIZED);
        break;

    case DE_INTERFACE_ALREADY_CONNECTED :
        throw EtherCANException("DE_INTERFACE_ALREADY_CONNECTED: EtherCAN Interface was already connected,"
                                " would need to disconnect() first.",
                                DE_INTERFACE_ALREADY_CONNECTED);
        break;

    case DE_INTERFACE_STILL_CONNECTED:
        throw EtherCANException("DE_INTERFACE_STILL_CONNECTED: EtherCAN interface is still connected",
                                DE_INTERFACE_STILL_CONNECTED);
        break;

    case DE_MAX_RETRIES_EXCEEDED :
        throw EtherCANException("DE_MAX_RETRIES_EXCEEDED: A command could not be"
                                " send in spite of several retries", DE_MAX_RETRIES_EXCEEDED);
        break;

    case DE_INVALID_WAVEFORM :
        throw EtherCANException("DE_INVALID_WAVEFORM: The passed waveform does not meet some general rule.",
                                DE_INVALID_WAVEFORM);
        break;

    case DE_INVALID_WAVEFORM_TOO_MANY_SECTIONS :
        throw EtherCANException("DE_INVALID_WAVEFORM_TOO_MANY_SECTIONS: The passed waveform has too many sections.",
                                DE_INVALID_WAVEFORM_TOO_MANY_SECTIONS);
        break;

    case DE_INVALID_WAVEFORM_RAGGED :
        throw EtherCANException("DE_INVALID_WAVEFORM_RAGGED: The passed waveform has different number of sections for different FPUs.",
                                DE_INVALID_WAVEFORM_RAGGED);
        break;

    case DE_INVALID_WAVEFORM_STEPCOUNT_TOO_LARGE :
        throw EtherCANException("DE_INVALID_WAVEFORM_STEP_COUNT_TOO_LARGE:"
                                " The passed waveform has a section with too many steps.",
                                DE_INVALID_WAVEFORM_STEPCOUNT_TOO_LARGE);
        break;

    case DE_INVALID_WAVEFORM_CHANGE :
        throw EtherCANException("DE_INVALID_WAVEFORM_CHANGE: The passed waveform has an"
                                " invalid change in step counts / speed between adjacent sections",
                                DE_INVALID_WAVEFORM_CHANGE);
        break;

    case DE_INVALID_WAVEFORM_TAIL :
        throw EtherCANException("DE_INVALID_WAVEFORM_TAIL: The passed waveform has an invalid tail section.",
                                DE_INVALID_WAVEFORM_TAIL);
        break;

    case DE_WAVEFORM_NOT_READY :
        throw EtherCANException("DE_WAVEFORM_NOT_READY: The FPU has no valid waveform configured for a movement.",
                                DE_WAVEFORM_NOT_READY);
        break;

    case DE_FPUS_NOT_CALIBRATED:
        throw EtherCANException("DE_FPUS_NOT_CALIBRATED: FPUs are lacking calibration by "
                                "a findDatum operation. For engineering or recovery use, consider"
                                " to set the 'allow_uninitialized' keyword argument to True",
                                DE_FPUS_NOT_CALIBRATED);
        break;

    case DE_NO_MOVABLE_FPUS :
        throw EtherCANException("DE_NO_MOVABLE_FPUS: No FPUs are currently movable.",
                                DE_NO_MOVABLE_FPUS);
        break;

    case DE_WAIT_TIMEOUT :
        throw EtherCANException("DE_WAIT_TIMEOUT: Response to a EtherCAN interface command surpassed the"
                                " waiting time parameter passed to waitForState(),"
                                " which caused the user command to return unfinished."
                                " (This is usually not an error.)",
                                DE_WAIT_TIMEOUT);
        break;

    case DE_IN_ABORTED_STATE :
        throw EtherCANException("DE_IN_ABORTED_STATE: There are FPUs in aborted state,"
                                " because of a previous abortMotion command or a step timing error"
                                "- use the enableMove (or resetFPUs) command to reset state.",
                                DE_IN_ABORTED_STATE);
        break;

    case DE_MOVEMENT_ABORTED :
        throw EtherCANException("DE_MOVEMENT_ABORTED: The FPU has entered the FPST_ABORTED state,"
                                " because of an abortMotion command or a step timing error "
                                "- use the enableMove (or resetFPUs) command to reset state.",
                                DE_MOVEMENT_ABORTED);
        break;

    case DE_DATUM_COMMAND_HW_TIMEOUT :
        throw EtherCANException("DE_DATUM_COMMAND_HW_TIMEOUT: The FPU firmware has timed-out"
                                " a datum operation because it took too long to complete. Potentially,"
                                " the datum switch is not working, or the FPU hardware is otherwise"
                                " damaged. It can also be that the datum command was just issued when"
				" the FPU was too far away from the datum switch.",
                                DE_DATUM_COMMAND_HW_TIMEOUT);
        break;

    case DE_ALPHA_ARM_ON_LIMIT_SWITCH:
        throw EtherCANException("DE_ALPHA_ARM_ON_LIMIT_SWITCH: Datum command rejected because"
                                " an FPU alpha arm is on its limit switch.",
                                DE_ALPHA_ARM_ON_LIMIT_SWITCH);
        break;

    case DE_HW_ALPHA_ARM_ON_LIMIT_SWITCH:
        throw EtherCANException("DE_HW_ALPHA_ARM_ON_LIMIT_SWITCH: Part of datum command rejected by"
                                " hardware because an FPU alpha arm is on its limit switch"
                                " before it started to move.",
                                DE_HW_ALPHA_ARM_ON_LIMIT_SWITCH);
        break;

    case DE_INCONSISTENT_STEP_COUNT:
        throw EtherCANException("The EtherCAN interface received an illegal counter value from"
                                " an FPU, so that it cannot correctly track the FPUs"
                                " any more. It is required to measure the"
                                " position and update the position database.",
                                DE_INCONSISTENT_STEP_COUNT);
        break;


    case DE_FPUS_LOCKED :
        throw EtherCANException("DE_FPUS_LOCKED: Some addressed FPUs are in locked state,"
                                " they need to be unlocked first.",
                                DE_FPUS_LOCKED);
        break;

    case DE_STEP_TIMING_ERROR:
        throw EtherCANException("DE_STEP_TIMING_ERROR: An FPU's controller"
                                " generated a step timing error"
                                " during movement. Possibly, reduce the microstepping level"
                                " to compute the step frequency in time.",
                                DE_STEP_TIMING_ERROR);
        break;


    case DE_INVALID_FPU_ID:
        throw EtherCANException("DE_INVALID_FPU_ID: A passed FPU id is out of range.",
                                DE_INVALID_FPU_ID);
        break;

    case DE_INVALID_FPU_STATE:
        throw EtherCANException("DE_INVALID_FPU_STATE: Command not allowed for present FPU state.",
                                DE_INVALID_FPU_STATE);
        break;

    case DE_PROTECTION_ERROR:
        throw EtherCANException("DE_PROTECTION_ERROR: Command might damage FPU, step count protection is enabled.",
                                DE_PROTECTION_ERROR);
        break;

    case DE_INVALID_PAR_VALUE:
        throw EtherCANException("DE_INVALID_PAR_VALUE: The passed parameter value is invalid.",
                                DE_INVALID_PAR_VALUE);
        break;

    case DE_DUPLICATE_SERIAL_NUMBER:
        throw EtherCANException("DE_DUPLICATE_SERIAL_NUMBER: The passed serial number is already in use.",
                                DE_DUPLICATE_SERIAL_NUMBER);
        break;

    case DE_FIRMWARE_UNIMPLEMENTED:
        throw EtherCANException("DE_FIRMWARE_UNIMPLEMENTED: Command or operation not implemented"
                                " for this protocol version",
                                DE_FIRMWARE_UNIMPLEMENTED);
        break;

    case DE_RESOURCE_ERROR:
        throw EtherCANException("DE_RESOURCE_ERROR: The EtherCAN interface could not acquire necessary"
                                " resources such as file descriptors from the OS, and can not operate.",
                                DE_RESOURCE_ERROR);
        break;

    case DE_OUT_OF_MEMORY:
        throw EtherCANException("DE_OUT_OF_MEMORY: The EtherCAN interface could not allocate the required memory, "
                                "and can not operate. Probable cause is a memory leak.",
                                DE_OUT_OF_MEMORY);
        break;

    case DE_INVALID_INTERFACE_STATE :
        throw EtherCANException("DE_INVALID_INTERFACE_STATE: The current state of the EtherCAN interface"
                                " does not allow the requested operation.",
                                DE_INVALID_INTERFACE_STATE);
        break;

    case DE_INVALID_CONFIG:
        throw EtherCANException("DE_INVALID_CONFIG: The EtherCAN interface configuration is not valid",
                                DE_INVALID_CONFIG);
        break;

    case DE_SYNC_CONFIG_FAILED:
        throw EtherCANException("DE_SYNC_CONFIG_FAILED: Sending the SYNC configuration to the gateways failed",
                                DE_SYNC_CONFIG_FAILED);
        break;

    case DE_ASSERTION_FAILED:
        throw EtherCANException("DE_ASSERTION_FAILED: The EtherCAN interface determined an internal logic error, "
                                "should probably be terminated.",
                                DE_ASSERTION_FAILED);
        break;

    case DE_ERROR_UNKNOWN:
        throw EtherCANException("DE_ERROR_UNKNOWN: An unknown error occurred, "
                                "should probably be terminated.",
                                DE_ERROR_UNKNOWN);
    }
}


// -----------------------------------------------------------------------------
