// -*- mode: c++ -*-

////////////////////////////////////////////////////////////////////////////////
//
// Copyright 2020 UKRI. See file "LICENSE" for license information.
//
//
// Who       When        What
// --------  ----------  -------------------------------------------------------
// bwillemse 2020-05-13  Separated this shared functionality out from
//                       original EtherCAN wrapper code.
//------------------------------------------------------------------------------

////////////////////////////////////////////////////////////////////////////////
// NAME WrapperSharedBase.C
//
// Grid driver Boost.Python wrapper functionality shared between the wrapped
// classes.
//
////////////////////////////////////////////////////////////////////////////////

#include "WrapperSharedBase.h"


//==============================================================================

// TODO: BW Comment: the following #pragma's (and the "#pragma GCC diagnostic pop"
// after the end of the function) suppress the following obscure error which
// happened when building ethercanif using "make wrapper":
//     "In member function ‘int WrapperSharedBase::convertGatewayAddresses(...)’:
//      python/src/WrapperSharedBase.C:28:5: error: assuming signed overflow
//      does not occur when changing X +- C1 cmp C2 to X cmp C2 -+ C1 [-Werror=strict-overflow]"
// This error is caused because the Makefile has the compiler option
// "-Wstrict-overflow=4" enabled (see CXXFLAGS), which apparently (see Google)
// causes the optimiser to flag obscure optimisation risks in highly
// code-specific conditions.
// TODO: Do we need the "-Wstrict-overflow=4" option enabled in the Makefile
// (e.g. has it caught known issues in the past), or does it just create these
// unneccessarily-pendantic error messages which need to be suppressed with
// these messy #pragma's?
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wstrict-overflow"

//------------------------------------------------------------------------------
int WrapperSharedBase::convertGatewayAddresses(const bp::list &list_gateway_addresses,
                                    t_gateway_address *address_array_to_fill)
{
    // N.B. Upon entry, address_array_to_fill[] must have been defined as an
    // array of size MAX_NUM_GATEWAYS, e.g.:
    //       t_gateway_address address_array[MAX_NUM_GATEWAYS];
    //
    // Returns the number of gateway addresses in list_gateway_addresses.

    const int actual_num_gw = bp::len(list_gateway_addresses);

    if (actual_num_gw > MAX_NUM_GATEWAYS)
    {
        throw EtherCANException("Number of EtherCAN gateways exceed EtherCAN interface limit",
                                DE_INVALID_CONFIG);
    }
    if (actual_num_gw == 0)
    {
        throw EtherCANException("Need to configure at least one EtherCAN gateway",
                                DE_INSUFFICENT_NUM_GATEWAYS);
    }

    for (int i = 0; i < actual_num_gw; i++)
    {
        // Extract entry
        WrapGatewayAddress address_entry =
                        extract<WrapGatewayAddress>(list_gateway_addresses[i]);
        // Cast (slice) to internal parameter type
        address_array_to_fill[i] = static_cast<t_gateway_address>(address_entry);
    }

    return actual_num_gw;
}

//------------------------------------------------------------------------------
void WrapperSharedBase::convertWavetable(const bp::dict &dict_waveforms,
                                         t_wtable &wavetable_to_fill)
{
    wavetable_to_fill.clear();

    bp::list fpu_id_list = dict_waveforms.keys();
    const int num_keys = bp::len(fpu_id_list);

    if (num_keys == 0)
    {
        throw EtherCANException("DE_INVALID_WAVEFORM: Waveform table needs to address at least one FPU.",
                                DE_INVALID_WAVEFORM);
    }

    for (int i = 0; i < num_keys; i++)
    {
        object fpu_key = fpu_id_list[i];
        int fpu_id = bp::extract<int>(fpu_key);
        bp::list step_list = bp::extract<bp::list>(dict_waveforms[fpu_key]);
        int num_steps = bp::len(step_list);

        if (num_steps == 0)
        {
            throw EtherCANException("DE_INVALID_WAVEFORM: Waveform entry needs to contain at least one step.",
                                    DE_INVALID_WAVEFORM);
        }

        std::vector<t_step_pair> steps;

        for (int j = 0; j < num_steps; j++)
        {
            bp::tuple tstep_pair = bp::extract<bp::tuple>(step_list[j]);
            int16_t alpha_steps = bp::extract<int>(tstep_pair[0]);
            int16_t beta_steps = bp::extract<int>(tstep_pair[1]);

            t_step_pair step_pair;
            step_pair.alpha_steps = alpha_steps;
            step_pair.beta_steps = beta_steps;
            steps.push_back(step_pair);
        }

        t_waveform wform;
        wform.fpu_id = fpu_id;
        wform.steps = steps;
        wavetable_to_fill.push_back(wform);
    }
}

//------------------------------------------------------------------------------
void WrapperSharedBase::getFPUSet(const bp::list &fpu_list, t_fpuset &fpuset) const
{
    for (int i = 0; i < MAX_NUM_POSITIONERS; i++)
    {
        fpuset[i] = false;
    }

    if (bp::len(fpu_list) == 0)
    {
        for (int i = 0; ((i < getConfig().num_fpus) && (i < MAX_NUM_POSITIONERS));
             i++)
        {
            fpuset[i] = true;
        }
    }
    else
    {
        for (int i = 0; i < bp::len(fpu_list); i++)
        {
            int fpu_id = bp::extract<int>(fpu_list[i]);
            if ((fpu_id < 0) ||
                (fpu_id >= MAX_NUM_POSITIONERS) ||
                (fpu_id >= getConfig().num_fpus))
            {
                throw EtherCANException("DE_INVALID_FPU_ID: Parameter contains invalid FPU IDs.",
                                        DE_INVALID_FPU_ID);
            }
            else
            {
                fpuset[fpu_id] = true;
            }
        }
    }
}

//------------------------------------------------------------------------------
void WrapperSharedBase::getDatumFlags(bp::dict &dict_search_modes,
                                      t_datum_search_flags &direction_flags,
                                      const t_fpuset &fpuset)
{
    bp::list fpu_id_list = dict_search_modes.keys();
    const int num_keys = bp::len(fpu_id_list);

    if (num_keys == 0)
    {
        // default -- everything is SEARCH_AUTO
        for (int i = 0; i < MAX_NUM_POSITIONERS; i++)
        {
            if (fpuset[i])
            {
                direction_flags[i] = SEARCH_AUTO;
            }
            else
            {
                direction_flags[i] = SKIP_FPU;
            }
        }
    }
    else
    {
        for (int i = 0; i < MAX_NUM_POSITIONERS; i++)
        {
            direction_flags[i] = SKIP_FPU;
        }

        const int num_fpus = getConfig().num_fpus;

        if (num_keys > num_fpus)
        {
            throw EtherCANException("DE_INVALID_FPU_ID: Parameter contains invalid FPU IDs.",
                                    DE_INVALID_FPU_ID);
        }

        for (int i = 0; i < num_keys; i++)
        {
            object fpu_key = fpu_id_list[i];
            int fpu_id = bp::extract<int>(fpu_key);

            if ((fpu_id >= num_fpus) || (fpu_id < 0))
            {
                throw EtherCANException("DE_INVALID_FPU_ID: Parameter contains invalid FPU IDs.",
                                        DE_INVALID_FPU_ID);
            }

            if (fpuset[fpu_id])
            {
                int mode = bp::extract<int>(dict_search_modes[fpu_key]);
                direction_flags[fpu_id] = static_cast<E_DATUM_SEARCH_DIRECTION>(mode);
            }
        }
    }
}

#pragma GCC diagnostic pop


//==============================================================================
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

//------------------------------------------------------------------------------
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

//------------------------------------------------------------------------------
void checkInterfaceError(E_EtherCANErrCode ecode)
{
    switch(ecode)
    {
    //..........................................................................
    // Everything worked
    case DE_OK:
        break;

    //..........................................................................
    // Non-error return codes

    case DE_WAIT_TIMEOUT:
        throw EtherCANException("DE_WAIT_TIMEOUT: Response to a EtherCAN interface command surpassed the"
                                " waiting time parameter passed to waitForState(),"
                                " which caused the user command to return unfinished."
                                " (This is usually not an error.)",
                                DE_WAIT_TIMEOUT);
        break;

    case DE_FIRMWARE_UNIMPLEMENTED:
        throw EtherCANException("DE_FIRMWARE_UNIMPLEMENTED: Command or operation not implemented"
                                " for this protocol version",
                                DE_FIRMWARE_UNIMPLEMENTED);
        break;

    //..........................................................................
    // Fatal system failure

    case DE_OUT_OF_MEMORY:
        throw EtherCANException("DE_OUT_OF_MEMORY: The EtherCAN interface could not allocate the required memory, "
                                "and can not operate. Probable cause is a memory leak.",
                                DE_OUT_OF_MEMORY);
        break;

    case DE_RESOURCE_ERROR:
        throw EtherCANException("DE_RESOURCE_ERROR: The EtherCAN interface could not acquire necessary"
                                " resources such as file descriptors from the OS, and can not operate.",
                                DE_RESOURCE_ERROR);
        break;

    case DE_ASSERTION_FAILED:
        throw EtherCANException("DE_ASSERTION_FAILED: The EtherCAN interface determined an internal logic error, "
                                "should probably be terminated.",
                                DE_ASSERTION_FAILED);
        break;

    //..........................................................................
    // State errors

    case DE_INTERFACE_NOT_INITIALIZED:
        throw EtherCANException("DE_INTERFACE_NOT_INITIALIZED: EtherCANInterface was not initialized "
                                "properly, possibly due to system error or out-of-memory condition.",
                                DE_INTERFACE_NOT_INITIALIZED);
        break;

    case DE_INTERFACE_ALREADY_INITIALIZED:
        throw EtherCANException("DE_INTERFACE_ALREADY_INITIALIZED: EtherCANInterface was already initialized properly.",
                                DE_INTERFACE_ALREADY_INITIALIZED);
        break;

    case DE_STILL_BUSY:
        throw EtherCANException("DE_STILL_BUSY: The EtherCAN interface is still busy "
                                "working on a previosu command",
                                DE_STILL_BUSY);
        break;

    case DE_UNRESOLVED_COLLISION:
        throw EtherCANException("DE_UNRESOLVED_COLLISION: A previous collision, limit breach,"
                                " or abort message needs to be resolved first",
                                DE_UNRESOLVED_COLLISION);
        break;

    case DE_FPU_NOT_INITIALIZED:
        throw EtherCANException("DE_FPU_NOT_INITIALIZED: A fibre positioner unit (FPU) was not initialized as"
                                " required, needs to do a datum search first",
                                DE_FPU_NOT_INITIALIZED);
        break;

    case DE_INTERFACE_ALREADY_CONNECTED:
        throw EtherCANException("DE_INTERFACE_ALREADY_CONNECTED: EtherCAN Interface was already connected,"
                                " would need to disconnect() first.",
                                DE_INTERFACE_ALREADY_CONNECTED);
        break;

    case DE_INTERFACE_STILL_CONNECTED:
        throw EtherCANException("DE_INTERFACE_STILL_CONNECTED: EtherCAN interface is still connected",
                                DE_INTERFACE_STILL_CONNECTED);
        break;

    case DE_WAVEFORM_NOT_READY:
        throw EtherCANException("DE_WAVEFORM_NOT_READY: The FPU has no valid waveform configured for a movement.",
                                DE_WAVEFORM_NOT_READY);
        break;

    case DE_FPUS_NOT_CALIBRATED:
        throw EtherCANException("DE_FPUS_NOT_CALIBRATED: FPUs are lacking calibration by "
                                "a findDatum operation. For engineering or recovery use, consider"
                                " to set the 'allow_uninitialized' keyword argument to True",
                                DE_FPUS_NOT_CALIBRATED);
        break;

    case DE_NO_MOVABLE_FPUS:
        throw EtherCANException("DE_NO_MOVABLE_FPUS: No FPUs are currently movable.",
                                DE_NO_MOVABLE_FPUS);
        break;

    case DE_INVALID_FPU_STATE:
        throw EtherCANException("DE_INVALID_FPU_STATE: Command not allowed for present FPU state.",
                                DE_INVALID_FPU_STATE);
        break;

    case DE_PROTECTION_ERROR:
        throw EtherCANException("DE_PROTECTION_ERROR: Command might damage FPU, step count protection is enabled.",
                                DE_PROTECTION_ERROR);
        break;

    case DE_INVALID_INTERFACE_STATE:
        throw EtherCANException("DE_INVALID_INTERFACE_STATE: The current state of the EtherCAN interface"
                                " does not allow the requested operation.",
                                DE_INVALID_INTERFACE_STATE);
        break;

    case DE_FPUS_LOCKED:
        throw EtherCANException("DE_FPUS_LOCKED: Some addressed FPUs are in locked state,"
                                " they need to be unlocked first.",
                                DE_FPUS_LOCKED);
        break;

    case DE_IN_ABORTED_STATE:
        throw EtherCANException("DE_IN_ABORTED_STATE: There are FPUs in aborted state,"
                                " because of a previous abortMotion command or a step timing error"
                                "- use the enableMove (or resetFPUs) command to reset state.",
                                DE_IN_ABORTED_STATE);
        break;

    case DE_ALPHA_ARM_ON_LIMIT_SWITCH:
        throw EtherCANException("DE_ALPHA_ARM_ON_LIMIT_SWITCH: Datum command rejected because"
                                " an FPU alpha arm is on its limit switch.",
                                DE_ALPHA_ARM_ON_LIMIT_SWITCH);
        break;

    //..........................................................................
    // Setup errors

    case DE_INSUFFICENT_NUM_GATEWAYS:
        throw EtherCANException("DE_INSUFFICENT_NUM_GATEWAYS: The number of EtherCAN gateways"
                                " configured is insufficient for the configured number of FPUs",
                                DE_INSUFFICENT_NUM_GATEWAYS);
        break;

    case DE_INVALID_CONFIG:
        throw EtherCANException("DE_INVALID_CONFIG: The EtherCAN interface configuration is not valid",
                                DE_INVALID_CONFIG);
        break;

    case DE_SYNC_CONFIG_FAILED:
        throw EtherCANException("DE_SYNC_CONFIG_FAILED: Sending the SYNC configuration to the gateways failed",
                                DE_SYNC_CONFIG_FAILED);
        break;

    case DE_WRITE_VERIFICATION_FAILED:
        throw EtherCANException("DE_WRITE_VERIFICATION_FAILED: After a write operation, a readback showed a different value",
                                DE_WRITE_VERIFICATION_FAILED);
        break;

    //..........................................................................
    // Invalid command parameters

    case DE_INVALID_FPU_ID:
        throw EtherCANException("DE_INVALID_FPU_ID: A passed FPU id is out of range.",
                                DE_INVALID_FPU_ID);
        break;

    case DE_INVALID_PAR_VALUE:
        throw EtherCANException("DE_INVALID_PAR_VALUE: The passed parameter value is invalid.",
                                DE_INVALID_PAR_VALUE);
        break;

    case DE_DUPLICATE_SERIAL_NUMBER:
        throw EtherCANException("DE_DUPLICATE_SERIAL_NUMBER: The passed serial number is already in use.",
                                DE_DUPLICATE_SERIAL_NUMBER);
        break;

    //..........................................................................
    // Connection failures

    case DE_MAX_RETRIES_EXCEEDED:
        throw EtherCANException("DE_MAX_RETRIES_EXCEEDED: A command could not be"
                                " send in spite of several retries", DE_MAX_RETRIES_EXCEEDED);
        break;

    case DE_CAN_COMMAND_TIMEOUT_ERROR:
        throw EtherCANException("DE_CAN_COMMAND_TIMEOUT_ERROR:"
                                " A CAN command to an FPU surpassed the maximum waiting time"
                                " determined by the CAN protocol."
                                " This likely indicates a failure of the controller or a"
                                " serious connection problem.",
                                DE_CAN_COMMAND_TIMEOUT_ERROR);
        break;

    case DE_NO_CONNECTION:
        throw EtherCANException("DE_NO_CONNECTION: The EtherCAN Interface is not connected to a gateway.",
                                DE_NO_CONNECTION);
        break;

    case DE_FIRMWARE_CAN_BUFFER_OVERFLOW:
        throw EtherCANException("DE_FIRMWARE_CAN_BUFFER_OVERFLOW:"
                                " A CAN command to an FPU could not be processed and was lost"
                                " because the FPU firmware buffer was full.",
                                DE_FIRMWARE_CAN_BUFFER_OVERFLOW);
        break;

    //..........................................................................
    // Invalid waveforms

    case DE_INVALID_WAVEFORM:
        throw EtherCANException("DE_INVALID_WAVEFORM: The passed waveform does not meet some general rule.",
                                DE_INVALID_WAVEFORM);
        break;

    case DE_INVALID_WAVEFORM_TOO_MANY_SECTIONS:
        throw EtherCANException("DE_INVALID_WAVEFORM_TOO_MANY_SECTIONS: The passed waveform has too many sections.",
                                DE_INVALID_WAVEFORM_TOO_MANY_SECTIONS);
        break;

    case DE_INVALID_WAVEFORM_RAGGED:
        throw EtherCANException("DE_INVALID_WAVEFORM_RAGGED: The passed waveform has different number of sections for different FPUs.",
                                DE_INVALID_WAVEFORM_RAGGED);
        break;

    case DE_INVALID_WAVEFORM_STEPCOUNT_TOO_LARGE:
        throw EtherCANException("DE_INVALID_WAVEFORM_STEP_COUNT_TOO_LARGE:"
                                " The passed waveform has a section with too many steps.",
                                DE_INVALID_WAVEFORM_STEPCOUNT_TOO_LARGE);
        break;

    case DE_INVALID_WAVEFORM_CHANGE:
        throw EtherCANException("DE_INVALID_WAVEFORM_CHANGE: The passed waveform has an"
                                " invalid change in step counts / speed between adjacent sections",
                                DE_INVALID_WAVEFORM_CHANGE);
        break;

    case DE_INVALID_WAVEFORM_TAIL:
        throw EtherCANException("DE_INVALID_WAVEFORM_TAIL: The passed waveform has an invalid tail section.",
                                DE_INVALID_WAVEFORM_TAIL);
        break;

    case DE_INVALID_WAVEFORM_REJECTED :
        throw EtherCANException("DE_INVALID_WAVEFORM_REJECTED: The passed waveform was not properly communicated. FPU state did not change.",
                                DE_INVALID_WAVEFORM_REJECTED);
        break;
        
    //..........................................................................
    // Errors which terminate movements

    case DE_NEW_COLLISION:
        throw EtherCANException("DE_NEW_COLLISION: A collision was detected,"
                                " movement for this FPU aborted.",
                                DE_NEW_COLLISION);
        break;

    case DE_NEW_LIMIT_BREACH:
        throw EtherCANException("DE_NEW_LIMIT_BREACH: An alpha limit breach was detected,"
                                " movement for this FPU aborted.",
                                DE_NEW_LIMIT_BREACH);
        break;

    case DE_STEP_TIMING_ERROR:
        throw EtherCANException("DE_STEP_TIMING_ERROR: An FPU's controller"
                                " generated a step timing error"
                                " during movement. Possibly, reduce the microstepping level"
                                " to compute the step frequency in time.",
                                DE_STEP_TIMING_ERROR);
        break;

    case DE_MOVEMENT_ABORTED:
        throw EtherCANException("DE_MOVEMENT_ABORTED: The FPU has entered the FPST_ABORTED state,"
                                " because of an abortMotion command or a step timing error "
                                "- use the enableMove (or resetFPUs) command to reset state.",
                                DE_MOVEMENT_ABORTED);
        break;

    case DE_HW_ALPHA_ARM_ON_LIMIT_SWITCH:
        throw EtherCANException("DE_HW_ALPHA_ARM_ON_LIMIT_SWITCH: Part of datum command rejected by"
                                " hardware because an FPU alpha arm is on its limit switch"
                                " before it started to move.",
                                DE_HW_ALPHA_ARM_ON_LIMIT_SWITCH);
        break;

    case DE_DATUM_COMMAND_HW_TIMEOUT:
        throw EtherCANException("DE_DATUM_COMMAND_HW_TIMEOUT: The FPU firmware has timed-out"
                                " a datum operation because it took too long to complete. Potentially,"
                                " the datum switch is not working, or the FPU hardware is otherwise"
                                " damaged. It can also be that the datum command was just issued when"
				                " the FPU was too far away from the datum switch.",
                                DE_DATUM_COMMAND_HW_TIMEOUT);
        break;

    case DE_INCONSISTENT_STEP_COUNT:
        throw EtherCANException("DE_INCONSISTENT_STEP_COUNT: The EtherCAN interface received an"
                                " illegal counter value from an FPU, so that it cannot correctly"
                                " track the FPUs any more. It is required to measure the"
                                " position and update the position database.",
                                DE_INCONSISTENT_STEP_COUNT);
        break;

    //..........................................................................
     // Database errors

    case DE_DB_ENV_VARIABLE_NOT_FOUND:
        throw EtherCANException("DE_DB_ENV_VARIABLE_NOT_FOUND: One or more of the Linux"
                                " environment variables which are needed to specify the"
                                " database directory are missing.",
                                DE_DB_ENV_VARIABLE_NOT_FOUND);
        break;

    case DE_DB_DIR_OR_FILE_NOT_FOUND:
        throw EtherCANException("DE_DB_DIR_OR_FILE_NOT_FOUND: Database opening error:"
                                " Invalid directory, or database file(s) were not found.",
                                DE_DB_DIR_OR_FILE_NOT_FOUND);
        break;

    case DE_DB_ACCESS_DENIED:
        throw EtherCANException("DE_DB_ACCESS_DENIED: Database opening error:"
                                " Access is denied - the grid driver does not have"
                                " sufficient privileges to access the database files"
                                " and/or the directory which they are in.",
                                DE_DB_ACCESS_DENIED);
        break;

    case DE_DB_OLD_FORMAT:
        throw EtherCANException("DE_DB_OLD_FORMAT: Database opening error:"
                                " The database files found are of the old incompatible"
                                " Python format, and cannot be used.",
                                DE_DB_OLD_FORMAT);
        break;

    case DE_DB_OTHER_OPENING_FAILURE:
        throw EtherCANException("DE_DB_OTHER_OPENING_FAILURE: Database opening error:"
                                " An other unspecified failure occurred.",
                                DE_DB_OTHER_OPENING_FAILURE);
        break;

    case DE_DB_TRANSACTION_CREATION_FAILED:
        throw EtherCANException("DE_DB_TRANSACTION_CREATION_FAILED:"
                                " An attempted database transaction creation failed.",
                                DE_DB_TRANSACTION_CREATION_FAILED);
        break;

    case DE_DB_MISSING_FPU_ENTRY_OR_READ_FAILED:
        throw EtherCANException("DE_DB_MISSING_FPU_ENTRY_OR_READ_FAILED:"
                                " For a physical FPU's serial number, one or more of its"
                                " required data items are missing from the database, or the"
                                " reading of these items failed for some reason.",
                                DE_DB_MISSING_FPU_ENTRY_OR_READ_FAILED);
        break;

    case DE_DB_WRITE_FAILED:
        throw EtherCANException("DE_DB_WRITE_FAILED: An attempted write of a data item failed.",
                                DE_DB_WRITE_FAILED);
        break;

    case DE_DB_SYNC_FAILED:
        throw EtherCANException("DE_DB_SYNC_FAILED:"
                                " A database synchronisation operation failed.",
                                DE_DB_SYNC_FAILED);
        break;

    //..........................................................................
    // Unknown error
    case DE_ERROR_UNKNOWN:
        throw EtherCANException("DE_ERROR_UNKNOWN: An unknown error occurred.",
                                DE_ERROR_UNKNOWN);

    //..........................................................................
    }
}

//------------------------------------------------------------------------------
