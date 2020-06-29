// -*- mode: c++ -*-
////////////////////////////////////////////////////////////////////////////////
//
// Copyright 2017 UKRI. See file "LICENSE" for license information.
//
// Who       When        What
// --------  ----------  -------------------------------------------------------
//------------------------------------------------------------------------------

////////////////////////////////////////////////////////////////////////////////
// NAME ethercanif.cpp
//
// This file implements the Python wrapper for the EtherCAN interface
// for the MOONS instrument fibre positioner unit.
//
////////////////////////////////////////////////////////////////////////////////

#include "FpuBPShared_General.h"

//................................................
#ifdef COMBINED_INCLUSION
//................................................


//................................................
#else // NOT COMBINED_INCLUSION
//................................................

#include <string.h>
#include <iostream>
#include <iomanip>
#include <string>
#include <vector>

#include <boost/python/class.hpp>
#include <boost/python/module.hpp>
#include <boost/python/def.hpp>
#include <boost/python/enum.hpp>
#include <boost/python/extract.hpp>
#include <boost/python/list.hpp>
#include <boost/python/dict.hpp>
#include <boost/python/tuple.hpp>
#include <boost/python/str.hpp>
#include <boost/python/suite/indexing/vector_indexing_suite.hpp>
#include <boost/python/operators.hpp>
#include <boost/python/exception_translator.hpp>

#include "../../include/ethercan/E_CAN_COMMAND.h"
#include "../../include/T_GridState.h"
#include "../../include/E_GridState.h"
#include "../../include/EtherCANInterface.h"
#include "../../include/GridState.h"

//................................................
#endif // NOT COMBINED_INCLUSION
//................................................


PyObject* EtherCANExceptionTypeObj = 0;
PyObject* InvalidWaveformExceptionTypeObj = 0;
PyObject* MovementErrorExceptionTypeObj = 0;
PyObject* CollisionErrorExceptionTypeObj = 0;
PyObject* LimitBreachErrorExceptionTypeObj = 0;
PyObject* AbortMotionErrorExceptionTypeObj = 0;
PyObject* FirmwareTimeOutExceptionTypeObj = 0;
PyObject* TimingErrorExceptionTypeObj = 0;
PyObject* InvalidStateExceptionTypeObj = 0;
PyObject* SystemFailureExceptionTypeObj = 0;
PyObject* SetupErrorExceptionTypeObj = 0;
PyObject* InvalidParameterExceptionTypeObj = 0;
PyObject* ConnectionFailureExceptionTypeObj = 0;
PyObject* SocketFailureExceptionTypeObj = 0;
PyObject* CommandTimeoutExceptionTypeObj = 0;
PyObject* CAN_OverflowExceptionTypeObj = 0;
PyObject* ProtectionErrorExceptionTypeObj = 0;
PyObject* HardwareProtectionErrorExceptionTypeObj = 0;



namespace   // Avoid cluttering the global namespace.
{

//................................................
#ifndef COMBINED_INCLUSION // NOT COMBINED_INCLUSION
//................................................

using namespace mpifps;

using namespace mpifps::ethercanif;

using boost::python::object;
using boost::python::extract;
using boost::python::list;
using boost::python::dict;
using boost::python::tuple;
using boost::python::str;

//................................................
#endif // NOT COMBINED_INCLUSION
//................................................

/* ---------------------------------------------------------------------------*/
E_GridState wrapGetGridStateSummary(WrapGridState& grid_state)
{
    return getGridStateSummary(grid_state);
}


/* ---------------------------------------------------------------------------*/
void translate_interface_error(EtherCANException const& e)
{
    // Use the Python 'C' API to set up an exception object
    switch (e.getErrCode())
    {

    case DE_INTERFACE_NOT_INITIALIZED :
    case DE_INTERFACE_ALREADY_INITIALIZED :
    case DE_STILL_BUSY :
    case DE_UNRESOLVED_COLLISION :
    case DE_FPU_NOT_INITIALIZED :
    case DE_INTERFACE_ALREADY_CONNECTED :
    case DE_INTERFACE_STILL_CONNECTED :
    case DE_WAVEFORM_NOT_READY :
    case DE_FPUS_NOT_CALIBRATED :
    case DE_NO_MOVABLE_FPUS :
    case DE_FPUS_LOCKED :
    case DE_INVALID_FPU_STATE :
    case DE_INVALID_INTERFACE_STATE :
    case DE_IN_ABORTED_STATE :
    case DE_ALPHA_ARM_ON_LIMIT_SWITCH:
        PyErr_SetString(InvalidStateExceptionTypeObj, e.what());
        break;

    case DE_PROTECTION_ERROR:
        PyErr_SetString(ProtectionErrorExceptionTypeObj, e.what());
        break;

    case DE_OUT_OF_MEMORY:
    case DE_RESOURCE_ERROR:
    case DE_ASSERTION_FAILED:
        PyErr_SetString(SystemFailureExceptionTypeObj, e.what());
        break;

    case DE_FIRMWARE_UNIMPLEMENTED:
    case DE_INSUFFICENT_NUM_GATEWAYS :
    case DE_INVALID_CONFIG :
    case DE_SYNC_CONFIG_FAILED:
        PyErr_SetString(SetupErrorExceptionTypeObj, e.what());
        break;

    case DE_INVALID_FPU_ID :
    case DE_INVALID_PAR_VALUE :
    case DE_DUPLICATE_SERIAL_NUMBER:
        PyErr_SetString(InvalidParameterExceptionTypeObj, e.what());
        break;

    case DE_WAIT_TIMEOUT :
        // this is normally not raised, because not necessarily an error
        PyErr_SetString(ConnectionFailureExceptionTypeObj, e.what());
        break;
    case DE_NO_CONNECTION :
        PyErr_SetString(SocketFailureExceptionTypeObj, e.what());
        break;
    case DE_MAX_RETRIES_EXCEEDED :
    case DE_CAN_COMMAND_TIMEOUT_ERROR:
        PyErr_SetString(CommandTimeoutExceptionTypeObj, e.what());
        break;

    case DE_FIRMWARE_CAN_BUFFER_OVERFLOW:
        PyErr_SetString(CAN_OverflowExceptionTypeObj, e.what());
        break;

    case DE_INVALID_WAVEFORM :
    case DE_INVALID_WAVEFORM_TAIL:
    case DE_INVALID_WAVEFORM_TOO_MANY_SECTIONS:
    case DE_INVALID_WAVEFORM_RAGGED:
    case DE_INVALID_WAVEFORM_STEPCOUNT_TOO_LARGE:
    case DE_INVALID_WAVEFORM_CHANGE:
        PyErr_SetString(InvalidWaveformExceptionTypeObj, e.what());
        break;

    case DE_NEW_COLLISION:
        PyErr_SetString(CollisionErrorExceptionTypeObj, e.what());
        break;
    case DE_NEW_LIMIT_BREACH:
        PyErr_SetString(LimitBreachErrorExceptionTypeObj, e.what());
        break;
    case DE_STEP_TIMING_ERROR:
        PyErr_SetString(TimingErrorExceptionTypeObj, e.what());
        break;

    case DE_MOVEMENT_ABORTED:
        PyErr_SetString(AbortMotionErrorExceptionTypeObj, e.what());
        break;

    case DE_DATUM_COMMAND_HW_TIMEOUT:
        PyErr_SetString(FirmwareTimeOutExceptionTypeObj, e.what());
        break;

    case DE_HW_ALPHA_ARM_ON_LIMIT_SWITCH:
        PyErr_SetString(HardwareProtectionErrorExceptionTypeObj, e.what());
        break;

    case DE_INCONSISTENT_STEP_COUNT:
        PyErr_SetString(HardwareProtectionErrorExceptionTypeObj, e.what());
        break;

    default:
        PyErr_SetString(EtherCANExceptionTypeObj, e.what());
    }
}


/* ---------------------------------------------------------------------------*/
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


/* ---------------------------------------------------------------------------*/
class WrapEtherCANInterface : public EtherCANInterface
{
private:
    const EtherCANInterfaceConfig config;

    void getFPUSet(const list& fpu_list, t_fpuset &fpuset) const
    {
        if (len(fpu_list) == 0)
        {
            for(int i=0; i < MAX_NUM_POSITIONERS; i++)
            {
                fpuset[i] = true;
            }
        }
        else
        {
            for(int i=0; i < MAX_NUM_POSITIONERS; i++)
            {
                fpuset[i] = false;
            }
            for(int i=0; i < len(fpu_list); i++)
            {
                int fpu_id = extract<int>(fpu_list[i]);
                if ((fpu_id < 0)
                        || (fpu_id >= MAX_NUM_POSITIONERS)
                        || (fpu_id >= config.num_fpus))
                {
                    throw EtherCANException("DE_INVALID_FPU_ID: Parameter contain invalid FPU IDs.",
                                            DE_INVALID_FPU_ID);
                }
                fpuset[fpu_id] = true;
            }
        }
    }

public:

    WrapEtherCANInterface(const EtherCANInterfaceConfig _config) : EtherCANInterface(_config), config(_config)
    {

        E_EtherCANErrCode ecode = initializeInterface();
        checkInterfaceError(ecode);
    };


    E_EtherCANErrCode connectGateways(list& list_gateway_addresses)
    {
        const int actual_num_gw = len(list_gateway_addresses);

        t_gateway_address address_array[MAX_NUM_GATEWAYS];

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

        for (int i=0; i < actual_num_gw; i++)
        {

            // extract entry
            WrapGatewayAddress address_entry =
                extract<WrapGatewayAddress>(
                    list_gateway_addresses[i]);
            // cast (slice) to internal parameter type
            address_array[i] = static_cast<t_gateway_address>(
                                   address_entry);
        }
        E_EtherCANErrCode ecode = connect(actual_num_gw, address_array);
        checkInterfaceError(ecode);
        return ecode;

    };

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wstrict-overflow"
#pragma GCC diagnostic error "-Wstrict-overflow=2"

    E_EtherCANErrCode configMotionWithDict(dict& dict_waveforms, WrapGridState& grid_state,
                                           list &fpu_list,
                                           bool allow_uninitialized=false,
					   int ruleset_version=DEFAULT_WAVEFORM_RULESET_VERSION)
  {
        t_fpuset fpuset;
        getFPUSet(fpu_list, fpuset);

        list fpu_id_list = dict_waveforms.keys();
        const int nkeys = len(fpu_id_list);

        if (nkeys == 0)
        {
            throw EtherCANException("DE_INVALID_WAVEFORM: Waveform table needs to address at least one FPU.",
                                    DE_INVALID_WAVEFORM);
        }

        t_wtable wtable;
        for(int i = 0; i < nkeys; i++)
        {
            object fpu_key = fpu_id_list[i];
            int fpu_id = extract<int>(fpu_key);
            list step_list = extract<list>(dict_waveforms[fpu_key]);
            int num_steps = len(step_list);

            if (num_steps == 0)
            {
                throw EtherCANException("DE_INVALID_WAVEFORM: Waveform entry needs to contain at least one step.",
                                        DE_INVALID_WAVEFORM);
            }

            std::vector<t_step_pair> steps;

            for(int j = 0; j < num_steps; j++)
            {
                tuple tstep_pair = extract<tuple>(step_list[j]);
                int16_t alpha_steps = extract<int>(tstep_pair[0]);
                int16_t beta_steps = extract<int>(tstep_pair[1]);

                t_step_pair step_pair;
                step_pair.alpha_steps = alpha_steps;
                step_pair.beta_steps = beta_steps;
                steps.push_back(step_pair);
            }

            t_waveform wform;
            wform.fpu_id = fpu_id;
            wform.steps = steps;
            wtable.push_back(wform);
        }
        E_EtherCANErrCode ecode = configMotion(wtable, grid_state, fpuset,
                                               allow_uninitialized, ruleset_version);
        checkInterfaceError(ecode);
        return ecode;

    };
#pragma GCC diagnostic pop

    WrapGridState wrap_getGridState()
    {
        WrapGridState grid_state;
        getGridState(grid_state);
        return grid_state;
    }

    E_EtherCANErrCode wrap_initializeGrid(WrapGridState& grid_state, list& fpu_list)
    {
        t_fpuset fpuset;
        getFPUSet(fpu_list, fpuset);

        E_EtherCANErrCode ecode = initializeGrid(grid_state, fpuset);
        checkInterfaceError(ecode);
        return ecode;
    }

    E_EtherCANErrCode wrap_resetFPUs(WrapGridState& grid_state, list& fpu_list)
    {
        t_fpuset fpuset;
        getFPUSet(fpu_list, fpuset);

        E_EtherCANErrCode ecode = resetFPUs(grid_state, fpuset);
        checkInterfaceError(ecode);
        return ecode;

    }


    E_EtherCANErrCode wrap_pingFPUs(WrapGridState& grid_state, list& fpu_list)
    {
        t_fpuset fpuset;
        getFPUSet(fpu_list, fpuset);
        E_EtherCANErrCode ecode = pingFPUs(grid_state, fpuset);
        checkInterfaceError(ecode);
        return ecode;
    }


    E_EtherCANErrCode wrap_readRegister(int read_address, WrapGridState& grid_state, list& fpu_list)
    {
        t_fpuset fpuset;
        getFPUSet(fpu_list, fpuset);

        if ( (read_address > 0xffff) || (read_address < 0))
        {
            checkInterfaceError(DE_INVALID_PAR_VALUE);
        }
        const uint16_t raddress = (uint16_t) read_address;
        E_EtherCANErrCode ecode = readRegister(raddress, grid_state, fpuset);
        checkInterfaceError(ecode);
        return ecode;
    }

    E_EtherCANErrCode wrap_getFirmwareVersion(WrapGridState& grid_state, list& fpu_list)
    {
        t_fpuset fpuset;
        getFPUSet(fpu_list, fpuset);

        E_EtherCANErrCode ecode = getFirmwareVersion(grid_state, fpuset);
        checkInterfaceError(ecode);
        return ecode;
    }



    void getDatumFlags(dict& dict_modes, t_datum_search_flags &direction_flags, const t_fpuset &fpuset)
    {
        list fpu_id_list = dict_modes.keys();
        const int nkeys = len(fpu_id_list);

        if (nkeys == 0)
        {
            // default -- everything is SEARCH_AUTO
            for(int i=0; i < MAX_NUM_POSITIONERS; i++)
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
            for(int i=0; i < MAX_NUM_POSITIONERS; i++)
            {
                direction_flags[i] = SKIP_FPU;
            }


            const int num_fpus = getNumFPUs();

            if (nkeys > num_fpus )
            {
                throw EtherCANException("DE_INVALID_FPU_ID: Parameter contain invalid FPU IDs.",
                                        DE_INVALID_FPU_ID);
            }


            for(int i = 0; i < nkeys; i++)
            {
                object fpu_key = fpu_id_list[i];
                int fpu_id = extract<int>(fpu_key);

                if ((fpu_id >= num_fpus) || (fpu_id < 0))
                {
                    throw EtherCANException("DE_INVALID_FPU_ID: Parameter contain invalid FPU IDs.",
                                            DE_INVALID_FPU_ID);
                }


                if (fpuset[fpu_id])
                {
                    int mode = extract<int>(dict_modes[fpu_key]);
                    direction_flags[fpu_id] = static_cast<E_DATUM_SEARCH_DIRECTION>(mode);
                }

            }
        }
    }

    E_EtherCANErrCode wrap_findDatum(WrapGridState& grid_state,
                                     dict &dict_modes,
                                     list& fpu_list,
                                     E_DATUM_SELECTION arm_selection=DASEL_BOTH,
                                     E_DATUM_TIMEOUT_FLAG timeout_flag=DATUM_TIMEOUT_ENABLE,
                                     bool count_protection=true)
    {
        t_fpuset fpuset;
        getFPUSet(fpu_list, fpuset);

        t_datum_search_flags direction_flags;
        getDatumFlags(dict_modes, direction_flags, fpuset);

        E_EtherCANErrCode ecode = findDatum(grid_state, direction_flags,
                                            arm_selection, timeout_flag,
                                            count_protection, &fpuset);
        checkInterfaceError(ecode);
        return ecode;
    }


    E_EtherCANErrCode wrap_startFindDatum(WrapGridState& grid_state,
                                          dict& dict_modes,
                                          list& fpu_list,
                                          E_DATUM_SELECTION arm_selection=DASEL_BOTH,
                                          E_DATUM_TIMEOUT_FLAG timeout_flag=DATUM_TIMEOUT_ENABLE,
                                          bool count_protection=true)
    {

        t_fpuset fpuset;
        getFPUSet(fpu_list, fpuset);

        t_datum_search_flags direction_flags;
        getDatumFlags(dict_modes, direction_flags, fpuset);

        E_EtherCANErrCode ecode = startFindDatum(grid_state,
                                  direction_flags,
                                  arm_selection,
                                  timeout_flag,
                                  count_protection, &fpuset);
        checkInterfaceError(ecode);
        return ecode;
    }

    E_EtherCANErrCode wrap_waitFindDatum(WrapGridState& grid_state, double max_wait_time, list& fpu_list)
    {
        E_EtherCANErrCode estatus;
        bool finished = false;

        t_fpuset fpuset;
        getFPUSet(fpu_list, fpuset);


        // FIXME: should return remaining wait time in tuple
        estatus =  waitFindDatum(grid_state, max_wait_time, finished, &fpuset);

        if (((! finished) && (estatus == DE_OK))
                || (estatus == DE_WAIT_TIMEOUT))
        {
            estatus = DE_WAIT_TIMEOUT;
            // we return because this is not exceptional or an error
            return estatus;
        }

        checkInterfaceError(estatus);
        return estatus;

    }


    E_EtherCANErrCode wrap_executeMotion(WrapGridState& grid_state, list& fpu_list, bool sync_command=false)
    {
        t_fpuset fpuset;
        getFPUSet(fpu_list, fpuset);

        E_EtherCANErrCode ecode =executeMotion(grid_state, fpuset, sync_command);
        checkInterfaceError(ecode);
        return ecode;
    }

    E_EtherCANErrCode wrap_startExecuteMotion(WrapGridState& grid_state, list& fpu_list, bool sync_command=false)
    {
        t_fpuset fpuset;
        getFPUSet(fpu_list, fpuset);

        E_EtherCANErrCode ecode =startExecuteMotion(grid_state, fpuset, sync_command);
        checkInterfaceError(ecode);
        return ecode;
    }

    E_EtherCANErrCode wrap_waitExecuteMotion(WrapGridState& grid_state, double max_wait_time, list& fpu_list)
    {
        E_EtherCANErrCode estatus;
        bool finished = false;

        t_fpuset fpuset;
        getFPUSet(fpu_list, fpuset);

        // FIXME: should return remaining wait time in tuple
        estatus =  waitExecuteMotion(grid_state, max_wait_time, finished, fpuset);
        if (((! finished) && (estatus == DE_OK))
                || (estatus == DE_WAIT_TIMEOUT))
        {
            estatus = DE_WAIT_TIMEOUT;
            // we return because this is not exceptional oo an error
            return estatus;
        }

        checkInterfaceError(estatus);
        return estatus;
    }

    E_EtherCANErrCode wrap_repeatMotion(WrapGridState& grid_state, list& fpu_list)
    {
        t_fpuset fpuset;
        getFPUSet(fpu_list, fpuset);

        E_EtherCANErrCode ecode =repeatMotion(grid_state, fpuset);
        checkInterfaceError(ecode);
        return ecode;
    }

    E_EtherCANErrCode wrap_reverseMotion(WrapGridState& grid_state, list& fpu_list)
    {
        t_fpuset fpuset;
        getFPUSet(fpu_list, fpuset);

        E_EtherCANErrCode ecode = reverseMotion(grid_state, fpuset);
        checkInterfaceError(ecode);
        return ecode;
    }

    E_EtherCANErrCode wrap_abortMotion(WrapGridState& grid_state, list& fpu_list, bool sync_command=true)
    {
        t_fpuset fpuset;
        getFPUSet(fpu_list, fpuset);

        E_EtherCANErrCode ecode = abortMotion(grid_state, fpuset, sync_command);
        checkInterfaceError(ecode);
        return ecode;
    }

    E_EtherCANErrCode wrap_enableMove(int fpu_id, WrapGridState& grid_state)
    {
        E_EtherCANErrCode ecode = enableMove(fpu_id, grid_state);
        checkInterfaceError(ecode);
        return ecode;
    }

    E_EtherCANErrCode wrap_setUStepLevel(int ustep_level, WrapGridState& grid_state, list& fpu_list)
    {
        t_fpuset fpuset;
        getFPUSet(fpu_list, fpuset);

        E_EtherCANErrCode ecode = setUStepLevel(ustep_level, grid_state, fpuset);
        checkInterfaceError(ecode);
        return ecode;
    }


    E_EtherCANErrCode wrap_freeBetaCollision(int fpu_id, E_REQUEST_DIRECTION request_direction,
            WrapGridState& grid_state)
    {
        E_EtherCANErrCode ecode = freeBetaCollision(fpu_id, request_direction, grid_state);
        checkInterfaceError(ecode);
        return ecode;
    }

    E_EtherCANErrCode wrap_enableBetaCollisionProtection(WrapGridState& grid_state)
    {
        E_EtherCANErrCode ecode = enableBetaCollisionProtection(grid_state);
        checkInterfaceError(ecode);
        return ecode;
    }

    E_EtherCANErrCode wrap_lockFPU(int fpu_id, WrapGridState& grid_state)
    {
        E_EtherCANErrCode ecode =lockFPU(fpu_id, grid_state);
        checkInterfaceError(ecode);
        return ecode;
    }

    E_EtherCANErrCode wrap_unlockFPU(int fpu_id, WrapGridState& grid_state)
    {
        E_EtherCANErrCode ecode =unlockFPU(fpu_id, grid_state);
        checkInterfaceError(ecode);
        return ecode;
    }

    E_EtherCANErrCode wrap_readSerialNumbers(WrapGridState& grid_state, list& fpu_list)
    {
        t_fpuset fpuset;
        getFPUSet(fpu_list, fpuset);

        E_EtherCANErrCode ecode = readSerialNumbers(grid_state, fpuset);
        checkInterfaceError(ecode);
        return ecode;
    }

    E_EtherCANErrCode wrap_writeSerialNumber(int fpu_id, str serial_number,
            WrapGridState& grid_state)
    {
        std::string cpp_serial_number =  extract<std::string>(serial_number);

        E_EtherCANErrCode ecode = writeSerialNumber(fpu_id, cpp_serial_number.c_str(), grid_state);
        checkInterfaceError(ecode);
        return ecode;
    }

    E_EtherCANErrCode wrap_resetStepCounters(long alpha_steps, long beta_steps,
					     WrapGridState& grid_state, list& fpu_list)
    {
        t_fpuset fpuset;
        getFPUSet(fpu_list, fpuset);

        E_EtherCANErrCode ecode = resetStepCounters(alpha_steps, beta_steps, grid_state, fpuset);
        checkInterfaceError(ecode);
        return ecode;

    }

    E_EtherCANErrCode wrap_checkIntegrity(WrapGridState& grid_state, list& fpu_list)
    {
        t_fpuset fpuset;
        getFPUSet(fpu_list, fpuset);

        E_EtherCANErrCode ecode = checkIntegrity(grid_state, fpuset);
        checkInterfaceError(ecode);
        return ecode;

    }

    boost::python::tuple wrap_getMinFirmwareVersion(WrapGridState& grid_state, list& fpu_list)
    {
        t_fpuset fpuset;
        getFPUSet(fpu_list, fpuset);
        uint8_t min_firmware_version[3];

        E_EtherCANErrCode ecode = getMinFirmwareVersion(fpuset, min_firmware_version, grid_state);
        checkInterfaceError(ecode);
        return boost::python::make_tuple(min_firmware_version[0], min_firmware_version[1], min_firmware_version[2]);
    }

    E_EtherCANErrCode wrap_setStepsPerSegment(int min_steps, int max_steps, WrapGridState& grid_state, list& fpu_list)
    {
        t_fpuset fpuset;
        getFPUSet(fpu_list, fpuset);

        E_EtherCANErrCode ecode = setStepsPerSegment(min_steps, max_steps, grid_state, fpuset);
        checkInterfaceError(ecode);
        return ecode;
    }
    E_EtherCANErrCode wrap_setTicksPerSegment(unsigned long ticks, WrapGridState& grid_state, list& fpu_list)
    {
        t_fpuset fpuset;
        getFPUSet(fpu_list, fpuset);

        E_EtherCANErrCode ecode = setTicksPerSegment(ticks, grid_state, fpuset);
        checkInterfaceError(ecode);
        return ecode;
    }

    E_EtherCANErrCode wrap_freeAlphaLimitBreach(int fpu_id, E_REQUEST_DIRECTION request_direction,
            WrapGridState& grid_state)
    {
        E_EtherCANErrCode ecode = freeAlphaLimitBreach(fpu_id, request_direction, grid_state);
        checkInterfaceError(ecode);
        return ecode;
    }

    E_EtherCANErrCode wrap_enableAlphaLimitProtection(WrapGridState& grid_state)
    {
        E_EtherCANErrCode ecode = enableAlphaLimitProtection(grid_state);
        checkInterfaceError(ecode);
        return ecode;
    }

};



}

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wmissing-declarations"


/* ---------------------------------------------------------------------------*/
PyObject* EtherCANExceptionClass(const char* name, PyObject* baseTypeObj = PyExc_Exception)
{
    using std::string;
    namespace bp = boost::python;

    string scopeName = bp::extract<string>(bp::scope().attr("__name__"));
    string qualifiedName0 = scopeName + "." + name;
    char* qualifiedName1 = const_cast<char*>(qualifiedName0.c_str());

    PyObject* typeObj = PyErr_NewException(qualifiedName1, baseTypeObj, 0);
    if(!typeObj) bp::throw_error_already_set();
    bp::scope().attr(name) = bp::handle<>(bp::borrowed(typeObj));
    return typeObj;
}
#pragma GCC diagnostic pop

BOOST_PYTHON_MODULE(ethercanif)
{
    using namespace boost::python;

    scope().attr("__version__") = (strlen(VERSION) > 1) ?  (((const char*)VERSION) +1) : "?.?.?";

    scope().attr("CAN_PROTOCOL_VERSION") = CAN_PROTOCOL_VERSION;

    scope().attr("DEFAULT_WAVEFORM_RULESET_VERSION") = DEFAULT_WAVEFORM_RULESET_VERSION;

    /* define the exception hierarchy */
    EtherCANExceptionTypeObj = EtherCANExceptionClass("EtherCANException");
    MovementErrorExceptionTypeObj = EtherCANExceptionClass("MovementError", EtherCANExceptionTypeObj);
    CollisionErrorExceptionTypeObj = EtherCANExceptionClass("CollisionError", MovementErrorExceptionTypeObj);
    LimitBreachErrorExceptionTypeObj = EtherCANExceptionClass("LimitBreachError", MovementErrorExceptionTypeObj);
    AbortMotionErrorExceptionTypeObj = EtherCANExceptionClass("AbortMotionError", MovementErrorExceptionTypeObj);
    FirmwareTimeOutExceptionTypeObj = EtherCANExceptionClass("FirmwareTimeoutError", MovementErrorExceptionTypeObj);
    TimingErrorExceptionTypeObj = EtherCANExceptionClass("StepTimingError", MovementErrorExceptionTypeObj);
    InvalidStateExceptionTypeObj = EtherCANExceptionClass("InvalidStateException", EtherCANExceptionTypeObj);
    SystemFailureExceptionTypeObj = EtherCANExceptionClass("SystemFailure", EtherCANExceptionTypeObj);
    InvalidParameterExceptionTypeObj  = EtherCANExceptionClass("InvalidParameterError", EtherCANExceptionTypeObj);
    SetupErrorExceptionTypeObj  = EtherCANExceptionClass("SetupError", InvalidParameterExceptionTypeObj);
    InvalidWaveformExceptionTypeObj = EtherCANExceptionClass("InvalidWaveformException", InvalidParameterExceptionTypeObj);
    ConnectionFailureExceptionTypeObj = EtherCANExceptionClass("ConnectionFailure", EtherCANExceptionTypeObj);
    SocketFailureExceptionTypeObj = EtherCANExceptionClass("SocketFailure", ConnectionFailureExceptionTypeObj);
    CommandTimeoutExceptionTypeObj = EtherCANExceptionClass("CommandTimeout", ConnectionFailureExceptionTypeObj);
    CAN_OverflowExceptionTypeObj = EtherCANExceptionClass("CAN_BufferOverflowException", ConnectionFailureExceptionTypeObj);
    ProtectionErrorExceptionTypeObj = EtherCANExceptionClass("ProtectionError", InvalidStateExceptionTypeObj);
    HardwareProtectionErrorExceptionTypeObj = EtherCANExceptionClass("HardwareProtectionError", MovementErrorExceptionTypeObj);





    register_exception_translator<EtherCANException>(&translate_interface_error);

    // include summary function
    def("getGridStateSummary", wrapGetGridStateSummary);

    //..........................................................................
    // Include shared Boost.Python module content
#include "FpuBPShared_ModuleContent.C"
    //..........................................................................

    class_<EtherCANInterfaceConfig>("EtherCANInterfaceConfig", init<>())
    .def_readwrite("num_fpus", &EtherCANInterfaceConfig::num_fpus)
    .def_readwrite("alpha_datum_offset", &EtherCANInterfaceConfig::alpha_datum_offset)
    .def_readwrite("motor_minimum_frequency", &EtherCANInterfaceConfig::motor_minimum_frequency)
    .def_readwrite("motor_maximum_frequency", &EtherCANInterfaceConfig::motor_maximum_frequency)
    .def_readwrite("motor_max_start_frequency", &EtherCANInterfaceConfig::motor_max_start_frequency)
    .def_readwrite("motor_max_rel_increase", &EtherCANInterfaceConfig::motor_max_rel_increase)
    .def_readwrite("motor_max_step_difference", &EtherCANInterfaceConfig::motor_max_step_difference)
    .def_readwrite("logLevel", &EtherCANInterfaceConfig::logLevel)
    .def_readwrite("waveform_upload_pause_us", &EtherCANInterfaceConfig::waveform_upload_pause_us)
    .def_readwrite("firmware_version_address_offset", &EtherCANInterfaceConfig::firmware_version_address_offset)
    .def_readwrite("confirm_each_step", &EtherCANInterfaceConfig::confirm_each_step)
    .def_readwrite("configmotion_confirmation_period", &EtherCANInterfaceConfig::configmotion_confirmation_period)
    .def_readwrite("configmotion_max_retry_count", &EtherCANInterfaceConfig::configmotion_max_retry_count)
    .def_readwrite("configmotion_max_resend_count", &EtherCANInterfaceConfig::configmotion_max_resend_count)
    .def_readwrite("can_command_priority", &EtherCANInterfaceConfig::can_command_priority)
    .def_readwrite("min_bus_repeat_delay_ms", &EtherCANInterfaceConfig::min_bus_repeat_delay_ms)
    .def_readwrite("min_fpu_repeat_delay_ms", &EtherCANInterfaceConfig::min_fpu_repeat_delay_ms)
    .def_readwrite("SocketTimeOutSeconds", &EtherCANInterfaceConfig::SocketTimeOutSeconds)
    .def_readwrite("TCP_IdleSeconds", &EtherCANInterfaceConfig::TCP_IdleSeconds)
    .def_readwrite("TCP_KeepaliveIntervalSeconds", &EtherCANInterfaceConfig::TCP_KeepaliveIntervalSeconds)
    .def_readwrite("TCP_KeepaliveIntervalSeconds", &EtherCANInterfaceConfig::TCP_KeepaliveIntervalSeconds)
    .def_readwrite("fd_controllog", &EtherCANInterfaceConfig::fd_controllog)
    .def_readwrite("fd_txlog", &EtherCANInterfaceConfig::fd_txlog)
    .def_readwrite("fd_rxlog", &EtherCANInterfaceConfig::fd_rxlog);


    class_<WrapEtherCANInterface, boost::noncopyable>("EtherCANInterface", init<EtherCANInterfaceConfig>())
    .def("getNumFPUs", &WrapEtherCANInterface::getNumFPUs)
    .def("connect", &WrapEtherCANInterface::connectGateways)
    .def("disconnect", &WrapEtherCANInterface::disconnect)
    .def("deInitializeInterface", &WrapEtherCANInterface::deInitializeInterface)
    .def("initializeGrid", &WrapEtherCANInterface::wrap_initializeGrid)
    .def("resetFPUs", &WrapEtherCANInterface::wrap_resetFPUs)
    .def("pingFPUs", &WrapEtherCANInterface::wrap_pingFPUs)
    .def("findDatum", &WrapEtherCANInterface::wrap_findDatum)
    .def("startFindDatum", &WrapEtherCANInterface::wrap_startFindDatum)
    .def("waitFindDatum", &WrapEtherCANInterface::wrap_waitFindDatum)
    .def("configMotion", &WrapEtherCANInterface::configMotionWithDict)
    .def("executeMotion", &WrapEtherCANInterface::wrap_executeMotion)
    .def("startExecuteMotion", &WrapEtherCANInterface::wrap_startExecuteMotion)
    .def("waitExecuteMotion", &WrapEtherCANInterface::wrap_waitExecuteMotion)
    .def("getGridState", &WrapEtherCANInterface::wrap_getGridState)
    .def("repeatMotion", &WrapEtherCANInterface::wrap_repeatMotion)
    .def("reverseMotion", &WrapEtherCANInterface::wrap_reverseMotion)
    .def("abortMotion", &WrapEtherCANInterface::wrap_abortMotion)
    .def("freeBetaCollision", &WrapEtherCANInterface::wrap_freeBetaCollision)
    .def("setUStepLevel", &WrapEtherCANInterface::wrap_setUStepLevel)
    .def("readRegister", &WrapEtherCANInterface::wrap_readRegister)
    .def("getFirmwareVersion", &WrapEtherCANInterface::wrap_getFirmwareVersion)
    .def("enableBetaCollisionProtection", &WrapEtherCANInterface::wrap_enableBetaCollisionProtection)
    .def("lockFPU", &WrapEtherCANInterface::wrap_lockFPU)
    .def("unlockFPU", &WrapEtherCANInterface::wrap_unlockFPU)
    .def("writeSerialNumber", &WrapEtherCANInterface::wrap_writeSerialNumber)
    .def("readSerialNumbers", &WrapEtherCANInterface::wrap_readSerialNumbers)

    
    .def("getMinFirmwareVersion", &WrapEtherCANInterface::wrap_getMinFirmwareVersion)
    .def("resetStepCounters", &WrapEtherCANInterface::wrap_resetStepCounters)
    .def("enableMove", &WrapEtherCANInterface::wrap_enableMove)
    .def("enableAlphaLimitProtection", &WrapEtherCANInterface::wrap_enableAlphaLimitProtection)
    .def("freeAlphaLimitBreach", &WrapEtherCANInterface::wrap_freeAlphaLimitBreach)
    .def("setStepsPerSegment", &WrapEtherCANInterface::wrap_setStepsPerSegment)
    .def("setTicksPerSegment", &WrapEtherCANInterface::wrap_setTicksPerSegment)
    .def("checkIntegrity", &WrapEtherCANInterface::wrap_checkIntegrity)

    .def_readonly("NumFPUs", &WrapEtherCANInterface::getNumFPUs)
    ;

}
