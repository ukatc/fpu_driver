// -*- mode: c++ -*-
////////////////////////////////////////////////////////////////////////////////
//
// Copyright 2017 UKRI. See file "LICENSE" for license information.
//
// Who       When        What
// --------  ----------  -------------------------------------------------------
//------------------------------------------------------------------------------

////////////////////////////////////////////////////////////////////////////////
// NAME ethercanif.C
//
// This file implements the Python wrappers for the grid driver and EtherCAN
// interface classes for the MOONS instrument fibre positioner unit.
// NOTE: These classes, and the Boost.Python data wrapper classes which are
// shared between them (the wrapper versions of E_FPU_STATE, E_InterfaceState
// and so forth) are all in this combined Boost.Python module file (rather than
// the ethercanif and grid driver modules being separate) because the shared
// object wrappers (E_FPU_STATE, E_InterfaceState) can only be defined once.
//
////////////////////////////////////////////////////////////////////////////////

#include "WrapEtherCANInterface.h"
#include "WrappedGridDriver.h"

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

//------------------------------------------------------------------------------
E_GridState wrapGetGridStateSummary(WrapGridState& grid_state)
{
    return getGridStateSummary(grid_state);
}

//------------------------------------------------------------------------------
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
    case DE_ERROR_UNKNOWN:
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

}

//------------------------------------------------------------------------------
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wmissing-declarations"
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


//==============================================================================
BOOST_PYTHON_MODULE(ethercanif)
{
    using namespace boost::python;

    scope().attr("__version__") = (strlen(VERSION) > 1) ?  (((const char*)VERSION) +1) : "?.?.?";

    scope().attr("CAN_PROTOCOL_VERSION") = CAN_PROTOCOL_VERSION;

    scope().attr("DEFAULT_WAVEFORM_RULESET_VERSION") = DEFAULT_WAVEFORM_RULESET_VERSION;

    // define the exception hierarchy
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

    //--------------------------------------------------------------------------
    // Data object wrapper definitions - used by both ethercanif and grid driver

    enum_<E_FPU_STATE>("E_FPU_STATE")
    .value("FPST_UNKNOWN", FPST_UNKNOWN)
    .value("FPST_UNINITIALIZED", FPST_UNINITIALIZED)
    .value("FPST_LOCKED", FPST_LOCKED)
    .value("FPST_DATUM_SEARCH", FPST_DATUM_SEARCH)
    .value("FPST_AT_DATUM", FPST_AT_DATUM)
    .value("FPST_LOADING", FPST_LOADING)
    .value("FPST_READY_FORWARD", FPST_READY_FORWARD)
    .value("FPST_READY_REVERSE", FPST_READY_REVERSE)
    .value("FPST_MOVING", FPST_MOVING)
    .value("FPST_RESTING", FPST_RESTING)
    .value("FPST_ABORTED", FPST_ABORTED)
    .value("FPST_OBSTACLE_ERROR", FPST_OBSTACLE_ERROR)
    .export_values();

    enum_<E_InterfaceState>("E_InterfaceState")
    .value("DS_UNINITIALIZED", DS_UNINITIALIZED)
    .value("DS_UNCONNECTED", DS_UNCONNECTED)
    .value("DS_CONNECTED", DS_CONNECTED)
    .value("DS_ASSERTION_FAILED", DS_ASSERTION_FAILED)
    .export_values();

    enum_<E_LogLevel>("E_LogLevel")
    .value("LOG_ERROR",               LOG_ERROR)
    .value("LOG_INFO",                LOG_INFO)
    .value("LOG_GRIDSTATE",           LOG_GRIDSTATE)
    .value("LOG_VERBOSE",             LOG_VERBOSE)
    .value("LOG_DEBUG",               LOG_DEBUG)
    .value("LOG_TRACE_CAN_MESSAGES",  LOG_TRACE_CAN_MESSAGES)
    .export_values();

    /* The following codes are used in the last_status flag.  These
       values depend on the firmware protocol. It is legitimate to use
       them for engineering and troubleshooting but thy should *not*
       be used by normal EtherCAN interface client code.
     */
    enum_<E_MOC_ERRCODE>("E_MOC_ERRCODE")
    .value("MCE_FPU_OK", MCE_FPU_OK)
    .value("MCE_WARN_COLLISION_DETECTED", MCE_WARN_COLLISION_DETECTED)
    .value("MCE_WARN_LIMIT_SWITCH_BREACH", MCE_WARN_LIMIT_SWITCH_BREACH)
    .value("MCE_ERR_INVALID_COMMAND", MCE_ERR_INVALID_COMMAND)
    .value("MCE_NOTIFY_COMMAND_IGNORED", MCE_NOTIFY_COMMAND_IGNORED)
    .value("MCE_ERR_WAVEFORM_NOT_READY", MCE_ERR_WAVEFORM_NOT_READY)
    .value("MCE_WAVEFORM_REJECTED", MCE_WAVEFORM_REJECTED)
    .value("MCE_WARN_STEP_TIMING_ERROR", MCE_WARN_STEP_TIMING_ERROR)
    .value("MCE_ERR_INVALID_PARAMETER", MCE_ERR_INVALID_PARAMETER)
    .value("MCE_ERR_DATUM_TIME_OUT", MCE_ERR_DATUM_TIME_OUT)
    .value("MCE_NOTIFY_DATUM_ALPHA_ONLY", MCE_NOTIFY_DATUM_ALPHA_ONLY)
    .value("MCE_NOTIFY_DATUM_BETA_ONLY", MCE_NOTIFY_DATUM_BETA_ONLY)
    .value("MCE_ERR_AUTO_DATUM_UNINITIALIZED", MCE_ERR_AUTO_DATUM_UNINITIALIZED)
    .value("MCE_ERR_DATUM_ON_LIMIT_SWITCH", MCE_ERR_DATUM_ON_LIMIT_SWITCH)
    .value("MCE_ERR_CAN_OVERFLOW_HW", MCE_ERR_CAN_OVERFLOW_HW)
    .value("MCE_ERR_CAN_OVERFLOW_SW", MCE_ERR_CAN_OVERFLOW_SW)
    .value("MCE_NO_CONFIRMATION_EXPECTED", MCE_NO_CONFIRMATION_EXPECTED)
    .value("MCE_COMMAND_TIMEDOUT", MCE_COMMAND_TIMEDOUT)
    .export_values();

    enum_<E_WAVEFORM_ERRCODE>("E_WAVEFORM_ERRCODE")
    .value("WAVEFORM_OK", WAVEFORM_OK)
    .value("WAVEFORM_TOO_BIG", WAVEFORM_TOO_BIG)
    .value("WAVEFORM_SEQUENCE", WAVEFORM_SEQUENCE)
    .value("WAVEFORM_BADVALUE", WAVEFORM_BADVALUE)
    .value("WAVEFORM_UNDEFINED", WAVEFORM_UNDEFINED)
    .export_values();

    enum_<E_CAN_COMMAND>("E_CAN_COMMAND")
    .value("CCMD_NO_COMMAND", CCMD_NO_COMMAND)
    .value("CCMD_CONFIG_MOTION", CCMD_CONFIG_MOTION)
    .value("CCMD_EXECUTE_MOTION", CCMD_EXECUTE_MOTION)
    .value("CCMD_ABORT_MOTION", CCMD_ABORT_MOTION)
    .value("CCMD_READ_REGISTER", CCMD_READ_REGISTER)
    .value("CCMD_READ_SERIAL_NUMBER", CCMD_READ_SERIAL_NUMBER)
    .value("CCMD_WRITE_SERIAL_NUMBER", CCMD_WRITE_SERIAL_NUMBER)
    .value("CCMD_PING_FPU", CCMD_PING_FPU)
    .value("CCMD_RESET_FPU", CCMD_RESET_FPU)
    .value("CCMD_FIND_DATUM", CCMD_FIND_DATUM)
    .value("CCMD_REPEAT_MOTION", CCMD_REPEAT_MOTION)
    .value("CCMD_REVERSE_MOTION", CCMD_REVERSE_MOTION)
    .value("CCMD_ENABLE_BETA_COLLISION_PROTECTION", CCMD_ENABLE_BETA_COLLISION_PROTECTION)
    .value("CCMD_FREE_BETA_COLLISION", CCMD_FREE_BETA_COLLISION)
    .value("CCMD_SET_USTEP_LEVEL", CCMD_SET_USTEP_LEVEL)
#if    (CAN_PROTOCOL_VERSION == 1)
    .value("CCMD_GET_STEPS_ALPHA", CCMD_GET_STEPS_ALPHA)
    .value("CCMD_GET_STEPS_BETA", CCMD_GET_STEPS_BETA)
    .value("CCMD_GET_ERROR_ALPHA", CCMD_GET_ERROR_ALPHA)
    .value("CCMD_GET_ERROR_BETA", CCMD_GET_ERROR_BETA)
#else
    .value("CCMD_LOCK_UNIT", CCMD_LOCK_UNIT)
    .value("CCMD_UNLOCK_UNIT", CCMD_UNLOCK_UNIT)
    .value("CCMD_GET_FIRMWARE_VERSION", CCMD_GET_FIRMWARE_VERSION)
    .value("CCMD_CHECK_INTEGRITY", CCMD_CHECK_INTEGRITY)
    .value("CCMD_FREE_ALPHA_LIMIT_BREACH", CCMD_FREE_ALPHA_LIMIT_BREACH)
    .value("CCMD_ENABLE_ALPHA_LIMIT_PROTECTION", CCMD_ENABLE_ALPHA_LIMIT_PROTECTION)
    .value("CCMD_SET_TICKS_PER_SEGMENT", CCMD_SET_TICKS_PER_SEGMENT)
    .value("CCMD_SET_STEPS_PER_SEGMENT", CCMD_SET_STEPS_PER_SEGMENT)
    .value("CCMD_ENABLE_MOVE", CCMD_ENABLE_MOVE)
    .value("CCMD_RESET_STEPCOUNTER", CCMD_RESET_STEPCOUNTER)
#endif

#if    (CAN_PROTOCOL_VERSION == 1)
    .value("CMSG_FINISHED_MOTION", CMSG_FINISHED_MOTION)
    .value("CMSG_FINISHED_DATUM", CMSG_FINISHED_DATUM)
    .value("CMSG_WARN_COLLISION_BETA", CMSG_WARN_COLLISION_BETA)
    .value("CMSG_WARN_LIMIT_ALPHA", CMSG_WARN_LIMIT_ALPHA)
#else
    .value("CMSG_FINISHED_MOTION", CMSG_FINISHED_MOTION)
    .value("CMSG_FINISHED_DATUM", CMSG_FINISHED_DATUM)
    .value("CMSG_WARN_COLLISION_BETA", CMSG_WARN_COLLISION_BETA)
    .value("CMSG_WARN_LIMIT_ALPHA", CMSG_WARN_LIMIT_ALPHA)
    .value("CMSG_WARN_TIMEOUT_DATUM", CMSG_WARN_TIMEOUT_DATUM)
#endif
    .value("NUM_CAN_COMMANDS", NUM_CAN_COMMANDS)
    .export_values();

    enum_<E_EtherCANErrCode>("E_EtherCANErrCode")
    .value("DE_OK",DE_OK)
    .value("DE_INTERFACE_NOT_INITIALIZED",DE_INTERFACE_NOT_INITIALIZED)
    .value("DE_INTERFACE_ALREADY_INITIALIZED",DE_INTERFACE_ALREADY_INITIALIZED)
    .value("DE_NO_CONNECTION",DE_NO_CONNECTION)
    .value("DE_INSUFFICENT_NUM_GATEWAYS", DE_INSUFFICENT_NUM_GATEWAYS)
    .value("DE_STILL_BUSY",DE_STILL_BUSY)
    .value("DE_MAX_RETRIES_EXCEEDED", DE_MAX_RETRIES_EXCEEDED)
    .value("DE_CAN_COMMAND_TIMEOUT_ERROR", DE_CAN_COMMAND_TIMEOUT_ERROR)
    .value("DE_FIRMWARE_CAN_BUFFER_OVERFLOW", DE_FIRMWARE_CAN_BUFFER_OVERFLOW)
    .value("DE_UNRESOLVED_COLLISION",DE_UNRESOLVED_COLLISION)
    .value("DE_NEW_COLLISION", DE_NEW_COLLISION)
    .value("DE_NEW_LIMIT_BREACH", DE_NEW_LIMIT_BREACH)
    .value("DE_INTERFACE_NOT_INITIALIZED",DE_INTERFACE_NOT_INITIALIZED)
    .value("DE_FPU_NOT_INITIALIZED",DE_FPU_NOT_INITIALIZED)
    .value("DE_INTERFACE_ALREADY_CONNECTED",DE_INTERFACE_ALREADY_CONNECTED)
    .value("DE_INTERFACE_STILL_CONNECTED",DE_INTERFACE_ALREADY_CONNECTED)
    .value("DE_ASSERTION_FAILED",DE_ASSERTION_FAILED)
    .value("DE_INVALID_WAVEFORM", DE_INVALID_WAVEFORM)
    .value("DE_INVALID_WAVEFORM_TOO_MANY_SECTIONS", DE_INVALID_WAVEFORM_TOO_MANY_SECTIONS)
    .value("DE_INVALID_WAVEFORM_RAGGED", DE_INVALID_WAVEFORM_RAGGED)
    .value("DE_INVALID_WAVEFORM_STEPCOUNT_TOO_LARGE", DE_INVALID_WAVEFORM_STEPCOUNT_TOO_LARGE)
    .value("DE_INVALID_WAVEFORM_CHANGE", DE_INVALID_WAVEFORM_CHANGE)
    .value("DE_INVALID_WAVEFORM_TAIL", DE_INVALID_WAVEFORM_TAIL)
    .value("DE_WAVEFORM_NOT_READY", DE_WAVEFORM_NOT_READY)
    .value("DE_NO_MOVABLE_FPUS", DE_NO_MOVABLE_FPUS)
    .value("DE_WAIT_TIMEOUT", DE_WAIT_TIMEOUT)
    .value("DE_IN_ABORTED_STATE", DE_IN_ABORTED_STATE)
    .value("DE_MOVEMENT_ABORTED", DE_MOVEMENT_ABORTED)
    .value("DE_DATUM_COMMAND_HW_TIMEOUT", DE_DATUM_COMMAND_HW_TIMEOUT)
    .value("DE_ALPHA_ARM_ON_LIMIT_SWITCH", DE_ALPHA_ARM_ON_LIMIT_SWITCH)
    .value("DE_INCONSISTENT_STEP_COUNT", DE_INCONSISTENT_STEP_COUNT)
    .value("DE_HW_ALPHA_ARM_ON_LIMIT_SWITCH", DE_HW_ALPHA_ARM_ON_LIMIT_SWITCH)
    .value("DE_FPUS_LOCKED", DE_FPUS_LOCKED)
    .value("DE_STEP_TIMING_ERROR", DE_STEP_TIMING_ERROR)
    .value("DE_INVALID_FPU_ID", DE_INVALID_FPU_ID)
    .value("DE_INVALID_FPU_STATE", DE_INVALID_FPU_STATE)
    .value("DE_PROTECTION_ERROR", DE_PROTECTION_ERROR)
    .value("DE_INVALID_PAR_VALUE", DE_INVALID_PAR_VALUE)
    .value("DE_DUPLICATE_SERIAL_NUMBER", DE_DUPLICATE_SERIAL_NUMBER)
    .value("DE_INVALID_CONFIG", DE_INVALID_CONFIG)
    .value("DE_SYNC_CONFIG_FAILED", DE_SYNC_CONFIG_FAILED)
    .value("DE_INVALID_INTERFACE_STATE", DE_INVALID_INTERFACE_STATE)
    .value("DE_OUT_OF_MEMORY", DE_OUT_OF_MEMORY)
    .value("DE_RESOURCE_ERROR", DE_RESOURCE_ERROR)
    .value("DE_FIRMWARE_UNIMPLEMENTED", DE_FIRMWARE_UNIMPLEMENTED)
    .value("DE_ERROR_UNKNOWN", DE_ERROR_UNKNOWN)
    .export_values();

    enum_<E_GridState>("E_GridState")
    .value("GS_UNKNOWN", GS_UNKNOWN)
    .value("GS_UNINITIALIZED", GS_UNINITIALIZED)
    .value("GS_LEAVING_DATUM", GS_LEAVING_DATUM)
    .value("GS_ABOVE_DATUM", GS_ABOVE_DATUM)
    .value("GS_DATUM_SEARCH", GS_DATUM_SEARCH)
    .value("GS_AT_DATUM", GS_AT_DATUM)
    .value("GS_LOADING", GS_LOADING)
    .value("GS_READY_FORWARD", GS_READY_FORWARD)
    .value("GS_READY_REVERSE", GS_READY_REVERSE)
    .value("GS_MOVING", GS_MOVING)
    .value("GS_FINISHED", GS_FINISHED)
    .value("GS_COLLISION", GS_COLLISION)
    .value("GS_ABORTED", GS_ABORTED)
    .export_values();

    // direction of a movement request from the user
    enum_<E_REQUEST_DIRECTION>("E_REQUEST_DIRECTION")
    .value("REQD_ANTI_CLOCKWISE", REQD_ANTI_CLOCKWISE)
    .value("REQD_CLOCKWISE", REQD_CLOCKWISE)
    .export_values();

    enum_<E_DATUM_TIMEOUT_FLAG>("E_DATUM_TIMEOUT_FLAG")
    .value("DATUM_TIMEOUT_ENABLE", DATUM_TIMEOUT_ENABLE)
    .value("DATUM_TIMEOUT_DISABLE", DATUM_TIMEOUT_DISABLE)
    .export_values();

    // direction of the current or last actually recorded movement of each FPU
    enum_<E_MOVEMENT_DIRECTION>("E_MOVEMENT_DIRECTION")
    .value("DIRST_UNKNOWN", DIRST_UNKNOWN)
    .value("DIRST_ANTI_CLOCKWISE", DIRST_ANTI_CLOCKWISE)
    .value("DIRST_CLOCKWISE", DIRST_CLOCKWISE)
    // the following two might not be needed
    .value("DIRST_RESTING_LAST_CW", DIRST_RESTING_LAST_CW)
    .value("DIRST_RESTING_LAST_ACW", DIRST_RESTING_LAST_ACW)
    .export_values();

    // selection which arms should perform a datum operation
    enum_<E_DATUM_SELECTION>("E_DATUM_SELECTION")
    .value("DASEL_BOTH",   DASEL_BOTH)
    .value("DASEL_ALPHA",  DASEL_ALPHA)
    .value("DASEL_BETA",   DASEL_BETA)
    .export_values();

    // operation mode for datum command
    enum_<E_DATUM_SEARCH_DIRECTION>("E_DATUM_SEARCH_DIRECTION")
    .value("SEARCH_CLOCKWISE",       SEARCH_CLOCKWISE)
    .value("SEARCH_ANTI_CLOCKWISE",  SEARCH_ANTI_CLOCKWISE)
    .value("SEARCH_AUTO",            SEARCH_AUTO)
    .value("SKIP_FPU",               SKIP_FPU)
    .export_values();

    class_<WrapFPUState>("FPUState")
    .def_readonly("state", &WrapFPUState::state)
    .def_readonly("last_command", &WrapFPUState::last_command)
    .def_readonly("last_status", &WrapFPUState::last_status)
    .def_readonly("alpha_steps", &WrapFPUState::alpha_steps)
    .def_readonly("beta_steps", &WrapFPUState::beta_steps)
    .def_readonly("ping_ok", &WrapFPUState::ping_ok)
    .def_readonly("alpha_deviation", &WrapFPUState::alpha_deviation)
    .def_readonly("beta_deviation", &WrapFPUState::beta_deviation)
    .def_readonly("timeout_count", &WrapFPUState::timeout_count)
    .def_readonly("num_active_timeouts", &WrapFPUState::num_active_timeouts)
    .def_readonly("sequence_number", &WrapFPUState::sequence_number)
    .def_readonly("alpha_was_referenced", &WrapFPUState::alpha_was_referenced)
    .def_readonly("beta_was_referenced", &WrapFPUState::beta_was_referenced)
    .def_readonly("is_locked", &WrapFPUState::is_locked)
    .def_readonly("alpha_datum_switch_active", &WrapFPUState::alpha_datum_switch_active)
    .def_readonly("beta_datum_switch_active", &WrapFPUState::beta_datum_switch_active)
    .def_readonly("at_alpha_limit", &WrapFPUState::at_alpha_limit)
    .def_readonly("beta_collision", &WrapFPUState::beta_collision)
    .def_readonly("direction_alpha", &WrapFPUState::direction_alpha)
    .def_readonly("num_waveform_segments", &WrapFPUState::num_waveform_segments)
    .def_readonly("waveform_status", &WrapFPUState::waveform_status)
    .def_readonly("direction_beta", &WrapFPUState::direction_beta)
    .def_readonly("waveform_valid", &WrapFPUState::waveform_valid)
    .def_readonly("waveform_ready", &WrapFPUState::waveform_ready)
    .def_readonly("waveform_reversed", &WrapFPUState::waveform_reversed)
    .def_readonly("pending_command_set", &WrapFPUState::pending_command_set)
    .def_readonly("register_address", &WrapFPUState::register_address)
    .def_readonly("fw_version_major", &WrapFPUState::fw_version_major)
    .def_readonly("fw_version_minor", &WrapFPUState::fw_version_minor)
    .def_readonly("fw_version_patch", &WrapFPUState::fw_version_patch)
    .def_readonly("fw_date_year", &WrapFPUState::fw_date_year)
    .def_readonly("fw_date_month", &WrapFPUState::fw_date_month)
    .def_readonly("fw_date_day", &WrapFPUState::fw_date_day)
    .def_readonly("register_value", &WrapFPUState::register_value)
    .def_readonly("serial_number", &WrapFPUState::serial_number)
    .def_readonly("sequence_number", &WrapFPUState::sequence_number)
    .def_readonly("num_active_timeouts", &WrapFPUState::num_active_timeouts)
    .def_readonly("crc32", &WrapFPUState::crc32)
    .def_readonly("checksum_ok", &WrapFPUState::checksum_ok)
    .def_readonly("last_updated", &WrapFPUState::last_updated_sec)
    .def("__repr__", &WrapFPUState::to_repr)
    ;

    class_<std::vector<WrapFPUState> >("StateVec")
    .def(vector_indexing_suite<std::vector<WrapFPUState> >());

    class_<std::vector<long> >("IntVec")
    .def(vector_indexing_suite<std::vector<long> >());

    class_<std::vector<WrapGatewayAddress> >("GatewayAddressVec")
    .def(vector_indexing_suite<std::vector<WrapGatewayAddress> >());

    class_<WrapGridState>("GridState")
    .def_readonly("FPU", &WrapGridState::getStateVec)
    .def_readonly("Counts", &WrapGridState::getCounts)
    .def_readonly("count_timeout", &WrapGridState::count_timeout)
    .def_readonly("count_can_overflow", &WrapGridState::count_can_overflow)
    .def_readonly("count_pending", &WrapGridState::count_pending)
    .def_readonly("interface_state", &WrapGridState::interface_state)
    .def("__str__", &WrapGridState::to_string)
    .def("__repr__", &WrapGridState::to_repr);

    class_<WrapGatewayAddress>("GatewayAddress", init<const char*, int>())
    .def_readwrite("ip", &WrapGatewayAddress::ip)
    .def_readwrite("port", &WrapGatewayAddress::port);

    //--------------------------------------------------------------------------
    // GridDriver wrapper definitions
    // All of these wrapped GridDriver functions support Python named and
    // arbitrarily-ordered arguments, and have argument defaulting.
    //
    // WrappedGridDriver constructor notes:
    //   - The WrappedGridDriver Boost.Python constructor also supports named
    //     and arbitrarily-ordered arguments. However, this has required a more
    //     complex Boost.Python custom constructor approach to be used:
    //       - Inheriting of UnprotectedGridDriver's constructor using C++11
    //         constructor inheritance
    //       - WrappedGridDriver::initWrapper() function
    //       - In the BOOST_PYTHON_MODULE()'s "class_" statement, use of the
    //         "no_init" specifier, and specifying
    //         WrappedGridDriver::initWrapper() as the custom class constructor
    //         function (which uses a boost::shared_ptr construct)
    //       - IMPORTANT: All of the constructor argument lists must exactly
    //         correspond in the various places
    //   - See e.g. the following web reference for more info:
    //     https://stackoverflow.com/questions/18793952/boost-python-how-do-i-provide-a-custom-constructor-wrapper-function

    class_<WrappedGridDriver, boost::shared_ptr<WrappedGridDriver> >
        ("GridDriver", no_init)
    .def("__init__", make_constructor(&WrappedGridDriver::initWrapper,
                                        bp::default_call_policies(),
            (bp::arg("nfpus") = DEFAULT_NUM_FPUS,
            bp::arg("SocketTimeOutSeconds") = 20.0,
            bp::arg("confirm_each_step") = false,
            bp::arg("waveform_upload_pause_us") = 0,
            bp::arg("configmotion_max_retry_count") = 5,
            bp::arg("configmotion_max_resend_count") = 10,
            bp::arg("min_bus_repeat_delay_ms") = 0,
            bp::arg("min_fpu_repeat_delay_ms") = 1,
            bp::arg("alpha_datum_offset") = ALPHA_DATUM_OFFSET,
            bp::arg("motor_minimum_frequency") = MOTOR_MIN_STEP_FREQUENCY,
            bp::arg("motor_maximum_frequency") = MOTOR_MAX_STEP_FREQUENCY,
            bp::arg("motor_max_start_frequency") = MOTOR_MAX_START_FREQUENCY,
            bp::arg("motor_max_rel_increase") = MAX_ACCELERATION_FACTOR,
            bp::arg("motor_max_step_difference") = MAX_STEP_DIFFERENCE)))

    .def("initialize", &WrappedGridDriver::wrapped_initialize,
            (bp::arg("logLevel") = DEFAULT_LOGLEVEL,
            bp::arg("log_dir") = DEFAULT_LOGDIR,
            bp::arg("firmware_version_address_offset") = 0x61,
            bp::arg("protection_logfile") = "_" DEFAULT_START_TIMESTAMP "-fpu_protection.log",
            bp::arg("control_logfile") = "_" DEFAULT_START_TIMESTAMP "-fpu_control.log",
            bp::arg("tx_logfile") = "_" DEFAULT_START_TIMESTAMP "-fpu_tx.log",
            bp::arg("rx_logfile") = "_" DEFAULT_START_TIMESTAMP "-fpu_rx.log",
            bp::arg("start_timestamp") = DEFAULT_START_TIMESTAMP,
            bp::arg("mockup") = false))

    .def("getGridState", &WrappedGridDriver::wrapped_getGridState)

    .def("connect", &WrappedGridDriver::wrapped_connect,
            // TODO: Add defaulting to the following argument - see
            // FpuGridDriver.py function equivalent, FPU grid driver
            // documentation etc – see DEFAULT_GATEWAY_ADDRESS_LIST /
            // MOCK_GATEWAY_ADDRESS_LIST stuff – need to use a bp::list
            // somehow I think - or, check if the argument is empty and
            // default it if so
            (bp::arg("address_list")))

    .def("setUStepLevel", &WrappedGridDriver::wrapped_setUStepLevel,
            (bp::arg("ustep_level"),
            bp::arg("grid_state"),
            bp::arg("fpuset") = bp::list()))

    .def("setTicksPerSegment", &WrappedGridDriver::wrapped_setTicksPerSegment,
            (bp::arg("nticks"),
            bp::arg("grid_state"),
            bp::arg("fpuset") = bp::list()))

    .def("setStepsPerSegment", &WrappedGridDriver::wrapped_setStepsPerSegment,
            (bp::arg("min_steps"),
            bp::arg("max_steps"),
            bp::arg("grid_state"),
            bp::arg("fpuset") = bp::list()))

    .def("findDatum", &WrappedGridDriver::wrapped_findDatum,
            (bp::arg("grid_state"),
            bp::arg("search_modes") = bp::dict(),
            bp::arg("selected_arm") = DASEL_BOTH,
            bp::arg("fpuset") = bp::list(),
            bp::arg("soft_protection") = true,
            bp::arg("count_protection") = true,
            bp::arg("support_uninitialized_auto") = true,
            bp::arg("timeout") = DATUM_TIMEOUT_ENABLE))

    .def("resetFPUs", &WrappedGridDriver::wrapped_resetFPUs,
            (bp::arg("grid_state"),
            bp::arg("fpuset") = bp::list()))

    .def("resetStepCounters", &WrappedGridDriver::wrapped_resetStepCounters,
            (bp::arg("new_alpha_steps"),
            bp::arg("new_beta_steps"),
            bp::arg("grid_state"),
            bp::arg("fpuset") = bp::list()))

    .def("readRegister", &WrappedGridDriver::wrapped_readRegister,
            (bp::arg("address"),
            bp::arg("grid_state"),
            bp::arg("fpuset") = bp::list()))

    .def("pingFPUs", &WrappedGridDriver::wrapped_pingFPUs,
            (bp::arg("grid_state"),
            bp::arg("fpuset") = bp::list()))

    .def("readSerialNumbers", &WrappedGridDriver::wrapped_readSerialNumbers,
            (bp::arg("grid_state"),
            bp::arg("fpuset") = bp::list()))

    .def("writeSerialNumber", &WrappedGridDriver::wrapped_writeSerialNumber,
            (bp::arg("fpu_id"),
            bp::arg("snstring"),
            bp::arg("grid_state")))

    .def("configMotion", &WrappedGridDriver::wrapped_configMotion,
            (bp::arg("wavetable"),
            bp::arg("grid_state"),
            bp::arg("fpuset") = bp::list(),
            bp::arg("soft_protection") = true,
            bp::arg("allow_uninitialized") = false,
            bp::arg("ruleset_version") = DEFAULT_WAVEFORM_RULESET_VERSION,
            // TODO: The following arguments are from FpuGridDriver.py ->
            // configMotion() - keep? (N.B. They aren't documented in the
            // FPU driver manual of January 22, 2020)
            bp::arg("warn_unsafe") = true,
            bp::arg("verbosity") = 3))

    .def("executeMotion", &WrappedGridDriver::wrapped_executeMotion,
            (bp::arg("grid_state"),
            bp::arg("fpuset") = bp::list(),
            // TODO: sync_command default is true here and in FpuGridDriver.py,
            // but is shown as False in grid driver document
            bp::arg("sync_command") = true))

    .def("abortMotion", &WrappedGridDriver::wrapped_abortMotion,
            (bp::arg("grid_state"),
            bp::arg("fpuset") = bp::list(),
            bp::arg("sync_command") = true))

    .def("freeBetaCollision", &WrappedGridDriver::wrapped_freeBetaCollision,
            (bp::arg("fpu_id"),
            bp::arg("direction"),
            bp::arg("grid_state"),
            bp::arg("soft_protection") = true))

    .def("enableBetaCollisionProtection",
                    &WrappedGridDriver::wrapped_enableBetaCollisionProtection,
            (bp::arg("grid_state")))

    .def("freeAlphaLimitBreach",
                    &WrappedGridDriver::wrapped_freeAlphaLimitBreach,
            (bp::arg("fpu_id"),
            bp::arg("direction"),
            bp::arg("grid_state"),
            bp::arg("soft_protection") = true))

    .def("enableAlphaLimitProtection",
                    &WrappedGridDriver::wrapped_enableAlphaLimitProtection,
            (bp::arg("grid_state")))

    .def("reverseMotion", &WrappedGridDriver::wrapped_reverseMotion,
            (bp::arg("grid_state"),
            bp::arg("fpuset") = bp::list(),
            bp::arg("soft_protection") = true))

    .def("repeatMotion", &WrappedGridDriver::wrapped_repeatMotion,
            (bp::arg("grid_state"),
            bp::arg("fpuset") = bp::list(),
            bp::arg("soft_protection") = true))

    .def("lockFPU", &WrappedGridDriver::wrapped_lockFPU,
            (bp::arg("fpu_id"),
            bp::arg("grid_state")))

    .def("unlockFPU", &WrappedGridDriver::wrapped_unlockFPU,
            (bp::arg("fpu_id"),
            bp::arg("grid_state")))

    .def("enableMove", &WrappedGridDriver::wrapped_enableMove,
            (bp::arg("fpu_id"),
            bp::arg("grid_state")))

    .def("checkIntegrity", &WrappedGridDriver::wrapped_checkIntegrity,
            (bp::arg("grid_state"),
            bp::arg("fpuset") = bp::list()))

    // TODO: Test function only - remove when no longer needed
    // Demonstrates named, arbitrarily-ordered arguments with defaulting
    .def("boostPythonDivide", &WrappedGridDriver::boostPythonDivide,
            (bp::arg("dividend") = 23.0,
            bp::arg("divisor") = 4.0))
    ;

    //--------------------------------------------------------------------------
    // EtherCANInterfaceConfig wrapper definitions
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
    .def_readwrite("fd_rxlog", &EtherCANInterfaceConfig::fd_rxlog)
    ;

    //--------------------------------------------------------------------------
    // WrapEtherCANInterface wrapper definitions
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

    //--------------------------------------------------------------------------
}

//==============================================================================
