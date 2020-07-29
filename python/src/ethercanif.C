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

    // Include shared Boost.Python module content
#include "FpuBPShared_ModuleContent.C"

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
