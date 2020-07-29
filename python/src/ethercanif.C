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

//------------------------------------------------------------------------------


//==============================================================================
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

    //--------------------------------------------------------------------------
    // Include shared Boost.Python module content
#include "FpuBPShared_ModuleContent.C"

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

    .def("initialize", &WrappedGridDriver::initialize,
            (bp::arg("logLevel") = DEFAULT_LOGLEVEL,
            bp::arg("log_dir") = DEFAULT_LOGDIR,
            bp::arg("firmware_version_address_offset") = 0x61,
            bp::arg("protection_logfile") = "_" DEFAULT_START_TIMESTAMP "-fpu_protection.log",
            bp::arg("control_logfile") = "_" DEFAULT_START_TIMESTAMP "-fpu_control.log",
            bp::arg("tx_logfile") = "_" DEFAULT_START_TIMESTAMP "-fpu_tx.log",
            bp::arg("rx_logfile") = "_" DEFAULT_START_TIMESTAMP "-fpu_rx.log",
            bp::arg("start_timestamp") = DEFAULT_START_TIMESTAMP))

    .def("getGridState", &WrappedGridDriver::wrapped_getGridState)

    .def("connect", &WrappedGridDriver::wrapped_connect,
        // TODO: Add defaulting to the following argument - see 
        // FpuGridDriver.py function equivalent, FPU grid driver
        // documentation etc – see DEFAULT_GATEWAY_ADDRESS_LIST / 
        // MOCK_GATEWAY_ADDRESS_LIST stuff – need to use a bp::list
        // somehow I think
            (bp::arg("address_list")))

    .def("disconnect", &WrappedGridDriver::disconnect)

    .def("findDatum", &WrappedGridDriver::wrapped_findDatum,
            (bp::arg("grid_state"),
            bp::arg("search_modes") = bp::dict(),
            bp::arg("selected_arm") = DASEL_BOTH,
            bp::arg("fpuset") = bp::list(),
            bp::arg("soft_protection") = true,
            bp::arg("count_protection") = true,
            bp::arg("support_uninitialized_auto") = true,
            bp::arg("timeout") = DATUM_TIMEOUT_ENABLE))

    .def("pingFPUs", &WrappedGridDriver::wrapped_pingFPUs,
            (bp::arg("grid_state"),
            bp::arg("fpuset") = bp::list()))

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

    .def("enableMove", &WrappedGridDriver::wrapped_enableMove,
            (bp::arg("fpu_id"),
            bp::arg("grid_state")))

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
