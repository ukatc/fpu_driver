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
// NAME FpuBPShared_ModuleContent.C
//
// Contains FPU Boost.Python module items to be shared between the ethercanif
// and griddriver Boost.Python BOOST_PYTHON_MODULE() wrapper modules.
//
// IMPORTANT: This file must be #include-d directly inside the
// BOOST_PYTHON_MODULE() module code. This is a bit of a non-standard approach,
// but is simple and effective.
//
// N.B. It isn't immediately clear how this might be done more nicely via
// header file inclusion or similar - had originally started with an approach
// of turning the items into macros in a header file which could be #include'd,
// but this was clunky and resulted in long multi-line macros.
//
////////////////////////////////////////////////////////////////////////////////


    enum_<E_LogLevel>("E_LogLevel")
    .value("LOG_ERROR",               LOG_ERROR)
    .value("LOG_INFO",                LOG_INFO)
    .value("LOG_GRIDSTATE",           LOG_GRIDSTATE)
    .value("LOG_VERBOSE",             LOG_VERBOSE)
    .value("LOG_DEBUG",               LOG_DEBUG)
    .value("LOG_TRACE_CAN_MESSAGES",  LOG_TRACE_CAN_MESSAGES)
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
    .export_values();
