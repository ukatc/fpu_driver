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
// Contains FPU Boost.Python module items which are shared between the
// ethercanif and griddriver Boost.Python BOOST_PYTHON_MODULE() wrapper
// modules.
//
// IMPORTANT: This C file must be #include-d directly inside the
// BOOST_PYTHON_MODULE() module code. Although this C file inclusion is a
// non-standard approach, it is a simple and effective way of achieving the
// required code sharing here.
//
// N.B. It isn't immediately clear how this sharing of BOOST_PYTHON_MODULE()
// content across multiple Boost.Python modules might be done more nicely via
// header file inclusion or similar - had originally started with an approach
// of turning the items into macros in a header file which could be #include'd,
// but this was clunky and resulted in long multi-line macros.
//
////////////////////////////////////////////////////////////////////////////////

// NOTE: Do not #include anything here

    scope().attr("DEFAULT_WAVEFORM_RULESET_VERSION") = DEFAULT_WAVEFORM_RULESET_VERSION;

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
    .def("__repr__", &WrapGridState::to_repr)
    ;

    class_<WrapGatewayAddress>("GatewayAddress", init<const char*, int>())
    .def_readwrite("ip", &WrapGatewayAddress::ip)
    .def_readwrite("port", &WrapGatewayAddress::port);

