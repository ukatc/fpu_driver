// -*- mode: c++ -*-

////////////////////////////////////////////////////////////////////////////////
//
// Copyright 2020 UKRI. See file "LICENSE" for license information.
//
// Who       When        What
// --------  ----------  -------------------------------------------------------
// bwillemse 2020-07-29  Adapted so can work alongside new grid driver wrapper.
//------------------------------------------------------------------------------

////////////////////////////////////////////////////////////////////////////////
// NAME WrapEtherCANInterface.h
//
// Boost.Python wrappers for public EtherCAN functions.
//
////////////////////////////////////////////////////////////////////////////////

#include "EtherCANInterface.h"
#include "WrapperSharedBase.h"


//******************************************************************************
// TODO: Note: This Boost.Python wrapper for the EtherCAN interface (which
// provides the wrapper for the original Python grid driver in FpuGridDriver.py)
// is not supported when the flexible CAN mapping is enabled, because the Python
// grid driver won't be modified to support this feature
#ifndef FLEXIBLE_CAN_MAPPING // NOT FLEXIBLE_CAN_MAPPING
//******************************************************************************

//==============================================================================

class WrapEtherCANInterface : public EtherCANInterface,
                              protected WrapperSharedBase
{
private:
    const EtherCANInterfaceConfig config;

    const EtherCANInterfaceConfig &getConfig() const override;

public:
    WrapEtherCANInterface(const EtherCANInterfaceConfig _config) :
        EtherCANInterface(_config), config(_config)
    {
        E_EtherCANErrCode ecode = initializeInterface();
        checkInterfaceError(ecode);
    }

    E_EtherCANErrCode connectGateways(list& list_gateway_addresses);
    E_EtherCANErrCode configMotionWithDict(dict& dict_waveforms,
                                           WrapGridState& grid_state,
                                           list &fpu_list,
                                           bool allow_uninitialized = false,
					   int ruleset_version = DEFAULT_WAVEFORM_RULESET_VERSION);
    WrapGridState wrap_getGridState();
    E_EtherCANErrCode wrap_initializeGrid(WrapGridState& grid_state,
                                          list& fpu_list);
    E_EtherCANErrCode wrap_resetFPUs(WrapGridState& grid_state, list& fpu_list);
    E_EtherCANErrCode wrap_pingFPUs(WrapGridState& grid_state, list& fpu_list);
    E_EtherCANErrCode wrap_readRegister(int read_address,
                                        WrapGridState& grid_state,
                                        list& fpu_list);
    E_EtherCANErrCode wrap_getFirmwareVersion(WrapGridState& grid_state,
                                              list& fpu_list);
    E_EtherCANErrCode wrap_findDatum(WrapGridState& grid_state,
                                     dict &dict_modes, list& fpu_list,
                    E_DATUM_SELECTION arm_selection = DASEL_BOTH,
                    E_DATUM_TIMEOUT_FLAG timeout_flag = DATUM_TIMEOUT_ENABLE,
                    bool count_protection = true);
    E_EtherCANErrCode wrap_startFindDatum(WrapGridState& grid_state,
                                          dict& dict_modes, list& fpu_list,
                    E_DATUM_SELECTION arm_selection = DASEL_BOTH,
                    E_DATUM_TIMEOUT_FLAG timeout_flag = DATUM_TIMEOUT_ENABLE,
                    bool count_protection = true);
    E_EtherCANErrCode wrap_waitFindDatum(WrapGridState& grid_state,
                                         double max_wait_time, list& fpu_list);
    E_EtherCANErrCode wrap_executeMotion(WrapGridState& grid_state,
                                         list& fpu_list,
                                         bool sync_command = false);
    E_EtherCANErrCode wrap_startExecuteMotion(WrapGridState& grid_state,
                                              list& fpu_list,
                                              bool sync_command = false);
    E_EtherCANErrCode wrap_waitExecuteMotion(WrapGridState& grid_state,
                                             double max_wait_time,
                                             list& fpu_list);
    E_EtherCANErrCode wrap_repeatMotion(WrapGridState& grid_state,
                                        list& fpu_list);
    E_EtherCANErrCode wrap_reverseMotion(WrapGridState& grid_state,
                                         list& fpu_list);
    E_EtherCANErrCode wrap_abortMotion(WrapGridState& grid_state,
                                       list& fpu_list, bool sync_command = true);
    E_EtherCANErrCode wrap_enableMove(int fpu_id, WrapGridState& grid_state);
    E_EtherCANErrCode wrap_setUStepLevel(int ustep_level, WrapGridState& grid_state,
                                         list& fpu_list);
    E_EtherCANErrCode wrap_freeBetaCollision(int fpu_id,
                                             E_REQUEST_DIRECTION request_direction,
                                             WrapGridState& grid_state);
    E_EtherCANErrCode wrap_enableBetaCollisionProtection(WrapGridState& grid_state);
    E_EtherCANErrCode wrap_lockFPU(int fpu_id, WrapGridState& grid_state);
    E_EtherCANErrCode wrap_unlockFPU(int fpu_id, WrapGridState& grid_state);
    E_EtherCANErrCode wrap_readSerialNumbers(WrapGridState& grid_state,
                                             list& fpu_list);
    E_EtherCANErrCode wrap_writeSerialNumber(int fpu_id, str serial_number,
                                             WrapGridState& grid_state);
    E_EtherCANErrCode wrap_resetStepCounters(long alpha_steps, long beta_steps,
					                         WrapGridState& grid_state,
                                             list& fpu_list);
    E_EtherCANErrCode wrap_checkIntegrity(WrapGridState& grid_state,
                                          list& fpu_list);
    boost::python::tuple wrap_getMinFirmwareVersion(WrapGridState& grid_state,
                                                    list& fpu_list);
    E_EtherCANErrCode wrap_setStepsPerSegment(int min_steps, int max_steps,
                                              WrapGridState& grid_state,
                                              list& fpu_list);
    E_EtherCANErrCode wrap_setTicksPerSegment(unsigned long ticks,
                                              WrapGridState& grid_state,
                                              list& fpu_list);
    E_EtherCANErrCode wrap_freeAlphaLimitBreach(int fpu_id,
                                        E_REQUEST_DIRECTION request_direction,
                                        WrapGridState& grid_state);
    E_EtherCANErrCode wrap_enableAlphaLimitProtection(WrapGridState& grid_state);
};

//==============================================================================

//******************************************************************************
#endif // NOT FLEXIBLE_CAN_MAPPING
//******************************************************************************
