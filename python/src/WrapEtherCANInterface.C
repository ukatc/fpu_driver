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
// NAME WrapEtherCANInterface.C
//
// Boost.Python wrappers for public EtherCAN functions.
//
////////////////////////////////////////////////////////////////////////////////

#include "WrapEtherCANInterface.h"


//******************************************************************************
// TODO: Note: This Boost.Python wrapper for the EtherCAN interface (which
// provides the wrapper for the original Python grid driver in FpuGridDriver.py)
// is not supported when the flexible CAN mapping is enabled, because the Python
// grid driver won't be modified to support this feature
#ifndef FLEXIBLE_CAN_MAPPING // NOT FLEXIBLE_CAN_MAPPING
//******************************************************************************

//------------------------------------------------------------------------------
const EtherCANInterfaceConfig &WrapEtherCANInterface::getConfig() const
{
    return config;
}

//------------------------------------------------------------------------------
E_EtherCANErrCode WrapEtherCANInterface::connectGateways(list& list_gateway_addresses)
{
    t_gateway_address address_array[MAX_NUM_GATEWAYS];
    const int actual_num_gw = convertGatewayAddresses(list_gateway_addresses,
                                                      address_array);
    E_EtherCANErrCode ecode = connect(actual_num_gw, address_array);
    checkInterfaceError(ecode);
    return ecode;
}

//------------------------------------------------------------------------------
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wstrict-overflow"
#pragma GCC diagnostic error "-Wstrict-overflow=2"
E_EtherCANErrCode WrapEtherCANInterface::configMotionWithDict(dict& dict_waveforms,
                                                    WrapGridState& grid_state,
                                                    list &fpu_list,
                                                    bool allow_uninitialized,
                                                    int ruleset_version)
{
    t_fpuset fpuset;
    getFPUSet(fpu_list, fpuset);

    t_wtable wtable;
    convertWavetable(dict_waveforms, wtable);

    E_EtherCANErrCode ecode = configMotion(wtable, grid_state, fpuset,
                                           allow_uninitialized,
                                           ruleset_version);
    checkInterfaceError(ecode);
    return ecode;
}
#pragma GCC diagnostic pop

//------------------------------------------------------------------------------
WrapGridState WrapEtherCANInterface::wrap_getGridState()
{
    WrapGridState grid_state;
    getGridState(grid_state);
    return grid_state;
}

//------------------------------------------------------------------------------
E_EtherCANErrCode WrapEtherCANInterface::wrap_initializeGrid(
                                    WrapGridState& grid_state, list& fpu_list)
{
    t_fpuset fpuset;
    getFPUSet(fpu_list, fpuset);

    E_EtherCANErrCode ecode = initializeGrid(grid_state, fpuset);
    checkInterfaceError(ecode);
    return ecode;
}

//------------------------------------------------------------------------------
E_EtherCANErrCode WrapEtherCANInterface::wrap_resetFPUs(WrapGridState& grid_state,
                                                        list& fpu_list)
{
    t_fpuset fpuset;
    getFPUSet(fpu_list, fpuset);

    E_EtherCANErrCode ecode = resetFPUs(grid_state, fpuset);
    checkInterfaceError(ecode);
    return ecode;
}

//------------------------------------------------------------------------------
E_EtherCANErrCode WrapEtherCANInterface::wrap_pingFPUs(WrapGridState& grid_state, 
                                                       list& fpu_list)
{
    t_fpuset fpuset;
    getFPUSet(fpu_list, fpuset);
    E_EtherCANErrCode ecode = pingFPUs(grid_state, fpuset);
    checkInterfaceError(ecode);
    return ecode;
}

//------------------------------------------------------------------------------
E_EtherCANErrCode WrapEtherCANInterface::wrap_readRegister(int read_address,
                                    WrapGridState& grid_state, list& fpu_list)
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

//------------------------------------------------------------------------------
E_EtherCANErrCode WrapEtherCANInterface::wrap_getFirmwareVersion(
                                    WrapGridState& grid_state, list& fpu_list)
{
    t_fpuset fpuset;
    getFPUSet(fpu_list, fpuset);

    E_EtherCANErrCode ecode = getFirmwareVersion(grid_state, fpuset);
    checkInterfaceError(ecode);
    return ecode;
}

//------------------------------------------------------------------------------
E_EtherCANErrCode WrapEtherCANInterface::wrap_findDatum(WrapGridState& grid_state,
                                            dict &dict_modes,
                                            list& fpu_list,
                                            E_DATUM_SELECTION arm_selection,
                                            E_DATUM_TIMEOUT_FLAG timeout_flag,
                                            bool count_protection)
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

//------------------------------------------------------------------------------
E_EtherCANErrCode WrapEtherCANInterface::wrap_startFindDatum(
                                            WrapGridState& grid_state,
                                            dict& dict_modes,
                                            list& fpu_list,
                                            E_DATUM_SELECTION arm_selection,
                                            E_DATUM_TIMEOUT_FLAG timeout_flag,
                                            bool count_protection)
{
    t_fpuset fpuset;
    getFPUSet(fpu_list, fpuset);

    t_datum_search_flags direction_flags;
    getDatumFlags(dict_modes, direction_flags, fpuset);

    E_EtherCANErrCode ecode = startFindDatum(grid_state, direction_flags,
                                             arm_selection, timeout_flag,
                                             count_protection, &fpuset);
    checkInterfaceError(ecode);
    return ecode;
}

//------------------------------------------------------------------------------
E_EtherCANErrCode WrapEtherCANInterface::wrap_waitFindDatum(
            WrapGridState& grid_state, double max_wait_time, list& fpu_list)
{
    E_EtherCANErrCode estatus;
    bool finished = false;

    t_fpuset fpuset;
    getFPUSet(fpu_list, fpuset);

    // FIXME: should return remaining wait time in tuple
    estatus = waitFindDatum(grid_state, max_wait_time, finished, &fpuset);

    if ( ((!finished) && (estatus == DE_OK))
         || (estatus == DE_WAIT_TIMEOUT) )
    {
        estatus = DE_WAIT_TIMEOUT;
        // we return because this is not exceptional or an error
        return estatus;
    }

    checkInterfaceError(estatus);
    return estatus;
}

//------------------------------------------------------------------------------
E_EtherCANErrCode WrapEtherCANInterface::wrap_executeMotion(
                WrapGridState& grid_state, list& fpu_list, bool sync_command)
{
    t_fpuset fpuset;
    getFPUSet(fpu_list, fpuset);

    E_EtherCANErrCode ecode =executeMotion(grid_state, fpuset, sync_command);
    checkInterfaceError(ecode);
    return ecode;
}

//------------------------------------------------------------------------------
E_EtherCANErrCode WrapEtherCANInterface::wrap_startExecuteMotion(
                WrapGridState& grid_state, list& fpu_list, bool sync_command)
{
    t_fpuset fpuset;
    getFPUSet(fpu_list, fpuset);

    E_EtherCANErrCode ecode =startExecuteMotion(grid_state, fpuset, sync_command);
    checkInterfaceError(ecode);
    return ecode;
}

//------------------------------------------------------------------------------
E_EtherCANErrCode WrapEtherCANInterface::wrap_waitExecuteMotion(
                WrapGridState& grid_state, double max_wait_time, list& fpu_list)
{
    E_EtherCANErrCode estatus;
    bool finished = false;

    t_fpuset fpuset;
    getFPUSet(fpu_list, fpuset);

    // FIXME: should return remaining wait time in tuple
    estatus =  waitExecuteMotion(grid_state, max_wait_time, finished, fpuset);
    if ( ((!finished) && (estatus == DE_OK))
         || (estatus == DE_WAIT_TIMEOUT) )
    {
        estatus = DE_WAIT_TIMEOUT;
        // we return because this is not exceptional oo an error
        return estatus;
    }

    checkInterfaceError(estatus);
    return estatus;
}

//------------------------------------------------------------------------------
E_EtherCANErrCode WrapEtherCANInterface::wrap_repeatMotion(
                                    WrapGridState& grid_state, list& fpu_list)
{
    t_fpuset fpuset;
    getFPUSet(fpu_list, fpuset);

    E_EtherCANErrCode ecode =repeatMotion(grid_state, fpuset);
    checkInterfaceError(ecode);
    return ecode;
}

//------------------------------------------------------------------------------
E_EtherCANErrCode WrapEtherCANInterface::wrap_reverseMotion(
                                    WrapGridState& grid_state, list& fpu_list)
{
    t_fpuset fpuset;
    getFPUSet(fpu_list, fpuset);

    E_EtherCANErrCode ecode = reverseMotion(grid_state, fpuset);
    checkInterfaceError(ecode);
    return ecode;
}

//------------------------------------------------------------------------------
E_EtherCANErrCode WrapEtherCANInterface::wrap_abortMotion(
                    WrapGridState& grid_state, list& fpu_list, bool sync_command)
{
    t_fpuset fpuset;
    getFPUSet(fpu_list, fpuset);

    E_EtherCANErrCode ecode = abortMotion(grid_state, fpuset, sync_command);
    checkInterfaceError(ecode);
    return ecode;
}

//------------------------------------------------------------------------------
E_EtherCANErrCode WrapEtherCANInterface::wrap_enableMove(int fpu_id,
                                                     WrapGridState& grid_state)
{
    E_EtherCANErrCode ecode = enableMove(fpu_id, grid_state);
    checkInterfaceError(ecode);
    return ecode;
}

//------------------------------------------------------------------------------
E_EtherCANErrCode WrapEtherCANInterface::wrap_setUStepLevel(int ustep_level,
                                    WrapGridState& grid_state, list& fpu_list)
{
    t_fpuset fpuset;
    getFPUSet(fpu_list, fpuset);

    E_EtherCANErrCode ecode = setUStepLevel(ustep_level, grid_state, fpuset);
    checkInterfaceError(ecode);
    return ecode;
}


//------------------------------------------------------------------------------
E_EtherCANErrCode WrapEtherCANInterface::wrap_freeBetaCollision(int fpu_id,
                                         E_REQUEST_DIRECTION request_direction,
                                         WrapGridState& grid_state)
{
    E_EtherCANErrCode ecode = freeBetaCollision(fpu_id, request_direction,
                                                grid_state);
    checkInterfaceError(ecode);
    return ecode;
}

//------------------------------------------------------------------------------
E_EtherCANErrCode WrapEtherCANInterface::wrap_enableBetaCollisionProtection(
                                                    WrapGridState& grid_state)
{
    E_EtherCANErrCode ecode = enableBetaCollisionProtection(grid_state);
    checkInterfaceError(ecode);
    return ecode;
}

//------------------------------------------------------------------------------
E_EtherCANErrCode WrapEtherCANInterface::wrap_lockFPU(int fpu_id,
                                                      WrapGridState& grid_state)
{
    E_EtherCANErrCode ecode =lockFPU(fpu_id, grid_state);
    checkInterfaceError(ecode);
    return ecode;
}

//------------------------------------------------------------------------------
E_EtherCANErrCode WrapEtherCANInterface::wrap_unlockFPU(int fpu_id,
                                                        WrapGridState& grid_state)
{
    E_EtherCANErrCode ecode =unlockFPU(fpu_id, grid_state);
    checkInterfaceError(ecode);
    return ecode;
}

//------------------------------------------------------------------------------
E_EtherCANErrCode WrapEtherCANInterface::wrap_readSerialNumbers(
                                    WrapGridState& grid_state, list& fpu_list)
{
    t_fpuset fpuset;
    getFPUSet(fpu_list, fpuset);

    E_EtherCANErrCode ecode = readSerialNumbers(grid_state, fpuset);
    checkInterfaceError(ecode);
    return ecode;
}

//------------------------------------------------------------------------------
E_EtherCANErrCode WrapEtherCANInterface::wrap_writeSerialNumber(int fpu_id,
                                str serial_number, WrapGridState& grid_state)
{
    std::string cpp_serial_number =  extract<std::string>(serial_number);

    E_EtherCANErrCode ecode = writeSerialNumber(fpu_id,
                                                cpp_serial_number.c_str(),
                                                grid_state);
    checkInterfaceError(ecode);
    return ecode;
}

//------------------------------------------------------------------------------
E_EtherCANErrCode WrapEtherCANInterface::wrap_resetStepCounters(long alpha_steps,
                                                                long beta_steps,
                                    WrapGridState& grid_state, list& fpu_list)
{
    t_fpuset fpuset;
    getFPUSet(fpu_list, fpuset);

    E_EtherCANErrCode ecode = resetStepCounters(alpha_steps, beta_steps,
                                                grid_state, fpuset);
    checkInterfaceError(ecode);
    return ecode;
}

//------------------------------------------------------------------------------
E_EtherCANErrCode WrapEtherCANInterface::wrap_checkIntegrity(
                                    WrapGridState& grid_state, list& fpu_list)
{
    t_fpuset fpuset;
    getFPUSet(fpu_list, fpuset);

    E_EtherCANErrCode ecode = checkIntegrity(grid_state, fpuset);
    checkInterfaceError(ecode);
    return ecode;
}

//------------------------------------------------------------------------------
boost::python::tuple WrapEtherCANInterface::wrap_getMinFirmwareVersion(
                                    WrapGridState& grid_state, list& fpu_list)
{
    t_fpuset fpuset;
    getFPUSet(fpu_list, fpuset);
    uint8_t min_firmware_version[3];

    E_EtherCANErrCode ecode = getMinFirmwareVersion(fpuset,
                                                    min_firmware_version,
                                                    grid_state);
    checkInterfaceError(ecode);
    return boost::python::make_tuple(min_firmware_version[0],
                                     min_firmware_version[1],
                                     min_firmware_version[2]);
}

//------------------------------------------------------------------------------
E_EtherCANErrCode WrapEtherCANInterface::wrap_setStepsPerSegment(int min_steps,
                     int max_steps, WrapGridState& grid_state, list& fpu_list)
{
    t_fpuset fpuset;
    getFPUSet(fpu_list, fpuset);

    E_EtherCANErrCode ecode = setStepsPerSegment(min_steps, max_steps,
                                                 grid_state, fpuset);
    checkInterfaceError(ecode);
    return ecode;
}

//------------------------------------------------------------------------------
E_EtherCANErrCode WrapEtherCANInterface::wrap_setTicksPerSegment(
                unsigned long ticks, WrapGridState& grid_state, list& fpu_list)
{
    t_fpuset fpuset;
    getFPUSet(fpu_list, fpuset);

    E_EtherCANErrCode ecode = setTicksPerSegment(ticks, grid_state, fpuset);
    checkInterfaceError(ecode);
    return ecode;
}

//------------------------------------------------------------------------------
E_EtherCANErrCode WrapEtherCANInterface::wrap_freeAlphaLimitBreach(int fpu_id,
                                         E_REQUEST_DIRECTION request_direction,
                                         WrapGridState& grid_state)
{
    E_EtherCANErrCode ecode = freeAlphaLimitBreach(fpu_id, request_direction,
                                                   grid_state);
    checkInterfaceError(ecode);
    return ecode;
}

//------------------------------------------------------------------------------
E_EtherCANErrCode WrapEtherCANInterface::wrap_enableAlphaLimitProtection(
                                                    WrapGridState& grid_state)
{
    E_EtherCANErrCode ecode = enableAlphaLimitProtection(grid_state);
    checkInterfaceError(ecode);
    return ecode;
}

//------------------------------------------------------------------------------

//******************************************************************************
#endif // NOT FLEXIBLE_CAN_MAPPING
//******************************************************************************

