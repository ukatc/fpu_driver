// -*- mode: c++ -*-

////////////////////////////////////////////////////////////////////////////////
//
// Copyright 2021 UKRI. See file "LICENSE" for license information.
//
//
// Who       When        What
// --------  ----------  -------------------------------------------------------
// bwillemse 2021-03-23  Created.
//------------------------------------------------------------------------------

////////////////////////////////////////////////////////////////////////////////
// NAME EtherCANInterfaceConfig.C
//
// TODO: Put description here
//
////////////////////////////////////////////////////////////////////////////////

#include <EtherCANInterfaceConfig.h>

namespace mpifps
{

#ifdef FLEXIBLE_CAN_MAPPING
//------------------------------------------------------------------------------
E_EtherCANErrCode EtherCANInterfaceConfig::initFpuIdList(
                                    const std::vector<int> &fpu_id_list_init)
{
    E_EtherCANErrCode ecan_result = DE_OK;

    // Clear fpuset
    for (size_t fpu_id = 0; fpu_id < MAX_NUM_POSITIONERS; fpu_id++)
    {
        fpuset[fpu_id] = false;
    }

    // Populate fpuset from fpu_id_list_init
    for (int fpu_id : fpu_id_list_init)
    {
        if ((fpu_id >= 0) && (fpu_id < MAX_NUM_POSITIONERS))
        {
            fpuset[fpu_id] = true;
        }
        else
        {
            ecan_result = DE_INVALID_FPU_ID;
            break;
        }
    }

    if (ecan_result == DE_OK)
    {
        // Store fpu_id_list_init
        fpu_id_list = fpu_id_list_init;
    }
    else
    {
        // Clear fpuset
        for (size_t fpu_id = 0; fpu_id < MAX_NUM_POSITIONERS; fpu_id++)
        {
            fpuset[fpu_id] = false;
        }
    }

    return ecan_result;
}

//------------------------------------------------------------------------------
bool EtherCANInterfaceConfig::isValidFpuId(int fpu_id) const
{
    // Checks if fpu_id is one of the valid FPU IDs which was specified in
    // the initFpuIdList() call.

    // First check for valid range - also, needs to be done before indexing
    // into config_fpuset below
    if ((fpu_id < 0) || (fpu_id >= MAX_NUM_POSITIONERS))
    {
        return false;
    }

    // Check if fpu_id is a valid configured ID
    if (fpuset[fpu_id])
    {
        return true;
    }
    else
    {
        return false;
    }
}

//------------------------------------------------------------------------------
const std::vector<int> &EtherCANInterfaceConfig::getFpuIdList() const
{
    return fpu_id_list;
}

//------------------------------------------------------------------------------
const t_fpuset &EtherCANInterfaceConfig::getFpuSet() const
{
    return fpuset;
}

//------------------------------------------------------------------------------
#endif // FLEXIBLE_CAN_MAPPING

} // namespace mpifps

