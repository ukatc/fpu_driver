// -*- mode: c++ -*-

////////////////////////////////////////////////////////////////////////////////
//
// Copyright 2020 UKRI. See file "LICENSE" for license information.
//
//
// Who       When        What
// --------  ----------  -------------------------------------------------------
// bwillemse 2020-06-15  TODO: Put comment here
//------------------------------------------------------------------------------

////////////////////////////////////////////////////////////////////////////////
// NAME GridDriver.C
//
// This GridDriver class adds a software protection layer on top of the basic
// UnprotectedGridDriver class.
//
////////////////////////////////////////////////////////////////////////////////


// ********** NOTE: This file (along with UnprotectedGridDriver.C) is Bart's
// work in progress for converting the classes and functions in FPUGridDriver.py
// from Python to C++.
#include <set>
#include <cstring>
#include "GridDriver.h"

#ifdef DEBUG
#include <stdio.h>
#endif

namespace mpifps
{

#ifdef ENABLE_PROTECTION_CODE
//==============================================================================
// Define FPU constants
// TODO: The following constants are converted from fpu_constants.py - do the
// following?:
//  - Standardise their naming styles to lowercase_with_underscores
//  - Move them into GridDriver.h -> GridDriver class's private scope?

static const double AlphaGearRatio = 2050.175633;   // Actual gear ratio
static const double BetaGearRatio = 1517.662482;    // actual gear ratio

// There are 20 steps per revolution on the non-geared side, so:
static const double StepsPerRevolution = 20.0;
static const double DegreePerRevolution = 360.0;

// Note that these numbers must not be confounded with actual calibrated values!
// TODO: The Python versions in fpu_constants.py have "float()" functions in
// them - what is the purpose of this, and is the following C++ version OK?
static const double StepsPerDegreeAlpha =
                (StepsPerRevolution * AlphaGearRatio) / DegreePerRevolution;
static const double StepsPerDegreeBeta =
                (StepsPerRevolution * BetaGearRatio) / DegreePerRevolution;

// Overflow / underflow representations in binary FPU step counters - these are
// intentionally asymmetric for the alpha arm counter
static const int ALPHA_UNDERFLOW_COUNT = -10000;
static const int ALPHA_OVERFLOW_COUNT = ALPHA_UNDERFLOW_COUNT + (1 << 16) - 1;

static const int BETA_UNDERFLOW_COUNT = -0x8000;
static const int BETA_OVERFLOW_COUNT = BETA_UNDERFLOW_COUNT + (1 << 16) - 1;
#endif // ENABLE_PROTECTION_CODE


//==============================================================================
E_EtherCANErrCode GridDriver::initProtection(bool mockup)
{
    if (initprotection_was_called_ok)
    {
        return DE_INTERFACE_ALREADY_INITIALIZED;
    }

    if (!initialize_was_called_ok)
    {
        return DE_INTERFACE_NOT_INITIALIZED;
    }

#ifdef ENABLE_PROTECTION_CODE

    // Initialise the private data which doesn't initialise itself
    for (int fpu_id = 0; fpu_id < MAX_NUM_POSITIONERS; fpu_id++)
    {
        aretries_cw[fpu_id] = 0;
        aretries_acw[fpu_id] = 0;
        bretries_cw[fpu_id] = 0;
        bretries_acw[fpu_id] = 0;
    }

    // TODO: Finish this function

    // Initialise LMDB protection database
    std::string dir_str = protectionDB_GetDirFromLinuxEnv(mockup);
    if (!dir_str.empty())
    {
        if (protection_db.open(dir_str))
        {
            initprotection_was_called_ok = true;
            return DE_OK;
        }
    }

    // TODO: Return more detailed error code eventually?
    return DE_RESOURCE_ERROR;

#else // NOT ENABLE_PROTECTION_CODE
    UNUSED_ARG(mockup);

    initprotection_was_called_ok = true;
    return DE_OK;
#endif // NOT ENABLE_PROTECTION_CODE
}

//------------------------------------------------------------------------------
bool GridDriver::initializedOk()
{
    if ((initialize_was_called_ok) && (initprotection_was_called_ok))
    {
        return true;
    }
    return false;
}

//------------------------------------------------------------------------------
void GridDriver::_post_connect_hook()
{
    // TODO
}

//------------------------------------------------------------------------------
void GridDriver::getDuplicateSerialNumbers(t_grid_state &grid_state,
                            std::vector<std::string> &duplicate_snumbers_ret)
{
    // Gets any duplicate serial numbers by attempting to insert each into a
    // std::set (which doesn't allow duplicates) and building a list of those
    // which fail. N.B. The FPU serial numbers are in:
    //      grid_state.FPU_state[0...config.num_fpus].serial_number 

    std::set<std::string> snumbers_set;

    duplicate_snumbers_ret.clear();
    for (int fpu_id = 0;
         ((fpu_id < config.num_fpus) && (fpu_id < MAX_NUM_POSITIONERS));
         fpu_id++)
    {
        // Get FPU serial number C string, add explicit null-terminator for
        // safety, and convert to std::string
        char snumber_terminated[LEN_SERIAL_NUMBER];
        memcpy(snumber_terminated, grid_state.FPU_state[fpu_id].serial_number,
               LEN_SERIAL_NUMBER - 1);
        snumber_terminated[LEN_SERIAL_NUMBER - 1] = '\0';
        std::string snumber = snumber_terminated;

        // If serial number is a duplicate then record it
        if (!snumbers_set.insert(snumber).second)
        {
            duplicate_snumbers_ret.push_back(snumber);
        }
    }
}

#ifdef ENABLE_PROTECTION_CODE
//------------------------------------------------------------------------------
double GridDriver::_alpha_angle(const t_fpu_state &fpu_state,
                                bool &alpha_underflow_ret,
                                bool &alpha_overflow_ret)
{
    alpha_underflow_ret = (fpu_state.alpha_steps == ALPHA_UNDERFLOW_COUNT);
    alpha_overflow_ret = (fpu_state.alpha_steps == ALPHA_OVERFLOW_COUNT);
    return ( ((double)fpu_state.alpha_steps) / StepsPerDegreeAlpha) +
           config.alpha_datum_offset;
}

//------------------------------------------------------------------------------
double GridDriver::_beta_angle(const t_fpu_state &fpu_state,
                               bool &beta_underflow_ret,
                               bool &beta_overflow_ret)
{
    beta_underflow_ret = (fpu_state.beta_steps == BETA_UNDERFLOW_COUNT);
    beta_overflow_ret = (fpu_state.beta_steps == BETA_OVERFLOW_COUNT);
    return ((double)fpu_state.beta_steps) / StepsPerDegreeBeta;
}
#endif // ENABLE_PROTECTION_CODE

//------------------------------------------------------------------------------
void GridDriver::_allow_find_datum_hook(t_grid_state &gs,
                                        t_datum_search_flags &search_modes,
                                        enum E_DATUM_SELECTION selected_arm,
                                        const t_fpuset &fpuset,
                                        bool support_uninitialized_auto)
{
    // TODO

    // Temporary for now
    UNUSED_ARG(gs);
    UNUSED_ARG(search_modes);
    UNUSED_ARG(selected_arm);
    UNUSED_ARG(fpuset);
    UNUSED_ARG(support_uninitialized_auto);
}

//------------------------------------------------------------------------------
void GridDriver::_start_find_datum_hook(t_grid_state &gs,
                                        const t_datum_search_flags &search_modes,
                                        enum E_DATUM_SELECTION selected_arm,
                                        const t_fpuset &fpuset,
                                        t_fpu_positions &initial_positions_ret,
                                        bool soft_protection)
{
    // TODO

    // Temporary for now
    UNUSED_ARG(gs);
    UNUSED_ARG(search_modes);
    UNUSED_ARG(selected_arm);
    UNUSED_ARG(fpuset);
    UNUSED_ARG(initial_positions_ret);
    UNUSED_ARG(soft_protection);
}

//------------------------------------------------------------------------------
void GridDriver::_cancel_find_datum_hook(t_grid_state &gs, 
                                         const t_fpuset &fpuset,
                                         const t_fpu_positions &initial_positions)
{
    // TODO

    // Temporary for now
    UNUSED_ARG(gs);
    UNUSED_ARG(fpuset);
    UNUSED_ARG(initial_positions);
}

//------------------------------------------------------------------------------
void GridDriver::_finished_find_datum_hook(t_grid_state &prev_gs,
                                           t_grid_state &datum_gs,
                                           const t_datum_search_flags &search_modes,
                                           const t_fpuset &fpuset,
                                           bool was_cancelled,
                                           const t_fpu_positions &initial_positions, 
                                           enum E_DATUM_SELECTION selected_arm)
{
    // TODO

    // Temporary for now
    UNUSED_ARG(prev_gs);
    UNUSED_ARG(datum_gs);
    UNUSED_ARG(search_modes);
    UNUSED_ARG(fpuset);
    UNUSED_ARG(was_cancelled);
    UNUSED_ARG(initial_positions);
    UNUSED_ARG(selected_arm);
}

//------------------------------------------------------------------------------
void GridDriver::_reset_hook(t_grid_state &old_state, t_grid_state &gs,
                             const t_fpuset &fpuset)
{
    // TODO

    // Temporary for now
    UNUSED_ARG(old_state);
    UNUSED_ARG(gs);
    UNUSED_ARG(fpuset);
}

//------------------------------------------------------------------------------
void GridDriver::_refresh_positions(t_grid_state &grid_state, bool store,
                                    const t_fpuset &fpuset)
{
    // TODO

    // Temporary for now
    UNUSED_ARG(grid_state);
    UNUSED_ARG(store);
    UNUSED_ARG(fpuset);
}

//------------------------------------------------------------------------------
void GridDriver::_update_error_counters(const t_fpu_state &prev_fpu,
                                        const t_fpu_state &moved_fpu,
                                        bool datum_cmd)
{

    // TODO
    // N.B. Also see the Python GridDriver::_update_error_counters() function,
    // and my new FpuErrorCounterType enum class in GridDriver.h for this

    // Temporary for now
    UNUSED_ARG(prev_fpu);
    UNUSED_ARG(moved_fpu);
    UNUSED_ARG(datum_cmd);
}

//------------------------------------------------------------------------------
void GridDriver::_pre_config_motion_hook(const t_wtable &wtable,
                                         t_grid_state &gs,
                                         const t_fpuset &fpuset, Range wmode)
{
    // TODO

    // Temporary for now
    UNUSED_ARG(wtable);
    UNUSED_ARG(gs);
    UNUSED_ARG(fpuset);
    UNUSED_ARG(wmode);
}

//------------------------------------------------------------------------------
void GridDriver::_post_config_motion_hook(const t_wtable &wtable, 
                                          t_grid_state &gs,
                                          const t_fpuset &fpuset)
{
    // TODO

    // Temporary for now
    UNUSED_ARG(wtable);
    UNUSED_ARG(gs);
    UNUSED_ARG(fpuset);
}

//------------------------------------------------------------------------------
void GridDriver::_start_execute_motion_hook(t_grid_state &gs,
                                            const t_fpuset &fpuset,
                                            const t_fpu_positions &initial_positions)
{
    // TODO

    // Temporary for now
    UNUSED_ARG(gs);
    UNUSED_ARG(fpuset);
    UNUSED_ARG(initial_positions);
}

//------------------------------------------------------------------------------
void GridDriver::_cancel_execute_motion_hook(t_grid_state &gs,
                                             const t_fpuset &fpuset,
                                             const t_fpu_positions &initial_positions)
{
    // TODO

    // Temporary for now
    UNUSED_ARG(gs);
    UNUSED_ARG(fpuset);
    UNUSED_ARG(initial_positions);
}

//------------------------------------------------------------------------------
void GridDriver::_post_execute_motion_hook(t_grid_state &gs,
                                           const t_grid_state &old_gs,
                                           const t_grid_state &move_gs,
                                           const t_fpuset &fpuset)
{
    // TODO

    // Temporary for now
    UNUSED_ARG(gs);
    UNUSED_ARG(old_gs);
    UNUSED_ARG(move_gs);
    UNUSED_ARG(fpuset);
}

//------------------------------------------------------------------------------
void GridDriver::getFpuSetForConfigNumFpus(t_fpuset &fpuset_ret)
{
    for (int i = 0; i < MAX_NUM_POSITIONERS; i++)
    {
        fpuset_ret[i] = false;
    }

    for (int i = 0; ((i < config.num_fpus) && (i < MAX_NUM_POSITIONERS));
         i++)
    {
        fpuset_ret[i] = true;
    }
}

//------------------------------------------------------------------------------
// TODO: Boost.Python wrapper test member function - remove eventually
double GridDriver::boostPythonDivide(double dividend, double divisor)
{
    return dividend / divisor;
}


//==============================================================================

} // namespace mpifps

