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
#include <cstdlib>
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

// Note that these numbers must not be confused with actual calibrated values!
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
ProtectionDB &GridDriver::getProtectionDB()
{
    return protection_db;
}

//------------------------------------------------------------------------------
void GridDriver::_post_connect_hook()
{

    //***************************
    // TODO: NOTE: This function uses the new std::vector<FpuData> data
    // structuring - an earlier version which used the old parallel std::vector
    // approach (i.e. the Python FpuGridDriver.py structure of separate 
    // apositions, bpositions,... etc each being in their own array of size
    // config.num_fpus) is #ifdef'd out at the bottom of this file
    //***************************

    //***************************
    // TODO: This function conversion from Python is WIP - finish it
    //***************************

#ifdef ENABLE_PROTECTION_CODE

    bool result_ok = false;

    // TODO: Check that the FPU database has been properly opened at this point
    // self.fpudb = self.env.open_db(ProtectionDB.dbname)

    // TODO: Implement the following? (from Python version)
    // self.healthlog = self.env.open_db(HealthLogDB.dbname)

    E_EtherCANErrCode result;

    t_grid_state grid_state;
    getGridState(grid_state);

    t_fpuset fpuset;
    createFpuSetFlags(config.num_fpus, fpuset);

    //*************** TODO: Do something with result value below
    result = readSerialNumbers(grid_state, fpuset);
    // Check for serial number uniqueness
    std::vector<std::string> duplicate_snumbers;
    getDuplicateSerialNumbers(grid_state, duplicate_snumbers);
    if (duplicate_snumbers.size() != 0)
    {
        // TODO: Return a suitable error code
    }

    //..........................................................................
    // Create temporary FPUs data vector

    std::vector<FpuData> fpus_data_temp(config.num_fpus);

    //..........................................................................

    // ********* TODO: What to do with the interval offsets in the FpuDbData
    // Interval items??

    for (int fpu_id = 0; fpu_id < config.num_fpus; fpu_id++)
    {
        // TODO: If amy of the following operations fail then break out of this
        // "for" loop

        FpuData single_fpu_data;

        // Read FPU data item from the protection database into the temporary
        // FPUs data vector
        result_ok = false;
        const char *serial_number = grid_state.FPU_state[fpu_id].serial_number;
        {
            // N.B. This transaction is in its own nested scope so that it
            // automatically closes as soon as the scope ends 
            auto txn = protection_db.createTransaction();
            if (txn)
            {
                result_ok = txn->fpuDbTransferFpu(DbTransferType::Read,
                                                  serial_number,
                                                  single_fpu_data.db);
            }
        }
        
        if (result_ok)
        {
            single_fpu_data.a_caloffset = Interval(0.0);
            single_fpu_data.b_caloffset = Interval(0.0);
            single_fpu_data._last_counters = single_fpu_data.db.counters;
            single_fpu_data.target_position = { single_fpu_data.db.apos,
                                                single_fpu_data.db.bpos };

            // TODO: Check that this operation does a DEEP copy of ALL
            // underlying nested data items
            fpus_data_temp[fpu_id] = single_fpu_data;
        }
        else
        {
            // TODO: Error
        }

    }

    //..........................................................................

    if (result_ok)
    {
        // Copy all temporary FPUs data into main FPU data store
        // TODO: Ensure that does a full deep copy
        fpus_data = fpus_data_temp;

        configuring_targets.clear();
        configured_targets.clear();

        // Query positions and compute offsets, if FPUs have been reset.
        // This assumes that the stored positions are correct.
    
        // TODO: Check result code of _pingFPUs()
        result = _pingFPUs(grid_state, fpuset);

        _reset_hook(grid_state, grid_state, fpuset);
        _refresh_positions(grid_state, false, fpuset);
    }
    
    //..........................................................................

#endif // ENABLE_PROTECTION_CODE

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
    for (int fpu_id = 0; fpu_id < config.num_fpus; fpu_id++)
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
E_EtherCANErrCode GridDriver::_refresh_positions(t_grid_state &grid_state,
                                                 bool store,
                                                 const t_fpuset &fpuset)
{
    // TODO

    // Temporary for now
    UNUSED_ARG(grid_state);
    UNUSED_ARG(store);
    UNUSED_ARG(fpuset);

    return DE_OK;
}

//------------------------------------------------------------------------------
E_EtherCANErrCode GridDriver::pingFPUs(t_grid_state &gs, const t_fpuset &fpuset)
{
    if (!initializedOk())
    {
        return DE_INTERFACE_NOT_INITIALIZED;
    }

    E_EtherCANErrCode result = check_fpuset(fpuset);
    if (result != DE_OK)
    {
        return result;
    }

    // TODO: Add C++/Linux equivalent of Python version's "with self.lock"
    // here 

    t_grid_state prev_gs;
    _gd->getGridState(prev_gs);

    result = _pingFPUs(gs, fpuset);
    if (result == DE_OK)
    {
        result = _refresh_positions(gs, true, fpuset);
    }

    for (int fpu_id = 0; fpu_id < config.num_fpus; fpu_id++)
    {
        _update_error_counters(gs.FPU_state[fpu_id],
                               prev_gs.FPU_state[fpu_id]);
    }

    return result;
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

#ifdef ENABLE_PROTECTION_CODE
//------------------------------------------------------------------------------
void GridDriver::_update_counters_execute_motion(int fpu_id,
                                                 FpuCounters &fpu_counters,
                                                 const t_waveform_steps &waveform,
                                                 bool is_reversed,
                                                 bool cancel)
{
    
    // ********* TODO: Test this function - only visually converted from its
    // Python equivalent so far, not yet tested at all
    
    
    if ((fpu_id < 0) || (fpu_id >= config.num_fpus))
    {
        return;
    }

    FpuCounterInt sum_alpha_steps = 0;
    FpuCounterInt sum_beta_steps = 0;
    FpuCounterInt alpha_reversals = 0;
    FpuCounterInt beta_reversals = 0;
    FpuCounterInt alpha_starts = 0;
    FpuCounterInt beta_starts = 0;
    FpuCounterInt alpha_sign = 0;
    FpuCounterInt beta_sign = 0;
    FpuCounterInt last_asteps = 0;
    FpuCounterInt last_bsteps = 0;

    int rsign;
    if (is_reversed)
    {
         rsign = -1;
    }
    else
    {
        rsign = 1;
    }

    if (cancel)
    {
        // TODO: Check that this "=" works as expected
        fpu_counters = fpus_data[fpu_id]._last_counters;
    }

    FpuCounterInt alpha_lsign =
        fpu_counters.getCount(FpuCounterId::sign_alpha_last_direction);
    FpuCounterInt beta_lsign =
        fpu_counters.getCount(FpuCounterId::sign_beta_last_direction);

    for (const auto &step : waveform)
    {
        int asteps = step.alpha_steps * rsign;
        int bsteps = step.beta_steps * rsign;

        sum_alpha_steps += abs(asteps);
        sum_beta_steps += abs(bsteps);

        alpha_sign = sign(asteps);
        if (alpha_sign != 0)
        {
            int alpha_nzsign = alpha_sign;

            if (alpha_lsign != alpha_nzsign)
            {
                if (alpha_lsign != 0)
                {
                    alpha_reversals += 1;
                }
                alpha_lsign = alpha_nzsign;
            }
        }

        beta_sign = sign(bsteps);
        if (beta_sign != 0)
        {
            int beta_nzsign = beta_sign;

            if (beta_lsign != beta_nzsign)
            {
                if (beta_lsign != 0)
                {
                    beta_reversals += 1;
                }
                beta_lsign = beta_nzsign;
            }
        }

        if ((last_asteps == 0) && (asteps != 0))
        {
            alpha_starts += 1;
        }

        if ((last_bsteps == 0) && (bsteps != 0))
        {
            beta_starts += 1;
        }

        last_asteps = asteps;
        last_bsteps = bsteps;
    }

    // Store values for case of subsequent cancellation
    // TODO: Check that this "=" works as expected
    fpus_data[fpu_id]._last_counters = fpu_counters;

    // Update sums for FPU
    fpu_counters.addToCount(FpuCounterId::executed_waveforms, 1);
    fpu_counters.addToCount(FpuCounterId::total_alpha_steps, sum_alpha_steps);
    fpu_counters.addToCount(FpuCounterId::total_beta_steps, sum_beta_steps);
    fpu_counters.addToCount(FpuCounterId::alpha_direction_reversals, alpha_reversals);
    fpu_counters.addToCount(FpuCounterId::beta_direction_reversals, beta_reversals);
    fpu_counters.setCount(FpuCounterId::sign_alpha_last_direction, alpha_lsign);
    fpu_counters.setCount(FpuCounterId::sign_beta_last_direction, beta_lsign);
    fpu_counters.addToCount(FpuCounterId::alpha_starts, alpha_starts);
    fpu_counters.addToCount(FpuCounterId::beta_starts, beta_starts);
}
#endif // ENABLE_PROTECTION_CODE

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

#if 0
//------------------------------------------------------------------------------
// TODO: Note: set_wtable_reversed() didn't effectively do anything in the
// original Python version in FpuGridDriver.py, and so is removed here. This is
// because:
//   - wf_reversed is not used in UnprotectedGridDriver
//   - Therefore, the original Python UnprotectedGridDriver virtual hook
//     function implementations of _post_config_motion_hook(),
//     _post_repeat_motion_hook() and _post_reverse_motion_hook() which call
//     set_wtable_reversed() are superfluous (N.B. But their overrides in
//     GridDriver are important)
void UnprotectedGridDriver::set_wtable_reversed(const t_fpuset &fpuset,
                                                bool is_reversed)
{
    // TODO: Add C++/Linux equivalent of Python version's "with self.lock"
    // here 

    for (int fpu_id = 0; fpu_id < config.num_fpus; fpu_id++)
    {
        if (fpuset[fpu_id])
        {
            wf_reversed[fpu_id] = is_reversed;
        }
    }
}
#endif // 0

//------------------------------------------------------------------------------
int GridDriver::sign(int64_t val)
{
    if (val > 0)
    {
        return 1;
    }
    if (val < 0)
    {
        return -1;
    }
    return 0;
}

//------------------------------------------------------------------------------
// TODO: Boost.Python wrapper test member function - remove eventually
double GridDriver::boostPythonDivide(double dividend, double divisor)
{
    return dividend / divisor;
}


//==============================================================================
// TODO: Old _post_connect_hook() with the old parallel std::vector data sets
// structure - keeping for now just in case need to revert to this approach,
// but hopefully not
#if 0
//==============================================================================
void GridDriver::_post_connect_hook()
{
    //***************************
    // TODO: This function conversion from Python is WIP - finish it
    //***************************

#ifdef ENABLE_PROTECTION_CODE

    bool result_ok = false;

    // TODO: Check that the FPU database has been properly opened at this point
    // self.fpudb = self.env.open_db(ProtectionDB.dbname)

    // TODO: Implement the following? (from Python version)
    // self.healthlog = self.env.open_db(HealthLogDB.dbname)

    E_EtherCANErrCode result;

    t_grid_state grid_state;
    getGridState(grid_state);

    t_fpuset fpuset;
    createFpuSetFlags(config.num_fpus, fpuset);

    //*************** TODO: Do something with result value below
    result = readSerialNumbers(grid_state, fpuset);
    // Check for serial number uniqueness
    std::vector<std::string> duplicate_snumbers;
    getDuplicateSerialNumbers(grid_state, duplicate_snumbers);
    if (duplicate_snumbers.size() != 0)
    {
        // TODO: Return a suitable error code
    }

    //..........................................................................
    // Create temporary FPU value vectors, with any required initialisation
    // N.B. std::vector storage is created on the heap, so no problem with
    // local stack space here (unlike std::array)
    std::vector<Interval> apositions_temp(config.num_fpus);
    std::vector<Interval> bpositions_temp(config.num_fpus);
    t_wtable wavetable_temp;
    std::vector<bool> wf_reversed_temp(config.num_fpus, false);
    std::vector<Interval> alimits_temp(config.num_fpus);
    std::vector<Interval> blimits_temp(config.num_fpus);
    std::vector<int64_t> maxaretries_temp(config.num_fpus, 0);
    std::vector<int64_t> aretries_cw_temp(config.num_fpus, 0);
    std::vector<int64_t> aretries_acw_temp(config.num_fpus, 0);
    std::vector<int64_t> maxbretries_temp(config.num_fpus, 0);
    std::vector<int64_t> bretries_cw_temp(config.num_fpus, 0);
    std::vector<int64_t> bretries_acw_temp(config.num_fpus, 0);
    std::vector<FpuCounters> counters_temp(config.num_fpus);

    std::vector<Interval> a_caloffsets_temp(config.num_fpus, Interval(0.0));
    std::vector<Interval> b_caloffsets_temp(config.num_fpus, Interval(0.0));

    //..........................................................................
    // Read all FPUs' data items from the protection database into the
    // temporary value vectors
    struct
    {
        FpuDbPositionType type;
        std::vector<Interval> &vector_ref;
    } position_items[(int)FpuDbPositionType::NumTypes] = 
    {
        { FpuDbPositionType::AlphaPos,   apositions_temp },
        { FpuDbPositionType::BetaPos,    bpositions_temp },
        { FpuDbPositionType::AlphaLimit, alimits_temp    },
        { FpuDbPositionType::BetaLimit,  blimits_temp    }
    };
    
    struct
    {
        FpuDbIntValType type;
        std::vector<int64_t> &vector_ref;
    } int64_items[(int)FpuDbIntValType::NumTypes] =
    {
        // TODO: Check that FreeAlphaRetries and FreeBetaRetries correspond to
        // maxaretries_temp and maxbretries_temp
        { FpuDbIntValType::FreeAlphaRetries, maxaretries_temp  },
        { FpuDbIntValType::AlphaRetries_CW,  aretries_cw_temp  },
        { FpuDbIntValType::AlphaRetries_ACW, aretries_acw_temp },
        { FpuDbIntValType::FreeBetaRetries,  maxbretries_temp  },
        { FpuDbIntValType::BetaRetries_CW,   bretries_cw_temp  },
        { FpuDbIntValType::BetaRetries_ACW,  bretries_acw_temp }
    };

    for (int fpu_id = 0; fpu_id < config.num_fpus; fpu_id++)
    {
        // TODO: If amy of the following operations fail then break out of this
        // "for" loop


        auto txn = protection_db.createTransaction();
        if (txn)
        {
            const char *serial_number = grid_state.FPU_state[fpu_id].serial_number;

            // Read the FPU's position values
      //********* TODO: What to do with the offsets??
            double datum_offset;
            result_ok = true;
            for (int i = 0; i < (int)FpuDbPositionType::NumTypes; i++)
            {
                if (!txn->fpuDbTransferPosition(DbTransferType::Read,
                                                position_items[i].type,
                                                serial_number,
                                                position_items[i].vector_ref[fpu_id],
                                                datum_offset))
                {
                    result_ok = false;
                    break;
                }
            }

            // Read the FPU's waveform
            if (result_ok)
            {
                t_waveform_steps waveform;
                result_ok = txn->fpuDbTransferWaveform(DbTransferType::Read,
                                                       serial_number,
                                                       waveform);
                if (result_ok)
                {
                    wavetable_temp.push_back({(int16_t)fpu_id, waveform});
                }
            }

            // Read the FPU's wf_reversed flag
            if (result_ok)
            {
                bool wf_reversed_flag = false;
                result_ok = txn->fpuDbTransferWfReversedFlag(DbTransferType::Read,
                                                             serial_number,
                                                             wf_reversed_flag);
                wf_reversed_temp[fpu_id] = wf_reversed_flag;
            }

            // Read the FPU's integer values
            if (result_ok)
            {
                for (int i = 0; i < (int)FpuDbIntValType::NumTypes; i++)
                {
                    int64_t int64_val;
                    if (!txn->fpuDbTransferInt64Val(DbTransferType::Read,
                                                    int64_items[i].type,
                                                    serial_number,
                                            int64_items[i].vector_ref[fpu_id]))
                    {
                        result_ok = false;
                        break;
                    }
                }
            }
        }
        else
        {
            // TODO: Error
        }

    }

    //..........................................................................

    apositions = apositions_temp;
    bpositions = bpositions_temp;
    wf_reversed = wf_reversed_temp;
    alimits = alimits_temp;
    blimits = blimits_temp;
    maxaretries = maxaretries_temp;
    aretries_cw = aretries_cw_temp;
    aretries_acw = aretries_acw_temp;
    maxbretries = maxbretries_temp;
    bretries_cw = bretries_cw_temp;
    bretries_acw = bretries_acw_temp;
    counters = counters_temp;

    //..........................................................................

    last_wavetable = wavetable_temp;
    a_caloffsets = a_caloffsets_temp;
    b_caloffsets = b_caloffsets_temp;
    _last_counters = counters_temp;

    //..........................................................................

    std::vector<t_fpu_position> target_positions_temp(config.num_fpus);
    for (int fpu_id = 0; fpu_id < config.num_fpus; fpu_id++)
    {
        target_positions_temp[fpu_id] = {apositions_temp[fpu_id], 
                                         bpositions_temp[fpu_id]};
    }
    target_positions = target_positions_temp;

    configuring_targets.clear();
    configured_targets.clear();

    //..........................................................................
    // Query positions and compute offsets, if FPUs have been reset.
    // This assumes that the stored positions are correct.
    
    // TODO: Check result code of _pingFPUs()
    result = _pingFPUs(grid_state, fpuset);

    _reset_hook(grid_state, grid_state, fpuset);
    _refresh_positions(grid_state, false, fpuset);
    
    //..........................................................................

#endif // ENABLE_PROTECTION_CODE

}
//------------------------------------------------------------------------------
#endif // 0 
//------------------------------------------------------------------------------

} // namespace mpifps

