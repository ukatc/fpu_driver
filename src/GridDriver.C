// -*- mode: c++ -*-

////////////////////////////////////////////////////////////////////////////////
//
// Copyright 2020 UKRI. See file "LICENSE" for license information.
//
//
// Who       When        What
// --------  ----------  -------------------------------------------------------
// bwillemse 2020-06-15  Created (translated from Python FpuGridDriver.py).
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
#include "FPUConstants.h"
#include "FPUCommands.h"

#ifdef DEBUG
#include <stdio.h>
#endif

namespace mpifps
{

//==============================================================================
GridDriver::~GridDriver()
{
    // Destructor
    // If connection is live, gets and stores the positions before exiting

    // TODO: Put C++ equivalent of the Python "with self.lock:" here

    if (_gd != nullptr)
    {
        t_grid_state grid_state;
        getGridState(grid_state);

#ifdef FLEXIBLE_CAN_MAPPING
        const t_fpuset &fpuset = config.getFpuSet();
#else // NOT FLEXIBLE_CAN_MAPPING
        t_fpuset fpuset;
        createFpuSetForNumFpus(config.num_fpus, fpuset);
#endif // NOT FLEXIBLE_CAN_MAPPING

        if (grid_state.interface_state == DS_CONNECTED)
        {
            // Fetch current positions
            _pingFPUs(grid_state, fpuset);
        }

#ifdef ENABLE_PROTECTION_CODE
        _refresh_positions(grid_state, true, fpuset);
#endif
    }
}

//------------------------------------------------------------------------------
E_EtherCANErrCode GridDriver::initProtection(bool use_mockup_db)
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

    // Open LMDB protection database
    std::string dir_str = ProtectionDB::getDirFromLinuxEnv(use_mockup_db);
    if (!dir_str.empty())
    {
        MdbResult mdb_result = protection_db.open(dir_str);
        switch (mdb_result)
        {
        case MDB_SUCCESS:
            // TODO: Implement the following? (from Python version ->
            // _post_connect_hook())
            // self.healthlog = self.env.open_db(HealthLogDB.dbname)
            initprotection_was_called_ok = true;
            return DE_OK;
            break;

        case ENOENT:
            return DE_DB_DIR_OR_FILE_NOT_FOUND;
            break;

        case EACCES:
            return DE_DB_ACCESS_DENIED;
            break;

        case MDB_OLD_INCOMPATIBLE_DB_FORMAT:
            return DE_DB_OLD_FORMAT;
            break;

        default:
            return DE_DB_OTHER_OPENING_FAILURE;
            break;
        }
    }
    else
    {
        return DE_DB_ENV_VARIABLE_NOT_FOUND;
    }

#else // NOT ENABLE_PROTECTION_CODE
    UNUSED_ARG(use_mockup_db);

    initprotection_was_called_ok = true;
    return DE_OK;
#endif // NOT ENABLE_PROTECTION_CODE
}

//------------------------------------------------------------------------------
bool GridDriver::initializedOk()
{
    if (initialize_was_called_ok && initprotection_was_called_ok)
    {
        return true;
    }
    return false;
}


//------------------------------------------------------------------------------
#ifdef ENABLE_PROTECTION_CODE
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
E_EtherCANErrCode GridDriver::_post_connect_hook()
{
    // Called from the end of the UnprotectedGridDriver::connect() function.
    // Populates FPU data structures in RAM from the FPU database contents, for
    // the grid FPUs,
    // N.B. To support testing of this function, can use
    // GridDriverTester::writeGridFpusToFpuDb() to create FPU test data in the
    // FPU database.

    //***************************
    // TODO: NOTE: This latest version of this function uses the new
    // std::vector<FpuData> data structuring. An earlier version (#ifdef'd out
    // at the bottom of this file) used a parallel std::vector approach
    // equivalent to the Python version's separate dictionaries for apositions,
    // bpositions,... etc, but that approach proved to be too clunky in C++. 
    //***************************

    //***************************
    // TODO: This function is mostly complete and working (apart from the TODOs
    // in it), but will still need final checking once the rest of the
    // GridDriver C++ functions have been implemented, because some FPU data
    // structuring might still change.
    //***************************

    E_EtherCANErrCode ecan_result;

    t_grid_state grid_state;
    getGridState(grid_state);

#ifdef FLEXIBLE_CAN_MAPPING
    const t_fpuset &fpuset = config.getFpuSet();
#else // NOT FLEXIBLE_CAN_MAPPING
    t_fpuset fpuset;
    createFpuSetForNumFpus(config.num_fpus, fpuset);
#endif // NOT FLEXIBLE_CAN_MAPPING

    // Read serial numbers from grid FPUs, and check for duplicates
    ecan_result = readSerialNumbers(grid_state, fpuset);
    if (ecan_result == DE_OK)
    {
        std::vector<std::string> duplicate_snumbers;
        getDuplicateSerialNumbers(grid_state, duplicate_snumbers);
        if (duplicate_snumbers.size() != 0)
        {
            return DE_DUPLICATE_SERIAL_NUMBER;
        }
    }

    // Read data from FPU database for all grid FPUs
    std::vector<FpuData> fpus_data_temp(config.num_fpus);
    if (ecan_result == DE_OK)
    {
#ifdef FLEXIBLE_CAN_MAPPING
        for (int fpu_id : config.getFpuIdList())
#else
        for (int fpu_id = 0; fpu_id < config.num_fpus; fpu_id++)
#endif
        {
            MdbResult mdb_result = MDB_PANIC;
            auto txn = protection_db.createTransaction(mdb_result);
            if (txn)
            {
                const char *serial_number = grid_state.FPU_state[fpu_id].serial_number;
                FpuData fpu_data;
                
                if (txn->fpuDbTransferFpu(DbTransferType::Read, serial_number,
                                          fpu_data.db) == MDB_SUCCESS)
                {
                    // Adjust alpha position and limits with datum offset
                    // TODO: Is this addition by Interval::operator+=()
                    // equivalent to the Python _post_connect_hook() code?
                    fpu_data.db.apos += config.alpha_datum_offset;
                    fpu_data.db.alimits += config.alpha_datum_offset;

                    // TODO: Will BETA datum offset adjustment also be required
                    // here eventually?

                    // Populate the remaining FPU data items
                    fpu_data.a_caloffset = Interval(0.0);
                    fpu_data.b_caloffset = Interval(0.0);
                    fpu_data._last_counters = fpu_data.db.counters;
                    fpu_data.target_position = { fpu_data.db.apos,
                                                 fpu_data.db.bpos };

                    fpus_data_temp[fpu_id] = fpu_data;
                }
                else
                {
                    ecan_result = DE_DB_MISSING_FPU_ENTRY_OR_READ_FAILED;
                    break;
                }
            }
            else
            {
                ecan_result = DE_DB_TRANSACTION_CREATION_FAILED;
                break;
            }
        }
    }

    //..........................................................................

    if (ecan_result == DE_OK)
    {
        // Copy all temporary FPUs data into main FPU data store
        fpus_data = fpus_data_temp;

        configuring_targets.clear();
        configured_targets.clear();

        ecan_result = _pingFPUs(grid_state, fpuset);
    }

    // Query positions and compute offsets, if FPUs have been reset.
    // This assumes that the stored positions are correct.

    // TODO: Are the following calls equivalent to the Python version, in terms
    // of how error conditions are handled? e.g. Should they be called
    // unconditionally, even if one of the earlier ones fails?

    if (ecan_result == DE_OK)
    {
        ecan_result = _reset_hook(grid_state, grid_state, fpuset);
    }

    if (ecan_result == DE_OK)
    {
        ecan_result = _refresh_positions(grid_state, false, fpuset);
    }
    
    //..........................................................................

    return ecan_result;
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
#ifdef FLEXIBLE_CAN_MAPPING
    for (int fpu_id : config.getFpuIdList())
#else
    for (int fpu_id = 0; fpu_id < config.num_fpus; fpu_id++)
#endif
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

//------------------------------------------------------------------------------
double GridDriver::_alpha_angle(const t_fpu_state &fpu_state,
                                bool &alpha_underflow_ret,
                                bool &alpha_overflow_ret)
{
    // TODO: The first argument of this function can just be alpha_steps

    alpha_underflow_ret = (fpu_state.alpha_steps == ALPHA_UNDERFLOW_COUNT);
    alpha_overflow_ret = (fpu_state.alpha_steps == ALPHA_OVERFLOW_COUNT);
    return ( ((double)fpu_state.alpha_steps) / STEPS_PER_DEGREE_ALPHA) +
           config.alpha_datum_offset;
}

//------------------------------------------------------------------------------
double GridDriver::_beta_angle(const t_fpu_state &fpu_state,
                               bool &beta_underflow_ret,
                               bool &beta_overflow_ret)
{
    // TODO: See comment in _alpha_angle() above

    beta_underflow_ret = (fpu_state.beta_steps == BETA_UNDERFLOW_COUNT);
    beta_overflow_ret = (fpu_state.beta_steps == BETA_OVERFLOW_COUNT);
    return ((double)fpu_state.beta_steps) / STEPS_PER_DEGREE_BETA;
}

//------------------------------------------------------------------------------
E_EtherCANErrCode GridDriver::_reset_hook(t_grid_state &old_state,
                                          t_grid_state &gs,
                                          const t_fpuset &fpuset)
{
    // This function needs to be called after a reset or hardware power-on
    // reset. It updates the offset between the stored FPU positions and
    // positions reported by ping.
    //
    // If the FPUs have been switched off and powered on again, or if they have
    // been reset for another reason, then the ping values will usually differ
    // from the recorded valid values, until a successful datum command has
    // been run. This difference defines an offset which must be added every
    // time in order to yield the correct position.
    //
    // This method needs to be run:
    //  - Every time the driver is initialising, after the initial ping
    //  - After every resetFPU command

    E_EtherCANErrCode ecan_result;

    t_grid_state grid_state;
    getGridState(grid_state);

    ecan_result = readSerialNumbers(gs, fpuset);
    if (ecan_result == DE_OK)
    {
#ifdef FLEXIBLE_CAN_MAPPING
        for (int fpu_id : config.getFpuIdList())
#else
        for (int fpu_id = 0; fpu_id < config.num_fpus; fpu_id++)
#endif
        {
            if (!fpuset[fpu_id])
            {
                continue;
            }

            // These offsets are the difference between the calibrated angle
            // and the uncalibrated angle - after a findDatum, they are set to
            // zero
            bool a_underflow, a_overflow;
            double alpha_angle = _alpha_angle(grid_state.FPU_state[fpu_id],
                                              a_underflow, a_overflow);
            if (a_underflow || a_overflow)  
            {
                // TODO: Nothing to do here? The Python code just prints a
                // warning
            }
            fpus_data[fpu_id].a_caloffset = fpus_data[fpu_id].db.apos - alpha_angle;

            bool b_underflow, b_overflow;
            double beta_angle = _beta_angle(grid_state.FPU_state[fpu_id],
                                            b_underflow, b_overflow);
            if (b_underflow || b_overflow)  
            {
                // TODO: Nothing to do here? The Python code just prints a
                // warning
            }
            fpus_data[fpu_id].b_caloffset = fpus_data[fpu_id].db.bpos - beta_angle;

            if (configured_ranges.find(fpu_id) != configured_ranges.end())
            {
                configured_ranges.erase(fpu_id);
            }

            fpus_data[fpu_id].db.last_waveform.clear();
        }
    }

    return ecan_result;
}

//------------------------------------------------------------------------------
void GridDriver::_reset_counter_hook(double alpha_target, double beta_target,
                                     t_grid_state &old_state, t_grid_state &gs,
                                     const t_fpuset &fpuset)
{
    // Similar to reset_hook, but run after resetStepCounter() and only
    // updating the caloffsets

#ifdef FLEXIBLE_CAN_MAPPING
    for (int fpu_id : config.getFpuIdList())
#else
    for (int fpu_id = 0; fpu_id < config.num_fpus; fpu_id++)
#endif
    {
        if (!fpuset[fpu_id])
        {
            continue;
        }

        FpuData &fpu_data = fpus_data[fpu_id];

        // These offsets are the difference between the calibrated angle and
        // the uncalibrated angle - after a findDatum, they are set to zero
        bool a_underflow, a_overflow;
        double alpha_angle = _alpha_angle(gs.FPU_state[fpu_id], a_underflow,
                                          a_overflow);
        if (a_underflow || a_overflow)
        {
            fpu_data.a_caloffset = fpu_data.db.apos - alpha_target;
        }
        else
        {
            fpu_data.a_caloffset = fpu_data.db.apos - alpha_angle;
        }

        bool b_underflow, b_overflow;
        double beta_angle = _beta_angle(gs.FPU_state[fpu_id], b_underflow,
                                        b_overflow);
        if (b_underflow || b_overflow)
        {
            fpu_data.b_caloffset = fpu_data.db.bpos - beta_target;
        }
        else
        {
            fpu_data.b_caloffset = fpu_data.db.bpos- beta_angle;
        }
    }
}

//------------------------------------------------------------------------------
E_EtherCANErrCode GridDriver::_allow_find_datum_hook(t_grid_state &gs,
                                        t_datum_search_flags &search_modes,
                                        enum E_DATUM_SELECTION selected_arm,
                                        const t_fpuset &fpuset,
                                        bool support_uninitialized_auto)
{
    // Checks whether a datum search is safe based upon the stored position,
    // and returns an error if not.

    E_EtherCANErrCode ecan_result = DE_ERROR_UNKNOWN;

    // Get fresh ping data
    ecan_result = _pingFPUs(gs, fpuset);
    if (ecan_result != DE_OK)
    {
        return ecan_result;
    }

    const Interval acw_range(0.0, std::numeric_limits<double>::max());
    const Interval cw_range(-std::numeric_limits<double>::max(), 0.0);
#ifdef FLEXIBLE_CAN_MAPPING
    for (int fpu_id : config.getFpuIdList())
#else
    for (int fpu_id = 0; fpu_id < config.num_fpus; fpu_id++)
#endif
    {
        if (!fpuset[fpu_id])
        {
            continue;
        }

        if ((selected_arm == DASEL_BETA) || (selected_arm == DASEL_BOTH))
        {
            const Interval &blim = fpus_data[fpu_id].db.blimits;
            const Interval &bpos = fpus_data[fpu_id].db.bpos;
            const Interval search_beta_clockwise_range = 
                                                blim.intersects(acw_range);
            const Interval search_beta_anti_clockwise_range = 
                                                blim.intersects(cw_range);

            if (search_modes[fpu_id] == SEARCH_AUTO)
            {
                if ((!gs.FPU_state[fpu_id].beta_was_referenced) ||
                    (gs.FPU_state[fpu_id].state == FPST_UNINITIALIZED))
                {
                    if (!blim.contains(bpos))
                    {
                        // Beta arm is not in safe range for datum search,
                        // probably needs manual move
                        return DE_PROTECTION_ERROR;
                    }

                    if (support_uninitialized_auto)
                    {
                        // Operator wants auto search but hardware is not
                        // initialised. If possible, we use the database value
                        // to set the correct direction
                        if (search_beta_clockwise_range.contains(bpos))
                        {
                            search_modes[fpu_id] = SEARCH_CLOCKWISE;
                        }
                        else if (search_beta_anti_clockwise_range.contains(bpos))
                        {
                            search_modes[fpu_id] = SEARCH_ANTI_CLOCKWISE;
                        }
                        else
                        {
                            // No directed datum search possible - position for
                            // FPU ambiguous (operator should consider moving
                            // the FPU into an unambigous range)
                            return DE_PROTECTION_ERROR;
                        }
                    }
                    else
                    {
                        // FPU is not initialized, support_uninitialized_auto
                        // is not set, cannot do protected automatic search
                        return DE_PROTECTION_ERROR;
                    }
                }
                else
                {
                    // Beta was zeroed, check automatic search is safe
                    if (!blim.contains(bpos))
                    {
                        // Beta arm is not in safe range for datum search,
                        // probably needs manual move
                        return DE_PROTECTION_ERROR;
                    }
                }
            }
            else if (search_modes[fpu_id] == SEARCH_CLOCKWISE)
            {
                if (!search_beta_clockwise_range.contains(bpos))
                {
                    // Beta arm of FPU is outside of safe clockwise search
                    // range
                    return DE_PROTECTION_ERROR;
                }
            }
            else if (search_modes[fpu_id] == SEARCH_ANTI_CLOCKWISE)
            {
                if (!search_beta_anti_clockwise_range.contains(bpos))
                {
                    // Beta arm of FPU is outside of safe anticlockwise search
                    // range
                    return DE_PROTECTION_ERROR;
                }
            }
        }

        // Check alpha arm
        if ((selected_arm == DASEL_ALPHA) || (selected_arm == DASEL_BOTH))
        {
            const Interval &alim = fpus_data[fpu_id].db.alimits;
            const Interval &apos = fpus_data[fpu_id].db.apos;
            if (!alim.contains(apos))
            {
                // Alpha arm of FPU is not in safe range for datum search
                return DE_PROTECTION_ERROR;
            }
        }
    }

    return DE_OK;
}

//------------------------------------------------------------------------------
E_EtherCANErrCode GridDriver::_start_find_datum_hook(t_grid_state &gs,
                                        const t_datum_search_flags &search_modes,
                                        enum E_DATUM_SELECTION selected_arm,
                                        const t_fpuset &fpuset,
                                        t_fpu_positions &initial_positions,
                                        bool soft_protection)
{
    // This function is run when a findDatum command is actually started.
    // It updates the new range of possible positions to include the zero
    // point of each arm.

    initial_positions.clear();

    // We allow 0.5 degree of imprecision. This is mainly for the FPU
    // simulator which steps at discrete intervals.

    MdbResult mdb_result = MDB_PANIC;
    auto txn = protection_db.createTransaction(mdb_result);
    if (txn)
    {
#ifdef FLEXIBLE_CAN_MAPPING
        for (int fpu_id : config.getFpuIdList())
#else
        for (int fpu_id = 0; fpu_id < config.num_fpus; fpu_id++)
#endif
        {
            if (!fpuset[fpu_id])
            {
                continue;
            }

            // Record initial position intervals so that the known range can
            // be restored if the datum search is rejected
            initial_positions[fpu_id] = { fpus_data[fpu_id].db.apos,
                                          fpus_data[fpu_id].db.bpos };

            Interval atarget = initial_positions[fpu_id].apos;
            Interval btarget = initial_positions[fpu_id].bpos;

            const char *serial_number = gs.FPU_state[fpu_id].serial_number;

            // Update stored intervals to include zero, and store in DB
            if ((selected_arm == DASEL_ALPHA) || (selected_arm == DASEL_BOTH))
            {
                if (soft_protection)
                {
                    Interval new_apos = fpus_data[fpu_id].db.apos.extend(0.0 +
                                                     config.alpha_datum_offset);
                    if (!_update_apos(txn, serial_number, fpu_id, new_apos))
                    {
                        return DE_DB_WRITE_FAILED;
                    }
                }
                else
                {
                    Interval protection_interval(ALPHA_MIN_HARDSTOP_DEGREE,
                                                 ALPHA_MAX_HARDSTOP_DEGREE);
                    Interval apos = fpus_data[fpu_id].db.apos;
                    Interval new_range = apos.combine(protection_interval);
                    if (!_update_apos(txn, serial_number, fpu_id, new_range))
                    {
                        return DE_DB_WRITE_FAILED;
                    }
                }
                atarget = 0.0;
            }

            if ((selected_arm == DASEL_BETA) || (selected_arm == DASEL_BOTH))
            {
                Interval bpos = fpus_data[fpu_id].db.bpos;
                if (soft_protection)
                {
                    Interval new_bpos = bpos.extend(0.0);
                    if (!_update_bpos(txn, serial_number, fpu_id, new_bpos))
                    {
                        return DE_DB_WRITE_FAILED;
                    }
                }
                else
                {
                    Interval new_range;
                    if (search_modes[fpu_id] == SEARCH_CLOCKWISE)
                    {
                        new_range = bpos.extend(BETA_MIN_HWPROT_DEGREE);
                    }
                    else if (search_modes[fpu_id] == SEARCH_ANTI_CLOCKWISE)
                    {
                        new_range = bpos.extend(BETA_MAX_HWPROT_DEGREE);
                    }
                    else
                    {
                        Interval protection_interval(BETA_MIN_HWPROT_DEGREE,
                                                     BETA_MAX_HWPROT_DEGREE);
                        new_range = bpos.combine(protection_interval);
                    }

                    if (!_update_bpos(txn, serial_number, fpu_id, new_range))
                    {
                        return DE_DB_WRITE_FAILED;
                    }
                }
                btarget = 0.0;
            }

            // Update target positions. The target positions are used as a
            // computed tracked value if the driver cannot retrieve the exact
            // counted position of the FPU because the firmware counter is in
            // underflow/overflow. (That should only happen if the datum
            // search does not move this arm).
            fpus_data[fpu_id].target_position = { atarget, btarget };
        }
    }
    else
    {
        return DE_DB_TRANSACTION_CREATION_FAILED;
    }

    if (protection_db.sync() != MDB_SUCCESS)
    {
        return DE_DB_SYNC_FAILED;
    }

    return DE_OK;
}

//------------------------------------------------------------------------------
E_EtherCANErrCode GridDriver::_cancel_find_datum_hook(t_grid_state &gs, 
                                         const t_fpuset &fpuset,
                                         const t_fpu_positions &initial_positions)
{
    MdbResult mdb_result = MDB_PANIC;
    auto txn = protection_db.createTransaction(mdb_result);
    if (txn)
    {
        for (const auto &it : initial_positions)
        {
            const int fpu_id = it.first;
            if (fpu_id >= config.num_fpus)
            {
                return DE_INVALID_FPU_ID;
            }

            const char *serial_number = gs.FPU_state[fpu_id].serial_number;

            // Get last stored positions
            const Interval &apos = it.second.apos;
            const Interval &bpos = it.second.bpos;

            // Revert stored intervals to old values
            bool success = false;
            if (_update_apos(txn, serial_number, fpu_id, apos))
            {
                if (_update_bpos(txn, serial_number, fpu_id, bpos))
                {
                    success = true;
                }
            }
            if (!success)
            {
                return DE_DB_WRITE_FAILED;
            }

            fpus_data[fpu_id].target_position = { apos, bpos };
        }
    }
    else
    {
        return DE_DB_TRANSACTION_CREATION_FAILED;
    }

    if (protection_db.sync() != MDB_SUCCESS)
    {
        return DE_DB_SYNC_FAILED;
    }

    return DE_OK;
}

//------------------------------------------------------------------------------
E_EtherCANErrCode GridDriver::_finished_find_datum_hook(
                                           const t_grid_state &prev_gs,
                                           t_grid_state &datum_gs,
                                           const t_datum_search_flags &search_modes,
                                           const t_fpuset &fpuset,
                                           bool was_cancelled,
                                           const t_fpu_positions &initial_positions, 
                                           enum E_DATUM_SELECTION selected_arm)
{
    //..........................................................................
    // TODO: Johannes' comment: "FIXME: check if the next block is still needed"
    t_fpuset fpuset_refresh;
    bool refresh_pending = false;
    for (int fpu_id = 0; fpu_id < MAX_NUM_POSITIONERS; fpu_id++)
    {
        if (fpu_id < config.num_fpus)
        {
            t_fpu_state &fpu_state = datum_gs.FPU_state[fpu_id];
            if (((fpu_state.state == FPST_MOVING) ||
                 (fpu_state.state == FPST_DATUM_SEARCH)) ||
                (!fpu_state.ping_ok))
            {
                fpuset_refresh[fpu_id] = true;
                refresh_pending = true;
            }
        }
        else
        {
            fpuset_refresh[fpu_id] = false;
        }
    }
    if (refresh_pending)
    {
        sleepSecs(0.1);
        E_EtherCANErrCode ping_result = _pingFPUs(datum_gs, fpuset_refresh);
        if (ping_result != DE_OK)
        {
            return ping_result;
        }
    }

    //..........................................................................

    MdbResult mdb_result = MDB_PANIC;
    auto txn = protection_db.createTransaction(mdb_result);
    if (txn)
    {
#ifdef FLEXIBLE_CAN_MAPPING
        for (int fpu_id : config.getFpuIdList())
#else
        for (int fpu_id = 0; fpu_id < config.num_fpus; fpu_id++)
#endif
        {
            if (!fpuset[fpu_id])
            {
                continue;
            }

            FpuData &fpu_data = fpus_data[fpu_id];
            t_fpu_state &datum_fpu_state = datum_gs.FPU_state[fpu_id];
            const char *serial_number = datum_fpu_state.serial_number;

            //..................................................................
            // Alpha arm: Set position intervals to zero, and store in DB
            if (datum_fpu_state.alpha_was_referenced &&
                (datum_fpu_state.alpha_steps == 0))
            {
                fpu_data.a_caloffset = Interval(0.0);
                if (!_update_apos(txn, serial_number, fpu_id,
                                  Interval(0.0) + config.alpha_datum_offset))
                {
                    return DE_DB_WRITE_FAILED;
                }

                // Reset retry count for freeAlphaLimitBreach, because we have
                // reached a safe position.

                if (fpu_data.db.aretries_acw > 0)
                {
                    fpu_data.db.aretries_acw = 0;
                    if (txn->fpuDbTransferInt64Val(DbTransferType::Write,
                                    FpuDbIntValType::AlphaRetries_ACW,
                                    serial_number,
                                    fpu_data.db.aretries_acw) != MDB_SUCCESS)
                    {
                        return DE_DB_WRITE_FAILED;
                    }
                }
                if (fpu_data.db.aretries_cw > 0)
                {
                    fpu_data.db.aretries_cw = 0;
                    if (txn->fpuDbTransferInt64Val(DbTransferType::Write,
                                    FpuDbIntValType::AlphaRetries_CW,
                                    serial_number,
                                    fpu_data.db.aretries_cw) != MDB_SUCCESS)
                    {
                        return DE_DB_WRITE_FAILED;
                    }
                }
            }
            else
            {
                // If ping_ok is set, we assume that even if the datum
                // operation itself did not succeed, the step counter was
                // successfully retrieved by a subsequent ping, and the offset
                // is unchanged.

                if (datum_fpu_state.ping_ok &&
                    ((selected_arm == DASEL_ALPHA) ||
                     (selected_arm == DASEL_BOTH)))
                {
                    bool a_underflow, a_overflow;
                    double alpha_angle = _alpha_angle(datum_fpu_state,
                                                      a_underflow, a_overflow);
                    Interval a_interval;
                    if (a_underflow || a_overflow)
                    {
                        // We know only we are in the interval between the
                        // overflow / underflow value and the datum position
                        a_interval = (Interval(alpha_angle) + 
                            fpu_data.a_caloffset).extend(0.0 + config.alpha_datum_offset);
                    }
                    else
                    {
                        a_interval = Interval(alpha_angle) + fpu_data.a_caloffset;
                    }

                    if (!_update_apos(txn, serial_number, fpu_id, a_interval))
                    {
                        return DE_DB_WRITE_FAILED;
                    }
                }
            }

            //..................................................................
            // Beta arm
            if (datum_fpu_state.beta_was_referenced &&
                (datum_fpu_state.beta_steps == 0))
            {
                fpu_data.b_caloffset = Interval(0.0);
                if (!_update_bpos(txn, serial_number, fpu_id, Interval(0.0)))
                {
                    return DE_DB_WRITE_FAILED;
                }

                if (fpu_data.db.bretries_acw > 0)
                {
                    fpu_data.db.bretries_acw = 0;
                    if (txn->fpuDbTransferInt64Val(DbTransferType::Write,
                                    FpuDbIntValType::BetaRetries_ACW,
                                    serial_number,
                                    fpu_data.db.bretries_acw) != MDB_SUCCESS)
                    {
                        return DE_DB_WRITE_FAILED;
                    }
                }

                if (fpu_data.db.bretries_cw > 0)
                {
                    fpu_data.db.bretries_cw = 0;
                    if (txn->fpuDbTransferInt64Val(DbTransferType::Write,
                                    FpuDbIntValType::BetaRetries_CW,
                                    serial_number,
                                    fpu_data.db.bretries_cw) != MDB_SUCCESS)
                    {
                        return DE_DB_WRITE_FAILED;
                    }
                }
            }
            else
            {
                if (datum_fpu_state.ping_ok &&
                    ((selected_arm == DASEL_BETA) ||
                     (selected_arm == DASEL_BOTH)))
                {
                    bool b_underflow, b_overflow;
                    double beta_angle = _beta_angle(datum_fpu_state,
                                                    b_underflow,
                                                    b_overflow);
                    Interval b_interval;
                    if (b_underflow || b_overflow)
                    {
                        // We know only we are in the interval between the
                        // overflow / underflow value and the datum position
                        b_interval = (Interval(beta_angle) + 
                                      fpu_data.b_caloffset).extend(0.0);
                    }
                    else
                    {
                        b_interval = Interval(beta_angle) + fpu_data.b_caloffset;
                    }

                    if (!_update_bpos(txn, serial_number, fpu_id, b_interval))
                    {
                        return DE_DB_WRITE_FAILED;
                    }
                }
            }

            //..................................................................

            const t_fpu_state &prev_fpu_state = prev_gs.FPU_state[fpu_id];

            // This passes prev_fpu_state and datum_fpu_state, to deduce the
            // timeout counts
            bool datum_cmd = true;
            _update_error_counters(fpu_data.db.counters, prev_fpu_state,
                                   datum_fpu_state, datum_cmd);

            // This passes prev_fpu_state and datum_fpu_state, to get the
            // counter deviations
            _update_counters_find_datum(fpu_data.db.counters, prev_fpu_state,
                                        datum_fpu_state);

            if (txn->fpuDbTransferCounters(DbTransferType::Write,
                                           serial_number,
                                           fpu_data.db.counters) != MDB_SUCCESS)
            {
                return DE_DB_WRITE_FAILED;
            }

            //..................................................................
        }
    }
    else
    {
        return DE_DB_TRANSACTION_CREATION_FAILED;
    }

    //..........................................................................

    // TODO here: Implement health log updating as per Python version of this
    // function

    //..........................................................................

    if (protection_db.sync() != MDB_SUCCESS)
    {
        return DE_DB_SYNC_FAILED;
    }

    //..........................................................................

    // TODO: Return proper error codes
    return DE_OK;
}

//------------------------------------------------------------------------------
void GridDriver::_update_counters_find_datum(FpuCounters &fpu_counters,
                                             const t_fpu_state &prev_fpu_state,
                                             const t_fpu_state &datum_fpu_state)
{
    fpu_counters.addToCount(FpuCounterId::datum_count, 1);

    // Discard error states, and states which were uninitialised before FPU
    // is none if datum command didn't succeed. Storing aberrations is skipped
    // in that case

    if (datum_fpu_state.timeout_count != prev_fpu_state.timeout_count)
    {
        fpu_counters.addToCount(FpuCounterId::datum_timeout, 1);
    }

    if (datum_fpu_state.last_status == MCE_FPU_OK)
    {
        if (prev_fpu_state.alpha_was_referenced &&
            datum_fpu_state.alpha_was_referenced)
        {
            fpu_counters.addToCount(FpuCounterId::alpha_aberration_count, 1);
            fpu_counters.addToCount(FpuCounterId::datum_sum_alpha_aberration,
                                    datum_fpu_state.alpha_deviation);
            fpu_counters.addToCount(FpuCounterId::datum_sqsum_alpha_aberration,
                                    ((FpuCounterInt)datum_fpu_state.alpha_deviation) *
                                     datum_fpu_state.alpha_deviation);
        }

        if (prev_fpu_state.beta_was_referenced &&
            datum_fpu_state.beta_was_referenced)
        {
            fpu_counters.addToCount(FpuCounterId::beta_aberration_count, 1);
            fpu_counters.addToCount(FpuCounterId::datum_sum_beta_aberration,
                                    datum_fpu_state.beta_deviation);
            fpu_counters.addToCount(FpuCounterId::datum_sqsum_beta_aberration,
                                    ((FpuCounterInt)datum_fpu_state.beta_deviation) *
                                     datum_fpu_state.beta_deviation);
        }
    }
}

//------------------------------------------------------------------------------
E_EtherCANErrCode GridDriver::trackedAnglesVals(const t_grid_state &gs,
                                                const t_fpuset &fpuset,
                                                t_fpu_positions &positions_ret)
{
    // Gets the tracked angle values for the FPUs in fpuset.
    // TODO: N.B. In terms of the original Python version, this function is
    // equivalent to calling trackedAngles() with retrieve=True.

    E_EtherCANErrCode ecan_result = checkInitializedAndFpuset(fpuset);
    if (ecan_result != DE_OK)
    {
        return ecan_result;
    }

    // TODO: Add C++/Linux equivalent of Python version's "with self.lock" here

    positions_ret.clear();
#ifdef FLEXIBLE_CAN_MAPPING
    for (int fpu_id : config.getFpuIdList())
#else
    for (int fpu_id = 0; fpu_id < config.num_fpus; fpu_id++)
#endif
    {
        if (fpuset[fpu_id])
        {
            positions_ret[fpu_id] = { fpus_data[fpu_id].db.apos,
                                      fpus_data[fpu_id].db.bpos };
        }
    }

    return DE_OK;
}

//------------------------------------------------------------------------------
E_EtherCANErrCode GridDriver::trackedAnglesString(const t_grid_state &gs,
                                                  const t_fpuset &fpuset,
                                                  std::string &return_string,
                                                  bool show_offsets,
                                                  bool active)
{
    // Creates a string containing tracked angles, offset, and waveform span
    // for configured waveforms, for each FPU in fpuset

    E_EtherCANErrCode ecan_result = checkInitializedAndFpuset(fpuset);
    if (ecan_result != DE_OK)
    {
        return ecan_result;
    }

    // TODO: Add C++/Linux equivalent of Python version's "with self.lock" here

    return_string.clear();

#ifdef FLEXIBLE_CAN_MAPPING
    for (int fpu_id : config.getFpuIdList())
#else
    for (int fpu_id = 0; fpu_id < config.num_fpus; fpu_id++)
#endif
    {
        if (fpuset[fpu_id])
        {
            bool a_underflow, a_overflow;
            double a_angle = _alpha_angle(gs.FPU_state[fpu_id], a_underflow,
                                          a_overflow);
            bool b_underflow, b_overflow;
            double b_angle = _beta_angle(gs.FPU_state[fpu_id], b_underflow,
                                         b_overflow);

            t_fpu_position fpu_position;
            std::string prefix;
            if (active)
            {
                auto it = configured_ranges.find(fpu_id);
                if (it != configured_ranges.end())
                {
                    fpu_position = it->second;
                }
                prefix = "active";
            }
            else
            {
                auto it = configuring_ranges.find(fpu_id);
                if (it != configuring_ranges.end())
                {
                    fpu_position = it->second;
                }
                prefix = "last";
            }

            FpuData &fpu_data = fpus_data[fpu_id];

            return_string += "FPU " + std::to_string(fpu_id) +
                             ": angles = (" +
                             fpu_data.db.apos.toString() + ", " +
                             fpu_data.db.bpos.toString() + "), ";

            if (show_offsets)
            {
                std::string aflag;
                if (a_underflow)
                {
                    aflag = "!u";
                }
                else if (a_overflow)
                {
                    aflag = "!o";
                }
                else
                {
                    aflag = "";
                }

                std::string bflag;
                if (b_underflow)
                {
                    bflag = "!u";
                }
                else if (b_overflow)
                {
                    bflag = "!o";
                }
                else
                {
                    bflag = "";
                }

                return_string += "offsets = (" + 
                                 fpu_data.a_caloffset.toString() + ", " +
                                 fpu_data.b_caloffset.toString() + "), ";
                return_string += "stepcount angle = (" +
                                 doubleToString(a_angle) + aflag + ", " +
                                 doubleToString(b_angle) + bflag + "), ";
            }

            return_string += prefix + "_wform_range = (" +
                             fpu_position.apos.toString() + "," +
                             fpu_position.bpos.toString() + ")\n";
        }
    }

    return DE_OK;
}

//------------------------------------------------------------------------------
bool GridDriver::_update_apos(const std::unique_ptr<ProtectionDbTxn> &txn,
                              const char *serial_number, int fpu_id,
                              const Interval &new_apos, bool store)
{
    fpus_data[fpu_id].db.apos = new_apos;
    if (store)
    {
        // TODO: Check that the const_cast<> below works OK
        if (txn->fpuDbTransferInterval(DbTransferType::Write,
                                       FpuDbIntervalType::AlphaPos,
                                       serial_number,
                                       const_cast<Interval &>(new_apos),
                                       config.alpha_datum_offset) == MDB_SUCCESS)
        {
            return true;
        }
        else
        {
            return false;
        }
    }
    else
    {
        return true;
    }
}

//------------------------------------------------------------------------------
bool GridDriver::_update_bpos(const std::unique_ptr<ProtectionDbTxn> &txn,
                              const char *serial_number, int fpu_id,
                              const Interval &new_bpos, bool store)
{
    fpus_data[fpu_id].db.bpos = new_bpos;
    if (store)
    {
        double beta_datum_offset = BETA_DATUM_OFFSET;
        // TODO: Check that the const_cast<> below works OK
        if (txn->fpuDbTransferInterval(DbTransferType::Write,
                                       FpuDbIntervalType::BetaPos,
                                       serial_number,
                                       const_cast<Interval &>(new_bpos),
                                       beta_datum_offset) == MDB_SUCCESS)
        {
            return true;
        }
        else
        {
            return false;
        }
    }
    else
    {
        return true;
    }
}

//------------------------------------------------------------------------------
E_EtherCANErrCode GridDriver::_refresh_positions(const t_grid_state &grid_state,
                                                 bool store,
                                                 const t_fpuset &fpuset)
{
    // This is to be run after any movement.

    // Computes new current positions from step count and offsets (if set), and
    // stores them to database.

    // Note: We do not try to recognize a reset behind the back of the driver
    // as there is no reliable indicator. Protocol 1 does not support to
    // recognise that. Protocol 2 allows to recognise it, so this might change.

    E_EtherCANErrCode ecan_result = DE_ERROR_UNKNOWN;
    bool inconsistency_abort = false;

    MdbResult mdb_result = MDB_PANIC;
    auto txn = protection_db.createTransaction(mdb_result);
    if (txn)
    {
        ecan_result = DE_OK;
#ifdef FLEXIBLE_CAN_MAPPING
        for (int fpu_id : config.getFpuIdList())
#else
        for (int fpu_id = 0; fpu_id < config.num_fpus; fpu_id++)
#endif
        {
            if (!fpuset[fpu_id])
            {
                continue;
            }

            if (!grid_state.FPU_state[fpu_id].ping_ok)
            {
                // Position is not known. This flag is set by a successful ping
                // or datum response, and cleared by every movement as well as
                // all movement time-outs
                continue;
            }

            bool a_underflow, a_overflow;
            double counted_alpha_angle = _alpha_angle(grid_state.FPU_state[fpu_id],
                                                      a_underflow, a_overflow);
            Interval new_alpha = fpus_data[fpu_id].a_caloffset + counted_alpha_angle;

            bool b_underflow, b_overflow;
            double counted_beta_angle = _beta_angle(grid_state.FPU_state[fpu_id],
                                                    b_underflow, b_overflow);
            Interval new_beta = fpus_data[fpu_id].b_caloffset + counted_beta_angle;

            // Compute alpha and beta position intervals and store both to DB
            if (a_underflow || a_overflow)
            {
                // TODO: The Python version has the following warning here -
                // is anything required here for this C++ version? 
                // print("Warning: _refresh_positions(): FPU id %i: using stored alpha target value"
                //       " to bypass counter underflow/overflow" % fpu_id)
                new_alpha = fpus_data[fpu_id].target_position.apos;
            }
            if (b_underflow || b_overflow)
            {
                // TODO: The Python version has the following warning here -
                // is anything required here for this C++ version? 
                // print("Warning: _refresh_positions(): FPU id %i: using stored alpha target value"
                //       " to bypass counter underflow/overflow" % fpu_id)
                new_beta = fpus_data[fpu_id].target_position.bpos;
            }
            fpus_data[fpu_id].target_position.apos = new_alpha;
            fpus_data[fpu_id].target_position.bpos = new_beta;

            if ( (!fpus_data[fpu_id].db.apos.contains(new_alpha, 0.25)) ||
                 (!fpus_data[fpu_id].db.bpos.contains(new_beta, 0.25)) )
            {
                inconsistency_abort = true;
            }

            const char *serial_number = grid_state.FPU_state[fpu_id].serial_number;
            bool success = _update_apos(txn, serial_number, fpu_id, new_alpha,
                                        store);
            if (success)
            {
                success = _update_bpos(txn, serial_number, fpu_id, new_beta,
                                       store);
            }
            if (!success)
            {
                ecan_result = DE_DB_WRITE_FAILED;
                break;
            }
        }
    }
    else
    {
        ecan_result = DE_DB_TRANSACTION_CREATION_FAILED;
    }

    if (protection_db.sync() != MDB_SUCCESS)
    {
        ecan_result = DE_DB_SYNC_FAILED;
    }

    if ((ecan_result == DE_OK) && inconsistency_abort)
    {
        ecan_result = DE_INCONSISTENT_STEP_COUNT;
    }

    return ecan_result;
}

//------------------------------------------------------------------------------
E_EtherCANErrCode GridDriver::pingFPUs(t_grid_state &gs, const t_fpuset &fpuset)
{
    E_EtherCANErrCode ecan_result = checkInitializedAndFpuset(fpuset);
    if (ecan_result != DE_OK)
    {
        return ecan_result;
    }

    // TODO: Add C++/Linux equivalent of Python version's "with self.lock" here 

    t_grid_state prev_gs;
    _gd->getGridState(prev_gs);

    ecan_result = _pingFPUs(gs, fpuset);
    if (ecan_result == DE_OK)
    {
        ecan_result = _refresh_positions(gs, true, fpuset);
    }

    updateErrorCountersForFpuSet(prev_gs, gs, fpuset);

    return ecan_result;
}

//------------------------------------------------------------------------------
E_EtherCANErrCode GridDriver::_check_allowed_range(int fpu_id, int stepnum,
                                                   const char *arm_name,
                                                   const Interval &xlimits,
                                                   const Interval &xpos,
                                                   Interval &new_range,
                                                   Range wmode)
{
    // Checks whether waveform step is in allowed interval for this FPU.
    // Parameters:
    //   xlimits:   Manually configured limits of the individual FPU, in degrees
    //   new_range: On entry: Current minimum / maximum interval of real
    //              position, in degrees
    //              On exit: New minimum / maximum range if movement is
    //              executed
    //   xpos:      New value of variable

    // TODO: The following arguments are in the original Python version of this
    // function (and are used for displaying diagnostic information), but aren't
    // needed in this C++ version? (or might they be useful for diagnostics
    // eventually?)
    UNUSED_ARG(fpu_id);
    UNUSED_ARG(stepnum);
    UNUSED_ARG(arm_name);

    if (wmode == Range::Ignore)
    {
        return DE_OK;
    }

    // If the movement extends the range of the FPU location, this is updated
    // in the movement range
    new_range.assignCombine(xpos);

    if (!xlimits.contains(xpos))
    {
        if (wmode == Range::Error)
        {
            // TODO: Is this the correct error code to use?
            return DE_PROTECTION_ERROR;
        }
        else if (wmode == Range::Warn)
        {
            // Nothing to do here (the original Python version just showed a
            // warning)
        }
    }
    
    return DE_OK;
}

//------------------------------------------------------------------------------


// The complexity of the wave table data flow which follows merits a bit of
// explanation.
//
// Generally, the protection wrapper tries to track the position of each FPU as
// it is moved. When a ping returns, the new position can generally be updated
// to that point.
//
// However, when a movement is started, there is no way to know the exact
// position before the movement finished regularly, or the next successful ping
// returns. When an abortMotion message is sent, or a collision occurs, the
// position is not known.
//
// The solution used here is to track not the position, but the worst-case
// minimum and maximum position. In other words, for each of the alpha and beta
// coordinates, an INTERVAL of positions is tracked. When a deterministic
// movement is added to the current interval, the minimum and maximum extent of
// that movement become added to the current minimum and maximum. In other
// words, movements expand the region of uncertainty, and regular terminations,
// pings and datum responses collapse it to an interval of length zero.
// 
// If a configMotion command is issued, this defines a tentative future
// interval, which becomes valid once the corresponding movement is started.
// If a reverseMotion is issued, this generates an interval with the opposite
// sign of movements.
//
// Tracking the region of uncertainty is not always required, as ping commands
// can often be used to narrow the possible range of positions down. But doing
// this has, first, the disadvantage that special-casing is needed for handling
// an abortMotion or collision situation. Yet exactly these are situations
// which need to be handled very robustly. And second, tracking the known state
// of positions during start-up also requires special-casing, which would
// require a lot of fiddly state handling. This could be error-prone, and the
// solution to track intervals is therefore much more general and clean.

//------------------------------------------------------------------------------
E_EtherCANErrCode GridDriver::_check_and_register_wtable(const t_wtable &wtable,
                                                         t_grid_state &gs,
                                                         const t_fpuset &fpuset,
                                                         Range wmode, int sign)
{
    // - Compute movement range for each FPU
    // - Add to current known min / max position
    // - Compare to allowed range
    // - If not in range then return error code, depending upon the protection
    //   setting

    E_EtherCANErrCode ecan_result = DE_ERROR_UNKNOWN;

    t_fpu_positions configuring_ranges_temp;
    t_fpu_positions configuring_targets_temp;
    for (size_t wtable_index = 0; wtable_index < wtable.size(); wtable_index++)
    {
        int fpu_id = wtable[wtable_index].fpu_id;
        if (!fpuset[fpu_id])
        {
            continue;
        }

        const t_waveform_steps &waveform_steps = wtable[wtable_index].steps;

        Interval alimits = fpus_data[fpu_id].db.alimits;
        Interval blimits = fpus_data[fpu_id].db.blimits;

        Interval alpha0 = fpus_data[fpu_id].db.apos;
        Interval beta0 = fpus_data[fpu_id].db.bpos;

        Interval wf_arange = alpha0;
        Interval wf_brange = beta0;

        ecan_result = _check_allowed_range(fpu_id, -1, "alpha", alimits,
                                           alpha0, wf_arange, wmode);
        if (ecan_result == DE_OK)
        {
            ecan_result = _check_allowed_range(fpu_id, -1, "beta", blimits,
                                               beta0, wf_brange, wmode);
        }
        if (ecan_result != DE_OK)
        {
            return ecan_result;
        }

        int asteps = 0;
        int bsteps = 0;
        for (size_t i = 0; i < waveform_steps.size(); i++)
        {
            // If the waveform is reversed, we need to go backwards with the
            // check!
            int step_index;
            if (sign == 1)
            {
                step_index = i;
            }
            else if (sign == -1)
            {
                step_index = (waveform_steps.size() - 1) - i;
            }
            else
            {
                return DE_INVALID_PAR_VALUE;
            }

            asteps += waveform_steps[step_index].alpha_steps * sign; 
            bsteps += waveform_steps[step_index].beta_steps * sign;
            Interval alpha_sect = alpha0 + 
                                  (((double)asteps) / STEPS_PER_DEGREE_ALPHA);
            Interval beta_sect = beta0 +
                                 (((double)bsteps) / STEPS_PER_DEGREE_BETA);

            ecan_result = _check_allowed_range(fpu_id, step_index, "alpha",
                                               alimits, alpha_sect,
                                               wf_arange, wmode);
            if (ecan_result == DE_OK)
            {
                ecan_result = _check_allowed_range(fpu_id, step_index, "beta",
                                                   blimits, beta_sect,
                                                   wf_brange, wmode);
            }
            if (ecan_result != DE_OK)
            {
                return ecan_result;
            }

            configuring_ranges_temp[fpu_id] = {wf_arange, wf_brange};
            configuring_targets_temp[fpu_id] = {alpha_sect, beta_sect};
        }
    }

    // This is the list of alpha/beta position intervals that will become
    // valid if and after an executeMotion is started, and before the new
    // positions can be retrieved via ping.
    // Merge the "temp" maps into the overall ones (overwriting any existing
    // FPU entries)
    for (const auto &it : configuring_ranges_temp)
    {
        configuring_ranges[it.first] = it.second;
    }
    for (const auto &it : configuring_targets_temp)
    {
        configuring_targets[it.first] = it.second;
    }

    return DE_OK;
}

//------------------------------------------------------------------------------
void GridDriver::_update_error_counters(FpuCounters &fpu_counters,
                                        const t_fpu_state &prev_fpu_state,
                                        const t_fpu_state &moved_fpu_state,
                                        bool datum_cmd)
{
    if (moved_fpu_state.beta_collision)
    {
        fpu_counters.addToCount(FpuCounterId::collisions, 1);
    }
    if ((moved_fpu_state.state == FPST_OBSTACLE_ERROR) &&
        moved_fpu_state.at_alpha_limit)
    {
        fpu_counters.addToCount(FpuCounterId::limit_breaches, 1);
    }

    if (moved_fpu_state.timeout_count != prev_fpu_state.timeout_count)
    {
        int diff = moved_fpu_state.timeout_count - prev_fpu_state.timeout_count;
        if (diff < 0)
        {
            // The underlying unsigned 16-bit value has wrapped around and
            // needs to be corrected
            diff += 1 << 16;
        }
        fpu_counters.addToCount(FpuCounterId::can_timeout, diff);
    }

    if ((moved_fpu_state.last_status == MCE_ERR_DATUM_TIME_OUT) ||
        ((moved_fpu_state.last_status == MCE_COMMAND_TIMEDOUT) && datum_cmd))
    {
        fpu_counters.addToCount(FpuCounterId::datum_timeout, 1);
    }

    if ((moved_fpu_state.last_status == MCE_COMMAND_TIMEDOUT) &&
        (moved_fpu_state.last_command == CCMD_EXECUTE_MOTION))
    {
        fpu_counters.addToCount(FpuCounterId::movement_timeout, 1);
    }
}

//------------------------------------------------------------------------------
E_EtherCANErrCode GridDriver::_pre_config_motion_hook(const t_wtable &wtable,
                                                      t_grid_state &gs,
                                                      const t_fpuset &fpuset,
                                                      Range wmode)
{
    return _check_and_register_wtable(wtable, gs, fpuset, wmode, 1);
}

//------------------------------------------------------------------------------
// TODO: Johannes' comment: "This can possibly be deleted, but do we want to
// store the full wavetable?"
E_EtherCANErrCode GridDriver::_save_wtable_direction(const t_fpuset &fpuset,
                                                     bool is_reversed,
                                                     t_grid_state &gs)
{
    MdbResult mdb_result = MDB_PANIC;
    auto txn = protection_db.createTransaction(mdb_result);
    if (txn)
    {
#ifdef FLEXIBLE_CAN_MAPPING
        for (int fpu_id : config.getFpuIdList())
#else
        for (int fpu_id = 0; fpu_id < config.num_fpus; fpu_id++)
#endif
        {
            if (!fpuset[fpu_id])
            {
                continue;
            }

            fpus_data[fpu_id].db.wf_reversed = is_reversed;
            if (txn->fpuDbTransferWfReversedFlag(DbTransferType::Write,
                            gs.FPU_state[fpu_id].serial_number,
                            fpus_data[fpu_id].db.wf_reversed) != MDB_SUCCESS)
            {
                return DE_DB_WRITE_FAILED;
            }
        }
    }
    else
    {
        return DE_DB_TRANSACTION_CREATION_FAILED;
    }

    return DE_OK;
}

//------------------------------------------------------------------------------
E_EtherCANErrCode GridDriver::_post_config_motion_hook(const t_wtable &wtable, 
                                                       t_grid_state &gs,
                                                       const t_fpuset &fpuset)
{
    // Update ranges that will become valid once executeMotion is started
    for (const auto &it : configuring_ranges)
    {
        int fpu_id = it.first;
        if (fpu_id >= config.num_fpus)
        {
            return DE_INVALID_FPU_ID;
        }

        if (!fpuset[fpu_id])
        {
            continue;
        }

        if (wavetable_was_received(wtable, fpu_id, gs.FPU_state[fpu_id]))
        {
            configured_ranges[fpu_id] = it.second;
            // Store actual target position
            configured_targets[fpu_id] = configuring_targets[fpu_id];
        }
    }

    // Create fpuset_wtable
    t_fpuset fpuset_wtable;
    clearFpuSet(fpuset_wtable);
    for (size_t i = 0; i < wtable.size(); i++)
    {
        if (wtable[i].fpu_id < config.num_fpus)
        {
            fpuset_wtable[wtable[i].fpu_id] = true;
        }
        else
        {
            return DE_INVALID_FPU_ID;
        }
    }

    const bool is_reversed = false;
    E_EtherCANErrCode ecan_result = _save_wtable_direction(fpuset_wtable,
                                                           is_reversed, gs);
    if (ecan_result != DE_OK) 
    {
        return ecan_result;
    }

    // Save the changed waveforms
    MdbResult mdb_result = MDB_PANIC;
    auto txn = protection_db.createTransaction(mdb_result);
    if (txn)
    {
        for (size_t i = 0; i < wtable.size(); i++)
        {
            int fpu_id = wtable[i].fpu_id;
            if (fpu_id < config.num_fpus)
            {
                if (txn->fpuDbTransferWaveform(DbTransferType::Write,
                        gs.FPU_state[fpu_id].serial_number,
                        const_cast<t_waveform_steps &>(wtable[i].steps)) != MDB_SUCCESS)
                {
                    return DE_DB_WRITE_FAILED;
                }
            }
            else
            {
                return DE_INVALID_FPU_ID;
            }
        }
    }
    else
    {
        return DE_DB_TRANSACTION_CREATION_FAILED;
    }

    return DE_OK;
}

//------------------------------------------------------------------------------
E_EtherCANErrCode GridDriver::_pre_repeat_motion_hook(const t_wtable &wtable,
                                                      t_grid_state &gs,
                                                      const t_fpuset &fpuset,
                                                      Range wmode)
{
    const int sign = 1;
    return _check_and_register_wtable(wtable, gs, fpuset, wmode, sign);
}

//------------------------------------------------------------------------------
E_EtherCANErrCode GridDriver::_post_repeat_motion_hook(const t_wtable &wtable,
                                                       t_grid_state &gs,
                                                       const t_fpuset &fpuset)
{
    return _post_repeat_reverse_motion_hook(wtable, gs, fpuset,
                                            FPST_READY_FORWARD, false);
}

//------------------------------------------------------------------------------
E_EtherCANErrCode GridDriver::_pre_reverse_motion_hook(const t_wtable &wtable,
                                                       t_grid_state &gs,
                                                       const t_fpuset &fpuset,
                                                       Range wmode)
{
    const int sign = -1;
    return _check_and_register_wtable(wtable, gs, fpuset, wmode, sign);
}

//------------------------------------------------------------------------------
E_EtherCANErrCode GridDriver::_post_reverse_motion_hook(const t_wtable &wtable,
                                                        t_grid_state &gs,
                                                        const t_fpuset &fpuset)
{
    return _post_repeat_reverse_motion_hook(wtable, gs, fpuset,
                                            FPST_READY_REVERSE, true);
}

//------------------------------------------------------------------------------
E_EtherCANErrCode GridDriver::_post_repeat_reverse_motion_hook(
                                                    const t_wtable &wtable,
                                                    t_grid_state &gs,
                                                    const t_fpuset &fpuset,
                                                    E_FPU_STATE target_state,
                                                    bool is_reversed)
{
    // Update ranges that become valid once executeMotion is started

    t_fpuset fpuset_to_save;
    clearFpuSet(fpuset_to_save);

    for (const auto &it : configuring_ranges)
    {
        int fpu_id = it.first;
        if (fpu_id >= config.num_fpus)
        {
            return DE_INVALID_FPU_ID;
        }
        if (!fpuset[fpu_id])
        {
            continue;
        }

        const bool allow_unconfirmed = true;
        if (wavetable_was_received(wtable, fpu_id, gs.FPU_state[fpu_id],
                                   allow_unconfirmed, target_state))
        {
            configured_ranges[fpu_id] = it.second;
            fpuset_to_save[fpu_id] = true;
            configured_targets[fpu_id] = configuring_targets[fpu_id];
        }
    }

    return _save_wtable_direction(fpuset_to_save, is_reversed, gs);
}

//------------------------------------------------------------------------------
void GridDriver::_update_counters_execute_motion(int fpu_id,
                                                 FpuCounters &fpu_counters,
                                                 const t_waveform_steps &waveform,
                                                 bool is_reversed,
                                                 bool cancel)
{
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

//------------------------------------------------------------------------------
E_EtherCANErrCode GridDriver::_start_execute_motion_hook(t_grid_state &gs,
                                                         const t_fpuset &fpuset,
                                            t_fpu_positions &initial_positions)
{
    // This runs before executeMotion command is started. After that point, the
    // FPU in fpuset should be moving within the ranges set by the last
    // configMotion(), repeatMotion() or reverseMotion() command. These ranges
    // are set as the intervals which define the possible positions until the
    // command has finished.
    //
    // initial_positions is filled with the positions as (alpha, beta) tuples
    // at the time the command was called.

    initial_positions.clear();

    E_EtherCANErrCode ecan_result = _pingFPUs(gs, fpuset);
    if (ecan_result != DE_OK)
    {
        return ecan_result;
    }

    MdbResult mdb_result = MDB_PANIC;
    auto txn = protection_db.createTransaction(mdb_result);
    if (txn)
    {
#ifdef FLEXIBLE_CAN_MAPPING
        for (int fpu_id : config.getFpuIdList())
#else
        for (int fpu_id = 0; fpu_id < config.num_fpus; fpu_id++)
#endif
        {
            if (!fpuset[fpu_id])
            {
                continue;
            }

            FpuData &fpu_data = fpus_data[fpu_id];

            initial_positions[fpu_id] = { fpu_data.db.apos, fpu_data.db.bpos };
            // Copy configured alpha and beta position intervals, and store
            // both to DB
            auto it = configured_ranges.find(fpu_id);
            if (it != configured_ranges.end())
            {
                const char *serial_number = gs.FPU_state[fpu_id].serial_number;
                if (!_update_apos(txn, serial_number, fpu_id, it->second.apos))
                {
                    return DE_DB_WRITE_FAILED;
                }
                if (!_update_bpos(txn, serial_number, fpu_id, it->second.bpos))
                {
                    return DE_DB_WRITE_FAILED;
                }
                // Update target position which is used in case of step counter
                // overflow
                fpu_data.target_position = configured_targets[fpu_id];

                _update_counters_execute_motion(fpu_id,
                                                fpu_data.db.counters,
                                                fpu_data.db.last_waveform,
                                                fpu_data.db.wf_reversed);

                if (txn->fpuDbTransferCounters(DbTransferType::Write,
                                               serial_number,
                                               fpu_data.db.counters) != MDB_SUCCESS)
                {
                    return DE_DB_WRITE_FAILED;
                }
            }
        }
    }
    else
    {
        return DE_DB_TRANSACTION_CREATION_FAILED;
    }

    if (protection_db.sync() != MDB_SUCCESS)
    {
        return DE_DB_SYNC_FAILED;
    }

    return DE_OK;
}

//------------------------------------------------------------------------------
E_EtherCANErrCode GridDriver::_cancel_execute_motion_hook(t_grid_state &gs,
                                                          const t_fpuset &fpuset,
                                    const t_fpu_positions &initial_positions)
{
    // Cancels registering an executeMotion command which was rejected by the
    // driver, before an actual movement was started.

    MdbResult mdb_result = MDB_PANIC;
    auto txn = protection_db.createTransaction(mdb_result);
    if (txn)
    {
        for (const auto &it : initial_positions)
        {
            const int fpu_id = it.first;
            if (fpu_id >= config.num_fpus)
            {
                return DE_INVALID_FPU_ID;
            }

            if (!fpuset[fpu_id])
            {
                continue;
            }

            const Interval &apos = it.second.apos;
            const Interval &bpos = it.second.bpos;
            const char *serial_number = gs.FPU_state[fpu_id].serial_number;
            if (!_update_apos(txn, serial_number, fpu_id, apos))
            {
                return DE_DB_WRITE_FAILED;
            }
            if (!_update_bpos(txn, serial_number, fpu_id, bpos))
            {
                return DE_DB_WRITE_FAILED;
            }

            FpuData &fpu_data = fpus_data[fpu_id];

            fpu_data.target_position = { apos, bpos };
            
            if (configured_ranges.find(fpu_id) != configured_ranges.end()) 
            {
                _update_counters_execute_motion(fpu_id, fpu_data.db.counters,
                                                fpu_data.db.last_waveform,
                                                fpu_data.db.wf_reversed, true);
            }
            if (txn->fpuDbTransferCounters(DbTransferType::Write,
                                           serial_number,
                                           fpu_data.db.counters) != MDB_SUCCESS)
            {
                return DE_DB_WRITE_FAILED;
            }
        }
    }
    else
    {
        return DE_DB_TRANSACTION_CREATION_FAILED;
    }

    if (protection_db.sync() != MDB_SUCCESS)
    {
        return DE_DB_SYNC_FAILED;
    }

    return DE_OK;
}

//------------------------------------------------------------------------------
E_EtherCANErrCode GridDriver::_post_execute_motion_hook(t_grid_state &gs,
                                                    const t_grid_state &old_gs,
                                                    const t_grid_state &move_gs,
                                                    const t_fpuset &fpuset)
{
    // This runs after both an executeMotion has run, and *also* a ping has
    // returned successfully.

    // What do we here if FPUs are still moving (this would happen in case of
    // an error)? Solution for now: wait.
    // TODO: The above were Johannes' comments - need to look at this further?
    for (int loopCount = 0; loopCount < 50; loopCount++)
    {
        if ((gs.Counts[FPST_MOVING] > 0) || (gs.Counts[FPST_DATUM_SEARCH] > 0))
        {
            // Build set of FPUs requiring refresh based upon their current
            // state
            t_fpuset fpuset_refresh;
            clearFpuSet(fpuset_refresh);
            bool refresh_required = false;
#ifdef FLEXIBLE_CAN_MAPPING
            for (int fpu_id : config.getFpuIdList())
#else
            for (int fpu_id = 0; fpu_id < config.num_fpus; fpu_id++)
#endif
            {
                if ((gs.FPU_state[fpu_id].state == FPST_MOVING) ||
                    (gs.FPU_state[fpu_id].state == FPST_DATUM_SEARCH))
                {
                    fpuset_refresh[fpu_id] = true;
                    refresh_required = true;
                }
            }
            if (!refresh_required)
            {
                break;
            }

            // Add FPUs requiring ping to the set
            t_fpuset fpuset_ping_notok;
            need_ping(gs, fpuset, fpuset_ping_notok);
#ifdef FLEXIBLE_CAN_MAPPING
            for (int fpu_id : config.getFpuIdList())
#else
            for (int fpu_id = 0; fpu_id < config.num_fpus; fpu_id++)
#endif
            {
                if (fpuset_ping_notok[fpu_id])
                {
                    fpuset_refresh[fpu_id] = true;
                }
            }

            sleepSecs(0.2);

            E_EtherCANErrCode ecan_result = _pingFPUs(gs, fpuset_refresh);
            EtherCANErrorGroup error_group = errorGroup(ecan_result);
            if (error_group == EtherCANErrorGroup::CommandTimeout)
            {
                break;
            }
            else if (ecan_result != DE_OK)
            {
                return ecan_result;
            }
        }
        else
        {
            break;
        }
    }

    // The assumption here is that the offsets did not change, even if the
    // waveform execution was aborted. This might not be correct in a sense
    // valid for science measurements, because collisions would compromise the
    // precision of step counts, but for protection purposes this should be OK.
    //
    // Thus, use the step counter positions to update the location. If the FPU
    // step counters are in underflow / overflow, use the registered target
    // positions, to continue tracking.
    // But, roll back the latter to the movement range for an FPU, if that FPU
    // is not yet at target.
#ifdef FLEXIBLE_CAN_MAPPING
    for (int fpu_id : config.getFpuIdList())
#else
    for (int fpu_id = 0; fpu_id < config.num_fpus; fpu_id++)
#endif
    {
        if (!fpuset[fpu_id])
        {
            continue;
        }

        if (gs.FPU_state[fpu_id].state != FPST_RESTING)        
        {
            fpus_data[fpu_id].target_position = { fpus_data[fpu_id].db.apos,
                                                  fpus_data[fpu_id].db.bpos };
        }
    }

    const bool store = true;
    E_EtherCANErrCode ecan_result = _refresh_positions(gs, store, fpuset);
    if (ecan_result != DE_OK)
    {
        return ecan_result;
    }

    MdbResult mdb_result = MDB_PANIC;
    auto txn = protection_db.createTransaction(mdb_result);
    if (txn)
    {
#ifdef FLEXIBLE_CAN_MAPPING
        for (int fpu_id : config.getFpuIdList())
#else
        for (int fpu_id = 0; fpu_id < config.num_fpus; fpu_id++)
#endif
        {
            if (!fpuset[fpu_id])
            {
                continue;
            }

            _update_error_counters(fpus_data[fpu_id].db.counters,
                                   old_gs.FPU_state[fpu_id],
                                   move_gs.FPU_state[fpu_id]);
            const char *serial_number = move_gs.FPU_state[fpu_id].serial_number;
            if (txn->fpuDbTransferCounters(DbTransferType::Write, serial_number,
                                fpus_data[fpu_id].db.counters) != MDB_SUCCESS)
            {
                return DE_DB_WRITE_FAILED;
            }
        }
    }
    else
    {
        return DE_DB_TRANSACTION_CREATION_FAILED;
    }

    // Clear wavetable spans for the addressed FPUs - they are no longer valid
#ifdef FLEXIBLE_CAN_MAPPING
    for (int fpu_id : config.getFpuIdList())
#else
    for (int fpu_id = 0; fpu_id < config.num_fpus; fpu_id++)
#endif
    {
        if (fpuset[fpu_id])
        {
            // Erase if exists
            configured_ranges.erase(fpu_id);
        }
    }

    return DE_OK;
}

//------------------------------------------------------------------------------
E_EtherCANErrCode GridDriver::_pre_free_beta_collision_hook(int fpu_id,
                                                E_REQUEST_DIRECTION direction,
                                                const t_grid_state &gs,
                                                bool soft_protection)
{
    FpuData &fpu_data = fpus_data[fpu_id];

    int64_t brcnt;
    if (direction == REQD_CLOCKWISE)
    {
        brcnt = fpu_data.db.bretries_cw;
    }
    else
    {
        brcnt = fpu_data.db.bretries_acw;
    }

    // NOTE: Maximum number of beta retries is now taken from the constants
    // rather than from the database.
    // if soft_protection and (brcnt >= self.maxbretries[fpu_id]): # Old Johannes Python code.
    if (soft_protection && (brcnt >= DEFAULT_FREE_BETA_RETRIES))
    {
        return DE_MAX_RETRIES_EXCEEDED;
    }

    int diff;
    if (direction == REQD_CLOCKWISE)
    {
        diff = -FREE_BETA_STEPCOUNT;
    }
    else
    {
        diff = FREE_BETA_STEPCOUNT;
    }

    Interval bpos = fpu_data.db.bpos;
    Interval new_bpos = bpos + (((double)diff) / STEPS_PER_DEGREE_BETA);

    fpu_data.target_position = { fpu_data.db.apos, new_bpos };

    MdbResult mdb_result = MDB_PANIC;
    auto txn = protection_db.createTransaction(mdb_result);
    if (txn)
    {
        const char *serial_number = gs.FPU_state[fpu_id].serial_number;
        if (!_update_bpos(txn, serial_number, fpu_id, bpos.combine(new_bpos)))
        {
            return DE_DB_WRITE_FAILED;
        }
    }
    else
    {
        return DE_DB_TRANSACTION_CREATION_FAILED;
    }

    return DE_OK;
}

//------------------------------------------------------------------------------
E_EtherCANErrCode GridDriver::_post_free_beta_collision_hook(int fpu_id,
                                                E_REQUEST_DIRECTION direction,
                                                const t_grid_state &gs)
{
    int64_t count;
    FpuDbIntValType db_cw_or_acw;
    if (direction == REQD_CLOCKWISE)
    {
        fpus_data[fpu_id].db.bretries_cw += 1;
        count = fpus_data[fpu_id].db.bretries_cw;
        db_cw_or_acw = FpuDbIntValType::BetaRetries_CW;
    }
    else
    {
        fpus_data[fpu_id].db.bretries_acw += 1;
        count = fpus_data[fpu_id].db.bretries_acw;
        db_cw_or_acw = FpuDbIntValType::BetaRetries_ACW;
    }
    
    // The following code block is inside its own scope so that txn is
    // automatically destroyed at the end of it - this is required because
    // the subsequent _refresh_positions() call also creates its own
    // internal transaction, and ProtectionDB/ProtectionDBTxn requires that
    // there is only one transaction instance in existence at a time.
    // TODO: This could instead be achieved by putting txn.reset() after this
    // block?
    {
        MdbResult mdb_result = MDB_PANIC;
        auto txn = protection_db.createTransaction(mdb_result);
        if (txn)
        {
            const char *serial_number = gs.FPU_state[fpu_id].serial_number;
            if (txn->fpuDbTransferInt64Val(DbTransferType::Write, db_cw_or_acw,
                                        serial_number, count) != MDB_SUCCESS)
            {
                return DE_DB_WRITE_FAILED;
            }
        }
        else
        {
            return DE_DB_TRANSACTION_CREATION_FAILED;
        }
    }

    // N.B. Old Johannes Python code here which he had commented out:
    // #self._pingFPUs(grid_state, fpuset=fpuset)

    t_fpuset fpuset;
    createFpuSetForSingleFpu(fpu_id, fpuset);

    const bool store = true;
    return _refresh_positions(gs, store, fpuset);
}

//------------------------------------------------------------------------------
E_EtherCANErrCode GridDriver::_pre_free_alpha_limit_breach_hook(int fpu_id,
                                                E_REQUEST_DIRECTION direction,
                                                const t_grid_state &gs,
                                                bool soft_protection)
{
    FpuData &fpu_data = fpus_data[fpu_id];

    int64_t brcnt;
    if (direction == REQD_CLOCKWISE)
    {
        // *********** TODO: The original Python version does NOT define brcnt
        // AT ALL here, which might be a bug? (because subsequent attempted use
        // of undefined brcnt would then fail). Also, what to do in this C++
        // code? (have just set to 0 for now)
        brcnt = 0;
    }
    else
    {
        brcnt = fpu_data.db.aretries_acw;
    }

    // NOTE: Maximum number of beta retries is now taken from the constants
    // rather than from the database.
    // #if soft_protection and (brcnt >= self.maxaretries[fpu_id]): # Old Johannes Python code.
    if (soft_protection && (brcnt >= DEFAULT_FREE_ALPHA_RETRIES))
    {
        return DE_MAX_RETRIES_EXCEEDED;
    }

    int diff;
    if (direction == REQD_CLOCKWISE)
    {
        diff = -FREE_ALPHA_STEPCOUNT;
    }
    else
    {
        diff = FREE_ALPHA_STEPCOUNT;
    }

    Interval apos = fpu_data.db.apos;
    Interval new_apos = apos + (((double)diff) / STEPS_PER_DEGREE_ALPHA);

    fpu_data.target_position = { new_apos, fpu_data.db.bpos };

    MdbResult mdb_result = MDB_PANIC;
    auto txn = protection_db.createTransaction(mdb_result);
    if (txn)
    {
        const char *serial_number = gs.FPU_state[fpu_id].serial_number;
        if (!_update_apos(txn, serial_number, fpu_id, apos.combine(new_apos)))
        {
            return DE_DB_WRITE_FAILED;
        }
    }
    else
    {
        return DE_DB_TRANSACTION_CREATION_FAILED;
    }

    return DE_OK;
}

//------------------------------------------------------------------------------
E_EtherCANErrCode GridDriver::_post_free_alpha_limit_breach_hook(int fpu_id,
                                                E_REQUEST_DIRECTION direction,
                                                const t_grid_state &gs)
{
    int64_t count;
    FpuDbIntValType db_cw_or_acw;
    if (direction == REQD_CLOCKWISE)
    {
        fpus_data[fpu_id].db.aretries_cw += 1;
        count = fpus_data[fpu_id].db.aretries_cw;
        db_cw_or_acw = FpuDbIntValType::AlphaRetries_CW;
    }
    else
    {
        fpus_data[fpu_id].db.aretries_acw += 1;
        count = fpus_data[fpu_id].db.aretries_acw;
        db_cw_or_acw = FpuDbIntValType::AlphaRetries_ACW;
    }

    // The following code block is inside its own scope so that txn is
    // automatically destroyed at the end of it - this is required because
    // the subsequent _refresh_positions() call also creates its own
    // internal transaction, and ProtectionDB/ProtectionDBTxn requires that
    // there is only one transaction instance in existence at a time.
    // TODO: This could instead be achieved by putting txn.reset() after this
    // block?
    {
        MdbResult mdb_result = MDB_PANIC;
        auto txn = protection_db.createTransaction(mdb_result);
        if (txn)
        {
            const char *serial_number = gs.FPU_state[fpu_id].serial_number;
            if (txn->fpuDbTransferInt64Val(DbTransferType::Write, db_cw_or_acw,
                                        serial_number, count) != MDB_SUCCESS)
            {
                return DE_DB_WRITE_FAILED;
            }
        }
        else
        {
            return DE_DB_TRANSACTION_CREATION_FAILED;
        }
    }

    // N.B. Old Johannes Python code here which he had commented out:
    // #self._pingFPUs(grid_state, fpuset=fpuset)

    t_fpuset fpuset;
    createFpuSetForSingleFpu(fpu_id, fpuset);

    const bool store = true;
    return _refresh_positions(gs, store, fpuset);
}

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
#endif // ENABLE_PROTECTION_CODE
//------------------------------------------------------------------------------

} // namespace mpifps

