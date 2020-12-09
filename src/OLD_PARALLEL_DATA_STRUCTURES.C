
// BW comment 12/11/2020: This file keeps my original C++ code for converting
// the Python grid driver version's "parallel" FPU data structures into C++,
// where each individual data item consists of a full FPU array (i.e. about
// 1000 items). However, this data structuring approach proved too cumbersome
// in C++, so the grid driver code now uses the FpuData and FpuDbData 
// structures, with an array of these structures. But am keeping this old
// code now for reference.


// TODO: OLD CODE: Old non-aggregated FPU data - replaced by FpuData and
// FpuDbData structures, but kept here for now - remove once no longer needed
    // N.B. These vectors all have their sizes set to the number of FPUs
    std::vector<Interval> apositions;
    std::vector<Interval> bpositions;
    // TODO: This wf_reversed vector was moved here into GridDriver from
    // UnprotectedGridDriver so that it can eventually be included into an FPU
    // database data structure - this is OK because it's not actually used in
    // UnprotectedGridDriver. N.B. The associated set_wtable_reversed() function
    // is no longer required so has been removed, but the getReversed() function
    // might still be required? (it's shown in the FPU grid driver document)
    std::vector<bool> wf_reversed; // N.B. Size is set to config.num_fpus
    std::vector<Interval> alimits;
    std::vector<Interval> blimits;
    std::vector<int64_t> maxaretries;
    std::vector<int64_t> aretries_cw;
    std::vector<int64_t> aretries_acw;
    std::vector<int64_t> maxbretries;
    std::vector<int64_t> bretries_cw;
    std::vector<int64_t> bretries_acw;
    std::vector<FpuCounters> counters;

    std::vector<Interval> a_caloffsets;
    std::vector<Interval> b_caloffsets;
    std::vector<FpuCounters> _last_counters;
    std::vector<t_fpu_position> target_positions;


//==============================================================================
// TODO: Old _post_connect_hook() with the old parallel std::vector data sets
// structure - keeping for now just in case need to revert to this approach,
// but hopefully not
//==============================================================================
E_EtherCANErrCode GridDriver::_post_connect_hook()
{
    bool result_ok = false;

    // TODO: Check that the FPU database has been properly opened at this point
    // self.fpudb = self.env.open_db(ProtectionDB.dbname)

    // TODO: Implement the following? (from Python version)
    // self.healthlog = self.env.open_db(HealthLogDB.dbname)

    E_EtherCANErrCode ecan_result;

    t_grid_state grid_state;
    getGridState(grid_state);

    t_fpuset fpuset;
    createFpuSetForNumFpus(config.num_fpus, fpuset);

    //**************************************
    // NOTE: OLD FUNCTION
    //**************************************

    //*************** TODO: Do something with ecan_result value below
    ecan_result = readSerialNumbers(grid_state, fpuset);
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

    //**************************************
    // NOTE: OLD FUNCTION
    //**************************************

    //..........................................................................
    // Read all FPUs' data items from the protection database into the
    // temporary value vectors
    struct
    {
        FpuDbIntervalType type;
        std::vector<Interval> &vector_ref;
    } interval_items[(int)FpuDbIntervalType::NumTypes] = 
    {
        { FpuDbIntervalType::AlphaPos,    apositions_temp },
        { FpuDbIntervalType::BetaPos,     bpositions_temp },
        { FpuDbIntervalType::AlphaLimits, alimits_temp    },
        { FpuDbIntervalType::BetaLimits,  blimits_temp    }
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

    //**************************************
    // NOTE: OLD FUNCTION
    //**************************************

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
            for (int i = 0; i < (int)FpuDbIntervalType::NumTypes; i++)
            {
                if (!txn->fpuDbTransferInterval(DbTransferType::Read,
                                                interval_items[i].type,
                                                serial_number,
                                                interval_items[i].vector_ref[fpu_id],
                                                datum_offset))
                {
                    result_ok = false;
                    break;
                }
            }

    //**************************************
    // NOTE: OLD FUNCTION
    //**************************************

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
            
    //**************************************
    // NOTE: OLD FUNCTION
    //**************************************

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
    
    // TODO: Check ecan_result code of _pingFPUs()
    ecan_result = _pingFPUs(grid_state, fpuset);

    _reset_hook(grid_state, grid_state, fpuset);
    _refresh_positions(grid_state, false, fpuset);
    
    //..........................................................................
}



