// -*- mode: c++ -*-

////////////////////////////////////////////////////////////////////////////////
//
// Copyright 2020 UKRI. See file "LICENSE" for license information.
//
//
// Who       When        What
// --------  ----------  -------------------------------------------------------
// bwillemse 2020-11-16  Created (adapted from Python fpu-admin script).
//------------------------------------------------------------------------------

////////////////////////////////////////////////////////////////////////////////
// NAME FPUAdmin.C
//
// FPU database administration functions.
//
////////////////////////////////////////////////////////////////////////////////

#include <iostream>
#include <string>
#include "FPUAdmin.h"
#include "UnprotectedGridDriver.h"
#include "T_GridState.h"
#include "ProtectionDB.h"
#include "FPUConstants.h"

namespace mpifps
{

//------------------------------------------------------------------------------
bool FPUAdmin::flash(ProtectionDbTxnPtr &txn, int fpu_id,
                     const char *new_serial_number, bool mockup,
                     bool reuse_snum, t_gateway_address gateway_address)
{
    // Flashes serial number to FPU with ID fpu_id. FPU must be connected.
    // If reuse_snum is true, it is allowed to use a serial number which was
    // used before.

    UnprotectedGridDriver ugd(fpu_id + 1);

#if 0
    //if gateway_address is None:
    const t_gateway_address gateway_address;
    if (mockup)
    {
        gateway_address = [ FpuGridDriver.GatewayAddress("127.0.0.1", p)
                            for p in [4700, 4701, 4702] ]
    }
    else
    {
        gateway_address = [ FpuGridDriver.GatewayAddress(GATEWAY0_ADDRESS, 4700) ]
    }


    // TODO: t_gateway_address::ip is only a pointer - dangerous? Change this
    // eventually? (e.g. to a std::string?)

    // ********* TODO: The following should be MULTIPLE gateways??? (see original
    // flash_FPU() function)
    E_EtherCANErrorCode ecan_result = ugd.connect(1, gateway_address);

    t_grid_state grid_state;
    if (ecan_result == DE_OK)
    {
        ugd.getGridState(grid_state);

        ecan_result = ugd.pingFPUs(grid_state);
    }

    if (ecan_result == DE_OK)
    {
        ecan_result = ugd.readSerialNumbers(grid_state);
    }

    if (ecan_result == DE_OK)
    {
        // print("flashing FPU #%i with serial number %r" % (fpu_id, serial_number))

        ecan_result = ugd.writeSerialNumber(fpu_id, serial_number, grid_state);
    }

    // TODO: Print ecan_result or similar to std::cout?
    
    return false;
#else
    return true;
#endif
}

//------------------------------------------------------------------------------
bool FPUAdmin::init(ProtectionDbTxnPtr &txn, const char *serial_number, 
                    double apos_min, double apos_max,
                    double bpos_min, double bpos_max,
                    bool reinitialize, double adatum_offset)
{
    // Initializes the FPU in the protection database, passing the initial alpha
    // and beta arm min and max positions in degrees. The optional adatum_offset
    // parameter is the alpha datum offset.
    // If reinitialize is true, it is allowed to redefine FPU positions
    // which already have been stored before.

    FpuDbData fpu_db_data;

    // TODO: In the Python fpu-admin version, the apos/bpos/alimits/blimits
    // Intervals stored into the database also include alpha and beta OFFSETS
    // - so need to replicate this somehow
    fpu_db_data.apos = Interval(apos_min, apos_max); // TODO: Include alpha_offset
    fpu_db_data.bpos = Interval(bpos_min, bpos_max); // TODO: Include BETA_DATUM_OFFSET
    fpu_db_data.wf_reversed = false;
    fpu_db_data.alimits = Interval(ALPHA_MIN_DEGREE, ALPHA_MAX_DEGREE); // TODO: Include alpha_offset
    fpu_db_data.blimits = Interval(BETA_MIN_DEGREE, BETA_MAX_DEGREE);   // TODO: Include BETA_DATUM_OFFSET
    fpu_db_data.maxaretries = DEFAULT_FREE_ALPHA_RETRIES;
    fpu_db_data.aretries_cw = 0;
    fpu_db_data.aretries_acw = 0;
    fpu_db_data.maxbretries = DEFAULT_FREE_BETA_RETRIES;
    fpu_db_data.bretries_cw = 0;
    fpu_db_data.bretries_acw = 0;
    if (!reinitialize)
    {
        // If the FPU has an existing counters entry in the database, then
        // retain this

        //*************
        // ************ TODO: Look more closely at the help comments and Python
        // code for the reinitialize flag, and check that this C++ code and its
        // comments reflect this
        //*************

        // TODO: Need to distinguish between serial number not being found vs
        // the database read failing at a lower level - if counters entry for
        // serial number doesn't yet exist then NOT a failure - the counters
        // just need to be initialised as empty
        if (!txn->fpuDbTransferCounters(DbTransferType::Read, serial_number,
                                        fpu_db_data.counters))
        {
            fpu_db_data.counters.zeroAll();
        }
    }
    fpu_db_data.last_waveform.clear();

    if (!txn->fpuDbTransferFpu(DbTransferType::Write, serial_number, fpu_db_data))
    {
        return false;
    }
    return true;
}

//------------------------------------------------------------------------------
bool FPUAdmin::listAll(ProtectionDbTxnPtr &txn)
{
    // Prints data for all FPUs in database using std::cout

    std::vector<std::string> serial_numbers;
    if (txn->fpuDbGetSerialNumbers(serial_numbers))
    {
        
    }
    else
    {
        // TODO: 
        return false;
    }
    return true;
}

//------------------------------------------------------------------------------
bool FPUAdmin::listOne(ProtectionDbTxnPtr &txn, const char *serial_number)
{
    // Prints serial number and data for one FPU using std::cout

    std::cout << "FPU serial number: " << serial_number << "\n";
    FpuDbData fpu_db_data;
    if (txn->fpuDbTransferFpu(DbTransferType::Read, serial_number, fpu_db_data))
    {
        printFpuDbData(fpu_db_data);
        return true;
    }
    else
    {
        std::cout << "**ERROR**: One or more of this FPU's data items are "
                     "missing from the database.\n" << std::endl;
        return false;
    }
}

//------------------------------------------------------------------------------
void FPUAdmin::printFpuDbData(FpuDbData &fpu_db_data)
{
    // Prints data for one FPU using std::cout

    // apos / bpos / wf_reversed
    // TODO: Also display alpha offset (and beta offset?) - see fpu driver
    // manual example output on page 24
    std::cout << "apos = " << fpu_db_data.apos.toString() <<
                 ", bpos = " << fpu_db_data.bpos.toString();
    const char *wf_reversed_str = "";
    if (fpu_db_data.wf_reversed)
    {
        wf_reversed_str = "true";
    }
    else
    {
        wf_reversed_str = "false";
    }
    std::cout << ", wf_reversed = " << wf_reversed_str << "\n";
    
    // alimits / blimits
    // TODO: Also display alpha offset (and beta offset?) - see fpu driver
    // manual example output on page 24
    std::cout << "alimits = " << fpu_db_data.alimits.toString() <<
                 ", blimits = " << fpu_db_data.blimits.toString() << "\n";

    // maxaretries / aretries_cw / aretries_acw
    std::cout << "max_a_retries = " << std::to_string(fpu_db_data.maxaretries) <<
       ", a_retries_cw = " << std::to_string(fpu_db_data.aretries_cw) <<
       ", a_retries_acw = " << std::to_string(fpu_db_data.aretries_acw) << "\n";

    // maxbretries / bretries_cw / bretries_acw
    std::cout << "max_b_retries = " << std::to_string(fpu_db_data.maxbretries) <<
       ", b_retries_cw = " << std::to_string(fpu_db_data.bretries_cw) <<
       ", b_retries_acw = " << std::to_string(fpu_db_data.bretries_acw) << "\n";

    // TODO: Also display fpu_db_data.counters and fpu_db_data.last_waveform?

    std::cout << std::endl;
}

//------------------------------------------------------------------------------
bool FPUAdmin::setALimits(ProtectionDbTxnPtr &txn, const char *serial_number, 
                          double alimit_min, double alimit_max,
                          double adatum_offset)
{
    // Sets safe limits for alpha arm of an FPU.

    Interval alimits_interval(alimit_min, alimit_max);
    // TODO: Modify below once have better return codes
    return txn->fpuDbTransferPosition(DbTransferType::Write,
                                      FpuDbPositionType::AlphaLimits,
                                      serial_number, alimits_interval,
                                      adatum_offset);
}

//------------------------------------------------------------------------------
bool FPUAdmin::setBLimits(ProtectionDbTxnPtr &txn, const char *serial_number, 
                          double blimit_min, double blimit_max)
{
    // Sets safe limits for beta arm of an FPU.

    Interval blimits_interval(blimit_min, blimit_max);
    double datum_offset = 0.0;
    // TODO: Modify below once have better return codes
    return txn->fpuDbTransferPosition(DbTransferType::Write,
                                      FpuDbPositionType::BetaLimits,
                                      serial_number, blimits_interval,
                                      datum_offset); //************* TODO: Put a beta datum offset constant here?
}

//------------------------------------------------------------------------------
bool FPUAdmin::setARetries(ProtectionDbTxnPtr &txn, const char *serial_number,
                           int64_t aretries)
{
    // Sets allowed number of freeAlphaLimitBreach commands in the same
    // direction before the software protection kicks in. The retry count is
    // reset to zero upon a successfully finished datum search.

    return txn->fpuDbTransferInt64Val(DbTransferType::Write,
                                      FpuDbIntValType::FreeAlphaRetries,
                                      serial_number, aretries);
}

//------------------------------------------------------------------------------
bool FPUAdmin::setBRetries(ProtectionDbTxnPtr &txn, const char *serial_number,
                           int64_t bretries)
{
    // Sets allowed number of freeBetaCollision commands in the same direction
    // before the software protection kicks in. The retry count is reset to
    // zero upon a successfully finished datum search.

    return txn->fpuDbTransferInt64Val(DbTransferType::Write,
                                      FpuDbIntValType::FreeBetaRetries,
                                      serial_number, bretries);
}

//------------------------------------------------------------------------------
bool FPUAdmin::printHealthLog(ProtectionDbTxnPtr &txn, const char *serial_number)
{
    // Prints an FPU's health log from the health log database to std::cout.
    // Output format details:
    //   - The index number is the count of finished datum searches
    //   - Each row also contains the UNIX time stamp which can be used to plot
    //     against time, or to identify events in the driver logs.

    // TODO: Health log isn't implemented yet

    std::cout << "**ERROR**: printHealthLog() command is not implemented yet.\n"
              << std::endl;
    return false;
}

//------------------------------------------------------------------------------

} // namespace mpifps




