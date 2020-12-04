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
#include <string.h>
#include "FPUAdmin.h"
#include "UnprotectedGridDriver.h"
#include "T_GridState.h"
#include "ProtectionDB.h"
#include "FPUConstants.h"

namespace mpifps
{

static const char *db_write_failed_str = "Error: FPU database write failed.";
static const char *fpu_snum_not_in_database_str =
                    "Error: FPU serial number is not yet defined in database.";

//------------------------------------------------------------------------------
AppReturnVal FPUAdmin::flash(ProtectionDbTxnPtr &txn, int fpu_id,
                             const char *new_serial_number, bool mockup,
                             bool reuse_snum,
                             const t_gateway_address *gateway_address_ptr)
{
    // Flashes serial number to FPU with ID of fpu_id. FPU must be connected.
    // If reuse_snum is true then a previously-defined serial number can be used.
    // If gateway_address_ptr is not nullptr then it uses that gateway
    // address, otherwise it uses the mockup flag to determine it.

    //..........................................................................
    // Check arguments
    bool args_are_ok = true;

    const int max_snum_len = ethercanif::DIGITS_SERIAL_NUMBER;
    if ((strlen(new_serial_number) == 0) ||
        (strlen(new_serial_number) > max_snum_len))
    {
        std::cout << "Error: Serial number length must be between 1 and " <<
                      std::to_string(max_snum_len) << "." << std::endl;
        args_are_ok = false;
    }

    if ((fpu_id < 0) || (fpu_id > (MAX_NUM_POSITIONERS - 1)))
    {
        std::cout << "Error: fpu_id must be in the range 0 to " <<
                     std::to_string((int)(MAX_NUM_POSITIONERS - 1)) <<
                     "." << std::endl;
        args_are_ok = false;
    }

    if (!args_are_ok)
    {
        return AppReturnError;
    }

    //..........................................................................
    // Check if serial number is already in use
    int64_t snum_flag_used_val = 0;
    bool read_was_ok = txn->fpuDbTransferInt64Val(DbTransferType::Read,
                                                  FpuDbIntValType::SnumUsedFlag,
                                                  new_serial_number,
                                                  snum_flag_used_val);
    if ((read_was_ok) && (snum_flag_used_val == SNUM_USED_CHECK_VAL))
    {
        // Serial number is already in use
        if (!reuse_snum)
        {
            std::cout << "Flash command rejected: Serial number is already in use.\n"
                         "Call with '--reuse_sn' to use it again." << std::endl;
            return AppReturnError;
        }
    }
    else
    {
        // Serial number is not in use yet - add it to database
        snum_flag_used_val = SNUM_USED_CHECK_VAL;
        if (!txn->fpuDbTransferInt64Val(DbTransferType::Write,
                                        FpuDbIntValType::SnumUsedFlag,
                                        new_serial_number,
                                        snum_flag_used_val))
        {
            std::cout << db_write_failed_str << std::endl;
            return AppReturnError;
        }
    }

    //..........................................................................
    // Connect to grid and write serial number to FPU

    // Create gateway address list
    t_gateway_address gateway_addresses[MAX_NUM_GATEWAYS];
    int num_gateways;
    if (gateway_address_ptr != nullptr)
    {
        gateway_addresses[0] = *gateway_address_ptr;
        num_gateways = 1;
    }
    else
    {
        if (mockup)
        {
            for (int i = 0; i < MAX_NUM_GATEWAYS; i++)
            {
                gateway_addresses[i] = { "127.0.0.1", (uint16_t)(4700 + i) };
            }
            num_gateways = MAX_NUM_GATEWAYS;
        }
        else
        {
            // ************ TODO: Get GATEWAY0_ADDRESS from Linux environment
            // variable of the same name - see the Python definition of
            // GATEWAY0_ADDRESS
            const char *dummy_gateway0_address = "192.168.0.10";
            //gateway_addresses[0] = { GATEWAY0_ADDRESS, 4700 };
            gateway_addresses[0] = { dummy_gateway0_address, 4700 };
            num_gateways = 1;
        }
    }

    // Connect to grid
    UnprotectedGridDriver ugd(fpu_id + 1);
    ugd.initialize();

    std::cout << "Connecting to grid..." << std::endl;
    E_EtherCANErrCode ecan_result = ugd.connect(num_gateways,
                                                gateway_addresses);
    t_grid_state grid_state;
    t_fpuset fpuset;

    // TODO: Currently pings and reads the serial numbers for all FPUs up to
    // fpu_id (which is the equivalent functionality to that in the original
    // Python flash_FPU() function in fpu-admin), but might only need to do
    // this for the single FPU?
    UnprotectedGridDriver::createFpuSetForNumFpus(fpu_id + 1, fpuset);

    if (ecan_result == DE_OK)
    {
        std::cout << "Pinging FPUs..." << std::endl;
        ugd.getGridState(grid_state);
        ecan_result = ugd.pingFPUs(grid_state, fpuset);
    }

    if (ecan_result == DE_OK)
    {
        std::cout << "Reading serial numbers..." << std::endl;
        ecan_result = ugd.readSerialNumbers(grid_state, fpuset);
    }

    // Write serial number to FPU
    if (ecan_result == DE_OK)
    {
        std::cout << "Flashing FPU " << std::to_string(fpu_id) <<
                     " with serial number " << new_serial_number << std::endl;
        ecan_result = ugd.writeSerialNumber(fpu_id, new_serial_number,
                                            grid_state);
    }

    if (ecan_result == DE_OK)
    {
        return AppReturnOk;
    }
    else
    {
        std::cout << "Error: Operation failed unexpectedly - error code = " <<
                     std::to_string(ecan_result) << "." << std::endl;
        return AppReturnError;
    }

    //..........................................................................
}

//------------------------------------------------------------------------------
AppReturnVal FPUAdmin::init(ProtectionDbTxnPtr &txn, const char *serial_number, 
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

        // TODO: Need to distinguish between serial number not being found vs
        // the database read failing at a lower level - if counters entry for
        // serial number doesn't yet exist then NOT a failure - the counters
        // just need to be initialised as empty
        // TODO: If the following fails due to a database read error then 
        // display error message and return AppReturnError
        if (!txn->fpuDbTransferCounters(DbTransferType::Read, serial_number,
                                        fpu_db_data.counters))
        {
            fpu_db_data.counters.zeroAll();
        }
    }
    fpu_db_data.last_waveform.clear();

    if (txn->fpuDbTransferFpu(DbTransferType::Write, serial_number, fpu_db_data))
    {
        return AppReturnOk;
    }
    else
    {
        std::cout << db_write_failed_str << std::endl;
        return AppReturnError;
    }
}

//------------------------------------------------------------------------------
AppReturnVal FPUAdmin::listAll(ProtectionDbTxnPtr &txn)
{
    // Prints data for all FPUs in database using std::cout

    std::vector<std::string> serial_numbers;
    int num_fpus_with_good_data = 0;
    if (txn->fpuDbGetAllSerialNumbers(serial_numbers))
    {
        for (size_t i = 0; i < serial_numbers.size(); i++)
        {
            if (printSingleFpu(txn, serial_numbers[i].c_str()))
            {
                num_fpus_with_good_data++;
            }
            std::cout << std::endl;
        }
    }
    else
    {
        std::cout << "Error: Unexpected failure while collating serial numbers "
                     "from FPU database." << std::endl;
        return AppReturnError;
    }
    
    std::cout << "*** SUMMARY ***: " << std::to_string(serial_numbers.size()) <<
                 " unique serial numbers were found in the FPU database,\n"
                 "of which " << std::to_string(num_fpus_with_good_data) <<
                 " have all of their FPU data items correctly present.\n" << std::endl;
    return AppReturnOk;
}

//------------------------------------------------------------------------------
AppReturnVal FPUAdmin::listOne(ProtectionDbTxnPtr &txn,
                               const char *serial_number)
{
    // Prints serial number and data for one FPU using std::cout. Returns an
    // application return value.

    if (printSingleFpu(txn, serial_number))
    {
        return AppReturnOk;
    }
    else
    {
        // N.B. Error message will have been generated during printSingleFpu()
        // call above
        return AppReturnError;
    }
}

//------------------------------------------------------------------------------
bool FPUAdmin::printSingleFpu(ProtectionDbTxnPtr &txn,
                              const char *serial_number)
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
        std::cout << "Error: One or more of this FPU's data items "
                     "are missing from the database." << std::endl;
        return false;
    }
}

//------------------------------------------------------------------------------
void FPUAdmin::printFpuDbData(FpuDbData &fpu_db_data)
{
    // Prints useful data for one FPU using std::cout

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
       ", b_retries_acw = " << std::to_string(fpu_db_data.bretries_acw);

    // TODO: Also display fpu_db_data.counters and fpu_db_data.last_waveform?

    std::cout << std::endl;
}

//------------------------------------------------------------------------------
AppReturnVal FPUAdmin::setALimits(ProtectionDbTxnPtr &txn,
                                  const char *serial_number, 
                                  double alimit_min, double alimit_max,
                                  double adatum_offset)
{
    // Sets safe limits for alpha arm of an FPU.

    if (isSerialNumberUsed(txn, serial_number))
    {
        Interval alimits_interval(alimit_min, alimit_max);
        if (txn->fpuDbTransferPosition(DbTransferType::Write,
                                       FpuDbPositionType::AlphaLimits,
                                       serial_number, alimits_interval,
                                       adatum_offset))
        {
            return AppReturnOk;
        }
        else
        {
            std::cout << db_write_failed_str << std::endl;
            return AppReturnError;
        }
    }
    else
    {
        std::cout << fpu_snum_not_in_database_str << std::endl;
        return AppReturnError;
    }
}

//------------------------------------------------------------------------------
AppReturnVal FPUAdmin::setBLimits(ProtectionDbTxnPtr &txn,
                                  const char *serial_number, 
                                  double blimit_min, double blimit_max)
{
    // Sets safe limits for beta arm of an FPU.

    if (isSerialNumberUsed(txn, serial_number))
    {
        Interval blimits_interval(blimit_min, blimit_max);
        double datum_offset = 0.0;
        if (txn->fpuDbTransferPosition(DbTransferType::Write,
                                       FpuDbPositionType::BetaLimits,
                                       serial_number, blimits_interval,
                                       datum_offset)) //************* TODO: Put a beta datum offset constant here?
        {
            return AppReturnOk;
        }
        else
        {
            std::cout << db_write_failed_str << std::endl;
            return AppReturnError;
        }
    }
    else
    {
        std::cout << fpu_snum_not_in_database_str << std::endl;
        return AppReturnError;
    }
}

//------------------------------------------------------------------------------
AppReturnVal FPUAdmin::setARetries(ProtectionDbTxnPtr &txn,
                                   const char *serial_number,
                                   int64_t aretries)
{
    // Sets allowed number of freeAlphaLimitBreach commands in the same
    // direction before the software protection kicks in. N.B. The retry count
    // is reset to zero upon a successfully finished datum search.

    if (isSerialNumberUsed(txn, serial_number))
    {
        if (txn->fpuDbTransferInt64Val(DbTransferType::Write,
                                       FpuDbIntValType::FreeAlphaRetries,
                                       serial_number, aretries))
        {
            return AppReturnOk;
        }
        else
        {
            std::cout << db_write_failed_str << std::endl;
            return AppReturnError;
        }
    }
    else
    {
        std::cout << fpu_snum_not_in_database_str << std::endl;
        return AppReturnError;
    }
}

//------------------------------------------------------------------------------
AppReturnVal FPUAdmin::setBRetries(ProtectionDbTxnPtr &txn,
                                   const char *serial_number,
                                   int64_t bretries)
{
    // Sets allowed number of freeBetaCollision commands in the same direction
    // before the software protection kicks in. N.B. The retry count is reset
    // to zero upon a successfully finished datum search.

    if (isSerialNumberUsed(txn, serial_number))
    {
        if (txn->fpuDbTransferInt64Val(DbTransferType::Write,
                                       FpuDbIntValType::FreeBetaRetries,
                                       serial_number, bretries))
        {
            return AppReturnOk;
        }
        else
        {
            std::cout << db_write_failed_str << std::endl;
            return AppReturnError;
        }
    }
    else
    {
        std::cout << fpu_snum_not_in_database_str << std::endl;
        return AppReturnError;
    }
}

//------------------------------------------------------------------------------
bool FPUAdmin::isSerialNumberUsed(ProtectionDbTxnPtr &txn,
                                  const char *serial_number)
{
    // TODO: Need to differentiate between serial number not in database,
    // vs database read failure (once implement more detailed return codes)
    int64_t snum_used_flag = 0;
    bool result_ok = txn->fpuDbTransferInt64Val(DbTransferType::Read, 
                                                FpuDbIntValType::SnumUsedFlag,
                                                serial_number, snum_used_flag);
    if (result_ok)
    {
        if (snum_used_flag == SNUM_USED_CHECK_VAL)
        {
            return true;
        }
    }
    return false;
}

//------------------------------------------------------------------------------
AppReturnVal FPUAdmin::printHealthLog(ProtectionDbTxnPtr &txn,
                                      const char *serial_number)
{
    // Prints an FPU's health log from the health log database to std::cout.
    // Output format details:
    //   - The index number is the count of finished datum searches
    //   - Each row also contains the UNIX time stamp which can be used to plot
    //     against time, or to identify events in the driver logs.

    // TODO: Health log isn't implemented yet

    std::cout << "Error: printHealthLog() command is not implemented yet." << std::endl;
    return AppReturnError;
}

//------------------------------------------------------------------------------

} // namespace mpifps




