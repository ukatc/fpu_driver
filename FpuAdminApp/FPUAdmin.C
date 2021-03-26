// -*- mode: c++ -*-

////////////////////////////////////////////////////////////////////////////////
//
// Copyright 2020 UKRI. See file "LICENSE" for license information.
//
//
// Who       When        What
// --------  ----------  -------------------------------------------------------
// bwillemse 2020-11-16  Created (adapted from Python fpu-admin script).
// bwillemse 2021-03-26  Modified for new non-contiguous FPU IDs and CAN mapping.
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
#include "FPUCommands.h"
#include "ErrorCodes.h"

namespace mpifps
{

static const char *fpu_snum_not_in_database_str =
                    "Error: FPU serial number is not yet defined in database.";

ProtectionDB FPUAdmin::protectiondb;
ProtectionDbTxnPtr FPUAdmin::txn;

//------------------------------------------------------------------------------
AppReturnVal FPUAdmin::createEmptyDb(const std::string &dir_str)
{
    if (dir_str.size() >= 1)
    {
        if (dir_str.back() == '/')
        {
            std::cout << "Error: Do not provide trailing /.\n" << std::endl;
            return AppReturnError;
        }

        MdbResult mdb_result = protectiondb.createEmpty(dir_str);
        if (mdb_result == MDB_SUCCESS)
        {
            std::cout << "Success - created empty grid driver database in " <<
                            dir_str << ".\n" << std::endl;
            return AppReturnOk;
        }
        else
        {
            std::cout << "Error: Command failed with the following result:\n";
            std::cout << ProtectionDB::getResultString(mdb_result) << std::endl;
            return AppReturnError;
        }
    }
    else
    {
        std::cout << "Error: Directory string is zero-length.\n" << std::endl;
        return AppReturnError;
    }
}

//------------------------------------------------------------------------------
AppReturnVal FPUAdmin::flash(bool mockup, int fpu_id,
                             const char *new_serial_number,
                             bool reuse_snum,
                             const t_gateway_address *gateway_address_ptr)
{
    // Flashes serial number to FPU with ID of fpu_id. FPU must be connected.
    // If reuse_snum is true then a previously-defined serial number can be used.
    // If gateway_address_ptr is not nullptr then it uses that gateway
    // address, otherwise it uses the mockup flag to determine it.

    //..........................................................................
    if (!openDbAndCreateTxnWithMessages(mockup))
    {
        return AppReturnError;
    }

    //..........................................................................
    // Check arguments
    bool args_are_ok = true;

    if (!checkAndMessageForSerialNumberLength(new_serial_number))
    {
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
    MdbResult mdb_result = txn->fpuDbTransferInt64Val(DbTransferType::Read,
                                                      FpuDbIntValType::SnumUsedFlag,
                                                      new_serial_number,
                                                      snum_flag_used_val);
    if (mdb_result == MDB_SUCCESS)
    {
        if (snum_flag_used_val == SNUM_USED_CHECK_VAL)
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
            std::cout << "Error: Serial number is already in use, AND its in-use flag value is incorrect" << std::endl;
            return AppReturnError;
        }
    }
    else if (mdb_result == MDB_NOTFOUND)
    {
        // Serial number is not in use yet - add it to database
        snum_flag_used_val = SNUM_USED_CHECK_VAL;
        mdb_result = txn->fpuDbTransferInt64Val(DbTransferType::Write,
                                        FpuDbIntValType::SnumUsedFlag,
                                        new_serial_number,
                                        snum_flag_used_val);
        if (mdb_result != MDB_SUCCESS) 
        {
            printUnexpectedDbResult(mdb_result);
            return AppReturnError;
        }
    }
    else
    {
        printUnexpectedDbResult(mdb_result);
        return AppReturnError;
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
#ifdef FLEXIBLE_CAN_MAPPING
    //*********************************
    //*********************************
    // TODO: Dummy file path string for now - need to get it working with a
    // proper path
    //*********************************
    //*********************************
    const std::string csv_file_path("dummy_csv_file_path");
    ugd.initialize(csv_file_path);
#else // NOT FLEXIBLE_CAN_MAPPING
    ugd.initialize();
#endif // NOT FLEXIBLE_CAN_MAPPING

    std::cout << "Connecting to grid..." << std::endl;
    E_EtherCANErrCode ecan_result = ugd.connect(num_gateways,
                                                gateway_addresses);
    t_grid_state grid_state;
    t_fpuset fpuset;

#ifdef FLEXIBLE_CAN_MAPPING
    //*********************************
    //*********************************
    // TODO: Will this work OK? It should do, once the non-contiguous FPU ID / CAN
    // mapping functionality works OK?
    clearFpuSet(fpuset);
    fpuset[fpu_id] = true;
    //*********************************
    //*********************************
#else // NOT FLEXIBLE_CAN_MAPPING
    // TODO: Currently pings and reads the serial numbers for all FPUs up to
    // fpu_id (which is the equivalent functionality to that in the original
    // Python flash_FPU() function in fpu-admin), but might only need to do
    // this for the single FPU?
    UnprotectedGridDriver::createFpuSetForNumFpus(fpu_id + 1, fpuset);
#endif // NOT FLEXIBLE_CAN_MAPPING

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
        // TODO: Also display a longer "DE_XXX" error code STRING and message -
        // need to first implement a function to return error code strings / 
        // longer messages, given the numerical error codes
        std::cout << "Error: Operation failed unexpectedly - error code = " <<
                     std::to_string(ecan_result) << "." << std::endl;
        return AppReturnError;
    }

    //..........................................................................
}

//------------------------------------------------------------------------------
AppReturnVal FPUAdmin::init(bool use_mockup_db, const char *serial_number,
                            double apos_min, double apos_max,
                            double bpos_min, double bpos_max,
                            bool reinitialize, double adatum_offset)
{
    // Initialises the FPU in the protection database. The initial alpha and
    // beta arm min and max positions are in degrees. If reinitialize is true,
    // it is allowed to redefine FPU positions which already have been stored
    // before.

    if (!openDbAndCreateTxnWithMessages(use_mockup_db))
    {
        return AppReturnError;
    }

    if (!checkAndMessageForSerialNumberLength(serial_number))
    {
        return AppReturnError;
    }

    FpuDbData fpu_db_data;

    fpu_db_data.snum_used_flag = SNUM_USED_CHECK_VAL;

    fpu_db_data.apos = Interval(apos_min, apos_max);
    fpu_db_data.datum_offsets[(int)FpuDbIntervalType::AlphaPos] = adatum_offset;

    fpu_db_data.bpos = Interval(bpos_min, bpos_max);
    fpu_db_data.datum_offsets[(int)FpuDbIntervalType::BetaPos] = BETA_DATUM_OFFSET;

    fpu_db_data.alimits = Interval(ALPHA_MIN_DEGREE, ALPHA_MAX_DEGREE);
    fpu_db_data.datum_offsets[(int)FpuDbIntervalType::AlphaLimits] = adatum_offset;

    fpu_db_data.blimits = Interval(BETA_MIN_DEGREE, BETA_MAX_DEGREE);
    fpu_db_data.datum_offsets[(int)FpuDbIntervalType::BetaLimits] = BETA_DATUM_OFFSET;

    fpu_db_data.wf_reversed = false;

    fpu_db_data.maxaretries = DEFAULT_FREE_ALPHA_RETRIES;
    fpu_db_data.aretries_cw = 0;
    fpu_db_data.aretries_acw = 0;
    fpu_db_data.maxbretries = DEFAULT_FREE_BETA_RETRIES;
    fpu_db_data.bretries_cw = 0;
    fpu_db_data.bretries_acw = 0;

    MdbResult mdb_result = MDB_PANIC;

    if (!reinitialize)
    {
        // If the FPU has an existing counters entry in the database then
        // retain this
        mdb_result = txn->fpuDbTransferCounters(DbTransferType::Read,
                                                serial_number,
                                                fpu_db_data.counters);
        if (mdb_result == MDB_NOTFOUND)
        {
            fpu_db_data.counters.zeroAll();
        }
        else if (mdb_result != MDB_SUCCESS)
        {
            printUnexpectedDbResult(mdb_result);
            return AppReturnError;
        }
    }

    fpu_db_data.last_waveform.clear();

    mdb_result = txn->fpuDbTransferFpu(DbTransferType::Write, serial_number,
                                       fpu_db_data);
    if (mdb_result == MDB_SUCCESS)
    {
        return AppReturnOk;
    }
    else
    {
        printUnexpectedDbResult(mdb_result);
        return AppReturnError;
    }
}

//------------------------------------------------------------------------------
AppReturnVal FPUAdmin::listAll(bool use_mockup_db)
{
    // Prints data for all FPUs in database using std::cout

    if (!openDbAndCreateTxnWithMessages(use_mockup_db))
    {
        return AppReturnError;
    }

    std::vector<std::string> serial_numbers;
    int num_fpus_with_good_data = 0;
    MdbResult mdb_result = txn->fpuDbGetAllSerialNumbers(serial_numbers);
    if (mdb_result == MDB_SUCCESS)
    {
        for (size_t i = 0; i < serial_numbers.size(); i++)
        {
            std::cout << "----------------------------------------" <<
                         "------------------------------\n";
            if (printSingleFpu(serial_numbers[i].c_str()))
            {
                num_fpus_with_good_data++;
            }
        }
        std::cout << "----------------------------------------" <<
                     "------------------------------\n" << std::endl;
    }
    else
    {
        std::cout << "Error: Unexpected failure while collating serial numbers "
                     "from FPU database:" << std::endl;
        printUnexpectedDbResult(mdb_result);
        return AppReturnError;
    }
    
    std::cout << "*** SUMMARY ***: " << std::to_string(serial_numbers.size()) <<
                 " unique serial numbers were found in the FPU database,\n"
                 "of which " << std::to_string(num_fpus_with_good_data) <<
                 " have all of their FPU data items correctly present.\n" << std::endl;
    return AppReturnOk;
}

//------------------------------------------------------------------------------
AppReturnVal FPUAdmin::listOne(bool use_mockup_db, const char *serial_number)
{
    // Prints serial number and data for one FPU using std::cout. Returns an
    // application return value.

    if (!openDbAndCreateTxnWithMessages(use_mockup_db))
    {
        return AppReturnError;
    }

    if (printSingleFpu(serial_number))
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
bool FPUAdmin::printSingleFpu(const char *serial_number)
{
    // Prints serial number and data for one FPU using std::cout

    std::string snum_err_str;
    if (strlen(serial_number) > ethercanif::DIGITS_SERIAL_NUMBER)
    {
        snum_err_str = " <<<<< ERROR: Serial number longer than max " +
                       std::to_string(ethercanif::DIGITS_SERIAL_NUMBER);
    }
    
    std::cout << "FPU serial number: " << serial_number << snum_err_str << "\n";
    FpuDbData fpu_db_data;
    // NOTE: Using DbTransferType::ReadRaw rather than just DbTransferType::Read
    // here, because the latter will subtract the alpha/beta datum offsets from
    // their corresponding intervals, but we just want the raw interval and
    // offset values
    // TODO: Is this correct? This is what the fpu-admin Python version's 
    // "list1" command seems to do - it calls getRawField() for all fields
    MdbResult mdb_result = txn->fpuDbTransferFpu(DbTransferType::ReadRaw,
                                                 serial_number, fpu_db_data);
    if (mdb_result == MDB_SUCCESS)  
    {
        printFpuDbData(fpu_db_data);
        return true;
    }
    else if (mdb_result == MDB_NOTFOUND)
    {
        std::cout << "Error: One or more of this FPU's data items "
                     "are missing from the database." << std::endl;
        return false;
    }
    else
    {
        printUnexpectedDbResult(mdb_result);
        return false;
    }
}

//------------------------------------------------------------------------------
void FPUAdmin::printFpuDbData(FpuDbData &fpu_db_data)
{
    // Prints useful data for one FPU using std::cout.

    // apos / bpos intervals + offsets
    std::cout << "apos = [" << fpu_db_data.apos.toString() << ", " <<
        doubleToString(fpu_db_data.datum_offsets[(int)FpuDbIntervalType::AlphaPos]) << "]\n";
    std::cout << "bpos = [" << fpu_db_data.bpos.toString() << ", " <<
        doubleToString(fpu_db_data.datum_offsets[(int)FpuDbIntervalType::BetaPos]) << "]\n";

    // alimits / blimits intervals + offsets
    std::cout << "alimits = [" << fpu_db_data.alimits.toString() << ", " <<
        doubleToString(fpu_db_data.datum_offsets[(int)FpuDbIntervalType::AlphaLimits]) << "]\n";
    std::cout << "blimits = [" << fpu_db_data.blimits.toString() << ", " <<
        doubleToString(fpu_db_data.datum_offsets[(int)FpuDbIntervalType::BetaLimits]) << "]\n";

    // wf_reversed flag
    const char *wf_reversed_str = "";
    if (fpu_db_data.wf_reversed)
    {
        wf_reversed_str = "true";
    }
    else
    {
        wf_reversed_str = "false";
    }
    std::cout << "wf_reversed = " << wf_reversed_str << "\n";

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
AppReturnVal FPUAdmin::setALimits(bool use_mockup_db, const char *serial_number,
                                  double alimit_min, double alimit_max,
                                  double adatum_offset)
{
    // Sets safe limits for alpha arm of an FPU.

    if (!openDbAndCreateTxnWithMessages(use_mockup_db))
    {
        return AppReturnError;
    }

    if (checkAndMessageBeforeSetting(serial_number))
    {
        Interval alimits_interval(alimit_min, alimit_max);
        MdbResult mdb_result = txn->fpuDbTransferInterval(DbTransferType::Write,
                                                FpuDbIntervalType::AlphaLimits,
                                                serial_number, alimits_interval,
                                                adatum_offset);
        if (mdb_result == MDB_SUCCESS)
        {
            return AppReturnOk;
        }
        else
        {
            printUnexpectedDbResult(mdb_result);
            return AppReturnError;
        }
    }
    else
    {
        return AppReturnError;
    }
}

//------------------------------------------------------------------------------
AppReturnVal FPUAdmin::setBLimits(bool use_mockup_db, const char *serial_number,
                                  double blimit_min, double blimit_max)
{
    // Sets safe limits for beta arm of an FPU.

    if (!openDbAndCreateTxnWithMessages(use_mockup_db))
    {
        return AppReturnError;
    }

    if (checkAndMessageBeforeSetting(serial_number))
    {
        Interval blimits_interval(blimit_min, blimit_max);
        double beta_datum_offset = BETA_DATUM_OFFSET;
        MdbResult mdb_result = txn->fpuDbTransferInterval(DbTransferType::Write,
                                                FpuDbIntervalType::BetaLimits,
                                                serial_number, blimits_interval,
                                                beta_datum_offset);
        if (mdb_result == MDB_SUCCESS)    
        {
            return AppReturnOk;
        }
        else
        {
            printUnexpectedDbResult(mdb_result);
            return AppReturnError;
        }
    }
    else
    {
        return AppReturnError;
    }
}

//------------------------------------------------------------------------------
AppReturnVal FPUAdmin::setARetries(bool use_mockup_db, const char *serial_number,
                                   int64_t aretries)
{
    // Sets allowed number of freeAlphaLimitBreach commands in the same
    // direction before the software protection kicks in. N.B. The retry count
    // is reset to zero upon a successfully finished datum search.

    if (!openDbAndCreateTxnWithMessages(use_mockup_db))
    {
        return AppReturnError;
    }

    if (checkAndMessageBeforeSetting(serial_number))
    {
        MdbResult mdb_result = txn->fpuDbTransferInt64Val(DbTransferType::Write,
                                            FpuDbIntValType::FreeAlphaRetries,
                                            serial_number, aretries);
        if (mdb_result == MDB_SUCCESS)
        {
            return AppReturnOk;
        }
        else
        {
            printUnexpectedDbResult(mdb_result);
            return AppReturnError;
        }
    }
    else
    {
        return AppReturnError;
    }
}

//------------------------------------------------------------------------------
AppReturnVal FPUAdmin::setBRetries(bool use_mockup_db, const char *serial_number,
                                   int64_t bretries)
{
    // Sets allowed number of freeBetaCollision commands in the same direction
    // before the software protection kicks in. N.B. The retry count is reset
    // to zero upon a successfully finished datum search.

    if (!openDbAndCreateTxnWithMessages(use_mockup_db))
    {
        return AppReturnError;
    }

    if (checkAndMessageBeforeSetting(serial_number))
    {
        MdbResult mdb_result = txn->fpuDbTransferInt64Val(DbTransferType::Write,
                                            FpuDbIntValType::FreeBetaRetries,
                                            serial_number, bretries);
        if (mdb_result == MDB_SUCCESS)    
        {
            return AppReturnOk;
        }
        else
        {
            printUnexpectedDbResult(mdb_result);
            return AppReturnError;
        }
    }
    else
    {
        return AppReturnError;
    }
}

//------------------------------------------------------------------------------
bool FPUAdmin::checkAndMessageBeforeSetting(const char *serial_number)
{
    if (!checkAndMessageForSerialNumberLength(serial_number))
    {
        return false;
    }

    MdbResult mdb_result = checkIfSerialNumberUsed(serial_number);
    if (mdb_result == MDB_SUCCESS)
    {
        return true;
    }
    else if (mdb_result == MDB_NOTFOUND)
    {
        std::cout << fpu_snum_not_in_database_str << std::endl;
        return false;
    }
    else
    {
        printUnexpectedDbResult(mdb_result);
        return false;
    }
}

//------------------------------------------------------------------------------
MdbResult FPUAdmin::checkIfSerialNumberUsed(const char *serial_number)
{
    // Checks if a serial number is currently in use in the database. Returns:
    //   - MDB_SUCCESS if found and its serial-number-used flag value is OK
    //   - MDB_NOTFOUND if not found
    //   - MDB_INCORRECT_SNUM_USED_FLAG_VAL if a serial number entry was found,
    //     but its check value was incorrect
    //   - Other codes if unexpected database error
    int64_t snum_used_flag = 0;
    MdbResult mdb_result = txn->fpuDbTransferInt64Val(DbTransferType::Read, 
                                                FpuDbIntValType::SnumUsedFlag,
                                                serial_number, snum_used_flag);
    if (mdb_result == MDB_SUCCESS)
    {
        if (snum_used_flag != SNUM_USED_CHECK_VAL)
        {
            mdb_result = MDB_INCORRECT_SNUM_USED_FLAG_VAL;
        }
    }
    return mdb_result;
}

//------------------------------------------------------------------------------
AppReturnVal FPUAdmin::printHealthLog(bool use_mockup_db,
                                      const char *serial_number)
{
    // Prints an FPU's health log from the health log database to std::cout.
    // Output format details:
    //   - The index number is the count of finished datum searches
    //   - Each row also contains the UNIX time stamp which can be used to plot
    //     against time, or to identify events in the driver logs.

    // TODO: Health log isn't implemented yet

    if (!openDbAndCreateTxnWithMessages(use_mockup_db))
    {
        return AppReturnError;
    }

    std::cout << "Error: printHealthLog() command is not implemented yet." << std::endl;
    return AppReturnError;
}

//------------------------------------------------------------------------------
bool FPUAdmin::openDbAndCreateTxnWithMessages(bool use_mockup_db)
{
    std::string dir_str = ProtectionDB::getDirFromLinuxEnv(use_mockup_db);
    if (dir_str.empty())
    {
        std::string main_dir_env_name;
        std::string mockup_dir_env_name;
        ProtectionDB::getLinuxEnvVariableNames(main_dir_env_name,
                                               mockup_dir_env_name);
        std::cout << "Error: Could not determine directory of protection database - are the\n";
        std::cout << "following Linux environment variables set correctly?:\n";
        std::cout << main_dir_env_name << ", " << mockup_dir_env_name << std::endl;
        return false;
    }

    MdbResult mdb_result = protectiondb.open(dir_str);
    if (mdb_result == MDB_SUCCESS)
    {
        txn = protectiondb.createTransaction(mdb_result);
        if (!txn)
        {
            std::cout << "Error: Could not create a database transaction:" <<
                            std::endl;
            FPUAdmin::printUnexpectedDbResult(mdb_result);
            return false;
        }
    }
    else
    {
        std::cout << "Error: Problem when opening protection database (in " <<
                     dir_str << "):" << std::endl;
        FPUAdmin::printUnexpectedDbResult(mdb_result);
        return false;
    }

    return true;
}

//------------------------------------------------------------------------------
bool FPUAdmin::checkAndMessageForSerialNumberLength(const char *serial_number)
{
    if ((strlen(serial_number) > 0) && 
        (strlen(serial_number) <= ethercanif::DIGITS_SERIAL_NUMBER))
    {
        return true;
    }
    else
    {
        std::cout << "Error: Serial number length must be between 1 and " <<
                      std::to_string(ethercanif::DIGITS_SERIAL_NUMBER) <<
                      "." << std::endl;
        return false;
    }
}

//------------------------------------------------------------------------------
void FPUAdmin::printUnexpectedDbResult(MdbResult mdb_result)
{
    std::cout << "Database error " <<
                 ProtectionDB::getResultString(mdb_result) << std::endl;
}

//------------------------------------------------------------------------------

} // namespace mpifps




