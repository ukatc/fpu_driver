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
// NAME FPUAdmin.h
//
// FPU database administration functions for the FPU database administration
// command-line app. They are intended to be executed from a command-line app:
//   - Some of these functions produce std::cout output
//   - The functions return AppReturnVal values, which can then be directly
//     returned from the app's main() function
//
////////////////////////////////////////////////////////////////////////////////

#ifndef FPUADMIN_H
#define FPUADMIN_H

#include "InterfaceState.h"
#include "T_GatewayAddress.h"
#include "ProtectionDB.h"

namespace mpifps
{

// AppReturnVal enum: Defines application return values, i.e. values returned
// from main()
typedef enum
{
    AppReturnOk = 0,
    AppReturnError = 1
} AppReturnVal;

//==============================================================================

class FPUAdmin
{
public:
    // N.B. The following functions use REFERENCES to ProtectionDbTxnPtr, so
    // the unique_ptr itself doesn't have its ownership transferred
    static AppReturnVal flash(ProtectionDbTxnPtr &txn, int fpu_id,
                              const char *new_serial_number, bool mockup,
                              bool reuse_snum, 
                              const t_gateway_address *gateway_address_ptr);
    static AppReturnVal init(ProtectionDbTxnPtr &txn, const char *serial_number, 
                             double apos_min, double apos_max,
                             double bpos_min, double bpos_max,
                             bool reinitialize, double adatum_offset);
    static AppReturnVal listAll(ProtectionDbTxnPtr &txn);
    static AppReturnVal listOne(ProtectionDbTxnPtr &txn,
                                const char *serial_number);
    static AppReturnVal setALimits(ProtectionDbTxnPtr &txn,
                                   const char *serial_number, 
                                   double alimit_min, double alimit_max,
                                   double adatum_offset);
    static AppReturnVal setBLimits(ProtectionDbTxnPtr &txn,
                                   const char *serial_number, 
                                   double blimit_min, double blimit_max);
    static AppReturnVal setARetries(ProtectionDbTxnPtr &txn,
                                    const char *serial_number,
                                    int64_t aretries);
    static AppReturnVal setBRetries(ProtectionDbTxnPtr &txn,
                                    const char *serial_number,
                                    int64_t bretries);
    static AppReturnVal printHealthLog(ProtectionDbTxnPtr &txn,
                                       const char *serial_number);
    static void printUnexpectedDbResult(MdbResult mdb_result);

private:
    static bool printSingleFpu(ProtectionDbTxnPtr &txn, 
                               const char *serial_number);
    static void printFpuDbData(FpuDbData &fpu_db_data);
    static MdbResult checkIfSerialNumberUsed(ProtectionDbTxnPtr &txn,
                                             const char *serial_number);
    static bool checkAndMessageBeforeSetting(ProtectionDbTxnPtr &txn,
                                             const char *serial_number);
    static bool checkAndMessageForSerialNumberLength(const char *serial_number);
};

//==============================================================================

} // namespace mpifps

#endif // FPUADMIN_H
