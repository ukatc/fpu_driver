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
// command-line app. N.B. Some of these functions produce std::cout output.
//
////////////////////////////////////////////////////////////////////////////////

#ifndef FPUADMIN_H
#define FPUADMIN_H

#include "InterfaceState.h"
#include "T_GatewayAddress.h"
#include "ProtectionDB.h"

namespace mpifps
{

//==============================================================================

class FPUAdmin
{
public:
    // N.B. The following functions use REFERENCES to ProtectionDbTxnPtr, so
    // the unique_ptr itself doesn't have its ownership transferred
    static bool flash(ProtectionDbTxnPtr &txn, int fpu_id,
                      const char *new_serial_number, bool mockup,
                      bool reuse_snum, t_gateway_address gateway_address);
    static bool init(ProtectionDbTxnPtr &txn, const char *serial_number, 
                     double apos_min, double apos_max,
                     double bpos_min, double bpos_max,
                     bool reinitialize, double adatum_offset);
    static bool listAll(ProtectionDbTxnPtr &txn);
    static bool listOne(ProtectionDbTxnPtr &txn, const char *serial_number);
    static bool setALimits(ProtectionDbTxnPtr &txn, const char *serial_number, 
                           double alimit_min, double alimit_max,
                           double adatum_offset);
    static bool setBLimits(ProtectionDbTxnPtr &txn, const char *serial_number, 
                           double blimit_min, double blimit_max);
    static bool setARetries(ProtectionDbTxnPtr &txn, const char *serial_number,
                            int64_t aretries);
    static bool setBRetries(ProtectionDbTxnPtr &txn, const char *serial_number,
                            int64_t bretries);
    static bool printHealthLog(ProtectionDbTxnPtr &txn,
                               const char *serial_number);

private:
    static void printFpuDbData(FpuDbData &fpu_db_data);
};

//==============================================================================

} // namespace mpifps

#endif // FPUADMIN_H
