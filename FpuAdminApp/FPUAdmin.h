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
// FPU database administration functions.
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
    // N.B. The following functions use REFERENCES to ProtectionDbTxnPtr,
    // so the unique_ptr<> itself doesn't have its ownership transferred
    static E_EtherCANErrCode flash(ProtectionDbTxnPtr &txn, int fpu_id,
                                   const char *new_serial_number,
                                   bool mockup, bool reuse_snum,
                                   t_gateway_address gateway_address);
    static E_EtherCANErrCode init(ProtectionDbTxnPtr &txn,
                                  const char *serial_number, 
                                  double apos_min, double apos_max,
                                  double bpos_min, double bpos_max,
                                  bool reinitialize, double adatum_offset);
    static E_EtherCANErrCode listAll(ProtectionDbTxnPtr &txn);
    static E_EtherCANErrCode listOne(ProtectionDbTxnPtr &txn,
                                     const char *serial_number);
    static E_EtherCANErrCode setALimits(ProtectionDbTxnPtr &txn,
                                        const char *serial_number, 
                                        double alimit_min, double alimit_max,
                                        double adatum_offset);
    static E_EtherCANErrCode setBLimits(ProtectionDbTxnPtr &txn,
                                        const char *serial_number, 
                                        double blimit_min, double blimit_max);
    static E_EtherCANErrCode setARetries(ProtectionDbTxnPtr &txn,
                                         const char *serial_number,
                                         int aretries);
    static E_EtherCANErrCode setBRetries(ProtectionDbTxnPtr &txn,
                                         const char *serial_number,
                                         int bretries);
    static E_EtherCANErrCode printHealthLog(ProtectionDbTxnPtr &txn,
                                            const char *serial_number);

private:
    static void printFpuDbData(FpuDbData &fpu_db_data);
};

//==============================================================================

} // namespace mpifps

#endif // FPUADMIN_H
