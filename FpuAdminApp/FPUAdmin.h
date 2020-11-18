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

namespace mpifps
{

//==============================================================================

class FPUAdmin
{
public:
    static void printHelp();
    static E_EtherCANErrCode flash(int fpu_id, const char *serial_number,
                                   bool reuse_snum);
    static E_EtherCANErrCode init(const char *serial_number, 
                                  double apos_min, double apos_max,
                                  double bpos_min, double bpos_max,
                                  bool reinitialize, double adatum_offset);
    static E_EtherCANErrCode listAll();
    static E_EtherCANErrCode listOne(const char *serial_number);
    static E_EtherCANErrCode setALimits(const char *serial_number, 
                                        double alimit_min, double alimit_max,
                                        double adatum_offset);
    static E_EtherCANErrCode setBLimits(const char *serial_number, 
                                        double blimit_min, double blimit_max);
    static E_EtherCANErrCode setARetries(const char *serial_number,
                                         int aretries);
    static E_EtherCANErrCode setBRetries(const char *serial_number,
                                         int bretries);
    static E_EtherCANErrCode printHealthLog(const char *serial_number);
};

//==============================================================================

} // namespace mpifps

#endif // FPUADMIN_H
