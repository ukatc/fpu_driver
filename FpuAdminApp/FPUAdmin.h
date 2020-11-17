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
    FPUAdmin() {}

    void dummyTest(void);    // TODO: For testing only - remove once done

    // TODO: Make the following functions static? OR have this FPUAdmin class
    // open an FPU database when it's created?

    void printHelp();

    // TODO: flashFPU() should be a private function?
    E_EtherCANErrCode flashFPU(int fpu_id, const char *serial_number,
                               bool reuse_snum);

    E_EtherCANErrCode init(const char *serial_number, 
                           double apos_min, double apos_max,
                           double bpos_min, double bpos_max,
                           bool reinitialize, double adatum_offset);
    E_EtherCANErrCode listAll();
    E_EtherCANErrCode listOne(const char *serial_number);
    E_EtherCANErrCode setALimits(const char *serial_number, 
                                 double alimit_min, double alimit_max,
                                 double adatum_offset);
    E_EtherCANErrCode setBLimits(const char *serial_number, 
                                 double blimit_min, double blimit_max);
    E_EtherCANErrCode setARetries(const char *serial_number, int aretries);
    E_EtherCANErrCode setBRetries(const char *serial_number, int bretries);
    E_EtherCANErrCode printHealthLog(const char *serial_number);

private:
    int dummy = 0;
};

//==============================================================================

} // namespace mpifps

#endif // FPUADMIN_H
