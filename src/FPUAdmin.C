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
#include "FPUAdmin.h"
#include "UnprotectedGridDriver.h"
#include "T_GridState.h"

namespace mpifps
{

//------------------------------------------------------------------------------
void FPUAdmin::dummyTest(void)
{
    std::cout << "Hello\n" << std::endl;
    dummy++;
}

//------------------------------------------------------------------------------
E_EtherCANErrCode FPUAdmin::flashFPU(int fpu_id, const char *serial_number,
                                     bool reuse_snum)
{
    // Flashes serial number to FPU with ID <fpu_id>. FPU must be connected.
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

    return ecan_result;
#else
    return DE_OK;
#endif
}

//------------------------------------------------------------------------------
E_EtherCANErrCode FPUAdmin::initAsPositions(const char *serial_number,
                                            double apos, double bpos,
                                            bool reinitialize,
                                            double adatum_offset)
{
    // Initializes the FPU in the protection database, passing the initial alpha
    // and beta arm positions in degrees. The optional last parameter is the
    // alpha datum offset.
    // If the reinitialize is true, it is allowed to redefine FPU positions
    // which already have been stored before.

}

//------------------------------------------------------------------------------
E_EtherCANErrCode FPUAdmin::initAsIntervals(const char *serial_number, 
                                            double apos_min, double apos_max,
                                            double bpos_min, double bpos_max,
                                            bool reinitialize,
                                            double adatum_offset)
{
    // Similar to initAsPositions() above, but defining position intervals
    // instead.

}

//------------------------------------------------------------------------------
E_EtherCANErrCode FPUAdmin::listAll()
{
    // Prints whole database.
    // TODO: Specify that prints to stdout/cout?


}

//------------------------------------------------------------------------------
E_EtherCANErrCode FPUAdmin::listOne(const char *serial_number)
{
    // Prints data for one FPU
    // TODO: Specify that prints to stdout/cout?


}

//------------------------------------------------------------------------------
E_EtherCANErrCode FPUAdmin::setALimits(const char *serial_number, 
                                       double alimit_min, double alimit_max,
                                       double adatum_offset)
{
    // Sets safe limits for alpha arm of an FPU.

}

//------------------------------------------------------------------------------
E_EtherCANErrCode FPUAdmin::setBLimits(const char *serial_number, 
                                       double blimit_min, double blimit_max)
{
    // Sets safe limits for beta arm of an FPU.

}

//------------------------------------------------------------------------------
E_EtherCANErrCode FPUAdmin::setBRetries(const char *serial_number, int bretries)
{
    // Sets allowed number of freeBetaCollision commands in the same direction
    // before the software protection kicks in. The retry count is reset to
    // zero upon a successfully finished datum search.
}

//------------------------------------------------------------------------------
E_EtherCANErrCode FPUAdmin::printHealthLog(const char *serial_number)
{
    // Prints an FPU's health log from the health log database. Output format
    // details:
    //   - The index number is the count of finished datum searches
    //   - Each row also contains the UNIX time stamp which can be used to plot
    //     against time, or to identify events in the driver logs.
    // TODO: Specify that prints to stdout/cout?

    // TODO: Health log isn't implemented yet

    return DE_OK;
}

//------------------------------------------------------------------------------

} // namespace mpifps




