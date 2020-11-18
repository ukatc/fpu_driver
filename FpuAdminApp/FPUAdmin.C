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
void FPUAdmin::printHelp()
{
    // TODO: Ensure that the help comments' arguments etc exactly match the
    // actual code
    // TODO: Add aretries command to help text? The command is checked for in
    // fpu-admin, but isn't shown in its help text

    std::cout << "\n";
    std::cout << 
        "help\n"
        "    - Print this message\n"
        "\n"
        "flash [--reuse_sn] <serial_number> <fpu_id>\n"
        "    - Flash serial number to FPU with ID <fpu_id>. FPU must be connected.\n"
        "      If the --reuse_sn flag is set, it is allowed to\n"
        "      use a serial number which was used before.\n"
        "\n"
        "init [--reinitialize] <serial_number> <alpha_pos> <beta_pos> [<adatum_offset>]\n"
        "    - Initialize protection database for FPU, passing the initial alpha\n"
        "      and beta arm positions in degree.\n"
        "      The optional last parameter is the alpha datum offset.\n"
        "\n"
        "      If the --reinitialize flag is set, it is allowed to redefine\n"
        "      FPU positions which already have been stored before.\n"
        "\n"
        "init [--reinitialize] <serial_number> [<apos_min>, <apos_max>] [<bpos_min>, <bpos_max>] [<adatum_offset>]\n"
        "    - As above, but defining position intervals instead.\n"
        "\n"
        "list\n"
        "    - List whole database.\n"
        "\n"
        "list1 <serial_number>\n"
        "    - List data for one FPU.\n"
        "\n"
        "alimits <serial_number> <alpha_limit_min> <alpha_limit_max> [<adatum_offset>]\n"
        "    - Set individual safe limits for alpha arm of this FPU.\n"
        "\n"
        "blimits <serial_number> <beta_limit_min> <beta_limit_max>\n"
        "    - Set safe limits for beta arm of this FPU.\n"
        "\n"
        "bretries <serial_number> <freebetatries>\n"
        "    - Set allowed number of freeBetaCollision command in the same\n"
        "      direction before the software protection kicks in.\n"
        "      The retry count is reset to zero upon a successfully finished\n"
        "      datum search.\n"
        "\n"
        "healthlog <serial_number>\n"
        "    - Print the content of the health log database for an FPU\n"
        "      to the screen. The index number is the count of finished\n"
        "      datum searches. Each row also contains the UNIX time stamp\n"
        "      which can be used to plot against time, or to identify\n"
        "      events in the driver logs.\n"
        "\n"
        "Default alpha datum offset: " << std::to_string(ALPHA_DATUM_OFFSET);
    std::cout << "\n" << std::endl; 
}

//------------------------------------------------------------------------------
E_EtherCANErrCode FPUAdmin::flash(int fpu_id, const char *serial_number,
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
E_EtherCANErrCode FPUAdmin::init(const char *serial_number, 
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
E_EtherCANErrCode FPUAdmin::setARetries(const char *serial_number, int aretries)
{
    // TODO: Add comment here - the aretries command isn't shown in the Python
    // fpu-admin version's help text, so figure out the correct text to put
    // here - something like in setBRetries() below
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




