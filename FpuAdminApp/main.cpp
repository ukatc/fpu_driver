// -*- mode: c++ -*-

////////////////////////////////////////////////////////////////////////////////
//
// Copyright 2020 UKRI. See file "LICENSE" for license information.
//
//
// Who       When        What
// --------  ----------  -------------------------------------------------------
// bwillemse 2020-11-17  Created (project adapted from Python fpu-admin file).
//------------------------------------------------------------------------------

////////////////////////////////////////////////////////////////////////////////
// NAME main.cpp
//
// Main file for FPUAdmin command-line application, which provides FPU database
// administration facilities.
//
////////////////////////////////////////////////////////////////////////////////

// NOTE: return's from main() are used rather than exit()'s, so that all
// classes created in main() have their destructors properly called - exit()
// doesn't do this.

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>

#include <vector>
#include <string>
#include <iostream>

#include "FPUAdmin.h"

using namespace mpifps;

static void printHelp();
static void stringArgsToDoubles(std::vector<std::string> &arg_strs,
                                int start_index,
                                std::vector<double> &doubles_ret);

//------------------------------------------------------------------------------
int main(int argc, char**argv)
{
    // TODO: Look for the is_mockup flag in the Linux environment variables -
    // see Python fpu-admin file
    bool is_mockup = false;

    // Capture argument strings into convenient vector
    std::vector<std::string> arg_strs;
    for (int i = 0; i < argc; i++)
    {
        arg_strs.push_back(argv[i]);
    }

    // First argument is the application file path - not interested in it,
    // so delete it
    arg_strs.erase(arg_strs.begin());
    
    //..........................................................................
    // Display help if requested, and return
    bool print_help = false;
    if (arg_strs.size() <= 1)
    {
        print_help = true;
    }
    else
    {
        if ((arg_strs[0] == "-h") || (arg_strs[0] == "-?") ||
            (arg_strs[0] == "--help") || (arg_strs[0] == "help"))
        {
            print_help = true;
        }
    }
    
    if (print_help)
    {
        printHelp();
        return 0;
    }
    
    //..........................................................................
    // Capture any specified options, and delete from arg_strs along the way
    // to keep its subsequent navigation simpler
    bool reinitialize = false;
    bool reuse_sn = false;
    t_gateway_address gateway_address = { nullptr, 0 };

    // N.B. Start at index of [1], because [0] should be command command
    for (auto it = arg_strs.begin() + 1; it != arg_strs.end();)
    {
        bool erase_item = false;
        if (*it == "--mockup")
        {
            is_mockup = true;
            erase_item = true;
        }
        else if (*it == "--reinitialize")
        {
            reinitialize = true;
            erase_item = true;
        }
        else if ((*it == "--reuse_sn") || (*it == "--reuse-sn"))
        {
            bool reuse_sn = true;
            erase_item = true;
        }
        else if ((*it == "--gateway_address"))
        {
            it = arg_strs.erase(it);
            if (it != arg_strs.end())
            {
                // NOTE: gateway_address.ip is a POINTER, and so the location
                // that it's pointing to needs to be persistent
                gateway_address.ip = it->c_str();
                gateway_address.port = 4700;
                erase_item = true;
            }
        }
        
        if (erase_item)
        {
            it = arg_strs.erase(it);
        }
        else
        {
            it++;
        }
    }

    //..........................................................................
    // Open database and create transaction
    ProtectionDB protection_db;
    ProtectionDbTxnPtr txn;
    std::string dir_str = ProtectionDB::getDirFromLinuxEnv(is_mockup);
    if (!dir_str.empty())
    {
        if (protection_db.open(dir_str))
        {
            txn = protection_db.createTransaction();
            if (!txn)
            {
                // TODO: Error: Print error message here
                
                return 1;
            }
        }
        else
        {
            // TODO: Error: Print error message here
            
            return 1;
        }
    }
    else
    {
        // TODO: Error: Print error message here
        
        return 1;
    }
        
        
        // ********************** TODO: call ProtectionDB::sync() somewhere? OR will it automatically
        // sync when the database is closed when this app finishes?
    
        // ********************** TODO: Need to close protection_db somewhere?

    // Process specified command - N.B. at this stage, the number of items
    // in arg_strs might be less than the original, because the options above
    // will have been removed, so need to use indexes accordingly
    std::string cmd_str = arg_strs[0];

    //..........................................................................
    if (cmd_str == "flash")
    {
    }
    
    //..........................................................................
    else if (cmd_str == "init")
    {
        const char *serial_number = arg_strs[1].c_str();
        
        std::vector<double> doubles_args;
        stringArgsToDoubles(arg_strs, 2, doubles_args);
        if ((doubles_args.size() + 2) == arg_strs.size())
        {
            bool correct_num_args = false;
            double apos_min, apos_max, bpos_min, bpos_max;
            double adatum_offset = ALPHA_DATUM_OFFSET;
            
            if ((doubles_args.size() == 2) || (doubles_args.size() == 3))
            {
                correct_num_args = true;

                // 2 x single position values
                apos_min = doubles_args[0];
                apos_max = apos_min;
                bpos_min = doubles_args[1];
                bpos_max = bpos_min;
                if (doubles_args.size() == 3)
                {
                    adatum_offset = doubles_args[2];
                }
            }

            else if ((arg_strs.size() == 4) || (arg_strs.size() == 5))
            {
                correct_num_args = true;

                // 2 x pairs of min/max intervals
                apos_min = doubles_args[0];
                apos_max = doubles_args[1];
                bpos_min = doubles_args[2];
                bpos_max = doubles_args[3];
                if (arg_strs.size() == 5)
                {
                    adatum_offset = doubles_args[4];
                }
            }
        
            if (correct_num_args)
            {
                E_EtherCANErrCode ecan_result = 
                        FPUAdmin::init(txn, serial_number,
                                       apos_min, apos_max,
                                       bpos_min, bpos_max, reinitialize,
                                       adatum_offset);
                if (ecan_result != DE_OK)
                {
                    // TODO: Error: Display error etc
                    
                    return 1;
                }
            }
            else
            {
                std::cout << "**ERROR**: Incorrect number of arguments - "
                             "see help for required argument lists.\n" << std::endl;
                return 1;
            }
        }
        else
        {
            std::cout << "**ERROR: An argument wasn't in the correct format -"
                         "see help for correct format.\n" << std::endl;
            return 1;
        }
    }

    //..........................................................................
    else if (cmd_str == "alimits")
    {
    }
    
    //..........................................................................
    else if (cmd_str == "blimits")
    {
    }

    //..........................................................................
    else if (cmd_str == "aretries")
    {
    }

    //..........................................................................
    else if (cmd_str == "bretries")
    {
    }

    //..........................................................................
    else if (cmd_str == "list")
    {
    }

    //..........................................................................
    else if (cmd_str == "list1")
    {
    }

    //..........................................................................
    else if (cmd_str == "healthlog")
    {
        // TODO: Not implemented yet, because no health log in LMDB database
        // yet
    }

    //..........................................................................

    return 0;
}

//------------------------------------------------------------------------------
void printHelp()
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
        "init [--reinitialize] <serial_number> <apos_min> <apos_max> <bpos_min> <bpos_max> [<adatum_offset>]\n"
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
void stringArgsToDoubles(std::vector<std::string> &arg_strs,
                         int start_index, std::vector<double> &doubles_ret)
{
    // Converts arg_strs items into doubles in doubles_ret, starting from
    // arg_strs[start_index]. Notes:
    //   - If start_index is >= number of items in arg_strs then just returns
    //     with doubles_ret size of 0
    //   - If an argument string couldn't be converted into a double, then
    //     doubles_ret will contain only the first items successfully converted
    
    doubles_ret.clear();
    if (start_index >= arg_strs.size())
    {
        return;
    }
    
    for (int i = start_index; i < arg_strs.size(); i++)
    {
        try
        {
            double double_val = std::stod(arg_strs[i]);
            doubles_ret.push_back(double_val);
        }
        catch (...)
        {
            break;
        }
    }
}

//------------------------------------------------------------------------------
