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

// NOTE: return statements from main() are used rather than exit()'s, so that
// all classes created in main() have their destructors properly called (exit()
// doesn't do this).

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>

#include <vector>
#include <string>
#include <iostream>

#include "FPUAdmin.h"

using namespace mpifps;

//..............................................................................

// Multi-use error strings
static const char *bad_num_args_str = "Error: Incorrect number of arguments.";
static const char *bad_numerical_format_str = "Error: Bad numerical argument format.";

// Multi-use command strings
static const char *aretries_cmd_str = "aretries";
static const char *bretries_cmd_str = "bretries";
static const char *alimits_cmd_str = "alimits";
static const char *blimits_cmd_str = "blimits";

//..............................................................................

static void printHelp();
static void stringArgsToDoubles(const std::vector<std::string> &arg_strs,
                                int start_index,
                                std::vector<double> &doubles_ret);
static bool stringToInt(const std::string &string_in, int &integer_ret);

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
    if (arg_strs.size() == 0)
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
        return AppReturnOk;
    }
    
    //..........................................................................
    // Capture any specified options, and delete from arg_strs along the way
    // to keep its subsequent navigation simpler
    bool reinitialize = false;
    bool reuse_sn = false;
    static std::string gateway_ip_str;  // Static to guarantee persistence
    gateway_ip_str.clear();
    static t_gateway_address gateway_address = { nullptr, 0 };
    t_gateway_address *gateway_address_ptr = nullptr;

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
            reuse_sn = true;
            erase_item = true;
        }
        else if ((*it == "--gateway_address"))
        {
            it = arg_strs.erase(it);
            if (it != arg_strs.end())
            {
                // NOTE: gateway_address.ip is a POINTER, and so the
                // gateway_ip_str.c_str() that it's pointing to is persistent
                // (and not potentially optimised away)
                gateway_ip_str = *it;
                gateway_address.ip = gateway_ip_str.c_str();
                gateway_address.port = 4700;
                
                gateway_address_ptr = &gateway_address;
                erase_item = true;
            }
            else
            {
                std::cout << "Error: --gateway_address does not have a "
                             "gateway address specified." << std::endl;
                return AppReturnError;
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
                std::cout << "Error: Could not create a database transaction." <<
                             std::endl;
                return AppReturnError;
            }
        }
        else
        {
            std::cout << "Error: Could not open protection database." << std::endl;
            return AppReturnError;
        }
    }
    else
    {
        std::cout << "Error: Could not determine directory of protection database -\n";
        std::cout << "are the Linux environment variables set correctly?" << std::endl;
        return AppReturnError;
    }
        
    // Process specified command - N.B. at this stage, the number of items
    // in arg_strs might be less than the original, because the options above
    // will have been removed, so need to use indexes accordingly
    std::string cmd_str = arg_strs[0];

    //..........................................................................
    if (cmd_str == "flash")
    {
        if (arg_strs.size() == 3)
        {
            const char *serial_number = arg_strs[1].c_str();
            int fpu_id = 9999;
            if (stringToInt(arg_strs[2], fpu_id))
            {
                return FPUAdmin::flash(txn, fpu_id, serial_number, is_mockup,
                                       reuse_sn, gateway_address_ptr);
            }
            else
            {
                std::cout << bad_numerical_format_str << std::endl;
                return AppReturnError;
            }
        }
        else
        {
            std::cout << bad_num_args_str << std::endl;
            return AppReturnError;
        }
    }
    
    //..........................................................................
    else if (cmd_str == "init")
    {
        if (arg_strs.size() >= 4)
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
                    return FPUAdmin::init(txn, serial_number,
                                          apos_min, apos_max,
                                          bpos_min, bpos_max,
                                          reinitialize, adatum_offset);
                }
                else
                {
                    std::cout << bad_num_args_str << std::endl;
                    return AppReturnError;
                }
            }
            else
            {
                std::cout << bad_numerical_format_str << std::endl;
                return AppReturnError;
            }
        }
        else
        {
            std::cout << bad_num_args_str << std::endl;
            return AppReturnError;
        }
    }

    //..........................................................................
    else if ((cmd_str == alimits_cmd_str) || (cmd_str == blimits_cmd_str))
    {
        if (arg_strs.size() >= 4)
        {
            const char *serial_number = arg_strs[1].c_str();
            std::vector<double> doubles_args;
            stringArgsToDoubles(arg_strs, 2, doubles_args);
            if ((doubles_args.size() + 2) == arg_strs.size())
            {
                double limit_min = doubles_args[0];
                double limit_max = doubles_args[1];
                if (cmd_str == alimits_cmd_str)
                {
                    double adatum_offset = ALPHA_DATUM_OFFSET;
                    if (doubles_args.size() == 3)
                    {
                        adatum_offset = doubles_args[2];
                    }
                    return FPUAdmin::setALimits(txn, serial_number,
                                                limit_min, limit_max,
                                                adatum_offset);
                }
                else
                {
                    return FPUAdmin::setBLimits(txn, serial_number,
                                                limit_min, limit_max);
                }
            }
            else
            {
                std::cout << bad_numerical_format_str << std::endl;
                return AppReturnError;
            }
        }
        else
        {
            std::cout << bad_num_args_str << std::endl;
            return AppReturnError;
        }
    }
    
    //..........................................................................
    else if ((cmd_str == aretries_cmd_str) || (cmd_str == bretries_cmd_str))
    {
        if (arg_strs.size() == 3)
        {
            const char *serial_number = arg_strs[1].c_str();

            int retries;
            if (stringToInt(arg_strs[2], retries))
            {
                if (cmd_str == aretries_cmd_str)
                {
                    return FPUAdmin::setARetries(txn, serial_number, retries);
                }
                else
                {
                    return FPUAdmin::setBRetries(txn, serial_number, retries);
                }
            }
            else
            {
                std::cout << bad_numerical_format_str << std::endl;
                return AppReturnError;
            } 
        }
        else
        {
            std::cout << bad_num_args_str << std::endl;
            return AppReturnError;
        }
    }

    //..........................................................................
    else if (cmd_str == "list")
    {
        if (arg_strs.size() == 1)
        {
            return FPUAdmin::listAll(txn);
        }
        else
        {
            std::cout << bad_num_args_str << std::endl;
            return AppReturnError;
        }
    }

    //..........................................................................
    else if (cmd_str == "list1")
    {
        if (arg_strs.size() == 2)
        {
            const char *serial_number = arg_strs[1].c_str();
            return FPUAdmin::listOne(txn, serial_number);
        }
        else
        {
            std::cout << bad_num_args_str << std::endl;
            return AppReturnError;
        }
    }

    //..........................................................................
    else if (cmd_str == "healthlog")
    {
        std::cout << "Error: healthlog command is not implemented yet." << std::endl;
        return AppReturnError;
    }

    //..........................................................................
    else
    {
        std::cout << "Error: Command not recognised." << std::endl;
        return AppReturnError;
    }

    //..........................................................................
}

//------------------------------------------------------------------------------
void printHelp()
{
    // TODO: Ensure that the help comments' arguments etc exactly match the
    // actual code

    std::cout << "\n";
    std::cout << 
        "help\n"
        "    - Prints this message\n"
        "\n"
        "flash [--reuse_sn] <serial_number> <fpu_id>\n"
        "    - Flashes serial number to FPU with ID <fpu_id>. FPU must be connected.\n"
        "      If the --reuse_sn flag is set, it is allowed to\n"
        "      use a serial number which was used before.\n"
        "\n"
        "init [--reinitialize] <serial_number> <alpha_pos> <beta_pos> [<adatum_offset>]\n"
        "    - Initializes FPU data in protection database, passing the initial alpha\n"
        "      and beta arm positions in degree.\n"
        "      The optional last parameter is the alpha datum offset.\n"
        "\n"
        "      If the --reinitialize flag is set, it is allowed to redefine\n"
        "      FPU positions which have already been stored before.\n"
        "\n"
        "init [--reinitialize] <serial_number> <apos_min> <apos_max> <bpos_min> <bpos_max> [<adatum_offset>]\n"
        "    - As above, but specifies the positions in terms of INTERVALS instead.\n"
        "\n"
        "list\n"
        "    - Lists the whole database.\n"
        "\n"
        "list1 <serial_number>\n"
        "    - Lists data for one FPU.\n"
        "\n"
        "alimits <serial_number> <alpha_limit_min> <alpha_limit_max> [<adatum_offset>]\n"
        "    - Sets safe limits for alpha arm of this FPU.\n"
        "\n"
        "blimits <serial_number> <beta_limit_min> <beta_limit_max>\n"
        "    - Sets safe limits for beta arm of this FPU.\n"
        "\n"
        "aretries <serial_number> <freealpharetries>\n"
        "    - Sets allowed number of freeAlphaLimitBreach commands in the same\n"
        "      direction before the software protection kicks in.\n"
        "      The retry count is reset to zero upon a successfully-completed\n"
        "      datum search.\n"
        "\n"
        "bretries <serial_number> <freebetaretries>\n"
        "    - Sets allowed number of freeBetaCollision commands in the same\n"
        "      direction before the software protection kicks in.\n"
        "      The retry count is reset to zero upon a successfully-completed\n"
        "      datum search.\n"
        "\n"
        "healthlog <serial_number>\n"
        "    - Prints the content of the health log database for an FPU\n"
        "      to the screen. The index number is the count of finished\n"
        "      datum searches. Each row also contains the UNIX timestamp\n"
        "      which can be used to plot against time, or to identify\n"
        "      events in the driver logs.\n"
        "\n"
        "Default alpha datum offset: " << std::to_string(ALPHA_DATUM_OFFSET);
    std::cout << "\n" << std::endl; 
}

//------------------------------------------------------------------------------
void stringArgsToDoubles(const std::vector<std::string> &arg_strs,
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
bool stringToInt(const std::string &string_in, int &integer_ret)
{
    // Converts a string to an integer. Returns true if OK, or false if the
    // conversion wasn't successful.
    
    try
    {
        integer_ret = std::stoi(string_in);
        return true;
    }
    catch (...)
    {
        return false;
    }
}

//------------------------------------------------------------------------------
