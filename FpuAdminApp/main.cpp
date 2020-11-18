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

// NOTE: return's from main() are used rather than exit()'s because runs any
// destructors before exiting

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>

#include <vector>
#include <string>
#include <iostream>

#include "FPUAdmin.h"

//------------------------------------------------------------------------------
int main(int argc, char**argv)
{
    // TODO: Look for the is_mockup flag in the Linux environment variables -
    // see Python fpu-admin file
    bool is_mockup = false;

    // Capture argument strings into convenient vector - N.B. argument 0 is
    // the the application file path
    std::vector<std::string> arg_strs;
    for (int i = 0; i < argc; i++)
    {
        arg_strs.push_back(argv[i]);
    }

    using namespace mpifps;  // For accessing FPUAdmin class

    //..........................................................................
    // Display help if requested, and return
    if ((arg_strs.size() < 2) ||
        ((arg_strs.size() == 2) &&
         ((arg_strs[1] == "-h") || (arg_strs[1] == "-?") ||
          (arg_strs[1] == "--help") || (arg_strs[1] == "help"))))
    {
        FPUAdmin::printHelp();
        return 0;
    }
    
    //..........................................................................
    // Capture any specified options
    bool re_initialize = false;
    bool reuse_sn = false;
    t_gateway_address gateway_address = { nullptr, 0 };

    // N.B. Start at index of [2], because [0] is app file path and [1] is
    // command



    // **************** TODO: Delete each option from arg_strs[] as encounter
    // (like Python fpu-admin does) so that subseqent command processing
    // can be done properly


    for (size_t i = 2; i < arg_strs.size(); i++)
    {
        if (arg_strs[i] == "--mockup")
        {
            is_mockup = true;
        }
        else if (arg_strs[i] == "--reinitialize")
        {
            re_initialize = true;
        }
        else if ((arg_strs[i] == "--reuse_sn") || (arg_strs[i] == "--reuse-sn"))
        {
            bool reuse_sn = true;
        }
        else if ((arg_strs[i] == "--gateway_address"))
        {
            if (arg_strs.size() > (i + 1))
            {
                // NOTE: gateway_address.ip is a POINTER, and so the location
                // that it's pointing to needs to be persistent
                gateway_address.ip = arg_strs[i + 1].c_str();
                gateway_address.port = 4700;
            }
        }
    }

    //..........................................................................
    // Process specified command
    std::string cmd_str = arg_strs[1];

    if (cmd_str == "init")
    {
        if ((arg_strs.size() != 5) && (arg_strs.size() != 6))
        {
            std::cout << "Usage: init <serial_number> <alpha_pos> <beta_pos> "
                         "[<adatum_offset>]\n" << std::endl;
            return 1;
        }

        std::string serial_number = arg_strs[2];
        
        // TODO
        //FPUAdmin::init(serial_number.c_str(), )
    }
    
    else if (cmd_str == "flash")
    {
    }
    
    else if (cmd_str == "alimits")
    {
    }
    
    else if (cmd_str == "blimits")
    {
    }
    
    else if (cmd_str == "aretries")
    {
    }
    
    else if (cmd_str == "bretries")
    {
    }
    
    else if (cmd_str == "list")
    {
    }
    
    else if (cmd_str == "list1")
    {
    }
    
    else if (cmd_str == "healthlog")
    {
        // TODO: Not implemented yet, because no health log in LMDB database
        // yet
    }

    //..........................................................................

    return 0;
}

//------------------------------------------------------------------------------
