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

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>

#include <string>

#include "FPUAdmin.h"

//------------------------------------------------------------------------------
int main(int argc, char**argv)
{
    int i;

    mpifps::FPUAdmin fpuadmin;
    fpuadmin.printHelp();

    // TODO: Temporary for testing only - needs to be set to argv[1], if argc
    // is >= 2 (argument 1 is normally the app name I think)
    std::string command_string = "init";

    if (command_string == "init")
    {
    }
    else if (command_string == "flash")
    {
    }
    else if (command_string == "alimits")
    {
    }
    else if (command_string == "blimits")
    {
    }
    else if (command_string == "aretries")
    {
    }
    else if (command_string == "bretries")
    {
    }
    else if (command_string == "list")
    {
    }
    else if (command_string == "list1")
    {
    }
    else if (command_string == "healthlog")
    {
        // TODO: Not implemented yet, because no health log in LMDB database
        // yet
    }

    return 0;

#if 1
    int aflag = 0;
    int bflag = 0;
    char *cvalue = NULL;
    int index;
    int c;

    opterr = 0;

    while ((c = getopt(argc, argv, "abc:")) != -1)
    {
        switch (c)
        {
        case 'a':
          aflag = 1;
          break;
          
        case 'b':
          bflag = 1;
          break;
          
        case 'c':
          cvalue = optarg;
          break;
          
        case '?':
          if (optopt == 'c')
          {
              fprintf(stderr, "Option -%c requires an argument.\n", optopt);
          }
          //else if (isprint(optopt))
          //{
          //    fprintf(stderr, "Unknown option `-%c'.\n", optopt);
          //}
          else
          {
              fprintf(stderr,
                      "Unknown option character `\\x%x'.\n",
                      optopt);
          }
          return 1;
          
        default:
            abort();
        }
    }

    printf("aflag = %d, bflag = %d, cvalue = %s\n",
           aflag, bflag, cvalue);

    for (index = optind; index < argc; index++)
    {
        printf("Non-option argument %s\n", argv[index]);
    }
    return 0;
  
#else
    // Prints arguments
    printf("Arguments:\n");
    for (i = 0; i < argc; i++) {
        printf("%i: %s\n", i, argv[i]);
    }

    return 0;
    
#endif // 0
}

//------------------------------------------------------------------------------
