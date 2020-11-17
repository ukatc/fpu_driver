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
// Main file for FPUAdmin command-line application, which performs FPU database
// administration.
//
////////////////////////////////////////////////////////////////////////////////

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <unistd.h>

//------------------------------------------------------------------------------
int main(int argc, char**argv)
{
    int i;
    
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
