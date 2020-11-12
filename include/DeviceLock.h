// -*- mode: c++ -*-

////////////////////////////////////////////////////////////////////////////////
//
// Copyright 2020 UKRI. See file "LICENSE" for license information.
//
//
// Who       When        What
// --------  ----------  -------------------------------------------------------
// bwillemse 2020-05-14  Created (translated from Python devicelock.py).
//------------------------------------------------------------------------------

////////////////////////////////////////////////////////////////////////////////
// NAME DeviceLock.h
//
// Device locking/unlocking functionality.
//
////////////////////////////////////////////////////////////////////////////////


//  *********** BW NOTE: The work on converting devicelock.py into C++ code
//  *********** in this module is paused for now, because it's still to be
//  *********** evaluated whether this device lock functionality is actually
//  *********** needed, particularly in the final ESO driver, and/or if it's
//  *********** better to replace it by e.g. using Linux named semaphores instead


#ifndef DEVICELOCK_H
#define DEVICELOCK_H

#include <sys/stat.h>
#include <sys/types.h>
#include <grp.h>
#include <sstream>
#include <fstream>
#include <iostream>
#include <time.h>

#include "Exceptions_INTERIM.h"

namespace mpifps
{

#define LOCKDIR     "/var/tmp/devicelocks"

// TODO: This module uses files with long timestamped filenames to handle
// locks - but could Linux OS-scope named semaphores just be used instead, to
// simplify things? These work across processes as well (see
// http://man7.org/linux/man-pages/man7/sem_overview.7.html)


// -----------------------------------------------------------------------------
// Creates a reliable inter-process lock for exclusive access to a resource
// with the unique name DEVICENAME. When the DeviceLock object is deleted
// using del, or the process exits, the lock file is auomatically deleted.
// Stale lockfiles are detected by probing for the process ID (pid) of the
// creating process.

class DeviceLock
{
public:
    DeviceLock(const std::string &devicename,
               const std::string &usergroup = "");

private:
    std::string pidfile;
};

// -----------------------------------------------------------------------------

} // namespace mpifps

#endif // DEVICELOCK_H
