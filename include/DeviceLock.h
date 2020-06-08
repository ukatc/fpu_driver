// -*- mode: c++ -*-

////////////////////////////////////////////////////////////////////////////////
//
// Copyright 2020 UKRI. See file "LICENSE" for license information.
//
//
// Who       When        What
// --------  ----------  -------------------------------------------------------
// bwillemse 2020-05-14  TODO: Put comment here
//------------------------------------------------------------------------------

////////////////////////////////////////////////////////////////////////////////
// NAME DeviceLock.h
//
// TODO: Put description here
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

#define LOCKDIR     "/var/tmp/devicelocks"



// TODO: This module uses files with long timestamped filenames to handle
// locks - but could Linux OS-scope named semaphores just be used instead, to
// simplify things? These work across processes as well (see
// http://man7.org/linux/man-pages/man7/sem_overview.7.html)


// -----------------------------------------------------------------------------
class DeviceLock
{
public:
    // Creates a reliable inter-process lock for exclusive access to a resource
    // with the unique name DEVICENAME. When the DeviceLock object is deleted
    // using del, or the process exits, the lock file is auomatically deleted.
    // Stale lockfiles are detected by probing for the process ID (pid) of the
    // creating process.

    DeviceLock(const std::string &devicename, const std::string &usergroup="")
    {
        if (mkdir(LOCKDIR, S_IRWXU | S_IRWXG | S_IROTH) == 0)
        {
            if (usergroup != "")
            {
                // Set group
                gid_t gid = getgrnam(usergroup.c_str())->gr_gid;
                chown(LOCKDIR, -1, gid);
            }
        }
        else
        {
            if (errno != EEXIST)
            {
                raiseException("");
            }
        }
                
        std::string mypidstr = std::to_string(getpid());
        std::ostringstream tmpname;

        // TODO: The following time code is used because it should be
        // equivalent to Python's time.time() function which was used
        // in the original Python code from which this C++ code was
        // converted. However, will the C/C++ equivalent have the same time
        // resolution, and will there be any potential problem with duplicate
        // filenames if operations are done within a small time of each other?
        struct timespec ts;
        if (timespec_get(&ts, TIME_UTC) == 0)
        {
            // TODO: Error - but unlikely to occur?
        }
        int64_t microsecs = ts.tv_sec * 1000000;  // Get secs value in microsecs
        microsecs += ts.tv_nsec / 1000;           // Add full microseconds  
        if ((ts.tv_nsec % 1000) >= 500)           // Round up if necessary
        {
            ++microsecs;
        }
        double time_secs_double = ((double)microsecs) / 1000000.0;

        // TODO: Test resultant filenames - e.g. will time_secs_double create
        // too many digits?
        tmpname << LOCKDIR << "/" << devicename << "_" << mypidstr << "-"
                << time_secs_double << ".lock";

        std::fstream f;
        f.open(tmpname.str().c_str(), std::ios::out | std::ios::trunc);
        f << mypidstr;

        std::ostringstream lockname;
        lockname << LOCKDIR << "/" << devicename << ".lock";

        // This is atomic and can only succeed if lockname does not exist
        // Give everyone read permission
        if (chmod(tmpname.str().c_str(), S_IRUSR | S_IRGRP | S_IROTH) == 0)
        {
            // Link to temporary lock file
            // This is important because we need to keep the lock while we
            // check whether the last owner of the lock is alive
            if (link(tmpname.str().c_str(), lockname.str().c_str()) != 0)
            {
                // TODO: Add devicename to error message eventually
                // raise SystemError("Error: can't lock Device %s" % devicename)
                raiseException("Error: can't lock Device");
            }
        }
        // TODO: The logic of this needs to be part of the code above (was
        // a try/finally block in Python)    
        // We don't need the temporary file name any more
        unlink(tmpname.str().c_str());

/*      
        pidfile = "%s/%s.pid" % (LOCKDIR,devicename)

        try:
            if os.path.exists(pidfile):
                oldpid = int(open(pidfile).readline())
                if pid_exists(oldpid):
                    raise DeviceIsLockedError("Device %r is locked with lock file %r by running process %i" % (devicename, pidfile, oldpid))
                else:
                    print("Info: replacing stale lock file for device %r of dead process %s" % (devicename, oldpid))
                
            # this is also atomic, and unlinks the old pidfile
            os.rename(lockname, pidfile)
            self.pidfile = pidfile

        finally:
            cleanup_lock(lockname)

            
        atexit.register(cleanup_lock, pidfile)

*/
    }

private:
    std::string pidfile;
};

// -----------------------------------------------------------------------------

#endif // DEVICELOCK_H