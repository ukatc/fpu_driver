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
// NAME DeviceLock.C
//
// Device locking/unlocking functionality.
//
////////////////////////////////////////////////////////////////////////////////


  // *********** BW NOTE: See comments near top of DeviceLock.h **************


#include <sys/types.h>
#include <signal.h>
#include <errno.h>
#include <unistd.h>
#include <string>

#include "DeviceLock.h"

namespace mpifps
{

//------------------------------------------------------------------------------
DeviceLock::DeviceLock(const std::string &devicename,
                       const std::string &usergroup)
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

    // TODO: The following time code is used because it should be equivalent
    // to Python's time.time() function which was used in the original Python
    // code from which this C++ code was converted. However, will the C/C++
    // equivalent have the same time resolution, and will there be any
    // potential problem with duplicate filenames if operations are done within
    // a small time of each other?
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

//------------------------------------------------------------------------------
bool pid_exists(int pid)
{
    if (pid < 0)
    {
        return false;
    }
    if (pid == 0)
    {
        raiseException("invalid PID 0");
    }

    if (kill(pid, 0) != 0)
    {
        if (errno == ESRCH)
        {
            // ESRCH == No such process
            return false;
        }
        else if (errno == EPERM)
        {
            // EPERM clearly means there's a process to deny access to
            return true;
        }
        else
        {
            // According to "man 2 kill" possible error values are
            // (EINVAL, EPERM, ESRCH)
            raiseException();
        }
    }
    else
    {
        return true;
    }
}

//------------------------------------------------------------------------------
void cleanup_lock(const std::string &lockname)
{
    if (access(lockname.c_str(), F_OK) == 0)
    {
         unlink(lockname.c_str());
    }
}

//-----------------------------------------------------------------------------

} // namespace mpifps
