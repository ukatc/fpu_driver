#!/usr/bin/python

import os
import errno
import grp
import time
from stat import *
import atexit

LOCKDIR = "/var/tmp/devicelocks"

def pid_exists(pid):
    if pid < 0:
        return False
    if pid == 0:
        raise ValueError('invalid PID 0')
    try:
        os.kill(pid, 0)
    except OSError as err:
        if err.errno == errno.ESRCH:
            # ESRCH == No such process
            return False
        elif err.errno == errno.EPERM:
            # EPERM clearly means there's a process to deny access to
            return True
        else:
            # According to "man 2 kill" possible error values are
            # (EINVAL, EPERM, ESRCH)
            raise
    else:
        return True

class DeviceIsLockedError(SystemError):
    pass


def cleanup_lock(lockname):
    if os.path.exists(lockname):
        os.unlink(lockname)


class DeviceLock:
    """Creates a reliable inter-process lock for exclusive
    access to a resource with the unique name DEVICENAME.
    When the DeviceLock object is deleted using del,
    or the process exists, the lock file is auomatically
    deleted. Stale lockfiles are detected by probing for
    the process ID (pid) of the creating process.
    """
    def __init__(self, devicename, usergroup=''):
        self.pidfile = None
        try:
            os.mkdir(LOCKDIR, S_IRWXU | S_IRWXG |  S_IROTH)
            if usergroup != '':
                # set group
                gid = grp.getgrnam(usergroup)[2]
                os.chown(LOCKDIR, -1, gid)
                
        except OSError  as e:
            if e.errno == errno.EEXIST:
                pass
            else:
                raise
            
        mypid = str(os.getpid())
        tmpname = "%s/%s_%s-%s.lock" % (LOCKDIR,devicename,mypid,time.time())
        with open(tmpname,"w") as f:
            f.writelines(mypid)
            
        lockname = "%s/%s.lock" % (LOCKDIR,devicename)
        # this is atomic and can only succeed if lockname
        # does not exist
        try:
            try:
                # give everyone read permission
                os.chmod(tmpname, S_IRUSR | S_IRGRP | S_IROTH)
                # link to temporary lock file
                # This is important because we need to keep
                # the lock while we check whether the last owner
                # of the lock is alive
                os.link(tmpname, lockname)
            except:
                # raise SystemError("Error: can't lock Device %s" % devicename)
                raise
            
        finally:
            # we don't need the temporary file name any more
            os.unlink(tmpname)
            
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
        

    def __del__(self):
        if not self.pidfile is None:
            cleanup_lock(self.pidfile)
                
                
        
