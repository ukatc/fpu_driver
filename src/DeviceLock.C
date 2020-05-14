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
// NAME DeviceLock.C
//
// TODO: Put description here
//
////////////////////////////////////////////////////////////////////////////////

#include <sys/types.h>
#include <signal.h>
#include <errno.h>

#include "DeviceLock.h"
#include "Exceptions_INTERIM.h"

#define LOCKDIR     "/var/tmp/devicelocks"


namespace mpifps
{

// -----------------------------------------------------------------------------
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

// -----------------------------------------------------------------------------



}