// -*- mode: c++ -*-

////////////////////////////////////////////////////////////////////////////////
//
// Copyright 2020 UKRI. See file "LICENSE" for license information.
//
//
// Who       When        What
// --------  ----------  -------------------------------------------------------
// bwillemse 2020-06-25  TODO: Put comment here
//------------------------------------------------------------------------------

////////////////////////////////////////////////////////////////////////////////
// NAME FpuBPShared_General.C
//
// TODO: Put description here
//
////////////////////////////////////////////////////////////////////////////////

#include "FpuBPShared_General.h"


// -----------------------------------------------------------------------------
std::ostringstream& operator<<(std::ostringstream &out, const E_FPU_STATE &s)
{
    switch(s)
    {
    case FPST_UNKNOWN:
        out << "'FPST_UNKNOWN'";
        break;
    case FPST_UNINITIALIZED:
        out << "'FPST_UNINITIALIZED'";
        break;
    case FPST_LOCKED:
        out << "'FPST_LOCKED'";
        break;
    case FPST_DATUM_SEARCH:
        out << "'FPST_DATUM_SEARCH'";
        break;
    case FPST_AT_DATUM:
        out << "'FPST_AT_DATUM'";
        break;
    case FPST_LOADING:
        out << "'FPST_LOADING'";
        break;
    case FPST_READY_FORWARD:
        out << "'FPST_READY_FORWARD'";
        break;
    case FPST_READY_REVERSE:
        out << "'FPST_READY_REVERSE'";
        break;
    case FPST_MOVING:
        out << "'FPST_MOVING'";
        break;
    case FPST_RESTING:
        out << "'FPST_RESTING'";
        break;
    case FPST_ABORTED:
        out << "'FPST_ABORTED'";
        break;
    case FPST_OBSTACLE_ERROR:
        out << "'FPST_OBSTACLE_ERROR'";
        break;
    }
    return out;
}


// -----------------------------------------------------------------------------
std::ostringstream& operator<<(std::ostringstream &out, const E_InterfaceState &s)
{
    switch(s)
    {
    case DS_UNINITIALIZED:
        out << "'DS_UNINITIALIZED'";
        break;
    case DS_UNCONNECTED:
        out << "'DS_UNCONNECTED'";
        break;
    case DS_CONNECTED:
        out << "'DS_CONNECTED'";
        break;
    case DS_ASSERTION_FAILED:
        out << "'DS_ASSERTION_FAILED'";
        break;
    }
    return out;
}

// -----------------------------------------------------------------------------
