// -*- mode: c++ -*-

////////////////////////////////////////////////////////////////////////////////
//
// Copyright 2017 UKRI. See file "LICENSE" for license information.
//
//
// Who       When        What
// --------  ----------  -------------------------------------------------------
// jnix      2017-10-18  Created driver class using Pablo Guiterrez' CAN client sample
//------------------------------------------------------------------------------

////////////////////////////////////////////////////////////////////////////////
// NAME GridState.h
//
// This header defines a global state description for the FPU grid.
//
//
////////////////////////////////////////////////////////////////////////////////

#ifndef E_GRID_STATE_H
#define E_GRID_STATE_H

namespace mpifps
{

enum E_GridState
{
    // the following operative states
    // are sum descriptors for the state of the FPU grid.
    // They are intended as human-friendly summary information,
    // and probably not suitable to control the driver.
    //
    // The global state is computed in a "least common denominator"
    // manner (e.g. if 950 FPUs are "READY", 45 are "LOADING", 9 are
    // "LOCKED", and 1 "INITIALIZED", the state would be
    // "INITIALIZED".

    GS_UNKNOWN        = (1 << 1), // no information available
    GS_UNINITIALIZED  = (1 << 2), // datum positions not known
    GS_LEAVING_DATUM  = (1 << 3), // datum is known, no waveforms loaded
    GS_ABOVE_DATUM    = (1 << 4), // datum is known, no waveforms loaded
    GS_DATUM_SEARCH   = (1 << 5), // some FPUs are searching datum
    GS_AT_DATUM       = (1 << 6), // datum is known, no waveforms loaded
    GS_LOADING        = (1 << 7), // loading waveforms
    GS_READY_FORWARD  = (1 << 8), // all FPUs are ready to go forward
    GS_READY_REVERSE = (1 << 9), // all FPUs are ready to go backward
    GS_MOVING         = (1 << 10), // all or some FPUs are moving
    GS_FINISHED       = (1 << 11), // all FPUs at target
    GS_COLLISION      = (1 << 12), // a collision or limit stop was detected
    GS_ABORTED        = (1 << 13), // movement was aborted, error not cleared
    // the following are pseudo-states which are used
    // as wait targets but are no actual grid states
    GS_NOPENDING      = (1 << 14), // no commands are left marked as pending
    GS_NOMOVING       = (1 << 15), // FPUS are in datum search, moving, or ready to move
    GS_TIMEOUT        = (1 << 16), // a new time-out occurred

    GS_ALL_UPDATED   = (1 << 17), // all fpus have been updated

} ;


// Target state bitmasks for the waitForState()
// method. These are bitmask for the grid states above.

// keep in mind that the target states names
// describe desired collective states of the
// FPU grid, but are matched by error conditions.

enum E_WaitTarget
{

    TGT_ABOVE_DATUM      = ( GS_ABOVE_DATUM
                             | GS_UNKNOWN
                             | GS_COLLISION
                             | GS_ABORTED),

    TGT_AT_DATUM      = (GS_AT_DATUM
                         | GS_UNKNOWN
                         | GS_COLLISION
                         | GS_ABORTED),

    TGT_LOADING      = (GS_AT_DATUM
                        | GS_LOADING),


    TGT_READY_TO_MOVE = (GS_READY_FORWARD
                         | GS_READY_REVERSE
                         | GS_AT_DATUM
                         | GS_UNINITIALIZED
                         | GS_COLLISION
                         | GS_ABORTED),

    TGT_MOVEMENT_FINISHED = (GS_FINISHED
                             | GS_COLLISION
                             | GS_ABORTED),

    // Target for info-requesting commands that don't change
    // the state of the FPUs
    TGT_NO_MORE_PENDING = GS_NOPENDING,

    // target for finish of movement commands
    // (findDatum and executeMotion) and pending commands.
    TGT_NO_MORE_MOVING = GS_NOMOVING,

    TGT_TIMEOUT = GS_TIMEOUT, // return on timeout

    TGT_ALL_UPDATED = GS_ALL_UPDATED, // return when all FPUs have fresh info


    // Note: Using this target requires much more
    // frequent signalling, this possibly
    // affects performance.
    TGT_ANY_CHANGE = 0xFFFFFFFF,
} ;



}
#endif
