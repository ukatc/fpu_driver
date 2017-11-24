
// -*- mode: c++ -*-
////////////////////////////////////////////////////////////////////////////////
// ESO - VLT Project
//
// Copyright 2017 E.S.O,
//
// Who       When        What
// --------  ----------  -------------------------------------------------------
// Pablo Gutierrez 2017-07-22  created CAN client sample
// jnix      2017-10-18  Created driver class using Pablo Guiterrez' CAN client sample
//------------------------------------------------------------------------------

////////////////////////////////////////////////////////////////////////////////
// NAME E_CAN_COMMAND.h
//
// This class implements the low-level CAN driver for the MOONS fiber
// positioner grid
//
////////////////////////////////////////////////////////////////////////////////
#ifndef E_CAN_COMMAND_H

#define E_CAN_COMMAND_H

#include <cassert>
#include <stdint.h>

namespace mpifps
{

namespace canlayer
{

enum E_CAN_COMMAND
{

    CCMD_NO_COMMAND        =  0, // reserved
    CCMD_CONFIG_MOTION     =  1, // configure waveform
    CCMD_EXECUTE_MOTION    =  2, // execute loaded waveform
    CCMD_ABORT_MOTION      =  3, // abort any ongoing movement
    // FIXME: next two  schould be combined
    CCMD_GET_STEPS_ALPHA   =  4, // get alpha counts
    CCMD_GET_STEPS_BETA    =  5, // get beta counts
    CCMD_PING_FPU          =  7, // check connectivity
    CCMD_RESET_FPU         =  8, // reset MCU
    CCMD_AUTO_MOVE_DATUM   =  9, // "automatic" datum search
    CCMD_RESET_STEPCOUNTER = 10, // only for debugging
    CCMD_REPEAT_MOTION     = 11, // re-use last waveform
    CCMD_REVERSE_MOTION    = 12, // invert last waveform
    CCMD_CLEAR_COLLISION   = 13, // "ENABLE_COLLIDE"
    CCMD_UNTANGLE_FPU      = 14, // "FREE_COLLIDE"
    // FIXME: is the following needed ?
    CCMD_SET_USTEP         = 15, // set motor micro-stepping (1,2,4,8 supported)
    // FIXME: next two  schould be combined
    CCMD_GET_ERROR_ALPHA   = 16, // get residue count at last datum hit
    CCMD_GET_ERROR_BETA    = 17, // get residue count at last datum hit

    // commands which are not yet implemented
    CCMD_MOVE_DATUM_OFF    = 18, // move until datum switch is off
    CCMD_MOVE_DATUM_ON     = 19, // move until datum switch is on
    CCMD_REQUEST_STATUS    = 20, // report status and flags
    CCMD_REPORT_POSITIONS  = 21, // report alpha and beta position
    CCMD_ASSIGN_POSITION   = 22, // assign measured position in recovery
    CCMD_CHECK_INTEGRITY   = 23, // report firmware version and CRC
    CCMD_LOCK_UNIT         = 24, // ignore any command except reset and unlock
    CCMD_UNLOCK_UNIT       = 25, // listen to commands again

    NUM_CAN_COMMANDS  = 26,
};



// defines 4-bit priority value of CAN message
inline uint8_t getMessagePriority(const E_CAN_COMMAND cmd)
{
    uint8_t priority = 0x0f;
    switch (cmd)
    {
        /* highest priority has smallest code */
                
        /* used for emergency stop */
    case CCMD_ABORT_MOTION      :
        priority = 0x01; break;

        /* movement commands */
    case CCMD_EXECUTE_MOTION    :
        priority = 0x02; break;

        /* datum finding */
    case CCMD_AUTO_MOVE_DATUM        :
    case CCMD_MOVE_DATUM_OFF    :
    case CCMD_MOVE_DATUM_ON     :
        priority = 0x03; break;

        /* fpu reset */
    case CCMD_RESET_FPU         :
        priority = 0x04; break;

        /* motion configuration */
    case CCMD_CONFIG_MOTION     :
    case CCMD_ASSIGN_POSITION   :
    case CCMD_RESET_STEPCOUNTER :
    case CCMD_REPEAT_MOTION     :
    case CCMD_REVERSE_MOTION    :
    case CCMD_SET_USTEP         :
        priority = 0x05; break;

        /* error recovery */
    case CCMD_CLEAR_COLLISION   :
    case CCMD_UNTANGLE_FPU      :
    case CCMD_LOCK_UNIT         :
    case CCMD_UNLOCK_UNIT       :
        priority = 0x08; break;

        /* connectivity check */
    case CCMD_PING_FPU          :
        priority = 0x09; break;

        /* status inquiry */
    case CCMD_GET_STEPS_ALPHA   :
    case CCMD_GET_STEPS_BETA    :
    case CCMD_GET_ERROR_ALPHA   :
    case CCMD_GET_ERROR_BETA    :
    case CCMD_REQUEST_STATUS    :
    case CCMD_REPORT_POSITIONS  :
    case CCMD_CHECK_INTEGRITY   :                
        priority = 0x0a; break;

        // invalid cases
    case CCMD_NO_COMMAND        :
    default:
        assert(false);

    }
    return priority;
}

}

} // end of namespace

#endif
