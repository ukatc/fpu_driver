
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

    CCMD_NO_COMMAND                       = 0, // reserved
    CCMD_CONFIG_MOTION                    = 1, // configure waveform
    CCMD_EXECUTE_MOTION                   = 2, // execute loaded waveform
    CCMD_ABORT_MOTION                     = 3, // abort any ongoing movement
    // FIXME: next two  schould be combined
    CCMD_GET_STEPS_ALPHA                  = 4, // get alpha counts
    CCMD_GET_STEPS_BETA                   = 5, // get beta counts
    CCMD_READ_REGISTER                    = 6, // read register - unused
    CCMD_PING_FPU                         = 7, // check connectivity
    CCMD_RESET_FPU                        = 8, // reset MCU
    CCMD_AUTO_MOVE_DATUM                  = 9, // "automatic" datum search
    CCMD_RESET_STEPCOUNTER                = 10, // only for debugging
    CCMD_REPEAT_MOTION                    = 11, // re-use last waveform
    CCMD_REVERSE_MOTION                   = 12, // invert last waveform
    CCMD_ENABLE_BETA_COLLISION_PROTECTION = 13, // "ENABLE_COLLIDE"
    CCMD_FREE_BETA_COLLISION              = 14, // "FREE_COLLIDE"
    CCMD_SET_USTEP                        = 15, // set motor micro-stepping (1,2,4,8 supported)
    // FIXME: next two  schould be combined
    CCMD_GET_ERROR_ALPHA                  = 16, // get residue count at last datum hit
    CCMD_GET_ERROR_BETA                   = 17, // get residue count at last datum hit

    // commands which are not yet implemented
    CCMD_REQUEST_STATUS = 20,   // report status and flags
    CCMD_REPORT_POSITIONS              = 21, // report alpha and beta position
    CCMD_GET_FIRMWARE_VERSION          = 22, // get firmware version
    CCMD_CHECK_INTEGRITY               = 23, // report firmware CRC
    CCMD_LOCK_UNIT                     = 24, // ignore any command except reset and unlock
    CCMD_UNLOCK_UNIT                   = 25, // listen to commands again
    CCMD_FREE_ALPHA_LIMIT_BREACH       = 26, // untangle alpha arm
    CCMD_ENABLE_ALPHA_LIMIT_PROTECTION = 27, // re-enable limit switch
    CCMD_SET_TIME_STEP                 = 28, // set movement time interval
    CCMD_SET_MIN_FREQUENCY             = 29, // set minimum step frequency

    NUM_CAN_COMMANDS = 30,

    /***************************************/
    /* FPU warning messages */
    /* code 101 unused */
    /* code 102 unused */
    CMSG_FINISHED_MOTION               = 103, // executeMotion finished
    CMSG_FINISHED_DATUM                = 104, // findDatum finished
    CMSG_WARNCOLLISION_BETA            = 105, // collision at beta arm
    CMSG_WARNLIMIT_ALPHA               = 106, // limit switch at alpha arm
    /* code 107 probably unused */

};


// this is a one-bit parameter to several commands
enum E_REQUEST_DIRECTION
{
    REQD_COUNTER_CLOCKWISE = 0,
    REQD_CLOCKWISE         = 1,
};

// this uses the unsigned specifier because
// the representation of signed bitfields is
// implementation-defined
typedef struct t_fpu_status_flags
{
    unsigned int alpha_at_datum : 1;
    unsigned int beta_at_datum : 1;
    unsigned int beta_collision_detected : 1;
    unsigned int alpha_limitswitch_triggered : 1;
    unsigned int is_locked : 1;
    unsigned int dir_alpha : 1; 
    unsigned int dir_beta : 1;
    unsigned int was_initialized : 1;
} t_fpu_status_flags;
 

// defines 4-bit priority value of CAN message
inline uint8_t getMessagePriority(const E_CAN_COMMAND cmd)
{
    uint8_t priority = 0x0f;
    switch (cmd)
    {
        /* highest priority has smallest code */

        /* priorities 0x00 and 0x01 are reserved for
           FPU warning messages and command responses. */
        
        /* used for emergency stop, usually broadcast */
    case CCMD_ABORT_MOTION      : 
        /* movement commands, usually broadcast */
    case CCMD_EXECUTE_MOTION    :
    case CCMD_AUTO_MOVE_DATUM        :
        priority = 0x02; break;

        /* motion configuration */
    case CCMD_CONFIG_MOTION     :
        /* error recovery */
    case CCMD_RESET_FPU         :
    case CCMD_RESET_STEPCOUNTER :
    case CCMD_ENABLE_BETA_COLLISION_PROTECTION   :
    case CCMD_FREE_BETA_COLLISION      : 
    case CCMD_FREE_ALPHA_LIMIT_BREACH       :
    case CCMD_ENABLE_ALPHA_LIMIT_PROTECTION :
    case CCMD_LOCK_UNIT         :
    case CCMD_UNLOCK_UNIT       :
    case CCMD_REPEAT_MOTION     :
    case CCMD_REVERSE_MOTION    :
        priority = 0x03; break;

        /* status inquiry */
    case CCMD_PING_FPU          :
    case CCMD_GET_STEPS_ALPHA   :
    case CCMD_GET_STEPS_BETA    :
    case CCMD_GET_ERROR_ALPHA   :
    case CCMD_GET_ERROR_BETA    :
    case CCMD_REQUEST_STATUS    :
    case CCMD_REPORT_POSITIONS  :
    case CCMD_CHECK_INTEGRITY   :                
    case CCMD_GET_FIRMWARE_VERSION          :
    case CCMD_SET_TIME_STEP                 :
    case CCMD_SET_MIN_FREQUENCY             :
    case CCMD_SET_USTEP         :
        priority = 0x04; break;

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
