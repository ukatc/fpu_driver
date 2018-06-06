
// -*- mode: c++ -*-
////////////////////////////////////////////////////////////////////////////////
//
// Copyright 2017 UKRI. See file "LICENSE" for license information.
//
// Who       When        What
// --------  ----------  -------------------------------------------------------
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

#define CAN_PROTOCOL_VERSION 1

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
#if (CAN_PROTOCOL_VERSION == 1)
    // In version 2, two are covered by the ping command, which also
    // reports the current positions.
    CCMD_GET_STEPS_ALPHA                  = 4, // get alpha counts
    CCMD_GET_STEPS_BETA                   = 5, // get beta counts
#else
    CCMD_LOCK_UNIT                        = 4, // ignore any command except reset and unlock
    CCMD_UNLOCK_UNIT                      = 5, // listen to commands again
#endif
    CCMD_READ_REGISTER                    = 6, // read register - unused
    CCMD_PING_FPU                         = 7, // check connectivity
    CCMD_RESET_FPU                        = 8, // reset MCU
    CCMD_FIND_DATUM                       = 9, // "automatic" datum search
    CCMD_RESET_STEPCOUNTER                = 10, // only for debugging
    CCMD_REPEAT_MOTION                    = 11, // re-use last waveform
    CCMD_REVERSE_MOTION                   = 12, // invert last waveform
    CCMD_ENABLE_BETA_COLLISION_PROTECTION = 13, // "ENABLE_COLLIDE"
    CCMD_FREE_BETA_COLLISION              = 14, // "FREE_COLLIDE"
    CCMD_SET_USTEP_LEVEL                  = 15, // set stepper motorf232 micro-stepping level (1,2,4,8 supported)

#if (CAN_PROTOCOL_VERSION == 1)
    // the next two are combined in version 2
    CCMD_GET_ERROR_ALPHA                  = 16, // get residue alpha count at last datum hit
    CCMD_GET_ERROR_BETA                   = 17, // get residue beta count at last datum hit
    CCMD_READ_SERIAL_NUMBER               = 18, // read serial number from NVRAM
    CCMD_WRITE_SERIAL_NUMBER              = 19, // write serial number to NVRAM

    NUM_CAN_COMMANDS = 20,
#else
    // commands which are not yet implemented
    CCMD_GET_COUNTER_DEVIATION            = 16, // get alpha and beta residue count
    CCMD_GET_FIRMWARE_VERSION             = 17, // get firmware version
    CCMD_CHECK_INTEGRITY                  = 18, // report firmware CRC
    CCMD_FREE_ALPHA_LIMIT_BREACH          = 19, // untangle alpha arm
    CCMD_ENABLE_ALPHA_LIMIT_PROTECTION    = 20, // re-enable limit switch
    CCMD_SET_TICKS_PER_SEGMENT            = 21, // set movement time interval
    CCMD_SET_STEPS_PER_SEGMENT            = 22, // set minimum step frequency
    CCMD_ENABLE_MOVE                      = 23, // leave aborted state
    CCMD_READ_SERIAL_NUMBER               = 24, // read serial number from NVRAM
    CCMD_WRITE_SERIAL_NUMBER              = 25, // write serial number to NVRAM

    NUM_CAN_COMMANDS = 26,

#endif


    /***************************************/
    /* FPU warning messages */
#if  (CAN_PROTOCOL_VERSION == 1)
    /* code 101 unused */
    /* code 102 unused */
    CMSG_FINISHED_MOTION     = 103, // executeMotion finished
    CMSG_FINISHED_DATUM      = 104, // findDatum finished
    CMSG_WARN_COLLISION_BETA = 105, // collision at beta arm
    CMSG_WARN_LIMIT_ALPHA    = 106, // limit switch at alpha arm
    CMSG_WARN_RACE           = 107, // step timing error
#else
    CMSG_FINISHED_MOTION     = 26, // executeMotion finished
    CMSG_FINISHED_DATUM      = 27, // findDatum finished
    CMSG_WARN_COLLISION_BETA = 28, // collision at beta arm
    CMSG_WARN_LIMIT_ALPHA    = 29, // limit switch at alpha arm
    CMSG_WARN_TIMEOUT_DATUM  = 30, // datum search time out
#endif

};


#if CAN_PROTOCOL_VERSION == 1
/* Error Codes from FPU response messages (note some codes are obsolete
   or only used internally */
enum E_MOC_ERRCODE
{
    ER_OK             = 0x00,   // no error
    ER_STALLX         = 0x01,   // x motor stall (obsolete)
    ER_STALLY         = 0x02,   // y motor stall (obsolete)
    ER_COLLIDE        = 0x03,   // FPU collision detected
    ER_INVALID        = 0x04,   // received command not valid
    ER_WAVENRDY       = 0x05,   // waveform not ready
    ER_WAVE2BIG       = 0x06,   // waveform exceeds memory allocation
    ER_TIMING         = 0x07,   // step timing error (interrupt race condition)
    ER_M1LIMIT        = 0x08,   // M1 Limit switch breached
    ER_M2LIMIT        = 0x09,   // no longer used
    ER_PARAM          = 0x10,   // parameter out of range
    ER_AUTO           = 0x11,   // FPU cannot datum automatically
    ER_OK_UNCONFIRMED = 0x12,   // command will not be confirmed if OK
    ER_TIMEDOUT       = 0x13,   // command hit time-out
};


/* Status bits in FPU response message (many only used
   internally in the controller) */

enum E_FPU_STATUS_BITS
{
    STBT_MSGRCV       = 1,      // message received over CANBUS
    STBT_WAVE_READY   = (1 << 1), // waveform good and ready for execution
    STBT_EXECUTE_WAVE = (1 << 2), // internal start flag to start executing waveform
    STBT_RUNNING_WAVE = (1 << 3), // FPU is running the waveform
    STBT_ABORT_WAVE   = (1 << 4), // abort waveform
    STBT_M1LIMIT      = (1 << 5), // M1 Limit breached
    STBT_M2LIMIT      = (1 << 6), // no longer used
    STBT_REVERSE_WAVE = (1 << 7), // waveform to be run in reverse
};

#endif

enum E_DATUM_SKIP_FLAG
{
    DATUM_SKIP_ALPHA = (1 << 0),
    DATUM_SKIP_BETA = (1 << 1),
};

enum E_DATUM_MODE_FLAG
{
    MODE_DATUM_AUTO = (1 << 2),
    MODE_DATUM_ANTI_CLOCKWISE = (1 << 3),
};



// this uses the unsigned specifier because
// the representation of signed bitfields is
// implementation-defined
typedef struct t_fpu_status_flags
{
#if (CAN_PROTOCOL_VERSION == 1)
    unsigned int message_received : 1; // unused
    unsigned int waveform_ready : 1;
    unsigned int _internal_execute_wave : 1; // only internally used
    unsigned int running_wave : 1; // waveform is being executed
    unsigned int abort_wave : 1;
    unsigned int alpha_limit_active;
    unsigned int _unused_m2limit_switch_active: 1; // no longer used
    unsigned int waveform_reversed : 1;
#else
    unsigned int alpha_datum_switch_active : 1;
    unsigned int beta_datum_switch_active : 1;
    unsigned int beta_collision_detected : 1;
    unsigned int alpha_limitswitch_active : 1;
    unsigned int is_locked : 1;
    unsigned int dir_alpha : 1;
    unsigned int dir_beta : 1;
    unsigned int was_initialized : 1;
    unsigned int waveform_valid : 1;
    unsigned int waveform_ready : 1;
    unsigned int waveform_reversed : 1; // 0 means anti-clockwise for positive step numbers
#endif
} t_fpu_status_flags;


// defines 4-bit priority value of CAN message
inline uint8_t getMessagePriority(const E_CAN_COMMAND cmd)
{

#if (CAN_PROTOCOL_VERSION == 1)
    // protocol version 1 requires a priority of zero.
    return 0;
#endif

    uint8_t priority = 0x0f;
    switch (cmd)
    {
    /* highest priority has smallest code */

    /* priorities 0x00 and 0x01 are reserved for
       FPU warning messages and command responses. */

    /* used for emergency stop, usually broadcast */
    case CCMD_ABORT_MOTION                       :
    /* movement commands, usually broadcast */
    case CCMD_EXECUTE_MOTION                     :
    case CCMD_FIND_DATUM                         :
        priority = 0x02;
        break;

    /* motion configuration */
    case CCMD_CONFIG_MOTION                      :
    /* error recovery */
    case CCMD_RESET_FPU                          :
    case CCMD_RESET_STEPCOUNTER                  :
    case CCMD_ENABLE_BETA_COLLISION_PROTECTION   :
    case CCMD_FREE_BETA_COLLISION                :
#if CAN_PROTOCOL_VERSION == 1
    case CCMD_GET_STEPS_ALPHA                    :
    case CCMD_GET_STEPS_BETA                     :
    case CCMD_GET_ERROR_ALPHA                    :
    case CCMD_GET_ERROR_BETA                     :
#else
    case CCMD_FREE_ALPHA_LIMIT_BREACH            :
    case CCMD_ENABLE_ALPHA_LIMIT_PROTECTION      :
    case CCMD_LOCK_UNIT                          :
    case CCMD_UNLOCK_UNIT                        :
    case CCMD_CHECK_INTEGRITY                    :
    case CCMD_GET_FIRMWARE_VERSION               :
    case CCMD_ENABLE_MOVE                        :
#endif
    case CCMD_REPEAT_MOTION                      :
    case CCMD_REVERSE_MOTION                     :
        priority = 0x03;
        break;

    /* status inquiry */
    case CCMD_PING_FPU                           :
#if CAN_PROTOCOL_VERSION != 1
    case CCMD_SET_TICKS_PER_SEGMENT              :
    case CCMD_SET_STEPS_PER_SEGMENT              :
#endif
    case CCMD_SET_USTEP_LEVEL                    :
        priority = 0x05;
        break;

    // invalid cases
    case CCMD_NO_COMMAND                         :
    default:
        assert(false);

    }
    return priority;
}

}

} // end of namespace

#endif
