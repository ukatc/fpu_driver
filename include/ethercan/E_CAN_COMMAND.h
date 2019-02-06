
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

#define CAN_PROTOCOL_VERSION 2

namespace mpifps
{

namespace ethercanif
{

enum E_CAN_COMMAND : uint8_t
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
    CCMD_FIND_DATUM                       = 9, // datum search
    CCMD_RESET_STEPCOUNTER                = 10, // only for debugging
    CCMD_REPEAT_MOTION                    = 11, // re-use last waveform
    CCMD_REVERSE_MOTION                   = 12, // invert last waveform
    CCMD_ENABLE_BETA_COLLISION_PROTECTION = 13, // "ENABLE_COLLIDE"
    CCMD_FREE_BETA_COLLISION              = 14, // "FREE_COLLIDE"
    CCMD_SET_USTEP_LEVEL                  = 15, // set stepper motor micro-stepping level (1,2,4,8 supported)

#if (CAN_PROTOCOL_VERSION == 1)
    // the next two are combined in version 2
    CCMD_GET_ERROR_ALPHA                  = 16, // get residue alpha count at last datum hit
    CCMD_GET_ERROR_BETA                   = 17, // get residue beta count at last datum hit
    CCMD_READ_SERIAL_NUMBER               = 18, // read serial number from NVRAM
    CCMD_WRITE_SERIAL_NUMBER              = 19, // write serial number to NVRAM

    NUM_CAN_COMMANDS = 20,
#else

    CCMD_GET_FIRMWARE_VERSION             = 16, // get firmware version
    CCMD_CHECK_INTEGRITY                  = 17, // report firmware CRC
    CCMD_FREE_ALPHA_LIMIT_BREACH          = 18, // untangle alpha arm
    CCMD_ENABLE_ALPHA_LIMIT_PROTECTION    = 19, // re-enable limit switch
    CCMD_SET_TICKS_PER_SEGMENT            = 20, // set movement time interval
    CCMD_SET_STEPS_PER_SEGMENT            = 21, // set minimum step frequency
    CCMD_ENABLE_MOVE                      = 22, // leave aborted state
    CCMD_READ_SERIAL_NUMBER               = 23, // read serial number from NVRAM
    CCMD_WRITE_SERIAL_NUMBER              = 24, // write serial number to NVRAM

    NUM_CAN_COMMANDS = 25,

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
    CMSG_WARN_CANOVERFLOW    = 108, // CAN buffer overflow warning
#else
    CMSG_FINISHED_MOTION     = 25, // executeMotion finished
    CMSG_FINISHED_DATUM      = 26, // findDatum finished
    CMSG_WARN_COLLISION_BETA = 27, // collision detected at beta arm
    CMSG_WARN_LIMIT_ALPHA    = 28, // limit switch breach detected at alpha arm
    CMSG_WARN_TIMEOUT_DATUM  = 29, // datum search time-out
    CMSG_WARN_CANOVERFLOW    = 30, // CAN buffer overflow warning
#endif

};

#define COMMAND_CODE_MASK 0x1f

#if CAN_PROTOCOL_VERSION == 1
/* Error Codes from FPU response messages (note some codes are obsolete
   or only used internally */
enum E_MOC_ERRCODE : uint8_t
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
    ER_CANOVRS        = 0x0A,   // can overflow firmware software buffer
    ER_CANOVRH        = 0x0B,   // can overflow FPU hardware buffer
    ER_PARAM          = 0x10,   // parameter out of range
    ER_AUTO           = 0x11,   // FPU step counters not initialized, cannot datum automatically
    ER_DATUMTO        = 0x12,   // hardware error: datum search timed out by firmware
    ER_DATUM_LIMIT    = 0x13,   // datum search denied, limit switch is active
    ER_OK_UNCONFIRMED = 0x14,   // command will not be confirmed if OK
    ER_TIMEDOUT       = 0x15,   // command hit driver time-out
};


#else
enum E_MOC_ERRCODE : uint8_t
{
    MCE_FPU_OK			     = 0x00,	// no error
    MCE_WARN_COLLISION_DETECTED	     = 0x01,	// beta collision warning
    MCE_WARN_LIMIT_SWITCH_BREACH     = 0x02,	// alpha limit switch breach
    MCE_ERR_INVALID_COMMAND	     = 0x03,	// invalid command received by motion controller
    MCE_NOTIFY_COMMAND_IGNORED	     = 0x04,	// command was ignored by FPU motion controller
    MCE_ERR_WAVEFORM_NOT_READY	     = 0x05,	// waveform not ready for execution
    MCE_WAVEFORM_REJECTED	     = 0x06,	// waveform not accepted by FPU, see following enumeration for detailed cause
    MCE_WARN_STEP_TIMING_ERROR	     = 0x07,	// Microstepping value is too high for step frequency
    MCE_ERR_INVALID_PARAMETER	     = 0x08,	// invalid parameter was rejected by motion controller
    MCE_ERR_DATUM_TIME_OUT	     = 0x09,	// datum search exceeded hardware time or step limit
    MCE_NOTIFY_DATUM_ALPHA_ONLY	     = 0x0a,	// only the alpha arm was moved to datum
    MCE_NOTIFY_DATUM_BETA_ONLY	     = 0x0b,	// only the beta arm was moved to datum
    MCE_ERR_AUTO_DATUM_UNINITIALIZED = 0x0c,	// automatic datum operation was requested, but FPU is not initialized
    MCE_ERR_DATUM_ON_LIMIT_SWITCH    = 0x0d,	// datum command was rejected because alpha arm is on limit switch
    MCE_ERR_CAN_OVERFLOW_HW	     = 0x0e,	// overflow in CAN hardware buffer
    MCE_ERR_CAN_OVERFLOW_SW	     = 0x0f,	// CAN overflow in motion controller firmware buffer
    // the following two codes are NEVER sent by the FPU - they are used for driver-internal book-keeping
    MCE_NO_CONFIRMATION_EXPECTED     = 0x10,    // command was sent for which no confirmation is expected
    MCE_COMMAND_TIMEDOUT             = 0x11,    // response for CAN command is missing (timed out)

};

enum E_WAVEFORM_ERRCODE : uint8_t
{
    WAVEFORM_OK		     = 0x00,	// no error
    WAVEFORM_TOO_BIG	     = 0x01,	// too many waveform entries
    WAVEFORM_SEQUENCE	     = 0x02,	// transmitted waveform sequence not consistent in respect to use of first and last flags
    WAVEFORM_BADVALUE	     = 0x03,	// the transmitted waveform value did not pass bounds checking
    WAVEFORM_UNDEFINED       = 0x04,    // no valid waveform data on FPU
};


/* Status bits in FPU response message (many only used
   internally in the controller) */


enum E_FPU_STATUS_BITS
{
    STBT_ALPHA_DATUM_ACTIVE   = 1,	        // alpha datum switch is active
    STBT_BETA_DATUM_ACTIVE    = (1 << 1),	// beta datum switch is active
    STBT_COLLISION_DETECTED   = (1 << 2),	// collision was detected
    STBT_ALPHA_AT_LIMIT	      = (1 << 3),	// alpha limit switch active
    STBT_FPU_LOCKED	      = (1 << 4),	// FPU is currently locked
    STBT_ALPHA_LAST_DIRECTION = (1 << 5),	// last movement direction of alpha arm
    STBT_BETA_LAST_DIRECTION  = (1 << 6),	// last movement direction of beta arm
    STBT_IS_REFERENCED	      = (1 << 7),	// both arms have been datumed
    STBT_WAVEFORM_VALID	      = (1 << 8),	// loaded waveform is valid
    STBT_WAVEFORM_READY	      = (1 << 9),	// ready to run waveform
    STBT_WAVEFORM_REVERSED    = (1 << 10),	// waveform is in reversed mode
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

enum E_DATUM_TIMEOUT_FLAG
{
    DATUM_TIMEOUT_ENABLE  = 0,
    DATUM_TIMEOUT_DISABLE = (1 << 4),
};


 
// defines 4-bit priority value of CAN message
inline uint8_t getMessagePriority(const E_CAN_COMMAND cmd)
{

#if (CAN_PROTOCOL_VERSION == 1)
    // protocol version 1 requires a priority of zero.
    return 0;
#endif

    uint8_t	priority = 0x0f;
    switch (cmd)
    {

    /* highest priority has smallest code */

    /* priorities 0x01 and 0x02 are reserved for
       FPU warning messages and command responses. */

    /* used for emergency stop, usually broadcast with priority zero */
    case CCMD_ABORT_MOTION                       :
    /* movement commands, usually broadcast with priority zero */
    case CCMD_EXECUTE_MOTION                     :
    case CCMD_FIND_DATUM                         :
        priority = 0x03;
        break;

    /* special motion commands */

    case CCMD_LOCK_UNIT                          :
    case CCMD_UNLOCK_UNIT                        :
    case CCMD_FREE_BETA_COLLISION                :
    case CCMD_FREE_ALPHA_LIMIT_BREACH            :
        priority = 0x04;
        break;

    /* error recovery */
    case CCMD_RESET_FPU                          :
    case CCMD_RESET_STEPCOUNTER                  :
    case CCMD_ENABLE_BETA_COLLISION_PROTECTION   :
    case CCMD_ENABLE_ALPHA_LIMIT_PROTECTION      :
    case CCMD_CHECK_INTEGRITY                    :
    case CCMD_ENABLE_MOVE                        :
        priority = 0x05;
        break;

    /* motion configuration */
    case CCMD_CONFIG_MOTION                      :
    case CCMD_REPEAT_MOTION                      :
    case CCMD_REVERSE_MOTION                     :
    /* configuration and status inquiry */
    case CCMD_GET_FIRMWARE_VERSION               :
    case CCMD_READ_REGISTER                    :
    case CCMD_SET_USTEP_LEVEL                  :
    case CCMD_READ_SERIAL_NUMBER               :
    case CCMD_WRITE_SERIAL_NUMBER              :
    case CCMD_PING_FPU                           :
    case CCMD_SET_TICKS_PER_SEGMENT              :
    case CCMD_SET_STEPS_PER_SEGMENT              :
        priority = 0x06;
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
