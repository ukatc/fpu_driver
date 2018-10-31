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
// NAME FPUState.h
//
// This class implements the low-level CAN driver for the MOONS fiber
// positioner grid
//
////////////////////////////////////////////////////////////////////////////////

#ifndef FPU_STATE_H
#define FPU_STATE_H

#include "ethercan/E_CAN_COMMAND.h"
#include "ethercan/CAN_Constants.h"
#include <time.h>

namespace mpifps
{

using ethercanif::E_CAN_COMMAND;
using ethercanif::E_MOC_ERRCODE;
using ethercanif::NUM_CAN_COMMANDS;

enum E_FPU_STATE
{
    FPST_UNKNOWN                 = 0,
    FPST_UNINITIALIZED           = 1,
    FPST_LOCKED                  = 2,
    FPST_DATUM_SEARCH            = 3,
    FPST_AT_DATUM                = 4,
    FPST_LOADING                 = 5,
    FPST_READY_FORWARD           = 6,
    FPST_READY_REVERSE          = 7,
    FPST_MOVING                  = 8,
    FPST_RESTING                 = 9,
    FPST_ABORTED                 = 10,
    FPST_OBSTACLE_ERROR          = 11,

    NUM_FPU_STATES               = 12,

};

// known direction of movement
// (keep in mind that the command parameter
// is differently encoded)
enum E_MOVEMENT_DIRECTION
{
    DIRST_UNKNOWN          = 0,
    DIRST_ANTI_CLOCKWISE   = 3, // we could also use WYDDERSHINS here
    DIRST_CLOCKWISE        = 4, // and "DEOSIL" here which is from Gaelic
    DIRST_RESTING_LAST_CW  = 5, // might or might not be needed
    DIRST_RESTING_LAST_ACW = 6, // might or might not be needed
};

const int MAX_NUM_TIMEOUTS = 4;


typedef struct __attribute__((packed)) tout_entry
{
    timespec tout_val;
    uint8_t cmd_code;
    uint8_t sequence_number;
} tout_entry;

// length of serial number string in state structure
const int LEN_SERIAL_NUMBER = (ethercanif::DIGITS_SERIAL_NUMBER + 1);

typedef struct __attribute__((packed)) t_fpu_state
{
    // time when any running command is considered timed out
    // Note: this time needs to use the monotonic linux system
    // clock so that leap seconds don't trigger bugs.
    tout_entry cmd_timeouts[MAX_NUM_TIMEOUTS];
    // this uses the monotonic system time (roughly, seconds since booting)
    timespec last_updated;
    // zero-terminated serial number of FPU, stored in controller NVRAM
    char serial_number[LEN_SERIAL_NUMBER];
    // set of any still running and incomplete commands
    uint32_t pending_command_set;
    uint8_t firmware_version[3];
    uint8_t firmware_date[3];

    // current state of FPU
    E_FPU_STATE state;
    // previous state of FPU (used for time-out handling etc.)
    E_FPU_STATE previous_state;
    // id of last command that was issued
    E_CAN_COMMAND last_command;
    // motion controller status response for last command
    E_MOC_ERRCODE last_status; /* note this is very low-level
                                  information which should only be used
                                  by the CAN driver */

    // these members are the individual values
    // reported by FPU responses
    int alpha_steps;
    int beta_steps;
    int alpha_deviation;
    int beta_deviation;
    uint32_t crc32;
    // Wrapping number of minor time-outs which have been observed.
    uint16_t timeout_count;
    // Wrapping count of step timing errors observed for this FPU.
    // These are caused by a problem in the FPU firmware which can
    // occur at higher microstepping levels, if the time is not long
    // enough for the microcontroller to compute the step frequency.
    uint16_t step_timing_errcount;
    uint16_t can_overflow_errcount;
    E_MOVEMENT_DIRECTION direction_alpha;
    E_MOVEMENT_DIRECTION direction_beta;
    int8_t num_active_timeouts;
    uint16_t register_address;
    uint8_t register_value;  // single-byte response value for readRegister command
    uint8_t sequence_number; // number of last pending / received command
    unsigned int num_waveform_segments: 9; /* number of loaded waveform segements */
    unsigned int alpha_was_zeroed: 1; /* alpha steps are validly calibrated by
                                         finding datum.  This is required
                                         for any science observations. */
    unsigned int beta_was_zeroed: 1; /* beta steps are validly calibrated by
                                        finding datum.  This is required
                                        for any science observations. */
    unsigned int is_locked: 1;  // FPU was locked by operator
    unsigned int ping_ok: 1;  // last ping command was successful
    unsigned int movement_complete: 1;  // last movement command was completed successfully.
    unsigned int alpha_datum_switch_active: 1; // alpha datum switch is on
    unsigned int beta_datum_switch_active: 1; // beta datum switch is on
    unsigned int at_alpha_limit: 1; // alpha arm has reached limit (detected by datum off)
    unsigned int beta_collision: 1;
    unsigned int waveform_valid: 1; /* waveform completely loaded, can be reversed, and is not
                                       invalidated by collision or abort message. */

    unsigned int waveform_ready: 1; // FPU can execute waveform
    unsigned int waveform_reversed: 1; // false means anti-clockwise for positive step numbers



} t_fpu_state;


namespace ethercanif
{

void initialize_fpu(t_fpu_state &fpu);

}

}


#endif

