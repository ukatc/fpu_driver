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
// NAME FPUState.cpp
//
// This class implements initialization of the FPU state description.
//
////////////////////////////////////////////////////////////////////////////////


#include "FPUState.h"

#include "ethercan/E_CAN_COMMAND.h"
#include "ethercan/TimeOutList.h"
#include <string.h>
#ifdef DEBUG
#include <stdio.h>
#endif

namespace mpifps
{

namespace ethercanif
{


void initialize_fpu(t_fpu_state &fpu)
{
    fpu.alpha_was_zeroed          = false;
    fpu.beta_was_zeroed           = false;
    fpu.is_locked                 = false;
    fpu.state                     = FPST_UNKNOWN;
    fpu.previous_state            = FPST_UNKNOWN;
    fpu.pending_command_set       = 0;
    for(int i=0; i < MAX_NUM_TIMEOUTS; i++)
    {
        fpu.cmd_timeouts[i].tout_val       = TimeOutList::MAX_TIMESPEC;
        fpu.cmd_timeouts[i].cmd_code       = CCMD_NO_COMMAND;
    }
    fpu.num_active_timeouts = 0;
    fpu.last_updated.tv_sec       = 0;
    fpu.last_updated.tv_nsec      = 0;
    fpu.timeout_count             = 0;
    fpu.step_timing_errcount      = 0;
    fpu.last_command              = CCMD_NO_COMMAND;
    fpu.last_status               = MCE_FPU_OK;
    fpu.sequence_number           = 0;
    fpu.can_overflow_errcount     = 0;
    fpu.crc32                     = 0;
    fpu.register_address          = 0;
    fpu.register_value            = 0;
    // the values below are not valid, they need proper
    // initialization from a physical fpu response.
    fpu.alpha_steps               = 0;
    fpu.beta_steps                = 0;
    fpu.alpha_datum_switch_active = false;
    fpu.beta_datum_switch_active  = false;
    fpu.at_alpha_limit            = false;
    fpu.beta_collision            = false;
    fpu.direction_alpha           = DIRST_UNKNOWN;
    fpu.direction_beta            = DIRST_UNKNOWN;
    fpu.num_waveform_segments     = 0;
    fpu.ping_ok                   = false;
    fpu.movement_complete         = false;
    fpu.waveform_valid            = false;
    fpu.waveform_ready            = false;
    fpu.waveform_reversed         = false;
    fpu.checksum_ok               = false;

    memset(&(fpu.firmware_version), FIRMWARE_NOT_RETRIEVED, sizeof(fpu.firmware_version));
    memset(&(fpu.firmware_date), 0, sizeof(fpu.firmware_version));
    strcpy(fpu.serial_number,"@@@@@\0");

}

}

}
