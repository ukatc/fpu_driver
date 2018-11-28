// -*- mode: c++ -*-

////////////////////////////////////////////////////////////////////////////////
//
// Copyright 2017 UKRI. See file "LICENSE" for license information.
//
// ESO - VLT Project
//
// Who       When        What
// --------  ----------  -------------------------------------------------------
// jnix      2017-10-18  Created driver class using Pablo Guiterrez' CAN client sample
//------------------------------------------------------------------------------

////////////////////////////////////////////////////////////////////////////////
// NAME FPUState.cpp
//
// This class implements the low-level CAN driver for the MOONS fiber
// positioner grid
//
////////////////////////////////////////////////////////////////////////////////


#include <cassert>


#include "ethercan/decode_CAN_response.h"
#include "ethercan/time_utils.h"

namespace mpifps
{

namespace ethercanif
{


// logs error status in CAN response
void logErrorStatus(const EtherCANInterfaceConfig &config, int fpu_id, int err_code)
{

    const char * err_msg = "(no error)";
    switch (err_code)
    {

    case MCE_FPU_OK:
        err_msg = "no error";
        break;

    case MCE_WARN_COLLISION_DETECTED:
        err_msg = "FPU beta collision detected";
        break;

    case MCE_ERR_INVALID_COMMAND:
        err_msg = "invalid command received by motion controller";
        break;

    case MCE_ERR_WAVEFORM_NOT_READY:
        err_msg = "waveform not ready for execution";
        break;

    case MCE_WARN_STEP_TIMING_ERROR:
        err_msg = "step timing error:microstepping value is too high for step frequency";
        break;

    case MCE_WARN_LIMIT_SWITCH_BREACH:
        err_msg = "alpha limit switch breach";
        break;

    case MCE_ERR_INVALID_PARAMETER:
        err_msg = "invalid parameter was rejected by motion controller";
        break;

    case MCE_ERR_DATUM_TIME_OUT:
        err_msg = "datum search exceeded hardware time or step limit";
        break;

    case MCE_NOTIFY_DATUM_ALPHA_ONLY:
        err_msg = "only the alpha arm was moved to datum";
        break;

    case MCE_NOTIFY_DATUM_BETA_ONLY:
        err_msg = "only the beta arm was moved to datum";
        break;

    case MCE_ERR_AUTO_DATUM_UNINITIALIZED:
        err_msg = "automatic datum operation was requested, but FPU is not initialized";
        break;

    case MCE_ERR_DATUM_ON_LIMIT_SWITCH:
        err_msg = "datum command was rejected because alpha arm is on limit switch";
        break;

    case MCE_ERR_CAN_OVERFLOW_HW:
        err_msg = "overflow in CAN hardware buffer";
        break;

    case MCE_ERR_CAN_OVERFLOW_SW:
        err_msg = "CAN overflow in motion controller firmware buffer";
        break;

    case MCE_NOTIFY_COMMAND_IGNORED:
        err_msg = "command was ignored by FPU motion controller";
        break;

    default:
        err_msg = "obsolete error code received";
        break;

    }

    LOG_RX(LOG_DEBUG, "%18.6f:FPU #%04i:error response msg = %s\n",
           get_realtime(),
           fpu_id, err_msg);

}



// Takes an unsiged 16-bit value.
// Decodes step count as a signed 16-bit value
// with an asymmetric range.
int unfold_stepcount_alpha(const uint16_t step_count)
{
    int val = static_cast<int>(step_count);
    const int lower_limit = -10000;
    const int upper_limit = lower_limit + (1 << 16) -1;
    if (val > upper_limit)
    {
        val -= (1 << 16);
    }
    return val;

}

// Takes an unsiged 16-bit value.
// Decodes step count as a signed 16-bit
// value with an symmetric range.
int unfold_stepcount_beta(const uint16_t step_count)
{
    int val = static_cast<int>(step_count);
    const int upper_limit = (1 << 15) -1;
    if (val > upper_limit)
    {
        val -= (1 << 16);
    }
    return val;

}

// Converts the response value for a datum search
// deviation into an integer. This is valid for both
// the alpha and the beta arm.
int unfold_steps_deviation(const uint16_t step_count)
{
    int val = static_cast<int>(step_count);
    if (val >= (1 << 15))
    {
        val -= (1 << 16);
    }
    return val;

}


/* this function takes the header part of a CAN response
and updates the status for this FPU according to the status data.
Because some commands have so much payload that they cannot
transmit all status data, the function takes
a bitmask parameter which tells which fields should be updated.

The bitmask depends on the command code. Normally, all fields
are updated.
*/

E_MOC_ERRCODE update_status_flags(t_fpu_state& fpu,
                                  const UPDATE_FIELDID req_fields,
                                  const t_response_buf& data)
{
    E_MOC_ERRCODE err_code = MCE_FPU_OK;

    fpu.last_command = static_cast<E_CAN_COMMAND>(data[1] & 0x1f);

    // store new state of FPU.
    if ((UPDATE_STATE & req_fields) != 0)
    {
        fpu.state = static_cast<E_FPU_STATE>(data[3] & 0x0F);
    }


    // update fpu status bits from transmitted status word
    {
        // assemble status word
        uint32_t stwd = ((data[1] >> 5) & 0x07) | (data[2] << 3);

        if ((UPDATE_STSWD & req_fields) != 0)
        {

            fpu.waveform_ready =       test_bit(stwd, STBT_WAVEFORM_READY);
            fpu.at_alpha_limit =       test_bit(stwd, STBT_ALPHA_AT_LIMIT);
            fpu.waveform_reversed =    test_bit(stwd, STBT_WAVEFORM_REVERSED);

            fpu.alpha_was_zeroed = fpu.alpha_was_zeroed || test_bit(stwd, STBT_IS_ZEROED);
            fpu.beta_was_zeroed = fpu.beta_was_zeroed || test_bit(stwd, STBT_IS_ZEROED);
            fpu.is_locked = test_bit(stwd, STBT_FPU_LOCKED);
            fpu.alpha_datum_switch_active = test_bit(stwd, STBT_ALPHA_DATUM_ACTIVE);
            fpu.beta_datum_switch_active = test_bit(stwd, STBT_BETA_DATUM_ACTIVE);
            fpu.beta_collision = test_bit(stwd, STBT_COLLISION_DETECTED);
            fpu.waveform_valid = test_bit(stwd, STBT_WAVEFORM_VALID);


            // set the direction enum according to whether the FPU is in moving state or not
            const bool ismoving = ( (fpu.state == FPST_DATUM_SEARCH) || (fpu.state == FPST_MOVING) );

            if (ismoving)
            {
                fpu.direction_alpha = test_bit(stwd, STBT_ALPHA_LAST_DIRECTION) == 1 ? DIRST_CLOCKWISE : DIRST_ANTI_CLOCKWISE;
                fpu.direction_beta = test_bit(stwd, STBT_BETA_LAST_DIRECTION) == 1 ? DIRST_CLOCKWISE : DIRST_ANTI_CLOCKWISE;
            }
            else
            {
                fpu.direction_alpha = test_bit(stwd, STBT_ALPHA_LAST_DIRECTION) == 1 ? DIRST_RESTING_LAST_CW : DIRST_RESTING_LAST_ACW;
                fpu.direction_beta = test_bit(stwd, STBT_BETA_LAST_DIRECTION) == 1 ? DIRST_RESTING_LAST_CW : DIRST_RESTING_LAST_ACW;
            }

            if ( test_bit(stwd, STBT_WAVEFORM_VALID) == 0)
            {
                fpu.num_waveform_segments = 0;
            }


        }
    }


    // store error status of command
    if ((UPDATE_ECODE & req_fields) != 0)
    {
        err_code = static_cast<E_MOC_ERRCODE>((data[3] & 0xF0) >> 4);
        fpu.last_status = err_code;
    }

    // update step counts, if they were transmitted
    if ((UPDATE_STEPS & req_fields) != 0)
    {

        // the finished_DATUM commands does not report the current step counts
        // (at datum, they are zero), bit the residual step count when the datum
        // position was reached. We need to check whether alpha or beta arm
        // were datumed separately.
        if ((fpu.last_command != CMSG_FINISHED_DATUM) || (err_code == MCE_NOTIFY_DATUM_ALPHA_ONLY))
        {
            fpu.alpha_steps = unfold_stepcount_alpha(data[4] | (data[5] << 8));
        }
        else
        {
            fpu.alpha_deviation = unfold_stepcount_alpha(data[4] | (data[5] << 8));
	    fpu.alpha_steps = 0;
        }
        if ((fpu.last_command != CMSG_FINISHED_DATUM) || (err_code == MCE_NOTIFY_DATUM_BETA_ONLY))
        {
            fpu.beta_steps = unfold_stepcount_beta(data[6] | (data[7] << 8));
        }
        else
        {
            fpu.beta_deviation = unfold_stepcount_beta(data[6] | (data[7] << 8));
	    fpu.beta_steps = 0;
        }
	fpu.ping_ok = int(true);
    }


    return err_code;


}

E_MOVEMENT_DIRECTION update_direction_stopping(E_MOVEMENT_DIRECTION last_direction)
{
    switch (last_direction)
    {

    case DIRST_CLOCKWISE:
        return DIRST_RESTING_LAST_CW;
    /* no break needed */

    case DIRST_ANTI_CLOCKWISE:
        return DIRST_RESTING_LAST_ACW;
    /* no break needed */

    case DIRST_UNKNOWN:
    default:
        return last_direction;
        /* no break needed */

    }
}

}
}
