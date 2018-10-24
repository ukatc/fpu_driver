
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
// NAME ConfigureMotionCommand.h
//
// This class implements the low-level CAN driver for the MOONS fiber
// positioner grid
//
////////////////////////////////////////////////////////////////////////////////

#ifndef CONFIGURE_MOTION_COMMAND_H
#define CONFIGURE_MOTION_COMMAND_H

#include <cassert>

#include <string.h>

#include "../CAN_Command.h"

namespace mpifps
{

namespace ethercanif
{

class ConfigureMotionCommand : public CAN_Command
{

public:

    // maximum number of sections the FPU can store
    static const unsigned int MAX_NUM_SECTIONS=256;

    static const E_CAN_COMMAND command_code = CCMD_CONFIG_MOTION;
    
    static E_CAN_COMMAND getCommandCode()
    {
        return command_code;
    };
    
    ConfigureMotionCommand() : CAN_Command(command_code)
    {
        asteps = 0;
        bsteps = 0;
        apause = false;
        bpause = false;
        aclockwise = false;
        bclockwise = false;
        fentry = false;
        lentry = false;
        confirm = true;
    };

    void parametrize(int f_id,
                     int16_t alpha_steps,
                     int16_t beta_steps,
                     bool first_entry, bool last_entry,
                     const int MIN_STEPCOUNT,
                     bool do_confirm)
    {
        fpu_id = f_id;


        // assert precondition of 14-bit step size
        const int abs_alpha_steps = abs(alpha_steps);
        const int abs_beta_steps = abs(beta_steps);

        assert( (abs_alpha_steps >> 14) == 0);
        assert( (abs_beta_steps >> 14) == 0);

        const bool alpha_pause = (abs_alpha_steps == 0);
        const int alpha_scount = (alpha_pause
                                  ? MIN_STEPCOUNT
                                  : abs_alpha_steps);

        const bool alpha_clockwise = (alpha_steps < 0);
        const bool beta_pause = (abs_beta_steps == 0);
        const int beta_scount = (beta_pause
                                 ? MIN_STEPCOUNT
                                 : abs_beta_steps);

        const bool beta_clockwise = (beta_steps < 0);


        asteps = alpha_scount;
        bsteps = beta_scount;
        apause = alpha_pause;
        aclockwise = alpha_clockwise;
        bpause = beta_pause;
        bclockwise = beta_clockwise;
        fentry = first_entry;
        lentry = last_entry;
        confirm = do_confirm;
    };


    // the internal member fpu_id is the logical number of the
    // fpu in the grid.
    // Busid is the bus number the command should be sent to.
    // canid is the id if the FPU on that bus.
    void SerializeToBuffer(const uint8_t busid,
                           const uint8_t fpu_canid,
                           int& buf_len,
                           t_CAN_buffer& can_buffer,
			   const uint8_t sequence_number)
    {

	set_msg_header(can_buffer, buf_len, busid, fpu_canid, bcast, sequence_number);



        // flags for first and last entry
        can_buffer.message.data[2] = ( (fentry ? 1 : 0)
                                       | ((lentry ? 1 : 0) << 1)
				       | ((confirm ? 1 : 0) << 2)
				     );
        // alpha and beta steps
        // NOTE: tx2 and tx3, and tx4 and tx5 had been swapped
        // to work around a bug in the firmware. This is fixed in v2.
        can_buffer.message.data[3] = 0xff & asteps;
        can_buffer.message.data[4] = (0x3f & (asteps >> 8))
                                     | ((apause ? 1 : 0) << 6)
                                     | ((aclockwise ? 1 : 0) << 7);

        can_buffer.message.data[5] = 0xff & bsteps;
        can_buffer.message.data[6] = (0x3f & (bsteps >> 8))
                                     | ((bpause ? 1 : 0) << 6)
                                     | ((bclockwise ? 1 : 0) << 7);

        buf_len += 5;


    };



    // boolean value indicating whether
    // the driver should wait for a response
    bool expectsResponse()
    {
        // send response if this is the first or last entry
        return confirm;
    };

    // time-out period for a response to the message
    timespec getTimeOut()
    {
        const struct timespec  toval =
        {
            /* .tv_sec = */ 10,
            /* .tv_nsec = */ 0
        };

        return toval;
    };

    bool doBroadcast()
    {
        return false;
    }

private:
    int16_t asteps;
    int16_t bsteps;
    bool apause;
    bool bpause;
    bool aclockwise;
    bool bclockwise;
    bool fentry;
    bool lentry;
    bool confirm;


};

}

}
#endif
