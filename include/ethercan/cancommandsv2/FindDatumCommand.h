
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
// NAME MoveDatumOnCommand.h
//
// This class implements the low-level CAN driver for the MOONS fiber
// positioner grid
//
//////////////////////////////////////////////////////////////////////////
//////

#ifndef MOVE_DATUM_COMMAND_H
#define MOVE_DATUM_COMMAND_H

#include <string.h>
#include <cassert>
#include "../CAN_Command.h"

namespace mpifps
{

namespace ethercanif
{

class FindDatumCommand : public CAN_Command
{

public:

    static const E_CAN_COMMAND command_code = CCMD_FIND_DATUM;

    static E_CAN_COMMAND getCommandCode()
    {
        return command_code;
    };


    FindDatumCommand() : CAN_Command(command_code),
        _arm_selection(DASEL_BOTH),
        _search_mode(SKIP_FPU),
        _timeout_flag(DATUM_TIMEOUT_ENABLE)
    {
    };

    ~FindDatumCommand() {};


    void parametrize(int f_id, bool broadcast,
                     E_DATUM_SEARCH_DIRECTION search_mode, E_DATUM_SELECTION arm_selection,
                     E_DATUM_TIMEOUT_FLAG timeout_flag)
    {
        fpu_id = f_id;
        bcast = broadcast;
        _search_mode = search_mode;
        _arm_selection = arm_selection;
        _timeout_flag = timeout_flag;
    };


    void SerializeToBuffer(const uint8_t busid,
                           const uint8_t fpu_canid,
                           int& buf_len,
                           t_CAN_buffer& can_buffer,
                           const uint8_t _sequence_number)
    {

        set_msg_header(can_buffer, buf_len, busid, fpu_canid, bcast, _sequence_number);

        bool skip_alpha = false;
        bool skip_beta = false;
        switch (_arm_selection)
        {
        case DASEL_BOTH:
            break;
        case DASEL_ALPHA:
            skip_beta = true;
            break;
        case DASEL_BETA:
            skip_alpha = true;
            break;
        case DASEL_NONE:
            skip_alpha = true;
            skip_beta = true;
            break;

        default:
            // Invalid values have been filtered out by
            // AsyncDriver, so this can't happen.
            assert(0);
        }

        bool _auto_datum = false;
        bool _anti_clockwise = false;
        switch (_search_mode)
        {
        case SEARCH_CLOCKWISE:
            _anti_clockwise = false;
            _auto_datum = false;
            break;

        case SEARCH_ANTI_CLOCKWISE:
            _anti_clockwise = true;
            _auto_datum = false;
            break;

        case SEARCH_AUTO          :
            _anti_clockwise = false;
            _auto_datum = true;
            break;

        // must not happen
        default:
        case SKIP_FPU             :
            assert(false);
            break;
        }
        switch (_timeout_flag)
        {
        case DATUM_TIMEOUT_ENABLE:
        case DATUM_TIMEOUT_DISABLE:
            break;
        default:
            assert(false);
        }

        const uint8_t flags = ( (skip_alpha ? DATUM_SKIP_ALPHA : 0)
                                | (skip_beta ? DATUM_SKIP_BETA : 0)
                                | (_auto_datum ? MODE_DATUM_AUTO : 0)
                                | (_anti_clockwise ? MODE_DATUM_ANTI_CLOCKWISE : 0)
                                | _timeout_flag);

        can_buffer.message.data[2] = flags;
        buf_len += 1;

    };


    // time-out period for a response to the message
    timespec getTimeOut()
    {
        // Largest possible waiting time for a working datum
        // search is 35 seconds.
        const struct timespec  toval =
        {
            /* .tv_sec = */ 60,
            /* .tv_nsec = */ 0
        };

        return toval;
    };

private:
    E_DATUM_SELECTION _arm_selection;
    E_DATUM_SEARCH_DIRECTION _search_mode;
    E_DATUM_TIMEOUT_FLAG _timeout_flag;

};

}

}
#endif
