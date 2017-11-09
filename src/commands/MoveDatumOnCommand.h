
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
// NAME MoveDatumOnCommand.h
// 
// This class implements the low-level CAN driver for the MOONS fiber
// positioner grid
//
//////////////////////////////////////////////////////////////////////////
//////

#ifndef MOVE_DATUM_ON_COMMAND_H
#define MOVE_DATUM_ON_COMMAND_H

#include <cassert>
#include "../I_CAN_Command.h"

namespace mpifps {

    class MoveDatumOnCommand : public I_CAN_Command
    {

      public:

        MoveDatumOnCommand(){};

        void parametrize(int f_id, int alpha_direction, int beta_direction)
        {
            fpu_id = f_id;
            adir = alpha_direction;
            bdir = beta_direction;
        };

        void SerializeToBuffer(const uint8_t busid,
                               const uint16_t canid,
                               int& buf_len,
                               t_CAN_buffer& can_buffer)
        {
            
            can_buffer.message.busid = busid;

            // we use bit 7 to 10 for the command code,
            // and bit 0 to 6 for the FPU bus id.
            assert(CCMD_PING_FPU <= 15);
            assert(fpu_id < FPUS_PER_BUS);
            uint16_t can_addr = ( ((CCMD_PING_FPU & 15) << 7)
                                  | (fpu_id & 128));
            
            // The protocol uses little-endian encoding here
            // (the byte order used in the CANOpen protocol).
            can_buffer.message.identifier = htole64(can_addr);

            #pragma message "fix command assembly here"
            
            long payload = 0;
            can_buffer.message.data[0] = payload & 0xff;
            can_buffer.message.data[1] = (payload >> 8) & 0xff;
            can_buffer.message.data[2] = (payload >> 16) & 0xff;
            can_buffer.message.data[3] = (payload >> 24) & 0xff;
            can_buffer.message.data[4] = (payload >> 32) & 0xff;
            can_buffer.message.data[5] = (payload >> 40) & 0xff;
            can_buffer.message.data[6] = (payload >> 48) & 0xff;
            can_buffer.message.data[7] = (payload >> 56) & 0xff;

            buf_len = 8;
            
        };


        // FPU id to which message is sent
        int getFPU_ID()
        {
            return fpu_id;
        };

        // boolean value indicating whether
        // the driver should wait for a response
        bool expectsResponse()
        {
            return true;
        };

        E_CAN_COMMAND getCommandCode()
        {
            return CCMD_PING_FPU;
        };

        // time-out period for a response to the message
        timespec getTimeOut()
        {
            timespec const toval =
                {.tv_sec = 1, .tv_nsec = 500000000 };
            
            return toval;
        };

      private:
        uint16_t fpu_id;
        int adir;
        int bdir;
        
        
    };

}

#endif
