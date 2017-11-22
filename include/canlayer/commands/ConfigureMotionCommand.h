
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
// NAME ConfigureMotionCommand.h
// 
// This class implements the low-level CAN driver for the MOONS fiber
// positioner grid
//
////////////////////////////////////////////////////////////////////////////////

#ifndef CONFIGURE_MOTION_COMMAND_H
#define CONFIGURE_MOTION_COMMAND_H

#include <cassert>

#include "../I_CAN_Command.h"

namespace mpifps
{

namespace canlayer
{

    class ConfigureMotionCommand : public I_CAN_Command
    {

      public:

        static E_CAN_COMMAND getCommandCode()
        {
            return CCMD_PING_FPU;
        };

        ConfigureMotionCommand(){};

        void parametrize(int f_id, int16_t alpha_steps, int16_t beta_steps, bool first_entry, bool last_entry)
        {
            fpu_id = f_id;
            asteps = alpha_steps;
            bsteps = beta_steps;
            fentry = first_entry;
            lentry = last_entry;                       
        };

        E_CAN_COMMAND getInstanceCommandCode()
        {
            return getCommandCode();
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
                                  | (canid & 128));
            
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

        // time-out period for a response to the message
        timespec getTimeOut()
        {
            const struct timespec  toval =
                {/* .tv_sec = */ 1,
                 /* .tv_nsec = */ 500000000 };
            
            return toval;
        };

      bool doBroadcast()
      {
        return false;
      }

    private:
        uint16_t fpu_id;
        int16_t asteps;
        int16_t bsteps;
        bool fentry;
        bool lentry;
        
        
    };

}

}
#endif
