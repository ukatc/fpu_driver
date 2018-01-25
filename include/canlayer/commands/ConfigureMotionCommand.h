
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

#include <string.h>

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
            return CCMD_CONFIG_MOTION;
        };

        ConfigureMotionCommand(){};

        void parametrize(int f_id,
                         int16_t alpha_steps,
                         bool alpha_pause,
                         bool alpha_clockwise,
                         int16_t beta_steps,
                         bool beta_pause,
                         bool beta_clockwise,
                         bool first_entry, bool last_entry)
        {
            fpu_id = f_id;
            asteps = alpha_steps;
            apause = alpha_pause;
            aclockwise = alpha_clockwise;
            bpause = beta_pause;
            bclockwise = beta_clockwise;            
            bsteps = beta_steps;
            fentry = first_entry;
            lentry = last_entry;                       
        };

        E_CAN_COMMAND getInstanceCommandCode()
        {
            return getCommandCode();
        };


        // the internal member fpu_id is the logical number of the
        // fpu in the grid.
        // Busid is the bus number the command should be sent to.
        // canid is the id if the FPU on that bus.
        void SerializeToBuffer(const uint8_t busid,
                               const uint8_t fpu_canid,
                               int& buf_len,
                               t_CAN_buffer& can_buffer)
        {

            // CAN bus id for that gateway to which message should go
            can_buffer.message.busid = busid;

            // we use bit 7 to 10 for the command code,
            // and bit 0 to 6 for the FPU bus id.
            assert(fpu_canid <= FPUS_PER_BUS);
            assert(fpu_canid > 0);
            

            // the CAN identifier is either all zeros (for a broadcast
            // message) or bits 7 - 10 are the proiority and bits 0 -
            // 6 the CAN id of the FPU.
            const E_CAN_COMMAND cmd_code = getCommandCode();
            
            const uint16_t can_identifier = (getMessagePriority(cmd_code)
                                             << 7) | fpu_canid;
                                   
            // The protocol uses little-endian encoding here
            // (the byte order used in the CANOpen protocol).
            // zero buffer to make sure no spurious DLEs are sent
            bzero(&can_buffer.message, sizeof(can_buffer.message));
            can_buffer.message.identifier = htole64(can_identifier);


            // CAN command code
            can_buffer.message.data[0] = cmd_code;

            // flags for first and last entry
            can_buffer.message.data[1] = ( (fentry ? 1 : 0)
                                           | ((lentry ? 1 : 0) << 1));
            // alpha and beta steps
            can_buffer.message.data[2] = 0xff & asteps;
            can_buffer.message.data[3] = (0x3f & (asteps >> 8))
                                          | ((apause ? 1 : 0) << 6)
                                          | ((aclockwise ? 1 : 0) << 7);
            
            can_buffer.message.data[4] = 0xff & bsteps;
            can_buffer.message.data[5] = (0x3f & (bsteps >> 8))
                                          | ((bpause ? 1 : 0) << 6)
                                          | ((bclockwise ? 1 : 0) << 7);
            
            buf_len = 8;


#ifdef DEBUG
            printf("ConfigureMotion for FPU canid #%i: "
                   "(asteps=%i, apause=%i, acw=%i, bsteps=%i, bpause=%i, bcw=%i)\n[",
                   fpu_id, asteps, apause, aclockwise,
                   bsteps, bpause, bclockwise);
            for (int i=0; i < buf_len; i++)
            {
                printf(" %02i", can_buffer.message.data[i]);
            }
            printf(" ]\n");
#endif    

            
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
          // send response if this is the first or last entry
          return (fentry || lentry);
        };

        // time-out period for a response to the message
        timespec getTimeOut()
        {
            const struct timespec  toval =
                {/* .tv_sec = */ 0,
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
        bool apause;
        bool bpause;
        bool aclockwise;
        bool bclockwise;
        bool fentry;
        bool lentry;
        
        
    };

}

}
#endif
