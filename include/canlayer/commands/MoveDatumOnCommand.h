
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

namespace canlayer
{

    class MoveDatumOnCommand : public I_CAN_Command
    {

      public:

        static E_CAN_COMMAND getCommandCode()
        {
            return CCMD_MOVE_DATUM_ON;
        };


        MoveDatumOnCommand(){};
      
        ~MoveDatumOnCommand(){};

        E_CAN_COMMAND getInstanceCommandCode()
        {
            return getCommandCode();
        };


        void parametrize(int f_id, bool broadcast, int alpha_direction, int beta_direction)
        {
            fpu_id = f_id;
            bcast = broadcast;
            adir = alpha_direction;
            bdir = beta_direction;
        };

        void SerializeToBuffer(const uint8_t busid,
                               const uint8_t fpu_canid,
                               int& buf_len,
                               t_CAN_buffer& can_buffer)
        {

            // CAN bus id for that gateway to which message should go
            can_buffer.message.busid = busid;

            // we use bit 7 to 10 for the command code,
            // and bit 0 to 6 for the FPU bus id.
            assert(fpu_canid < FPUS_PER_BUS);
            if (! bcast)
            {
                assert(fpu_canid > 0);
            }
            

            // the CAN identifier is either all zeros (for a broadcast
            // message) or bits 7 - 10 are the proiority and bits 0 -
            // 6 the CAN id of the FPU.
            const E_CAN_COMMAND cmd_code = getCommandCode();

            uint16_t can_identifier = 0;

            if (! bcast)
            {
                can_identifier = (getMessagePriority(cmd_code)
                                  << 7) | fpu_canid;
            }
                                   
            // The protocol uses little-endian encoding here
            // (the byte order used in the CANOpen protocol).            
            can_buffer.message.identifier = htole64(can_identifier);


            // CAN command code
            can_buffer.message.data[0] = cmd_code;

            can_buffer.message.data[1] = 0xff & adir;
            can_buffer.message.data[2] = 0xff & bdir;

            buf_len = 3;
            
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
        return bcast;
      }

      private:
        uint16_t fpu_id;
        int adir;
        int bdir;
        bool bcast;
        
        
    };

}

}
#endif
