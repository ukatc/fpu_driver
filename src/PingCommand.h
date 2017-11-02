
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
// NAME FPU_CAN_driver.h
// 
// This class implements the low-level CAN driver for the MOONS fiber
// positioner grid
//
////////////////////////////////////////////////////////////////////////////////

#include "I_CAN_Command.h"

namespace mpifps {

    class PingCommand : public I_CAN_COmmand
    {

      public:

        PingCommand(){};

        void parametrize(int f_id, long pl)
        {
            fpu_id = f_id;
            payload = pl;
        };

        void SerializeToBuffer(const uint8_t busid,
                               const uint16_t canid,
                               int& buf_len,
                               t_CAN_buffer& can_buffer)
        {
            
            can_buffer.msg.node = 1;
            // The protocol uses little-endian encoding here
            // (the byte order used in the CANOpen protocol).
            can_buffer.msg.id = htole64(fpu_id);
            
            can_buffer.msg.data[0] = payload & 0xff;
            can_buffer.msg.data[1] = (payload >> 8) & 0xff;
            can_buffer.msg.data[2] = (payload >> 16) & 0xff;
            can_buffer.msg.data[3] = (payload >> 24) & 0xff;
            can_buffer.msg.data[4] = (payload >> 32) & 0xff;
            can_buffer.msg.data[5] = (payload >> 40) & 0xff;
            can_buffer.msg.data[6] = (payload >> 48) & 0xff;
            can_buffer.msg.data[7] = (payload >> 56) & 0xff;

            buf_len = 8;
            
        };


        // FPU id to which message is sent
        int getFPU_ID()
        {
            return fpu_id;
        };

        uint8 getMessageLength()
        {
            return mlen;
        }

        // boolean value indicating whether
        // the driver should wait for a response
        bool expectsResponse()
        {
            return true;
        };

        E_CAN_COMMAND getCommandCode()
        {
            return PING_FPU;
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
        long payload;
        
        
    }

}
