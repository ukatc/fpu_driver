// -*- mode: c++ -*-

////////////////////////////////////////////////////////////////////////////////
// ESO - VLT Project
//
// Copyright 2017 E.S.O,
//
// Who       When        What
// --------  ----------  -------------------------------------------------------
// jnix      2017-10-18  Created driver class using Pablo Guiterrez' CAN client sample
//------------------------------------------------------------------------------

////////////////////////////////////////////////////////////////////////////////
// NAME T_GridState.h
//
// This header defines a struct which holds the mirrored state
// if the whole FPU array
//
////////////////////////////////////////////////////////////////////////////////

#ifndef T_GATEWAY_ADDRESS_H
#define T_GATEWAY_ADDRESS_H

namespace mpifps
{
    // define default value for more convenient testing.
    static const char * DEFAULT_GATEWAY_IP= "192.168.0.10";
    static const int DEFAULT_GATEWAY_PORT = 4700;
    
    typedef struct t_gateway_address
    {
        public:
        const char * ip;
        uint16_t port;
        t_gateway_address()
            {
                ip = DEFAULT_GATEWAY_IP;
                port = DEFAULT_GATEWAY_PORT;
            };
        t_gateway_address(const char * new_ip, const int new_port)
            {
                ip = new_ip;
                port = new_port;
            };
        t_gateway_address(const char * new_ip)
            {
                ip = new_ip;
                port = DEFAULT_GATEWAY_PORT;
            };

        bool operator==(const  t_gateway_address &a) const
            {
                return (*this) == a;
            };
        
    } t_gateway_address;

}

#endif
