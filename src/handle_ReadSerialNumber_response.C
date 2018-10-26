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


#include "FPUState.h"
#include "ethercan/response_handlers/handle_ReadSerialNumber_response.h"
#include "ethercan/time_utils.h"
#include "ethercan/FPUArray.h"


#ifdef DEBUG
#include <stdio.h>
#endif

namespace mpifps
{

namespace ethercanif
{

handle_ReadSerialNumber_response(const EtherCANInterfaceConfig&config,
                                 const int fpu_id,
                                 t_fpu_state& fpu,
                                 int &count_pending
                                 const t_response_buf&data,
                                 const int blen, TimeOutList&  timeout_list,
                                 const E_CAN_COMMAND cmd_id,
				 const uint8_t sequence_number)
{

    // this message does not transmit the error code, for space reasons
    // clear time-out flag
    remove_pending(config, fpu, fpu_id,  cmd_id, response_errcode, timeout_list, count_pending, sequence_number);

    memset(fpu.serial_number, 0, sizeof(fpu.serial_number));
    static_assert(DIGITS_SERIAL_NUMBER < sizeof(fpu.serial_number), "buffer overflow");
    strncpy(fpu.serial_number, (char*) (data + 2), DIGITS_SERIAL_NUMBER);

    LOG_RX(LOG_VERBOSE, "%18.6f : RX : "
	   "Serial number for FPU %i is reported as %s\n",
	   get_realtime(),
	   fpu_id,
	   fpu.serial_number);

    fpu.last_updated = cur_time;

}

}

}
