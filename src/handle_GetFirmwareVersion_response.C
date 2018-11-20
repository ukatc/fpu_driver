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
#include "ethercan/response_handlers/handle_GetFirmwareVersion_response.h"
#include "ethercan/time_utils.h"
#include "ethercan/FPUArray.h"


#ifdef DEBUG
#include <stdio.h>
#endif

namespace mpifps
{

namespace ethercanif
{

void handle_GetFirmwareVersion_response(const EtherCANInterfaceConfig&config,
					const int fpu_id,
					t_fpu_state& fpu,
					int &count_pending,
					const t_response_buf&data,
					const int blen, TimeOutList&  timeout_list,
					const E_CAN_COMMAND cmd_id,
					const uint8_t sequence_number)
{
    // update status, without extracting error code or update state
    // (they don't fit into this response).
    assert(blen == 8);
    update_status_flags(fpu, UPDATE_FIELDS_NOSTATE, data);

    // clear time-out flag
    //
    // Note: This message has no room for an error code, we assume it
    // succeeded and set the return code to MCE_FPU_OK.
    // 
    // Because the driver checks the version, an invalid version such
    // as (0,0,0) will be detected safely and trigger an error.
    // FIXME: Might need to be defined better.
    
    E_MOC_ERRCODE response_errcode = MCE_FPU_OK;
    remove_pending(config, fpu, fpu_id,  cmd_id, response_errcode, timeout_list, count_pending, sequence_number);

    // get firmware version and date
    fpu.firmware_version[0] = data[3];
    fpu.firmware_version[1] = data[4];
    fpu.firmware_version[2] = data[5];

    // unpack calendar date
    const uint16_t fw_date = (data[6] | (data[7] << 8));
    fpu.firmware_date[0] = fw_date & 0x07f;
    fpu.firmware_date[1] = (fw_date >> 7) & 0x0f;
    fpu.firmware_date[2] = (fw_date >> 11) & 0x1f;;

    fpu.ping_ok = true;
    LOG_RX(LOG_ERROR, "%18.6f : RX : "
	   "GetFirmwareVersion result for FPU %i : version = %i.%i.%i, date = 20%02i-%02i-%02i/,\n",
	   get_realtime(), fpu_id,
	   fpu.firmware_version[0], fpu.firmware_version[1], fpu.firmware_version[2],
	   fpu.firmware_date[0], fpu.firmware_date[1], fpu.firmware_date[2]);



}

}

}
