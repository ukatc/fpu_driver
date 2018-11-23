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
// NAME handleFPUResponse.h
//
// This function implements the handler for FPU CAN responses.
// It is not a class method, stressing that the FPU status data
// has any "hidden" private fields.
//
////////////////////////////////////////////////////////////////////////////////


#ifndef DECODE_CAN_RESPONSE_H
#define DECODE_CAN_RESPONSE_H

#include "FPUState.h"
#include "ethercan/E_CAN_COMMAND.h"
#include "EtherCANInterfaceConfig.h"
#include "CAN_Command.h"

namespace mpifps
{
namespace ethercanif
{
enum UPDATE_FIELDID
{
    UPDATE_STSWD = 0x01, // update status word
    UPDATE_STATE = 0x02, // update FPU state
    UPDATE_ECODE = 0x04, // extract and update error code
    UPDATE_STEPS = 0x08, // update step counts
    UPDATE_STOPPING = 0x10 // update direction indication to stopped direction
};

const UPDATE_FIELDID UPDATE_FIELDS_DEFAULT = static_cast<UPDATE_FIELDID>(UPDATE_STSWD | UPDATE_STATE | UPDATE_ECODE | UPDATE_STEPS);
const UPDATE_FIELDID UPDATE_FIELDS_NOSTEPS = static_cast<UPDATE_FIELDID>(UPDATE_STSWD | UPDATE_STATE | UPDATE_ECODE);
const UPDATE_FIELDID UPDATE_FIELDS_NOECODE = static_cast<UPDATE_FIELDID>(UPDATE_STSWD | UPDATE_STATE);
const UPDATE_FIELDID UPDATE_FIELDS_NOSTATE = UPDATE_STSWD;
const UPDATE_FIELDID UPDATE_LAST_COMMAND = static_cast<UPDATE_FIELDID>(0);

// test a bit in the 32-bit status word, and return 1 if set, 0 otherwise
inline unsigned int test_bit (uint32_t stwd, E_FPU_STATUS_BITS bit)
{
    // the following test looks sligthly long-winded. However
    // we need to keep in mind that the result is stored in
    // a one-bit bit field, so we need to be careful to
    // not cause overflow.

    return ((stwd & bit) != 0) ? 1 : 0;
}


void logErrorStatus(const EtherCANInterfaceConfig &config, int fpu_id, int err_code);

int unfold_stepcount_alpha(const uint16_t step_count);
int unfold_stepcount_beta(const uint16_t step_count);
int unfold_steps_deviation(const uint16_t step_count);

E_MOC_ERRCODE update_status_flags(t_fpu_state& fpu,
                                  const UPDATE_FIELDID req_fields,
                                  const t_response_buf& data);

E_MOVEMENT_DIRECTION update_direction_stopping(E_MOVEMENT_DIRECTION last_direction);
}
}

#endif
