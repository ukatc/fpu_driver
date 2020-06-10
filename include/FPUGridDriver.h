// -*- mode: c++ -*-

////////////////////////////////////////////////////////////////////////////////
//
// Copyright 2020 UKRI. See file "LICENSE" for license information.
//
//
// Who       When        What
// --------  ----------  -------------------------------------------------------
// bwillemse 2020-04-28  TODO: Put comment here
//------------------------------------------------------------------------------

////////////////////////////////////////////////////////////////////////////////
// NAME FPUGridDriver.h
//
// TODO: Put description here
//
////////////////////////////////////////////////////////////////////////////////

#ifndef FPUGRIDDRIVER_H
#define FPUGRIDDRIVER_H

#include <string>
#include <map>
#include <memory>
#include "E_LogLevel.h"
#include "InterfaceConstants.h"
#include "FPUConstants.h"
#include "EtherCANInterface.h"
#include "AsyncInterface.h"

// TODO: FOR TESTING ONLY
#include "ProtectionDB.h"

namespace mpifps
{
using namespace std;
using namespace mpifps::ethercanif;

// *** TODO: Temporary dummy values for now - need to get from the Linux 
// environment variables in same way as is done in FpuGridDriver.py - need to
// figure out the C/C++ equivalents 
#define DEFAULT_NUM_FPUS    (1)
#define DEFAULT_LOGLEVEL    (LOG_ERROR) 
#define DEFAULT_LOGDIR      ("$HOME")

//*************** TODO: Await Steven's OK for FPU set being a std::vector<>
// (rather than the t_fpuset array of bools)
#define FPU_SET_IS_VECTOR

#ifdef FPU_SET_IS_VECTOR
// TODO: This is named "FpuSelection" to clearly differentiate its type from
// the EtherCAN layer's t_fpuset type
using FpuSelection = std::vector<uint16_t>;
#endif

//..............................................................................
    
class UnprotectedGridDriver
{
public:
    UnprotectedGridDriver(
        // NOTE: Boost.Python only allows up to 14 function arguments in
        // function wrappers, so need to keep number of arguments within this
        int nfpus = DEFAULT_NUM_FPUS,
        bool confirm_each_step = false,
        int configmotion_max_retry_count = 5,
        int configmotion_max_resend_count = 10,
        int min_bus_repeat_delay_ms = 0,
        int min_fpu_repeat_delay_ms = 1,
        enum E_LogLevel logLevel = DEFAULT_LOGLEVEL,
        const string &log_dir = DEFAULT_LOGDIR,
        double motor_minimum_frequency = MOTOR_MIN_STEP_FREQUENCY,
        double motor_maximum_frequency = MOTOR_MAX_STEP_FREQUENCY,
        double motor_max_start_frequency = MOTOR_MAX_START_FREQUENCY,
        double motor_max_rel_increase = MAX_ACCELERATION_FACTOR
        );

    // TODO: Ad-hoc test functions only - remove when no longer needed
    int testIncrement();
    double testDivide(double dividend, double divisor);
    int testGetNumFPUs();

    void _post_connect_hook(const EtherCANInterfaceConfig &config);

    E_EtherCANErrCode connect(const int ngateways,
                              const t_gateway_address gateway_addresses[]);

    
#ifdef FPU_SET_IS_VECTOR
    E_EtherCANErrCode check_fpuset(const FpuSelection &fpu_selection);
#else // NOT FPU_SET_IS_VECTOR
    E_EtherCANErrCode check_fpuset(const AsyncInterface::t_fpuset &fpuset);
#endif // NOT FPU_SET_IS_VECTOR

    void need_ping(const t_grid_state &grid_state,
                   const FpuSelection &fpu_selection,
                   FpuSelection &fpu_ping_selection_ret);

    // TODO: Check if this virtual destructor stuff is correct
    // TODO: Need a real destructor as well?? Or are all member objects RAII ones?
    virtual ~UnprotectedGridDriver();

private:
    EtherCANInterfaceConfig config;

    // TODO: Eventually merge t_wtable and the "reversed" bool into a single
    // data structure, and just have one map instead of two ?
    std::map<int, AsyncInterface::t_wtable> last_wavetable; // <fpu_id, wavetable>
    std::map<int, bool> wf_reversed; // <fpu_id, reversed>

    bool wavetables_incomplete = false;

    // _gd: N.B. Can't use a unique_ptr for EtherCANInterface, probably
    // because it's explicitly non-movable
    // TODO: Rename to something like etherCanIfPtr eventually - is  _gd so that
    // it's same as in FpuGridDriver.py for now
    EtherCANInterface *_gd = nullptr;

    // TODO: Add locked_gateways here, but need to convert Python devicelock.py
    // to C++ first (and use a Linux named semaphore instead of a lock file for this?)

    // TODO: Ad-hoc test variable only - remove when no longer needed
    int dummyCounter = 0;

};

class GridDriver
{
public:
    GridDriver();

private:    

};


} // namespace mpifps

#endif // FPUGRIDDRIVER_H
