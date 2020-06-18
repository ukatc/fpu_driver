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
// NAME UnprotectedGridDriver.h
//
// TODO: Put description here
//
////////////////////////////////////////////////////////////////////////////////

#ifndef UNPROTECTEDGRIDDRIVER_H
#define UNPROTECTEDGRIDDRIVER_H

#include <string>
#include <map>
#include <memory>
#include "E_LogLevel.h"
#include "InterfaceConstants.h"
#include "FPUConstants.h"
#include "EtherCANInterface.h"
#include "AsyncInterface.h"
#include "InterfaceState.h"
#include "Interval.h"

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
// BUT use t_fpuset from EtherCAN after all?
//#define FPU_SET_IS_VECTOR

#ifdef FPU_SET_IS_VECTOR
// TODO: This is named "FpuSelection" to clearly differentiate its type from
// the EtherCAN layer's t_fpuset type
using FpuSelection = std::vector<uint16_t>;
#endif

// TODO: Not sure about whether these FPU position data structures are fully
// correct and suitable yet - keep under review
typedef struct
{
    Interval apos;
    Interval bpos;
} FpuPosition;

typedef FpuPosition FpuPositions[MAX_NUM_POSITIONERS];

class UnprotectedGridDriverTester;  // Forward reference


//==============================================================================

class UnprotectedGridDriver
{
    // Declare test class as friend so that it can access protected/private
    // member variables and functions
    friend class UnprotectedGridDriverTester;
  
    //..........................................................................
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

    E_EtherCANErrCode connect(const int ngateways,
                              const t_gateway_address gateway_addresses[]);

#ifndef FPU_SET_IS_VECTOR
    // TODO: In some of the following functions, reversed the 
    // fpuset/selected_arm arguments relative to the equivalent Python
    // functions, so that can have argument defaults

    E_EtherCANErrCode findDatum(t_grid_state &gs, 
                                const t_datum_search_flags &search_modes,
                                enum E_DATUM_SELECTION selected_arm,
                                const t_fpuset &fpuset,
                                bool soft_protection, bool count_protection,
                                bool support_uninitialized_auto,
                                enum E_DATUM_TIMEOUT_FLAG timeout);


#endif // FPU_SET_IS_VECTOR

    // TODO: Check if this virtual destructor stuff is correct
    // TODO: Need a real destructor as well?? Or are all member objects RAII ones?
    virtual ~UnprotectedGridDriver();

    // TODO: Ad-hoc functions for Boost.Python wrapper testing only - remove
    // when no longer needed
    int boostPythonIncrement();
    double boostPythonDivide(double dividend, double divisor);
    int boostPythonGetNumFPUs();

    //..........................................................................
protected:
    // NOTE: The following virtual functions are overridden in GridDriver

    virtual void _post_connect_hook(const EtherCANInterfaceConfig &config) {}

    // TODO: Do the t_grid_state's below need to be const? Or will they
    // possibly be altered inside the functions?
    virtual void _allow_find_datum_hook(t_grid_state &gs,
                                        t_datum_search_flags &search_modes, // Modifiable
                                        enum E_DATUM_SELECTION selected_arm,
                                        const t_fpuset &fpuset,
                                        bool support_uninitialized_auto) {}
    virtual void _start_find_datum_hook(t_grid_state &gs,
                                        const t_datum_search_flags &search_modes,
                                        enum E_DATUM_SELECTION selected_arm,
                                        const t_fpuset &fpuset,
                                        FpuPositions &initial_positions_ret,
                                        bool soft_protection) {}
    virtual void _cancel_find_datum_hook(t_grid_state &gs,
                                         // TODO: These arguments (which are in the Python version)
                                         // are not used so can remove?
                                         //const t_datum_search_flags &search_modes,
                                         //enum E_DATUM_SELECTION selected_arm,
                                         const t_fpuset &fpuset, 
                                         const FpuPositions &initial_positions) {}
    virtual void _finished_find_datum_hook(t_grid_state &prev_gs,
                                           t_grid_state &datum_gs,
                                           const t_datum_search_flags &search_modes,
                                           const t_fpuset &fpuset,
                                           bool was_cancelled,
                                           const FpuPositions &initial_positions, 
                                           enum E_DATUM_SELECTION selected_arm) {}

    //..........................................................................
private:
  
#ifdef FPU_SET_IS_VECTOR
    E_EtherCANErrCode check_fpuset(const FpuSelection &fpu_selection);
    void need_ping(const t_grid_state &gs,
                   const FpuSelection &fpu_selection,
                   FpuSelection &fpu_ping_selection_ret);
#else // NOT FPU_SET_IS_VECTOR
    E_EtherCANErrCode check_fpuset(const t_fpuset &fpuset);
    void need_ping(const t_grid_state &gs, const t_fpuset &fpuset,
                   t_fpuset &pingset_ret);
#endif // NOT FPU_SET_IS_VECTOR

    E_EtherCANErrCode _pingFPUs(t_grid_state &gs, const t_fpuset &fpuset);

    EtherCANInterfaceConfig config;

    // TODO: Eventually merge t_wtable and the "reversed" bool into a single
    // data structure, and just have one map instead of two ?
    std::map<int, t_wtable> last_wavetable; // <fpu_id, wavetable>
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

    //..........................................................................
};


//==============================================================================

class UnprotectedGridDriverTester
{
    // N.B. This class is friend-ed from UnprotectedGridDriver, so it can
    // access its private and protected member variables and functions
public:
    void doTests();
    
private:
    void test_check_fpuset();
    void test_need_ping();
    void test_connect();
    void test_FindDatum();
};


//==============================================================================

} // namespace mpifps

#endif // UNPROTECTEDGRIDDRIVER_H
