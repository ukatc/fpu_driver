// -*- mode: c++ -*-

////////////////////////////////////////////////////////////////////////////////
//
// Copyright 2020 UKRI. See file "LICENSE" for license information.
//
// Who       When        What
// --------  ----------  -------------------------------------------------------
// bwillemse 2020-07-29  Created.
//------------------------------------------------------------------------------

////////////////////////////////////////////////////////////////////////////////
// NAME WrappedGridDriver.h
//
// TODO: Put comments here
//
////////////////////////////////////////////////////////////////////////////////

#include <string>
#include "WrapperSharedBase.h"
#include "GridDriver.h"
#include "InterfaceConstants.h"


//==============================================================================

class WrappedGridDriver : public GridDriver,
                          protected WrapperSharedBase
{
public:
    using GridDriver::GridDriver; // Inherit constructor

    // NOTE: Static function
    // TODO: Check if needs to be static - OR move out of this wrapper class
    // into a standalone function for better clarity?
    // Original from https://stackoverflow.com/questions/18793952/boost-python-how-do-i-provide-a-custom-constructor-wrapper-function:
    //static boost::shared_ptr<CppClass> initWrapper( object const & p )
    //{
    //    SpecialParameters sp = ... // do complicated extraction here.
    //    return boost::shared_ptr<CppClass>( new CppClass(sp) );
    //}

    static boost::shared_ptr<WrappedGridDriver> initWrapper(
        int nfpus,
        double SocketTimeOutSeconds,
        bool confirm_each_step,
        long waveform_upload_pause_us,
        int configmotion_max_retry_count,
        int configmotion_max_resend_count,
        int min_bus_repeat_delay_ms,
        int min_fpu_repeat_delay_ms,
        double alpha_datum_offset,
        double motor_minimum_frequency,
        double motor_maximum_frequency,
        double motor_max_start_frequency,
        double motor_max_rel_increase,
        double motor_max_step_difference);

    E_EtherCANErrCode wrapped_initialize(E_LogLevel logLevel,
                                         const std::string &log_dir,
                                         int firmware_version_address_offset,
                                         const std::string &protection_logfile,
                                         const std::string &control_logfile,
                                         const std::string &tx_logfile,
                                         const std::string &rx_logfile,
                                         const std::string &start_timestamp,
                                         bool mockup);
    WrapGridState wrapped_getGridState();
    E_EtherCANErrCode wrapped_connect(bp::list &list_gateway_addresses);
    E_EtherCANErrCode wrapped_disconnect();
    E_EtherCANErrCode wrapped_setUStepLevel(int ustep_level, 
                                            WrapGridState &grid_state,
                                            bp::list &fpu_list);
    E_EtherCANErrCode wrapped_setTicksPerSegment(unsigned long ticks,
                                                 WrapGridState &grid_state,
                                                 bp::list &fpu_list);
    E_EtherCANErrCode wrapped_setStepsPerSegment(int min_steps, int max_steps,
                                                 WrapGridState &grid_state,
                                                 bp::list &fpu_list);
    E_EtherCANErrCode wrapped_findDatum(WrapGridState &grid_state,
                                        bp::dict &dict_search_modes,
                                        E_DATUM_SELECTION selected_arm,
                                        bp::list &fpu_list,
                                        bool soft_protection,
                                        bool count_protection,
                                        bool support_uninitialized_auto,
                                        E_DATUM_TIMEOUT_FLAG timeout);
    E_EtherCANErrCode wrapped_resetFPUs(WrapGridState& grid_state, list& fpu_list);
    E_EtherCANErrCode wrapped_resetStepCounters(long new_alpha_steps,
                                                long new_beta_steps,
					                            WrapGridState &grid_state,
                                                bp::list &fpu_list);
    E_EtherCANErrCode wrapped_readRegister(int read_address,
                                           WrapGridState &grid_state,
                                           bp::list &fpu_list);
    E_EtherCANErrCode wrapped_getDiagnostics(WrapGridState &grid_state,
                                             bp::list &fpu_list);
    E_EtherCANErrCode wrapped_pingFPUs(WrapGridState &grid_state,
                                       bp::list &fpu_list);
    E_EtherCANErrCode wrapped_getFirmwareVersion(WrapGridState &grid_state,
                                                 bp::list & fpu_list);
    E_EtherCANErrCode wrapped_readSerialNumbers(WrapGridState &grid_state,
                                                bp::list &fpu_list);
    E_EtherCANErrCode wrapped_writeSerialNumber(int fpu_id,
                                                bp::str serial_number,
                                                WrapGridState &grid_state);
    E_EtherCANErrCode wrapped_configMotion(bp::dict &dict_waveforms,
                                           WrapGridState &grid_state,
                                           bp::list &fpu_list,
                                           bool soft_protection,
                                           bool allow_uninitialized,
					                       int ruleset_version,
                                           // TODO: The following arguments
                                           // are from FpuGridDriver.py ->
                                           // configMotion() - keep?
                                           bool warn_unsafe, int verbosity);
    E_EtherCANErrCode wrapped_executeMotion(WrapGridState &grid_state,
                                            bp::list &fpu_list,
                                            bool sync_command);
    E_EtherCANErrCode wrapped_abortMotion(WrapGridState &grid_state,
                                          bp::list &fpu_list,
                                          bool sync_command = true);
    E_EtherCANErrCode wrapped_freeBetaCollision(int fpu_id,
                                                E_REQUEST_DIRECTION direction,
                                                WrapGridState &grid_state,
                                                bool soft_protection);
    E_EtherCANErrCode wrapped_enableBetaCollisionProtection(
                                                WrapGridState &grid_state);
    E_EtherCANErrCode wrapped_freeAlphaLimitBreach(int fpu_id,
                                                   E_REQUEST_DIRECTION direction,
                                                   WrapGridState &grid_state,
                                                   bool soft_protection);
    E_EtherCANErrCode wrapped_enableAlphaLimitProtection(WrapGridState &grid_state);
    E_EtherCANErrCode wrapped_reverseMotion(WrapGridState &grid_state,
                                            bp::list &fpu_list,
                                            bool soft_protection);
    E_EtherCANErrCode wrapped_repeatMotion(WrapGridState &grid_state,
                                           bp::list &fpu_list,
                                           bool soft_protection);
    E_EtherCANErrCode wrapped_lockFPU(int fpu_id, WrapGridState &grid_state);
    E_EtherCANErrCode wrapped_unlockFPU(int fpu_id, WrapGridState &grid_state);
    E_EtherCANErrCode wrapped_enableMove(int fpu_id, WrapGridState &grid_state);
    E_EtherCANErrCode wrapped_checkIntegrity(WrapGridState &grid_state,
                                             bp::list &fpu_list);
    E_EtherCANErrCode wrapped_trackedAngles(WrapGridState &grid_state,
                                            bp::list &fpu_list,
                                            bool show_offsets, bool active);

private:
    bool checkAndMessageIfInitializedOk();
    const EtherCANInterfaceConfig &getConfig() const override;
};

//==============================================================================

