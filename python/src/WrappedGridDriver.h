// -*- mode: c++ -*-

////////////////////////////////////////////////////////////////////////////////
//
// Copyright 2020 UKRI. See file "LICENSE" for license information.
//
// Who       When        What
// --------  ----------  -------------------------------------------------------
// bwillemse 2020-07-29  TODO: Put comment here
//------------------------------------------------------------------------------

////////////////////////////////////////////////////////////////////////////////
// NAME WrappedGridDriver.h
//
// TODO: Put comments here
//
////////////////////////////////////////////////////////////////////////////////

#include <iostream>

#include "FpuBPShared_General.h"
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
        double motor_max_step_difference)
    {
        std::cout << "Grid driver object successfully created (new C++ version) - now call initialize().\n";
        std::cout << "*** NOTE: Soft protection is not implemented yet ***" << std::endl;

        if (confirm_each_step)
        {
            std::cout << "\nconfirm_each_step is set to True, which requires extra confirmation\n";
            std::cout << "requests of waveform step upload, and reduces performance\n" << std::endl;
        }

        if (min_bus_repeat_delay_ms > 0)
        {
            std::cout << "\nmin_bus_repeat_delay_ms is set to value above 0.\n";
            std::cout << "Decrease if message rate is too low.\n" << std::endl;
        }

        return boost::shared_ptr<WrappedGridDriver>(new WrappedGridDriver(
            nfpus,
            SocketTimeOutSeconds,
            confirm_each_step,
            waveform_upload_pause_us,
            configmotion_max_retry_count,
            configmotion_max_resend_count,
            min_bus_repeat_delay_ms,
            min_fpu_repeat_delay_ms,
            alpha_datum_offset,
            motor_minimum_frequency,
            motor_maximum_frequency,
            motor_max_start_frequency,
            motor_max_rel_increase,
            motor_max_step_difference));
    }

    WrapGridState wrapped_getGridState()
    {
        WrapGridState grid_state;
        if (checkAndMessageIfInitializeCalledOk())
        {
            E_GridState grid_state_enum = getGridState(grid_state);
        }
        else
        {
            // TODO: Zero grid_state here, using e.g. memset? BUT WrapGridState
            // is a class rather than a POD structure, so would this be OK?
        }
        return grid_state;
    }

    E_EtherCANErrCode wrapped_connect(bp::list &list_gateway_addresses)
    {
        if (!checkAndMessageIfInitializeCalledOk())
        {
            return DE_INTERFACE_NOT_INITIALIZED;
        }

        t_gateway_address address_array[MAX_NUM_GATEWAYS];
        const int actual_num_gw = convertGatewayAddresses(list_gateway_addresses,
                                                          address_array);
        E_EtherCANErrCode ecode = connect(actual_num_gw, address_array);
        checkInterfaceError(ecode);
        return ecode;
    }

    E_EtherCANErrCode wrapped_findDatum(WrapGridState &grid_state,
                                        bp::dict &dict_search_modes,
                                        E_DATUM_SELECTION selected_arm,
                                        bp::list &fpu_list,
                                        bool soft_protection,
                                        bool count_protection,
                                        bool support_uninitialized_auto,
                                        E_DATUM_TIMEOUT_FLAG timeout)
    {
        if (!checkAndMessageIfInitializeCalledOk())
        {
            return DE_INTERFACE_NOT_INITIALIZED;
        }

        t_fpuset fpuset;
        getFPUSet(fpu_list, fpuset);

        t_datum_search_flags direction_flags;
        getDatumFlags(dict_search_modes, direction_flags, fpuset);

        E_EtherCANErrCode ecode = findDatum(grid_state, direction_flags,
                                            selected_arm, fpuset,
                                            soft_protection,
                                            count_protection,
                                            support_uninitialized_auto,
                                            timeout);
        checkInterfaceError(ecode);
        return ecode;
    }

    E_EtherCANErrCode wrapped_pingFPUs(WrapGridState &grid_state, list &fpu_list)
    {
        if (!checkAndMessageIfInitializeCalledOk())
        {
            return DE_INTERFACE_NOT_INITIALIZED;
        }

        t_fpuset fpuset;
        getFPUSet(fpu_list, fpuset);

        E_EtherCANErrCode ecode = pingFPUs(grid_state, fpuset);
        checkInterfaceError(ecode);
        return ecode;
    }

    E_EtherCANErrCode wrapped_configMotion(bp::dict &dict_waveforms,
                                           WrapGridState &grid_state,
                                           bp::list &fpu_list,
                                           bool soft_protection,
                                           bool allow_uninitialized,
					                       int ruleset_version,
                                           // TODO: The following arguments
                                           // are from FpuGridDriver.py ->
                                           // configMotion() - keep?
                                           bool warn_unsafe, int verbosity)
    {
        // Configures movement by sending a waveform table to a group of FPUs.
        // Call signature is:
        // configMotion( { fpuid0 : { (asteps, bsteps), (asteps, bsteps), ...],
        //                 fpuid1 : { ... }, ...}})
        if (!checkAndMessageIfInitializeCalledOk())
        {
            return DE_INTERFACE_NOT_INITIALIZED;
        }

        t_fpuset fpuset;
        getFPUSet(fpu_list, fpuset);

        t_wtable wtable;
        convertWavetable(dict_waveforms, wtable);

        E_EtherCANErrCode ecode = configMotion(wtable, grid_state, fpuset,
                                               soft_protection,
                                               allow_uninitialized,
                                               ruleset_version, warn_unsafe,
                                               verbosity);
        checkInterfaceError(ecode);
        return ecode;
    }

    E_EtherCANErrCode wrapped_executeMotion(WrapGridState &grid_state,
                                            bp::list &fpu_list,
                                            bool sync_command)
    {
        if (!checkAndMessageIfInitializeCalledOk())
        {
            return DE_INTERFACE_NOT_INITIALIZED;
        }

        t_fpuset fpuset;
        getFPUSet(fpu_list, fpuset);

        E_EtherCANErrCode ecode = executeMotion(grid_state, fpuset, sync_command);
        checkInterfaceError(ecode);
        return ecode;
    }

    E_EtherCANErrCode wrapped_enableMove(int fpu_id, WrapGridState &grid_state)
    {
        if (!checkAndMessageIfInitializeCalledOk())
        {
            return DE_INTERFACE_NOT_INITIALIZED;
        }

        E_EtherCANErrCode ecode = enableMove(fpu_id, grid_state);
        checkInterfaceError(ecode);
        return ecode;
    }

private:
    bool checkAndMessageIfInitializeCalledOk()
    {
        if (initializeWasCalledOk())
        {
            return true;
        }
        std::cout << "**Error**: initialize() needs to be called first.\n" << std::endl;
        return false;
    }

    const EtherCANInterfaceConfig &getConfig() const override
    {
        return config;
    }
};

//==============================================================================

