// -*- mode: c++ -*-

////////////////////////////////////////////////////////////////////////////////
//
// Copyright 2020 UKRI. See file "LICENSE" for license information.
//
// Who       When        What
// --------  ----------  -------------------------------------------------------
// bwillemse 2020-04-28  TODO: Put comment here
//------------------------------------------------------------------------------

////////////////////////////////////////////////////////////////////////////////
// NAME griddriver_wrapper.C
//
// This file implements the Python wrapper for the FPU grid driver interface
// for the MOONS instrument fibre positioner unit.
//
////////////////////////////////////////////////////////////////////////////////


// BW NOTE: This file is initial work in progress for now

// TODO: Update the following comments eventually - the comments below were for
// early Boost.Python experimentation

// Usage examples
// --------------
//
// Firstly, set things up:
//   - Open a Bash shell in this directory
//   - Do: source build_griddriver_wrapped.sh   (produces griddriver.so library file)
//   - Open interactive Python shell from Bash shell by typing "python -i"
//   - Do: from griddriver import *
//
// Basic GridDriver instantiation and usage:
//   - Do: gd=GridDriver(123)    (N.B. Dummy values for now)
//   - Do: gd.boostPythonDivide(23.0, 5.0)
//             4.6
//   - Do: gd.connect([1,2,3])
//             griddriver.E_EtherCANErrCode.DE_OK

#include <iostream>

#include "FpuBPShared_General.h"
#include "GridDriver.h"
#include "InterfaceConstants.h"

// NOTE: The name in BOOST_PYTHON_MODULE() below should match the griddriver.C
// filename, otherwise get the following error when try to import the module in
// Python: // "ImportError: dynamic module does not define init function
// (initgriddriver)" - see 
// https://stackoverflow.com/questions/24226001/importerror-dynamic-module-does-not-define-init-function-initfizzbuzz

// All of the wrapped functions support Python named and arbitrarily-ordered 
// arguments, and have argument defaulting.

// WrappedGridDriver constructor notes:
//   - The WrappedGridDriver Boost.Python constructor also supports named and
//     arbitrarily-ordered arguments. However, this has required a more complex 
//     Boost.Python custom constructor approach to be used:
//       - Inheriting of UnprotectedGridDriver's constructor using C++11
//         constructor inheritance
//       - WrappedGridDriver::initWrapper() function
//       - In the BOOST_PYTHON_MODULE()'s "class_" statement, use of the
//         "no_init" specifier, and specifying WrappedGridDriver::initWrapper()
//         as the custom class constructor function (which uses a
//         boost::shared_ptr construct)
//       - IMPORTANT: All of the constructor argument lists must exactly
//         correspond in the various places
//   - See e.g. the following web reference for more info:
//     https://stackoverflow.com/questions/18793952/boost-python-how-do-i-provide-a-custom-constructor-wrapper-function


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

BOOST_PYTHON_MODULE(griddriver)   
{
    using namespace boost::python;

    scope().attr("__version__") = (strlen(VERSION) > 1) ? (((const char *)VERSION) + 1) : "?.?.?";

    // Include shared Boost.Python module content
#include "FpuBPShared_ModuleContent.C"

    class_<WrappedGridDriver, boost::shared_ptr<WrappedGridDriver> >
        ("GridDriver", no_init)
        .def("__init__", make_constructor(&WrappedGridDriver::initWrapper,
                                          bp::default_call_policies(),
             (bp::arg("nfpus") = DEFAULT_NUM_FPUS,
              bp::arg("SocketTimeOutSeconds") = 20.0,
              bp::arg("confirm_each_step") = false,
              bp::arg("waveform_upload_pause_us") = 0,
              bp::arg("configmotion_max_retry_count") = 5,
              bp::arg("configmotion_max_resend_count") = 10,
              bp::arg("min_bus_repeat_delay_ms") = 0,
              bp::arg("min_fpu_repeat_delay_ms") = 1,
              bp::arg("alpha_datum_offset") = ALPHA_DATUM_OFFSET,
              bp::arg("motor_minimum_frequency") = MOTOR_MIN_STEP_FREQUENCY,
              bp::arg("motor_maximum_frequency") = MOTOR_MAX_STEP_FREQUENCY,
              bp::arg("motor_max_start_frequency") = MOTOR_MAX_START_FREQUENCY,
              bp::arg("motor_max_rel_increase") = MAX_ACCELERATION_FACTOR,
              bp::arg("motor_max_step_difference") = MAX_STEP_DIFFERENCE)))

        .def("initialize", &WrappedGridDriver::initialize,
             (bp::arg("logLevel") = DEFAULT_LOGLEVEL,
              bp::arg("log_dir") = DEFAULT_LOGDIR,
              bp::arg("firmware_version_address_offset") = 0x61,
              bp::arg("protection_logfile") = "_" DEFAULT_START_TIMESTAMP "-fpu_protection.log",
              bp::arg("control_logfile") = "_" DEFAULT_START_TIMESTAMP "-fpu_control.log",
              bp::arg("tx_logfile") = "_" DEFAULT_START_TIMESTAMP "-fpu_tx.log",
              bp::arg("rx_logfile") = "_" DEFAULT_START_TIMESTAMP "-fpu_rx.log",
              bp::arg("start_timestamp") = DEFAULT_START_TIMESTAMP))

        .def("getGridState", &WrappedGridDriver::wrapped_getGridState)

        .def("connect", &WrappedGridDriver::wrapped_connect,
            // TODO: Add defaulting to the following argument - see 
            // FpuGridDriver.py function equivalent, FPU grid driver
            // documentation etc – see DEFAULT_GATEWAY_ADDRESS_LIST / 
            // MOCK_GATEWAY_ADDRESS_LIST stuff – need to use a bp::list
            // somehow I think
             (bp::arg("address_list")))

        .def("disconnect", &WrappedGridDriver::disconnect)

        .def("findDatum", &WrappedGridDriver::wrapped_findDatum,
             (bp::arg("grid_state"),
              bp::arg("search_modes") = bp::dict(),
              bp::arg("selected_arm") = DASEL_BOTH,
              bp::arg("fpuset") = bp::list(),
              bp::arg("soft_protection") = true,
              bp::arg("count_protection") = true,
              bp::arg("support_uninitialized_auto") = true,
              bp::arg("timeout") = DATUM_TIMEOUT_ENABLE))

        .def("pingFPUs", &WrappedGridDriver::wrapped_pingFPUs,
             (bp::arg("grid_state"),
              bp::arg("fpuset") = bp::list()))

        .def("configMotion", &WrappedGridDriver::wrapped_configMotion,
             (bp::arg("wavetable"),
              bp::arg("grid_state"),
              bp::arg("fpuset") = bp::list(),
              bp::arg("soft_protection") = true,
              bp::arg("allow_uninitialized") = false,
              bp::arg("ruleset_version") = DEFAULT_WAVEFORM_RULESET_VERSION,
              // TODO: The following arguments are from FpuGridDriver.py ->
              // configMotion() - keep? (N.B. They aren't documented in the
              // FPU driver manual of January 22, 2020)
              bp::arg("warn_unsafe") = true,
              bp::arg("verbosity") = 3))

        .def("executeMotion", &WrappedGridDriver::wrapped_executeMotion,
             (bp::arg("grid_state"),
              bp::arg("fpuset") = bp::list(),
              // TODO: sync_command default is true here and in FpuGridDriver.py,
              // but is shown as False in grid driver document
              bp::arg("sync_command") = true))

        //........................................
        // TODO: Test function only - remove when no longer needed
        // Demonstrates named, arbitrarily-ordered arguments with defaulting
        .def("boostPythonDivide", &WrappedGridDriver::boostPythonDivide,
             (bp::arg("dividend") = 23.0,
              bp::arg("divisor") = 4.0))

        //........................................
    ;
}


//==============================================================================
