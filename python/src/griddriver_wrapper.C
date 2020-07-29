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

//**********************************
#if 0
//**********************************

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

BOOST_PYTHON_MODULE(griddriver)   
{
    using namespace boost::python;

    scope().attr("__version__") = (strlen(VERSION) > 1) ? (((const char *)VERSION) + 1) : "?.?.?";

    // Include shared Boost.Python module content
#include "FpuBPShared_ModuleContent.C"

//**********************************
#endif // 0
//**********************************


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

        .def("enableMove", &WrappedGridDriver::wrapped_enableMove,
             (bp::arg("fpu_id"),
              bp::arg("grid_state")))

        //........................................
        // TODO: Test function only - remove when no longer needed
        // Demonstrates named, arbitrarily-ordered arguments with defaulting
        .def("boostPythonDivide", &WrappedGridDriver::boostPythonDivide,
             (bp::arg("dividend") = 23.0,
              bp::arg("divisor") = 4.0))

        //........................................
    ;

//**********************************
#if 0
//**********************************

}


//**********************************
#endif // 0
//**********************************


//==============================================================================
