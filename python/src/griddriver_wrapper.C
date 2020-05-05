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

// Example of using this Python wrapper:
//   - Open a Bash shell in this directory
//   - Do: source build_griddriver_wrapper.sh   (produces griddriver.so wrapper library file)
//   - Open interactive Python shell from Bash shell by typing "python -i"
//   - Do: from griddriver import *
//   - Do: ugd=UnprotectedGridDriver(1, 32.0, False, 3, 4, 5, 6, 7, 9.0, "blah", 1.0, 2.0, 3.0)
//              (***** NOTE: Does not yet have all required parameters *****)
//   - Do: ugd.testFunction()  repeatedly - on first invocation should display 1,
//     and then increment with each subsequent invocation


#include <boost/python.hpp>

#include "FPUGridDriver.h"
#include "InterfaceConstants.h"

using namespace boost::python;
namespace bp = boost::python;

using namespace mpifps;

// NOTE: The name in BOOST_PYTHON_MODULE() below should match the griddriver.C
// filename, otherwise get the following error when try to import the module in
// Python: // "ImportError: dynamic module does not define init function
// (initgriddriver)" - see 
// https://stackoverflow.com/questions/24226001/importerror-dynamic-module-does-not-define-init-function-initfizzbuzz
 
BOOST_PYTHON_MODULE(griddriver)   
{
    class_<UnprotectedGridDriver>("UnprotectedGridDriver", init<
        int,
        double,
        bool,
        int,
        int,
        int,
        int,
        int,
        double,  
        //enum E_LogLevel,  // TODO: Figure out how to implement enums in Boost.Python
        const string&,
        double,
        double,
        double>())

#if 0
        // NOTE: Boost.Python only allows up to 14 function params, so the
        // following can't be added
        double,
        int,
        int,
        const string&,
        const string&,
        const string&,
        const string&,
        const string&
#endif // 0



        // TODO: If use the following then need to use shared consts for
        // ALL of the following (so that shared with FPUGridDriver.h
        // constructor defaults)

#if 0        
        bp::arg("nfpus") = DEFAULT_NUM_FPUS,
        bp::arg("socketTimeOutSeconds") = 20.0,
        bp::arg("confirm_each_step") = false,
        bp::arg("waveform_upload_pause_us") = 0,
        bp::arg("configmotion_max_retry_count") = 5,
        bp::arg("configmotion_max_resend_count") = 10,
        bp::arg("min_bus_repeat_delay_ms") = 0,
        bp::arg("min_fpu_repeat_delay_ms") = 1,
        bp::arg("alpha_datum_offset") = ALPHA_DATUM_OFFSET,
        bp::arg("E_LogLevel logLevel") = DEFAULT_LOGLEVEL,
        bp::arg("string &log_dir") = DEFAULT_LOGDIR,
        bp::arg("motor_minimum_frequency") = MOTOR_MIN_STEP_FREQUENCY,
        bp::arg("motor_maximum_frequency") = MOTOR_MAX_STEP_FREQUENCY,
        bp::arg("motor_max_start_frequency") = MOTOR_MAX_START_FREQUENCY

        bp::arg("motor_max_rel_increase") = MAX_ACCELERATION_FACTOR,
        bp::arg("motor_max_step_difference") = MAX_STEP_DIFFERENCE,
        bp::arg("firmware_version_address_offset") = 0x61,
        bp::arg("string &protection_logfile") = "_{start_timestamp}-fpu_protection.log",
        bp::arg("string &control_logfile") = "_{start_timestamp}-fpu_control.log",
        bp::arg("string &tx_logfile") = "_{start_timestamp}-fpu_tx.log",
        bp::arg("string &rx_logfile") = "_{start_timestamp}-fpu_rx.log",
        bp::arg("string &start_timestamp") = "ISO8601"
#endif // 0        


        .def("testFunction", &UnprotectedGridDriver::testFunction)
    ;
}




//******************************************************************************
//******************************************************************************
// TODO: Experimental code only - delete once no longer required
#if 0
//******************************************************************************

// Adapted the following sample code from 
// https://www.boost.org/doc/libs/1_63_0/libs/python/doc/html/tutorial/tutorial/exposing.html
// to experiment with basic Python bindings for a simple C++ class

// To use this experimental stuff:
//   - Open a Bash shell in this directory
//   - Do: source build_griddriver_wrapper.sh   (produces griddriver.so library file)
//   - Open interactive Python shell from Bash shell by typing "python -i"
//   - Do: import griddriver
//   - Do: blah=griddriver.MessageStorer()
//   - Do: blah.set("abc")
//   - Do: blah.get()
//   - Displays: 'abc', showing that the C++ MessageStorer class's functionality
//     has worked OK

//..............................................................................
// Simple C++ test class

#include <string>

class MessageStorer
{
public:
    void set(std::string msg)
    {
        this->msg = msg;
    }

    std::string get()
    {
        return msg;
    }

private:    
    std::string msg;
};

//..............................................................................
// Boost.Python bindings

#include <boost/python.hpp>
using namespace boost::python;

// NOTE: The name in BOOST_PYTHON_MODULE() below should match the griddriver.C
// filename, otherwise get the following error when try to import the module in
// Python: // "ImportError: dynamic module does not define init function
// (initgriddriver)" - see 
// https://stackoverflow.com/questions/24226001/importerror-dynamic-module-does-not-define-init-function-initfizzbuzz
 
BOOST_PYTHON_MODULE(griddriver)   
{
    class_<MessageStorer>("MessageStorer")
        .def("set", &MessageStorer::set)
        .def("get", &MessageStorer::get)
    ;
}

//******************************************************************************
#endif // 0
//******************************************************************************
//******************************************************************************
