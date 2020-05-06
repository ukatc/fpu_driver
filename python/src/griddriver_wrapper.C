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
//   - Do: source build_griddriver_wrapped.sh   (produces griddriver.so library file)
//   - Open interactive Python shell from Bash shell by typing "python -i"
//   - Do: from griddriver import *
//   - Do: ugd=UnprotectedGridDriver(1, False, 1, 2, 3, 4, "blah", 5.5, 6.6, 7.7, 8.8)
//     (N.B. Dummy values for now)
//   - Do: ugd.testIncrement()  repeatedly - on first invocation should display 1,
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
        // NOTE: Boost.Python only allows up to 14 function arguments
        int,            // nfpus
        bool,           // confirm_each_step
        int,            // configmotion_max_retry_count
        int,            // configmotion_max_resend_count
        int,            // min_bus_repeat_delay_ms
        int,            // min_fpu_repeat_delay_ms
        //enum E_LogLevel // logLevel // TODO: Figure out how to implement enums in Boost.Python
        const string &, // log_dir
        double,         // motor_minimum_frequency
        double,         // motor_maximum_frequency
        double,         // motor_max_start_frequency
        double          // motor_max_rel_increase
        >())


        // TODO: If use the following then need to use shared consts for
        // ALL of the following (so that shared with FPUGridDriver.h
        // constructor defaults)

#if 0        
        bp::arg("nfpus") = DEFAULT_NUM_FPUS,
        bp::arg("confirm_each_step") = false,
        bp::arg("configmotion_max_retry_count") = 5,
        bp::arg("configmotion_max_resend_count") = 10,
        bp::arg("min_bus_repeat_delay_ms") = 0,
        bp::arg("min_fpu_repeat_delay_ms") = 1,
        bp::arg("E_LogLevel logLevel") = DEFAULT_LOGLEVEL,
        bp::arg("string &log_dir") = DEFAULT_LOGDIR,
        bp::arg("motor_minimum_frequency") = MOTOR_MIN_STEP_FREQUENCY,
        bp::arg("motor_maximum_frequency") = MOTOR_MAX_STEP_FREQUENCY,
        bp::arg("motor_max_start_frequency") = MOTOR_MAX_START_FREQUENCY
        bp::arg("motor_max_rel_increase") = MAX_ACCELERATION_FACTOR,
#endif // 0        

        // TODO: Ad-hoc test functions only - remove when no longer needed
        .def("testIncrement", &UnprotectedGridDriver::testIncrement)
        .def("testDivide", &UnprotectedGridDriver::testDivide)
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
//   - Do: source build_griddriver_wrapped.sh   (produces griddriver.so file)
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
