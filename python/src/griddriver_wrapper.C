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
//   - Do: gd.boostPythonIncrement() repeatedly - on first invocation should
//     display 1, and then increment with each subsequent invocation
//   - Do: gd.boostPythonDivide(23.0, 5.0)
//             4.6
//   - Do: gd.connect([1,2,3])
//             griddriver.E_EtherCANErrCode.DE_OK

#include <boost/python.hpp>
#include <boost/python/suite/indexing/vector_indexing_suite.hpp>

#include "GridDriver.h"
#include "InterfaceConstants.h"
#include "FpuBPShared_General.h"

using namespace boost::python;
namespace bp = boost::python;

using namespace mpifps;

using boost::python::object;
using boost::python::extract;
using boost::python::list;
using boost::python::dict;
using boost::python::tuple;
using boost::python::str;


// NOTE: The name in BOOST_PYTHON_MODULE() below should match the griddriver.C
// filename, otherwise get the following error when try to import the module in
// Python: // "ImportError: dynamic module does not define init function
// (initgriddriver)" - see 
// https://stackoverflow.com/questions/24226001/importerror-dynamic-module-does-not-define-init-function-initfizzbuzz


class WrappedGridDriver : public GridDriver
{
public:
    WrappedGridDriver(int dummy_val) : GridDriver{ dummy_val }
    {

    }

    WrapGridState wrapped_getGridState()
    {
        WrapGridState grid_state;
        getGridState(grid_state);
        return grid_state;
    }

    E_EtherCANErrCode wrapped_connect(list& list_gateway_addresses)
    {
        // TODO: Implement this - needs to be similar to
        // WrapEtherCANInterface::connectGateways()

        return DE_OK; 
    }

    E_EtherCANErrCode wrapped_findDatum(WrapGridState &grid_state)
    {

        return DE_OK; 
    }

    E_EtherCANErrCode wrapped_configMotion(void)
    {
        
        return DE_OK; 
    }

    E_EtherCANErrCode wrapped_executeMotion(void)
    {
        
        return DE_OK; 
    }

};


// ************* TODO: Create a WrappedUnprotectedGridDriver here, and specify it
// in the class_ below (rather than UnprotectedGridDriver)


BOOST_PYTHON_MODULE(griddriver)   
{
    scope().attr("__version__") = (strlen(VERSION) > 1) ? (((const char *)VERSION) + 1) : "?.?.?";

    //..........................................................................
    // Include shared Boost.Python module content
#include "FpuBPShared_ModuleContent.C"
    //..........................................................................

#if 0
    // Old UnprotectedGridDriver wrapper stuff - was for initial experimentation only

    // To use:
    // Firstly, set things up:
    //   - Open a Bash shell in this directory
    //   - Do: source build_griddriver_wrapped.sh   (produces griddriver.so library file)
    //   - Open interactive Python shell from Bash shell by typing "python -i"
    //   - Do: from griddriver import *
    // Basic UnprotectedGridDriver instantiation and test function:
    //   - Do: ugd=UnprotectedGridDriver(1, False, 1, 2, 3, 4, LOG_INFO, "blah", 5.5, 6.6, 7.7, 8.8)
    //     (N.B. Dummy values for now)
    //   - Do: ugd.boostPythonIncrement() repeatedly - on first invocation should
    //     display 1, and then increment with each subsequent invocation

    class_<UnprotectedGridDriver>("UnprotectedGridDriver", init<
        // NOTE: Boost.Python only allows up to 14 function arguments
        int,              // nfpus
        bool,             // confirm_each_step
        int,              // configmotion_max_retry_count
        int,              // configmotion_max_resend_count
        int,              // min_bus_repeat_delay_ms
        int,              // min_fpu_repeat_delay_ms
        enum E_LogLevel,  // logLevel
        const string &,   // log_dir
        double,           // motor_minimum_frequency
        double,           // motor_maximum_frequency
        double,           // motor_max_start_frequency
        double            // motor_max_rel_increase
        >())

        // TODO: Ad-hoc test functions only - remove when no longer needed
        .def("boostPythonIncrement", &UnprotectedGridDriver::boostPythonIncrement)
        .def("boostPythonDivide", &UnprotectedGridDriver::boostPythonDivide)
        .def("boostPythonGetNumFPUs", &UnprotectedGridDriver::boostPythonGetNumFPUs)
    ;
#endif // 0

    // TODO: Figure out how constructor needs to be specified for this derived
    // class, because WrappedGridDriver is derived from GridDriver, which is
    // derived from UnprotectedGridDriver
    class_<WrappedGridDriver>("GridDriver", init<
        // NOTE: Boost.Python only allows up to 14 function arguments
        int               // dummy_val
        >())
        .def("getGridState", &WrappedGridDriver::wrapped_getGridState)
        .def("connect", &WrappedGridDriver::wrapped_connect)
        .def("findDatum", &WrappedGridDriver::wrapped_findDatum)
        .def("configMotion", &WrappedGridDriver::wrapped_configMotion)
        .def("executeMotion", &WrappedGridDriver::wrapped_executeMotion)

        // TODO: Ad-hoc test functions only - remove when no longer needed
        .def("boostPythonIncrement", &WrappedGridDriver::boostPythonIncrement)
        .def("boostPythonDivide", &WrappedGridDriver::boostPythonDivide)
        .def("boostPythonGetNumFPUs", &WrappedGridDriver::boostPythonGetNumFPUs)

#if 0
        // TODO: If use the following then need to use shared consts for
        // ALL of the following (so that shared with FPUGridDriver.h
        // constructor defaults)
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
