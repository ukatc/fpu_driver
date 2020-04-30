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
//   - Do: import griddriver
//   - Do: ugd=griddriver.UnprotectedGridDriver()
//   - Do: ugd.testFunction()  repeatedly - on first invocation should display 1,
//     and then increment with each subsequent invocation


#include <boost/python.hpp>

#include "FPUGridDriver.h"

using namespace boost::python;
using namespace mpifps;

// NOTE: The name in BOOST_PYTHON_MODULE() below should match the griddriver.C
// filename, otherwise get the following error when try to import the module in
// Python: // "ImportError: dynamic module does not define init function
// (initgriddriver)" - see 
// https://stackoverflow.com/questions/24226001/importerror-dynamic-module-does-not-define-init-function-initfizzbuzz
 
BOOST_PYTHON_MODULE(griddriver)   
{
    class_<UnprotectedGridDriver>("UnprotectedGridDriver")
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
