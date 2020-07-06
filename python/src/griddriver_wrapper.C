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

#include "FpuBPShared_General.h"
#include "GridDriver.h"
#include "InterfaceConstants.h"



#define CONSTRUCTOR_NAMED_ARGS_1   (1)
#define CONSTRUCTOR_NAMED_ARGS_2   (2)
#define CONSTRUCTOR_ORIGINAL       (3)

#if 0
// See https://stackoverflow.com/questions/36138533/boost-python-constructor-with-optional-arguments
#define CONSTRUCTOR_TYPE    (CONSTRUCTOR_NAMED_ARGS_1)
#else
// See https://stackoverflow.com/questions/18793952/boost-python-how-do-i-provide-a-custom-constructor-wrapper-function
#define CONSTRUCTOR_TYPE    (CONSTRUCTOR_NAMED_ARGS_2)
#endif

// NOTE: The name in BOOST_PYTHON_MODULE() below should match the griddriver.C
// filename, otherwise get the following error when try to import the module in
// Python: // "ImportError: dynamic module does not define init function
// (initgriddriver)" - see 
// https://stackoverflow.com/questions/24226001/importerror-dynamic-module-does-not-define-init-function-initfizzbuzz


//............................................
// TODO: For testing only
static int testVal = 99;
//............................................


class WrappedGridDriver : public GridDriver,
                          protected WrapperSharedBase
{
public:
    WrappedGridDriver(int nfpus) : GridDriver(nfpus)
    {
        // TODO: For testing only
        testVal = nfpus;
    }

#if (CONSTRUCTOR_TYPE == CONSTRUCTOR_NAMED_ARGS_1)
    boost::shared_ptr<WrappedGridDriver> wrapper_init(int dummy)
    {

        boost::shared_ptr<WrappedGridDriver> wrapped_grid_driver(new WrappedGridDriver(dummy));

        return wrapped_grid_driver;
    }
#elif (CONSTRUCTOR_TYPE == CONSTRUCTOR_NAMED_ARGS_2)
    // NOTE: Static function
    // TODO: Check if needs to be static - OR move out of this wrapper class
    // into a standalone function for better clarity?
    // Original from https://stackoverflow.com/questions/18793952/boost-python-how-do-i-provide-a-custom-constructor-wrapper-function:
    //static boost::shared_ptr<CppClass> initWrapper( object const & p )
    //{
    //    SpecialParameters sp = ... // do complicated extraction here.
    //    return boost::shared_ptr<CppClass>( new CppClass(sp) );
    //}
    static boost::shared_ptr<WrappedGridDriver> initWrapper(int nfpus)
    {
        return boost::shared_ptr<WrappedGridDriver>(new WrappedGridDriver(nfpus));
    }

#endif

    WrapGridState wrapped_getGridState()
    {
        WrapGridState grid_state;
        getGridState(grid_state);
        return grid_state;
    }

    E_EtherCANErrCode wrapped_connect(bp::list &list_gateway_addresses)
    {
        t_gateway_address address_array[MAX_NUM_GATEWAYS];
        const int actual_num_gw = convertGatewayAddresses(list_gateway_addresses,
                                                          address_array);
        E_EtherCANErrCode ecode = connect(actual_num_gw, address_array);
        checkInterfaceError(ecode);
        return ecode;
    }

    E_EtherCANErrCode wrapped_findDatum(WrapGridState &grid_state,
                                        E_DATUM_SELECTION selected_arm,
                                        bp::list &fpu_list,
                                        bool soft_protection,
                                        bool count_protection,
                                        bool support_uninitialized_auto,
                                        E_DATUM_TIMEOUT_FLAG timeout)
    {
        // TODO: "dict &dict_modes" is specified as an argument in
        // WrapEtherCANInterface.wrap_findDatum() - is it needed here?

#if 0
        // TODO: Adapt the following which was copied from 
        // WrapEtherCANInterface.wrap_findDatum()

        t_fpuset fpuset;
        getFPUSet(fpu_list, fpuset);

        t_datum_search_flags direction_flags;
        getDatumFlags(dict_modes, direction_flags, fpuset);

        E_EtherCANErrCode ecode = findDatum(grid_state, direction_flags,
                                            arm_selection, timeout_flag,
                                            count_protection, &fpuset);
        checkInterfaceError(ecode);
        return ecode;
#endif // 0        
    }

    E_EtherCANErrCode wrapped_configMotion(void)
    {
        
        return DE_OK; 
    }

    E_EtherCANErrCode wrapped_executeMotion(void)
    {
        
        return DE_OK; 
    }

    //............................................
    // TODO: For testing only
    int getTestVal(void)
    {
        return testVal;
    }
    //............................................
};


// ************* TODO: Create a WrappedUnprotectedGridDriver here, and specify it
// in the class_ below (rather than UnprotectedGridDriver)


BOOST_PYTHON_MODULE(griddriver)   
{
    using namespace boost::python;

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

#if (CONSTRUCTOR_TYPE == CONSTRUCTOR_NAMED_ARGS_1)
    class_<WrappedGridDriver, boost::shared_ptr<WrappedGridDriver>, 
           boost::noncopyable>("GridDriver", bp::no_init)
        // NOTE: Boost.Python only allows up to 14 function arguments
        .def("__init__", bp::make_constructor(&WrappedGridDriver::wrapper_init,
                                              bp::default_call_policies(), 
                                              (bp::arg("dummy") = 0)))
#elif (CONSTRUCTOR_TYPE == CONSTRUCTOR_NAMED_ARGS_2)
    class_<WrappedGridDriver, boost::shared_ptr<WrappedGridDriver> >
        ("GridDriver", no_init)
        .def("__init__", make_constructor(&WrappedGridDriver::initWrapper,
                                          bp::default_call_policies(), 
                                          (bp::arg("nfpus") = DEFAULT_NUM_FPUS)))
#else
    class_<WrappedGridDriver>("GridDriver", init<
        // NOTE: Boost.Python only allows up to 14 function arguments
        int               // dummy_val
        >())
#endif
        .def("getGridState", &WrappedGridDriver::wrapped_getGridState)
        .def("connect", &WrappedGridDriver::wrapped_connect)
        .def("disconnect", &WrappedGridDriver::disconnect)
        .def("findDatum", &WrappedGridDriver::wrapped_findDatum,
             (bp::arg("grid_state"),    // Compulsory argument, so no default
              bp::arg("selected_arm") = DASEL_BOTH,
              bp::arg("fpuset") = bp::list(),
              bp::arg("soft_protection") = true,
              bp::arg("count_protection") = true,
              bp::arg("support_uninitialized_auto") = true,
              bp::arg("timeout") = DATUM_TIMEOUT_ENABLE))

        .def("configMotion", &WrappedGridDriver::wrapped_configMotion)
        .def("executeMotion", &WrappedGridDriver::wrapped_executeMotion)

        //........................................
        // TODO: Ad-hoc test functions only - remove when no longer needed
        .def("boostPythonIncrement", &WrappedGridDriver::boostPythonIncrement)
        // TODO: EXPERIMENTAL NAMED ARGUMENTS
        .def("boostPythonDivide", &WrappedGridDriver::boostPythonDivide,
             (bp::arg("dividend") = 23.0,
              bp::arg("divisor") = 4.0))
        .def("boostPythonGetNumFPUs", &WrappedGridDriver::boostPythonGetNumFPUs)
        .def("getTestVal", &WrappedGridDriver::getTestVal)
        //........................................


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
