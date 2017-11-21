// Copyright Ralf W. Grosse-Kunstleve 2002-2004. Distributed under the Boost
// Software License, Version 1.0. (See accompanying
// file LICENSE_1_0.txt or copy at http://www.boost.org/LICENSE_1_0.txt)

#include <boost/python/class.hpp>
#include <boost/python/module.hpp>
#include <boost/python/def.hpp>
#include <boost/python/enum.hpp>
#include <boost/python/suite/indexing/vector_indexing_suite.hpp>
#include <iostream>
#include <string>
#include <vector>
#include "../../src/T_GridState.h"
#include "../../src/E_GridState.h"
#include "../../src/GridDriver.h"


namespace   // Avoid cluttering the global namespace.
{

using namespace mpifps;

// A friendly class.
class hello
{
public:
    hello(const std::string& country)
    {
        this->country = country;
    }
    std::string greet() const
    {
        return "Hello from " + country;
    }
private:
    std::string country;
};

// A function taking a hello object as an argument.
std::string invite(const hello& w)
{
    return w.greet() + "! Please come soon!";
}


class WrapGridState : public t_grid_state
{
public:
    std::vector<t_fpu_state>getStateVec()
    {
        std::vector<t_fpu_state> state_vec;
        for (int i=0; i < MAX_NUM_POSITIONERS; i++)
        {
            state_vec.push_back(FPU_state[i]);
        }
        return state_vec;
    }

    std::vector<long>getCounts()
    {
        std::vector<long> count_vec;
        for (int i=0; i < NUM_FPU_STATES; i++)
        {
            count_vec.push_back(Counts[i]);
        }
        return count_vec;
    }
};

}

BOOST_PYTHON_MODULE(fpu_driver)
{
    using namespace boost::python;
    class_<hello>("hello", init<std::string>())
    // Add a regular member function.
    .def("greet", &hello::greet)
    // Add invite() as a member of hello!
    .def("invite", invite)
    ;

    // Also add invite() as a regular function to the module.
    def("invite", invite);

    enum_<E_FPU_STATE>("E_FPU_STATE")
    .value("FPST_UNKNOWN", FPST_UNKNOWN             )
    .value("FPST_UNINITIALISED", FPST_UNINITIALISED       )
    .value("FPST_LOCKED", FPST_LOCKED              )
    .value("FPST_COORDINATE_RECOVERY", FPST_COORDINATE_RECOVERY )
    .value("FPST_LEAVING_DATUM", FPST_LEAVING_DATUM       )
    .value("FPST_ABOVE_DATUM", FPST_ABOVE_DATUM         )
    .value("FPST_DATUM_SEARCH", FPST_DATUM_SEARCH        )
    .value("FPST_INITIALISED", FPST_INITIALISED         )
    .value("FPST_LOADING", FPST_LOADING             )
    .value("FPST_READY_FORWARD", FPST_READY_FORWARD       )
    .value("FPST_READY_BACKWARD", FPST_READY_BACKWARD      )
    .value("FPST_MOVING", FPST_MOVING              )
    .value("FPST_FINISHED", FPST_FINISHED            )
    .value("FPST_ABORTED", FPST_ABORTED             )
    .value("FPST_COLLISION_DETECTED", FPST_COLLISION_DETECTED  )
    .value("FPST_LIMIT_STOP", FPST_LIMIT_STOP          )
    .value("FPST_COLLISION_RECOVERY", FPST_COLLISION_RECOVERY  )
    .export_values();


    enum_<E_DriverState>("E_DriverState")
    .value("DS_UNINITIALISED", DS_UNINITIALISED       )
    .value("DS_UNCONNECTED", DS_UNCONNECTED         )
    .value("DS_CONNECTED", DS_CONNECTED           )
    .value("DS_ASSERTION_FAILED", DS_ASSERTION_FAILED    )
    .export_values();

    enum_<E_GridState>("E_GridState")
    .value("GS_UNKNOWN", GS_UNKNOWN         )
    .value("GS_UNINITIALISED", GS_UNINITIALISED   )
    .value("GS_LEAVING_DATUM", GS_LEAVING_DATUM   )
    .value("GS_ABOVE_DATUM", GS_ABOVE_DATUM     )
    .value("GS_DATUM_SEARCH", GS_DATUM_SEARCH    )
    .value("GS_INITIALISED", GS_INITIALISED     )
    .value("GS_LOADING", GS_LOADING         )
    .value("GS_READY_FORWARD", GS_READY_FORWARD   )
    .value("GS_READY_BACKWARD", GS_READY_BACKWARD  )
    .value("GS_MOVING", GS_MOVING          )
    .value("GS_FINISHED", GS_FINISHED        )
    .value("GS_LIMITSTOP", GS_LIMITSTOP       )
    .value("GS_COLLISION", GS_COLLISION       )
    .value("GS_ABORTED", GS_ABORTED         )
    .export_values();

    class_<t_fpu_state>("FpuState")
    .def_readwrite("state", &t_fpu_state::state)
    .def_readonly("alpha_steps", &t_fpu_state::alpha_steps)
    .def_readwrite("beta_steps", &t_fpu_state::beta_steps)
    .def_readwrite("is_initialized", &t_fpu_state::is_initialized)
    .def_readwrite("on_alpha_datum", &t_fpu_state::on_alpha_datum)
    .def_readwrite("on_beta_datum", &t_fpu_state::on_beta_datum)
    .def_readwrite("alpha_collision", &t_fpu_state::alpha_collision)
    .def_readwrite("at_alpha_limit", &t_fpu_state::at_alpha_limit)
    .def_readwrite("beta_collision", &t_fpu_state::beta_collision)
    .def_readwrite("ping_ok", &t_fpu_state::ping_ok)
    ;


    class_<std::vector<t_fpu_state> >("StateVec")
    .def(vector_indexing_suite<std::vector<t_fpu_state> >());

    class_<std::vector<long> >("IntVec")
    .def(vector_indexing_suite<std::vector<long> >());

    class_<WrapGridState>("GridState")
    .def_readonly("Fpu_state", &WrapGridState::getStateVec)
    .def_readonly("Counts", &WrapGridState::getCounts)
    .def_readwrite("count_timeout", &WrapGridState::count_timeout)
    .def_readwrite("count_pending", &WrapGridState::count_pending)
    .def_readwrite("driver_state", &WrapGridState::driver_state)
    ;

    class_<GridDriver, boost::noncopyable>("GridDriver", init<int>())
    .def("connect", &GridDriver::connect)
    .def("disconnect", &GridDriver::disconnect)
    .def("initializeDriver", &GridDriver::initializeDriver)
    .def("initializeGrid", &GridDriver::initializeGrid)
    .def("resetFPUs", &GridDriver::resetFPUs)
    .def("findDatum", &GridDriver::findDatum)
    .def("configMotion", &GridDriver::configMotion)
    .def("exectuteMotion", &GridDriver::executeMotion)
    .def("repeatMotion", &GridDriver::repeatMotion)
    .def("reverseMotion", &GridDriver::reverseMotion)
    .def("abortMotion", &GridDriver::abortMotion)
    .def("assignPositions", &GridDriver::assignPositions)
    .def("lockFPU", &GridDriver::lockFPU)
    .def("unlockFPU", &GridDriver::unlockFPU)
    ;

}
