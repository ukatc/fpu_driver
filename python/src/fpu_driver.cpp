// Copyright Ralf W. Grosse-Kunstleve 2002-2004. Distributed under the Boost
// Software License, Version 1.0. (See accompanying
// file LICENSE_1_0.txt or copy at http://www.boost.org/LICENSE_1_0.txt)

#include <boost/python/class.hpp>
#include <boost/python/module.hpp>
#include <boost/python/def.hpp>
#include <boost/python/enum.hpp>
#include <boost/python/extract.hpp>
#include <boost/python/list.hpp>
#include <boost/python/dict.hpp>
#include <boost/python/tuple.hpp>
#include <boost/python/suite/indexing/vector_indexing_suite.hpp>
#include <iostream>
#include <string>
#include <vector>

#include "../../include/T_GridState.h"
#include "../../include/E_GridState.h"
#include "../../include/GridDriver.h"


namespace   // Avoid cluttering the global namespace.
{

using namespace mpifps;

using boost::python::object;
using boost::python::extract;
using boost::python::list;
using boost::python::dict;
using boost::python::tuple;



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

struct TooManyGatewaysException : std::exception
{
  char const* what() const throw() { return "Number of gateways exceeded driver limit"; }
};

struct TooFewGatewaysException : std::exception
{
  char const* what() const throw() { return "Need to configure at least one gateway"; }
};

struct TooFewFPUsException : std::exception
{
  char const* what() const throw() { return "Waveform table needs to address at least one FPU."; }
};

struct TooFewStepsException : std::exception
{
  char const* what() const throw() { return "Waveform entry needs to contain at least one step."; }
};

void translate(TooManyGatewaysException const& e)
{
    // Use the Python 'C' API to set up an exception object
    PyErr_SetString(PyExc_RuntimeError, e.what());
}

void translate(TooFewGatewaysException const& e)
{
    // Use the Python 'C' API to set up an exception object
    PyErr_SetString(PyExc_RuntimeError, e.what());
}

void translate(TooFewFPUsException const& e)
{
    // Use the Python 'C' API to set up an exception object
    PyErr_SetString(PyExc_RuntimeError, e.what());
}

void translate(TooFewStepsException const& e)
{
    // Use the Python 'C' API to set up an exception object
    PyErr_SetString(PyExc_RuntimeError, e.what());
}


class WrapGatewayAddress : public t_gateway_address
    {
        public:
        WrapGatewayAddress()
            {
                ip = DEFAULT_GATEWAY_IP;
                port = DEFAULT_GATEWAY_PORT;
            };
        WrapGatewayAddress(const std::string& new_ip, const int new_port)
            {
                _ip = new_ip;
                ip = _ip.c_str();
                port = new_port;
            };
        WrapGatewayAddress(const std::string& new_ip)
            {
                _ip = new_ip;
                ip = _ip.c_str();
                port = DEFAULT_GATEWAY_PORT;
            };

        bool operator==(const  t_gateway_address &a) const
            {
                return (*this) == a;
            };
        private:
        std::string _ip;
        
    };

class WrapGridDriver : public GridDriver
{
    public:
    
    WrapGridDriver(int nfpus) : GridDriver(nfpus)
        {
        };

    
    E_DriverErrCode connectGateways(list& list_gateway_addresses)
        {
            const int actual_num_gw = len(list_gateway_addresses);
            
            int num_gateways = 0;
            t_gateway_address address_array[MAX_NUM_GATEWAYS];
            
            if (actual_num_gw > MAX_NUM_GATEWAYS)
            {
                throw TooManyGatewaysException();
            }
            if (actual_num_gw == 0)
            {
                throw TooFewGatewaysException();
            }
            
            for (int i=0; i < actual_num_gw; i++)
            {

                // extract entry
                WrapGatewayAddress address_entry =
                    extract<WrapGatewayAddress>(
                    list_gateway_addresses[i]);
                // cast (slice) to internal parameter type
                address_array[i] = static_cast<t_gateway_address>(
                    address_entry);
            }
            return connect(actual_num_gw, address_array);            
            
        };

    E_DriverErrCode configMotionWithDict(dict& dict_waveforms, WrapGridState& grid_state)
        {
            list fpu_id_list = dict_waveforms.keys();
            const int nkeys = len(fpu_id_list);

            if (nkeys == 0)
            {
                throw TooFewFPUsException();
            }

            t_wtable wtable;
            for(int i = 0; i < nkeys; i++)
            {
                object fpu_key = fpu_id_list[i];
                int fpu_id = extract<int>(fpu_key);
                list step_list = extract<list>(dict_waveforms[fpu_key]);
                int num_steps = len(step_list);

                if (num_steps == 0)
                {
                    throw TooFewStepsException();
                }

                std::vector<t_step_pair> steps;

                for(int j = 0; j < num_steps; j++)
                {
                    tuple tstep_pair = extract<tuple>(step_list[j]);
                    int16_t alpha_steps = extract<int>(tstep_pair[0]);
                    int16_t beta_steps = extract<int>(tstep_pair[1]);
                    t_step_pair step_pair;
                    step_pair.alpha_steps = alpha_steps;
                    step_pair.beta_steps = beta_steps;
                    steps.push_back(step_pair);                    
                }

                t_waveform wform;
                wform.fpu_id = fpu_id;
                wform.steps = steps;
                wtable.push_back(wform);
            }
            return configMotion(wtable, grid_state);
        };

    WrapGridState wrap_getGridState()
        {
            WrapGridState grid_state;
            getGridState(grid_state);
            return grid_state;
        }

    E_DriverErrCode wrap_initializeGrid(WrapGridState& grid_state)
        {
            return initializeGrid(grid_state);
        }

    E_DriverErrCode wrap_resetFPUs(WrapGridState& grid_state)
        {
            return resetFPUs(grid_state);
        }





    E_DriverErrCode wrap_getPositions(WrapGridState& grid_state)
        {
            return getPositions(grid_state);
        }

    E_DriverErrCode wrap_findDatum(WrapGridState& grid_state)
        {
            return findDatum(grid_state);
        }


    E_DriverErrCode wrap_executeMotion(WrapGridState& grid_state)
        {
            return executeMotion(grid_state);
        }

    E_DriverErrCode wrap_repeatMotion(WrapGridState& grid_state)
        {
            return repeatMotion(grid_state);
        }

    E_DriverErrCode wrap_reverseMotion(WrapGridState& grid_state)
        {
            return reverseMotion(grid_state);
        }

    E_DriverErrCode wrap_abortMotion(WrapGridState& grid_state)
        {
            return abortMotion(grid_state);
        }

    E_DriverErrCode wrap_lockFPU(WrapGridState& grid_state)
        {
            return lockFPU(grid_state);
        }

    E_DriverErrCode wrap_unlockFPU(WrapGridState& grid_state)
        {
            return unlockFPU(grid_state);
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
    .value("FPST_UNINITIALIZED", FPST_UNINITIALIZED       )
    .value("FPST_LOCKED", FPST_LOCKED              )
    .value("FPST_DATUM_SEARCH", FPST_DATUM_SEARCH        )
    .value("FPST_AT_DATUM", FPST_AT_DATUM         )
    .value("FPST_LOADING", FPST_LOADING             )
    .value("FPST_READY_FORWARD", FPST_READY_FORWARD       )
    .value("FPST_READY_BACKWARD", FPST_READY_BACKWARD      )
    .value("FPST_MOVING", FPST_MOVING              )
    .value("FPST_FINISHED", FPST_FINISHED            )
    .value("FPST_ABORTED", FPST_ABORTED             )
    .value("FPST_OBSTACLE_ERROR", FPST_OBSTACLE_ERROR  )
    .export_values();


    enum_<E_DriverState>("E_DriverState")
    .value("DS_UNINITIALIZED", DS_UNINITIALIZED       )
    .value("DS_UNCONNECTED", DS_UNCONNECTED         )
    .value("DS_CONNECTED", DS_CONNECTED           )
    .value("DS_ASSERTION_FAILED", DS_ASSERTION_FAILED    )
    .export_values();


    enum_<E_DriverErrCode>("E_DriverErrCode")
    .value("DE_OK",DE_OK)
    .value("DE_DRIVER_NOT_INITIALIZED",DE_DRIVER_NOT_INITIALIZED)
    .value("DE_DRIVER_ALREADY_INITIALIZED",DE_DRIVER_ALREADY_INITIALIZED)
    .value("DE_DRIVER_ALREADY_INITIALIZED",DE_DRIVER_ALREADY_INITIALIZED)
    .value("DE_NO_CONNECTION",DE_NO_CONNECTION)
    .value("DE_STILL_BUSY",DE_STILL_BUSY)
    .value("DE_UNRESOLVED_COLLISION",DE_UNRESOLVED_COLLISION)
    .value("DE_DRIVER_NOT_INITIALIZED",DE_DRIVER_NOT_INITIALIZED)
    .value("DE_FPU_NOT_INITIALIZED",DE_FPU_NOT_INITIALIZED)
    .value("DE_DRIVER_ALREADY_CONNECTED",DE_DRIVER_ALREADY_CONNECTED)
    .value("DE_DRIVER_STILL_CONNECTED",DE_DRIVER_ALREADY_CONNECTED)
    .value("DE_ASSERTION_FAILED",DE_ASSERTION_FAILED)

.export_values();
    
    enum_<E_GridState>("E_GridState")
    .value("GS_UNKNOWN", GS_UNKNOWN         )
    .value("GS_UNINITIALIZED", GS_UNINITIALIZED   )
    .value("GS_LEAVING_DATUM", GS_LEAVING_DATUM   )
    .value("GS_ABOVE_DATUM", GS_ABOVE_DATUM     )
    .value("GS_DATUM_SEARCH", GS_DATUM_SEARCH    )
    .value("GS_AT_DATUM", GS_AT_DATUM     )
    .value("GS_LOADING", GS_LOADING         )
    .value("GS_READY_FORWARD", GS_READY_FORWARD   )
    .value("GS_READY_BACKWARD", GS_READY_BACKWARD  )
    .value("GS_MOVING", GS_MOVING          )
    .value("GS_FINISHED", GS_FINISHED        )
    .value("GS_COLLISION", GS_COLLISION       )
    .value("GS_ABORTED", GS_ABORTED         )
    .export_values();



    enum_<E_MOVEMENT_DIRECTION>("E_MOVEMENT_DIRECTION")
        .value("DIRST_UNKNOWN"         , DIRST_UNKNOWN         )
        .value("DIRST_ANTI_CLOCKWISE"  , DIRST_ANTI_CLOCKWISE  )
        .value("DIRST_CLOCKWISE"       , DIRST_CLOCKWISE       )
        // the following two might not be needed
        .value("DIRST_RESTING_LAST_CW" , DIRST_RESTING_LAST_CW )
        .value("DIRST_RESTING_LAST_ACW", DIRST_RESTING_LAST_ACW)
        .export_values();

    
    class_<t_fpu_state>("FPUState")
    .def_readonly("state", &t_fpu_state::state)
    .def_readonly("alpha_steps", &t_fpu_state::alpha_steps)
    .def_readonly("beta_steps", &t_fpu_state::beta_steps)
    .def_readonly("was_zeroed", &t_fpu_state::was_zeroed)
    .def_readonly("is_locked", &t_fpu_state::is_locked)
    .def_readonly("alpha_datum_switch_active", &t_fpu_state::alpha_datum_switch_active)
    .def_readonly("beta_datum_switch_active", &t_fpu_state::beta_datum_switch_active)
    .def_readonly("at_alpha_limit", &t_fpu_state::at_alpha_limit)
    .def_readonly("beta_collision", &t_fpu_state::beta_collision)
    .def_readonly("direction_alpha", &t_fpu_state::direction_alpha)
    .def_readonly("direction_beta", &t_fpu_state::direction_beta)
    .def_readonly("ping_ok", &t_fpu_state::ping_ok)
    .def_readonly("waveform_valid", &t_fpu_state::waveform_valid)
    .def_readonly("waveform_ready", &t_fpu_state::waveform_ready)
    .def_readonly("waveform_reversed", &t_fpu_state::waveform_reversed)
    ;


    class_<std::vector<t_fpu_state> >("StateVec")
    .def(vector_indexing_suite<std::vector<t_fpu_state> >());

    class_<std::vector<long> >("IntVec")
    .def(vector_indexing_suite<std::vector<long> >());

    class_<std::vector<WrapGatewayAddress> >("GatewayAddressVec")
        .def(vector_indexing_suite<std::vector<WrapGatewayAddress> >());

    class_<WrapGridState>("GridState")
    .def_readonly("FPU", &WrapGridState::getStateVec)
    .def_readonly("Counts", &WrapGridState::getCounts)
    .def_readonly("count_timeout", &WrapGridState::count_timeout)
    .def_readonly("count_pending", &WrapGridState::count_pending)
    .def_readonly("driver_state", &WrapGridState::driver_state)
    ;

    class_<WrapGatewayAddress>("GatewayAddress", init<const char*, int>())
        .def_readwrite("ip", &WrapGatewayAddress::ip)
        .def_readwrite("port", &WrapGatewayAddress::port);

    class_<WrapGridDriver, boost::noncopyable>("GridDriver", init<int>())
    .def("connect", &WrapGridDriver::connectGateways)
    .def("disconnect", &WrapGridDriver::disconnect)
    .def("initializeDriver", &WrapGridDriver::initializeDriver)
    .def("deInitializeDriver", &WrapGridDriver::deInitializeDriver)
    .def("initializeGrid", &WrapGridDriver::wrap_initializeGrid)
    .def("resetFPUs", &WrapGridDriver::wrap_resetFPUs)
    .def("getPositions", &WrapGridDriver::wrap_getPositions)
    .def("findDatum", &WrapGridDriver::wrap_findDatum)
        // call signature is configMotion({ fpuid0 : {(asteps,bsteps), (asteps, bsteps), ...], fpuid1 : { ... }, ...}})
    .def("configMotion", &WrapGridDriver::configMotionWithDict)
    .def("executeMotion", &WrapGridDriver::wrap_executeMotion)
    .def("getGridState", &WrapGridDriver::wrap_getGridState)
    .def("getPositions", &WrapGridDriver::wrap_getPositions)
    .def("repeatMotion", &WrapGridDriver::wrap_repeatMotion)
    .def("reverseMotion", &WrapGridDriver::wrap_reverseMotion)
    .def("abortMotion", &WrapGridDriver::wrap_abortMotion)
    .def("lockFPU", &WrapGridDriver::lockFPU)
    .def("unlockFPU", &WrapGridDriver::unlockFPU)
    .def_readonly("NumFPUs", &WrapGridDriver::getNumFPUs)
    ;

}
