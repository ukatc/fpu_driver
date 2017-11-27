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
        WrapGatewayAddress(const char * new_ip, const int new_port)
            {
                ip = new_ip;
                port = new_port;
            };
        WrapGatewayAddress(const char * new_ip)
            {
                ip = new_ip;
                port = DEFAULT_GATEWAY_PORT;
            };

        bool operator==(const  t_gateway_address &a) const
            {
                return (*this) == a;
            };
        
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

    E_DriverErrCode configMotionWithDict(dict& dict_waveforms, t_grid_state& grid_state)
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
    .value("FPST_AT_DATUM", FPST_AT_DATUM         )
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


    enum_<E_DriverErrCode>("E_DriverErrCode")
    .value("DE_OK",DE_OK)
    .value("DE_DRIVER_NOT_INITIALISED",DE_DRIVER_NOT_INITIALISED)
    .value("DE_DRIVER_ALREADY_INITIALISED",DE_DRIVER_ALREADY_INITIALISED)
    .value("DE_NO_CONNECTION",DE_NO_CONNECTION)
    .value("DE_STILL_BUSY",DE_STILL_BUSY)
    .value("DE_UNRESOLVED_COLLISION",DE_UNRESOLVED_COLLISION)
    .value("DE_NOT_INITIALISED",DE_NOT_INITIALISED)
    .value("DE_ASSERTION_FAILED",DE_ASSERTION_FAILED)
    .export_values();
    
    enum_<E_GridState>("E_GridState")
    .value("GS_UNKNOWN", GS_UNKNOWN         )
    .value("GS_UNINITIALISED", GS_UNINITIALISED   )
    .value("GS_LEAVING_DATUM", GS_LEAVING_DATUM   )
    .value("GS_ABOVE_DATUM", GS_ABOVE_DATUM     )
    .value("GS_DATUM_SEARCH", GS_DATUM_SEARCH    )
    .value("GS_AT_DATUM", GS_AT_DATUM     )
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
    .def_readonly("state", &t_fpu_state::state)
    .def_readonly("alpha_steps", &t_fpu_state::alpha_steps)
    .def_readonly("beta_steps", &t_fpu_state::beta_steps)
    .def_readonly("is_initialized", &t_fpu_state::is_initialized)
    .def_readonly("on_alpha_datum", &t_fpu_state::on_alpha_datum)
    .def_readonly("on_beta_datum", &t_fpu_state::on_beta_datum)
    .def_readonly("alpha_collision", &t_fpu_state::alpha_collision)
    .def_readonly("at_alpha_limit", &t_fpu_state::at_alpha_limit)
    .def_readonly("beta_collision", &t_fpu_state::beta_collision)
    .def_readonly("ping_ok", &t_fpu_state::ping_ok)
    ;


    class_<std::vector<t_fpu_state> >("StateVec")
    .def(vector_indexing_suite<std::vector<t_fpu_state> >());

    class_<std::vector<long> >("IntVec")
    .def(vector_indexing_suite<std::vector<long> >());

    class_<std::vector<WrapGatewayAddress> >("GatewayAddressVec")
        .def(vector_indexing_suite<std::vector<WrapGatewayAddress> >());

    class_<WrapGridState>("GridState")
    .def_readonly("Fpu_state", &WrapGridState::getStateVec)
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
    .def("initializeGrid", &WrapGridDriver::initializeGrid)
    .def("resetFPUs", &WrapGridDriver::resetFPUs)
    .def("findDatum", &WrapGridDriver::findDatum)
    .def("getPositions", &WrapGridDriver::getPositions)
    .def("autoFindDatum", &WrapGridDriver::autoFindDatum)
        // call signature is configMotion({ fpuid0 : {(asteps,bsteps), (asteps, bsteps), ...], fpuid1 : { ... }, ...}})
    .def("configMotion", &WrapGridDriver::configMotionWithDict)
    .def("executeMotion", &WrapGridDriver::executeMotion)
    .def("getPositions", &WrapGridDriver::getPositions)
    .def("repeatMotion", &WrapGridDriver::repeatMotion)
    .def("reverseMotion", &WrapGridDriver::reverseMotion)
    .def("abortMotion", &WrapGridDriver::abortMotion)
    .def("assignPositions", &WrapGridDriver::assignPositions)
    .def("lockFPU", &WrapGridDriver::lockFPU)
    .def("unlockFPU", &WrapGridDriver::unlockFPU)
    ;

}
