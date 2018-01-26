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

#include "../../include/canlayer/E_CAN_COMMAND.h"
#include "../../include/T_GridState.h"
#include "../../include/E_GridState.h"
#include "../../include/GridDriver.h"
#include "../../include/GridState.h"


namespace   // Avoid cluttering the global namespace.
{

using namespace mpifps;

using namespace mpifps::canlayer;

using boost::python::object;
using boost::python::extract;
using boost::python::list;
using boost::python::dict;
using boost::python::tuple;


///   
///   // A friendly class.
///   class hello
///   {
///   public:
///       hello(const std::string& country)
///       {
///           this->country = country;
///       }
///       std::string greet() const
///       {
///           return "Hello from " + country;
///       }
///   private:
///       std::string country;
///   };
///   
///   // A function taking a hello object as an argument.
///   std::string invite(const hello& w)
///   {
///       return w.greet() + "! Please come soon!";
///   }
///


class WrapFPUState : public t_fpu_state
{

    public:

    bool was_zeroed;
    bool is_locked;
    bool alpha_datum_switch_active;
    bool beta_datum_switch_active;
    bool at_alpha_limit;
    bool beta_collision;
    bool waveform_valid;
    bool waveform_ready; 
    bool waveform_reversed; 

    WrapFPUState(){}

    WrapFPUState(const t_fpu_state& fpu_state)
    {
        memcpy(cmd_timeouts,fpu_state.cmd_timeouts, sizeof(cmd_timeouts));
        last_updated              = fpu_state.last_updated;                 
        pending_command_set       = fpu_state.pending_command_set;          
        state                     = fpu_state.state;                        
        last_command              = fpu_state.last_command;
        last_status               = fpu_state.last_status;
        alpha_steps               = fpu_state.alpha_steps;                  
        beta_steps                = fpu_state.beta_steps;                   
        timeout_count             = fpu_state.timeout_count;                
        direction_alpha           = fpu_state.direction_alpha;              
        direction_beta            = fpu_state.direction_beta;               
        num_waveforms             = fpu_state.num_waveforms;                
        num_active_timeouts       = fpu_state.num_active_timeouts;          
        sequence_number           = fpu_state.sequence_number;              
        was_zeroed                = fpu_state.was_zeroed;                   
        is_locked                 = fpu_state.is_locked;                    
        alpha_datum_switch_active = fpu_state.alpha_datum_switch_active;    
        beta_datum_switch_active  = fpu_state.beta_datum_switch_active;     
        at_alpha_limit            = fpu_state.at_alpha_limit;               
        beta_collision            = fpu_state.beta_collision;               
        waveform_valid            = fpu_state.waveform_valid;               
        waveform_ready            = fpu_state.waveform_ready;               
        waveform_reversed         = fpu_state.waveform_reversed;            
        
    }
    
    bool operator==(const  WrapFPUState &a) const
    {
        return (*this) == a;
    }

};

class WrapGridState : public t_grid_state
{
public:
    std::vector<WrapFPUState>getStateVec()
    {
        std::vector<WrapFPUState> state_vec;
        for (int i=0; i < MAX_NUM_POSITIONERS; i++)
        {
            WrapFPUState fpu_state(FPU_state[i]);
                state_vec.push_back(fpu_state);
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

E_GridState wrapGetGridStateSummary(WrapGridState& grid_state)
{
    return getGridStateSummary(grid_state);
}


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

    E_DriverErrCode wrap_pingFPUs(WrapGridState& grid_state)
        {
            return pingFPUs(grid_state);
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
    
     // include summary function
     def("getGridStateSummary", wrapGetGridStateSummary);

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
    .value("FPST_RESTING", FPST_RESTING            )
    .value("FPST_ABORTED", FPST_ABORTED             )
    .value("FPST_OBSTACLE_ERROR", FPST_OBSTACLE_ERROR  )
    .export_values();


    enum_<E_DriverState>("E_DriverState")
    .value("DS_UNINITIALIZED", DS_UNINITIALIZED       )
    .value("DS_UNCONNECTED", DS_UNCONNECTED         )
    .value("DS_CONNECTED", DS_CONNECTED           )
    .value("DS_ASSERTION_FAILED", DS_ASSERTION_FAILED    )
    .export_values();

    /* The following codes are used in the last_status flag.  These
       values depend on the firmware protocol. It is legitimate to use
       them for engineering and troubleshooting but thy should *not*
       be used by normal driver client code.
     */
    enum_<E_MOC_ERRCODE>("E_MOC_ERRCODE")
    .value("_ER_OK"            ,ER_OK            )
    .value("_ER_COLLIDE"       ,ER_COLLIDE       )
    .value("_ER_INVALID"       ,ER_INVALID       )
    .value("_ER_WAVENRDY"      ,ER_WAVENRDY      )
    .value("_ER_WAVE2BIG"      ,ER_WAVE2BIG      )
    .value("_ER_TIMING"        ,ER_TIMING        )
    .value("_ER_M1LIMIT"       ,ER_M1LIMIT       )
    .value("_ER_PARAM"         ,ER_PARAM         )
    .value("_ER_OK_UNCONFIRMED",ER_OK_UNCONFIRMED)
    .value("_ER_TIMEDOUT"      ,ER_TIMEDOUT      )
    .export_values();


    enum_<E_CAN_COMMAND>("E_CAN_COMMAND")
        .value("CCMD_NO_COMMAND", CCMD_NO_COMMAND)                       
        .value("CCMD_CONFIG_MOTION", CCMD_CONFIG_MOTION)                    
        .value("CCMD_EXECUTE_MOTION", CCMD_EXECUTE_MOTION)                   
        .value("CCMD_ABORT_MOTION", CCMD_ABORT_MOTION)                     
#if    (CAN_PROTOCOL_VERSION == 1)
        .value("CCMD_GET_STEPS_ALPHA", CCMD_GET_STEPS_ALPHA)                  
        .value("CCMD_GET_STEPS_BETA", CCMD_GET_STEPS_BETA)                   
        .value("CCMD_GET_ERROR_ALPHA", CCMD_GET_ERROR_ALPHA)                  
        .value("CCMD_GET_ERROR_BETA", CCMD_GET_ERROR_BETA)                   
#else
        .value("CCMD_LOCK_UNIT", CCMD_LOCK_UNIT)                        
        .value("CCMD_UNLOCK_UNIT", CCMD_UNLOCK_UNIT)                      
        .value("CCMD_GET_COUNTER_DEVIATION", CCMD_GET_COUNTER_DEVIATION)            
        .value("CCMD_GET_FIRMWARE_VERSION", CCMD_GET_FIRMWARE_VERSION)             
        .value("CCMD_CHECK_INTEGRITY", CCMD_CHECK_INTEGRITY)                  
        .value("CCMD_FREE_ALPHA_LIMIT_BREACH", CCMD_FREE_ALPHA_LIMIT_BREACH)          
        .value("CCMD_ENABLE_ALPHA_LIMIT_PROTECTION", CCMD_ENABLE_ALPHA_LIMIT_PROTECTION)    
        .value("CCMD_SET_TIME_STEP", CCMD_SET_TIME_STEP)                    
        .value("CCMD_SET_STEPS_PER_FRAME", CCMD_SET_STEPS_PER_FRAME)              
        .value("CCMD_ENABLE_MOVE", CCMD_ENABLE_MOVE)                      
#endif
        .value("CCMD_READ_REGISTER", CCMD_READ_REGISTER)                    
        .value("CCMD_PING_FPU", CCMD_PING_FPU)                         
        .value("CCMD_RESET_FPU", CCMD_RESET_FPU)                        
        .value("CCMD_FIND_DATUM", CCMD_FIND_DATUM)                       
        .value("CCMD_RESET_STEPCOUNTER", CCMD_RESET_STEPCOUNTER)                
        .value("CCMD_REPEAT_MOTION", CCMD_REPEAT_MOTION)                    
        .value("CCMD_REVERSE_MOTION", CCMD_REVERSE_MOTION)                   
        .value("CCMD_ENABLE_BETA_COLLISION_PROTECTION", CCMD_ENABLE_BETA_COLLISION_PROTECTION) 
        .value("CCMD_FREE_BETA_COLLISION", CCMD_FREE_BETA_COLLISION)              
        .value("CCMD_SET_USTEP", CCMD_SET_USTEP)                        
   
   
   
#if    (CAN_PROTOCOL_VERSION == 1)       
        .value("CMSG_FINISHED_MOTION", CMSG_FINISHED_MOTION)               
        .value("CMSG_FINISHED_DATUM", CMSG_FINISHED_DATUM)                
        .value("CMSG_WARN_COLLISION_BETA", CMSG_WARN_COLLISION_BETA)           
        .value("CMSG_WARN_LIMIT_ALPHA", CMSG_WARN_LIMIT_ALPHA)              
#else
        .value("CMSG_FINISHED_MOTION", CMSG_FINISHED_MOTION)               
        .value("CMSG_FINISHED_DATUM", CMSG_FINISHED_DATUM)                
        .value("CMSG_WARN_COLLISION_BETA", CMSG_WARN_COLLISION_BETA)           
        .value("CMSG_WARN_LIMIT_ALPHA", CMSG_WARN_LIMIT_ALPHA)              
        .value("CMSG_WARN_TIMEOUT_DATUM", CMSG_WARN_TIMEOUT_DATUM)            
#endif
   
        .value("NUM_CAN_COMMANDS", NUM_CAN_COMMANDS)                        
    .export_values();
   
    
    enum_<E_DriverErrCode>("E_DriverErrCode")
    .value("DE_OK",DE_OK)
    .value("DE_DRIVER_NOT_INITIALIZED",DE_DRIVER_NOT_INITIALIZED)
    .value("DE_DRIVER_ALREADY_INITIALIZED",DE_DRIVER_ALREADY_INITIALIZED)
    .value("DE_DRIVER_ALREADY_INITIALIZED",DE_DRIVER_ALREADY_INITIALIZED)
    .value("DE_NO_CONNECTION",DE_NO_CONNECTION)
    .value("DE_STILL_BUSY",DE_STILL_BUSY)
    .value("DE_MAX_RETRIES_EXCEEDED", DE_MAX_RETRIES_EXCEEDED)
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

    
    class_<WrapFPUState>("FPUState")
    .def_readonly("state", &WrapFPUState::state)
    .def_readonly("last_command", &WrapFPUState::last_command)
    .def_readonly("_last_status", &WrapFPUState::last_status)
    .def_readonly("alpha_steps", &WrapFPUState::alpha_steps)
    .def_readonly("beta_steps", &WrapFPUState::beta_steps)
    .def_readonly("was_zeroed", &WrapFPUState::was_zeroed)
    .def_readonly("is_locked", &WrapFPUState::is_locked)
    .def_readonly("alpha_datum_switch_active", &WrapFPUState::alpha_datum_switch_active)
    .def_readonly("beta_datum_switch_active", &WrapFPUState::beta_datum_switch_active)
    .def_readonly("at_alpha_limit", &WrapFPUState::at_alpha_limit)
    .def_readonly("beta_collision", &WrapFPUState::beta_collision)
    .def_readonly("direction_alpha", &WrapFPUState::direction_alpha)
    .def_readonly("direction_beta", &WrapFPUState::direction_beta)
    .def_readonly("waveform_valid", &WrapFPUState::waveform_valid)
    .def_readonly("waveform_ready", &WrapFPUState::waveform_ready)
    .def_readonly("waveform_reversed", &WrapFPUState::waveform_reversed)
    ;


    class_<std::vector<WrapFPUState> >("StateVec")
    .def(vector_indexing_suite<std::vector<WrapFPUState> >());

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
    .def("pingFPUs", &WrapGridDriver::wrap_pingFPUs)
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
