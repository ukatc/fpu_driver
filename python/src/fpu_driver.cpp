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
#include <boost/python/operators.hpp>
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


std::ostringstream& operator<<(std::ostringstream &out, const E_FPU_STATE &s)
{
    switch(s)
    {
    case FPST_UNKNOWN:
        out << "'UNKNOWN'";
        break;
    case FPST_UNINITIALIZED:
        out << "'UNINITIALIZED'";
        break;
    case FPST_LOCKED:
        out << "'LOCKED'";
        break;
    case FPST_DATUM_SEARCH:
        out << "'DATUM_SEARCH'";
        break;
    case FPST_AT_DATUM:
        out << "'AT_DATUM'";
        break;
    case FPST_LOADING:
        out << "'LOADING'";
        break;
    case FPST_READY_FORWARD:
        out << "'READY_FORWARD'";
        break;
    case FPST_READY_BACKWARD:
        out << "'READY_BACKWARD'";
        break;
    case FPST_MOVING:
        out << "'MOVING'";
        break;
    case FPST_RESTING:
        out << "'RESTING'";
        break;
    case FPST_ABORTED:
        out << "'ABORTED'";
        break;
    case FPST_OBSTACLE_ERROR:
        out << "'OBSTACLE_ERROR'";
        break;
    }
    return out;
}

std::ostringstream& operator<<(std::ostringstream &out, const E_DriverState &s)
{
    switch(s)
    {
    case DS_UNINITIALIZED:
        out << "'DS_UNINITIALIZED'";
        break;
    case DS_UNCONNECTED:
        out << "'DS_UNCONNECTED'";
        break;
    case DS_CONNECTED:
        out << "'DS_CONNECTED'";
        break;
    case DS_ASSERTION_FAILED:
        out << "'DS_ASSERTION_FAILED'";
        break;
    }
    return out;
}



class WrapFPUState : public t_fpu_state
{

public:

    bool was_zeroed;
    bool is_locked;
    bool ping_ok;
    bool alpha_datum_switch_active;
    bool beta_datum_switch_active;
    bool at_alpha_limit;
    bool beta_collision;
    bool waveform_valid;
    bool waveform_ready;
    bool waveform_reversed;
    int num_waveform_segments;
    int num_active_timeouts;
    int sequence_number;
    int movement_complete;

    WrapFPUState() {}

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
        alpha_deviation           = fpu_state.alpha_deviation;
        beta_deviation            = fpu_state.beta_deviation;
        timeout_count             = fpu_state.timeout_count;
        step_timing_errcount      = fpu_state.step_timing_errcount;
        direction_alpha           = fpu_state.direction_alpha;
        direction_beta            = fpu_state.direction_beta;
        num_waveform_segments     = fpu_state.num_waveform_segments;
        num_active_timeouts       = fpu_state.num_active_timeouts;
        sequence_number           = fpu_state.sequence_number;
        was_zeroed                = fpu_state.was_zeroed;
        ping_ok                   = fpu_state.ping_ok;
        is_locked                 = fpu_state.is_locked;
        movement_complete         = fpu_state.movement_complete;
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

    static std::string to_repr(const WrapFPUState &fpu)
    {
        std::ostringstream s;
        /* The following formatting breaks the chain of method
           calls at some points, and starts a new statement, for
           example when left-shifting fpu.state.  The reason for
           this is that doing not so will break the overloading of
           operator<<() for the enumeration types.  Do not change
           this without extensive testing.  This is probably a bug
           in the streams standard library.
        */

        s << "{ 'last_updated' : " << s.precision(10) << (1.0 * fpu.last_updated.tv_sec
                + 1.0e-9 * fpu.last_updated.tv_nsec)
          << " 'pending_command_set' : " << fpu.pending_command_set
          << " 'pending_command_set' : " << fpu.pending_command_set
          << " 'state' : ";
        s << fpu.state
          << " 'last_command' : " << fpu.last_command
          << " 'last_status' : " << fpu.last_status
          << " 'alpha_steps' : " << fpu.alpha_steps
          << " 'beta_steps' : " << fpu.beta_steps
          << " 'alpha_deviation' : " << fpu.alpha_deviation
          << " 'beta_deviation' : " << fpu.beta_deviation
          << " 'timeout_count' : " << fpu.timeout_count
          << " 'step_timing_errcount' : " << fpu.step_timing_errcount
          << " 'direction_alpha' : " << fpu.direction_alpha
          << " 'direction_beta' : " << fpu.direction_beta
          << " 'num_waveform_segments' : " << fpu.num_waveform_segments
          << " 'num_active_timeouts' : " << fpu.num_active_timeouts
          << " 'sequence_number' : " << fpu.sequence_number
          << " 'ping_ok' : " << fpu.ping_ok
          << " 'movement_complete' : " << fpu.movement_complete
          << " 'was_zeroed' : " << fpu.was_zeroed
          << " 'is_locked' : " << fpu.is_locked
          << " 'alpha_datum_switch_active' : " << fpu.alpha_datum_switch_active
          << " 'beta_datum_switch_active' : " << fpu.beta_datum_switch_active
          << " 'at_alpha_limit' : " << fpu.at_alpha_limit
          << " 'beta_collision' : " << fpu.beta_collision
          << " 'waveform_valid' : " << fpu.waveform_valid
          << " 'waveform_ready' : " << fpu.waveform_ready
          << " 'waveform_reversed' : " << fpu.waveform_reversed
          << " }";
        return s.str();
    }


};

class WrapGridState : public t_grid_state
{
public:
    std::vector<WrapFPUState>getStateVec()
    {
        int count_fpus = 0;
        for(int k=0; k < NUM_FPU_STATES; k++)
        {
            count_fpus += Counts[k];
        }
        assert(count_fpus <= MAX_NUM_POSITIONERS);
        std::vector<WrapFPUState> state_vec;
        for (int i=0; i < count_fpus; i++)
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

    static std::string to_string(const WrapGridState &gs)
    {
        std::ostringstream s;
        // Please observe the comment to WrapFPUState::to_repr,
        // this applies here as well.
        s << "count_pending=" << gs.count_pending << ", "
          << "num_queued=" << gs.num_queued << ", "
          << "count_timeout=" << gs.count_timeout << ", "
          << "driver_state=";
        s << gs.driver_state << ", "
          << "Counts= [ ";
        int num_fpus = 0;
        for(int i=0; i < NUM_FPU_STATES; i++)
        {
            s << gs.Counts[i];
            num_fpus += gs.Counts[i];
            if (i < (NUM_FPU_STATES -1))
            {
                s <<", ";
            }
        }
        s << " ]" << ", FPU[0 : " << num_fpus << "]=..." ;
        return s.str();
    }

    static std::string to_repr(const WrapGridState &gs)
    {
        std::ostringstream s;
        // Please observe the comment to WrapFPUState::to_repr,
        // this applies here as well.
        s << "{ 'count_pending' :" << gs.count_pending << ", "
          << "'num_queued' :" << gs.num_queued << ", "
          << "'count_timeout' :" << gs.count_timeout << ", "
          << "'driver_state' :";
        s << gs.driver_state << ", "
          << "'Counts' : { ";

        int num_fpus = 0;
        for(int i=0; i < NUM_FPU_STATES; i++)
        {
            s << static_cast<E_FPU_STATE>(i)
              << " : "
              << gs.Counts[i];
            num_fpus += gs.Counts[i];
            if (i < (NUM_FPU_STATES -1))
            {
                s <<", ";
            }
        }
        s << " ]" << ", FPU[0 : " << num_fpus << "]=..." ;
        return s.str();
    }

};

E_GridState wrapGetGridStateSummary(WrapGridState& grid_state)
{
    return getGridStateSummary(grid_state);
}


struct TooManyGatewaysException : std::exception
{
    char const* what() const throw()
    {
        return "Number of EtherCAN gateways exceed driver limit";
    }
};

struct NoGatewaysException : std::exception
{
    char const* what() const throw()
    {
        return "Need to configure at least one EtherCAN gateway";
    }
};


struct InsufficientNumGatewaysException : std::exception
{
    char const* what() const throw()
    {
        return "DE_INSUFFICENT_NUM_GATEWAYS: The number of EtherCAN gateways"
            " configured is insufficient for the configured number of FPUs";
    }
};

struct TooFewFPUsException : std::exception
{
    char const* what() const throw()
    {
        return "DE_INVALID_WAVEFORM: Waveform table needs to address at least one FPU.";
    }
};

struct TooFewStepsException : std::exception
{
    char const* what() const throw()
    {
        return "DE_INVALID_WAVEFORM: Waveform entry needs to contain at least one step.";
    }
};

struct DriverNotInitializedException
    : std::exception


{
    char const* what() const throw()
    {
        return "DE_DRIVER_NOT_INITIALIZED: GridDriver was not initialized "
               "properly, possibly due to system error or out-of-memory condition.";
    }
};

void translate(DriverNotInitializedException const& e)
{
    // Use the Python 'C' API to set up an exception object
    PyErr_SetString(PyExc_RuntimeError, e.what());
}

struct DriverAlreadyInitializedException : std::exception
{
    char const* what() const throw()
    {
        return "DE_DRIVER_ALREADY_INITIALIZED: GridDriver was already initialized properly.";
    }
};

void translate(DriverAlreadyInitializedException const& e)
{
    // Use the Python 'C' API to set up an exception object
    PyErr_SetString(PyExc_RuntimeError, e.what());
}


struct NoConnectionException : std::exception
{
    char const* what() const throw()
    {
        return "DE_NO_CONNECTION: The FPU Driver is not connected to a gateway.";
    }
};

void translate(NoConnectionException const& e)
{
    // Use the Python 'C' API to set up an exception object
    PyErr_SetString(PyExc_RuntimeError, e.what());
}


struct DriverStillBusyException : std::exception
{
    char const* what() const throw()
    {
        return "DE_STILL_BUSY: The FPU driver is still busy working on a previosu command";
    }
};

void translate(DriverStillBusyException const& e)
{
    // Use the Python 'C' API to set up an exception object
    PyErr_SetString(PyExc_RuntimeError, e.what());
}

struct NewCollisionException : std::exception
{
    char const* what() const throw()
    {
        return "DE_NEW_COLLISION: A collision was detected, movement for this FPU aborted.";
    }
};

void translate(NewCollisionException const& e)
{
    // Use the Python 'C' API to set up an exception object
    PyErr_SetString(PyExc_RuntimeError, e.what());
}


struct NewLimitBreachException : std::exception
{
    char const* what() const throw()
    {
        return "DE_NEW_LIMIT_BREACH: An alpha limit breach was detected, movement for this FPU aborted.";
    }
};

void translate(NewLimitBreachException const& e)
{
    // Use the Python 'C' API to set up an exception object
    PyErr_SetString(PyExc_RuntimeError, e.what());
}

struct UnresolvedCollisionException : std::exception
{
    char const* what() const throw()
    {
        return "DE_UNRESOLVED_COLLISION: A previous collision, limit breach, or abort message needs to be resolved first";
    }
};

void translate(UnresolvedCollisionException const& e)
{
    // Use the Python 'C' API to set up an exception object
    PyErr_SetString(PyExc_RuntimeError, e.what());
}

struct FpuNotInitializedException : std::exception
{
    char const* what() const throw()
    {
        return "DE_FPU_NOT_INITIALIZED: A fibre positioner unit (FPU) was not initialized as"
               " required, needs to do a datum search first";
    }
};

void translate(FpuNotInitializedException const& e)
{
    // Use the Python 'C' API to set up an exception object
    PyErr_SetString(PyExc_RuntimeError, e.what());
}

struct DriverAlreadyConnectedException : std::exception
{
    char const* what() const throw()
    {
        return "DE_DRIVER_ALREADY_CONNECTED: Driver was already connected, would need to disconnect() first.";
    }
};

void translate(DriverAlreadyConnectedException const& e)
{
    // Use the Python 'C' API to set up an exception object
    PyErr_SetString(PyExc_RuntimeError, e.what());
}

struct DriverStillConnectedException : std::exception
{
    char const* what() const throw()
    {
        return "DE_DRIVER_STILL_CONNECTED: FPU driver is still connected";
    }
};

void translate(DriverStillConnectedException const& e)
{
    // Use the Python 'C' API to set up an exception object
    PyErr_SetString(PyExc_RuntimeError, e.what());
}

struct MaxRetriesExceededException : std::exception
{
    char const* what() const throw()
    {
        return "DE_MAX_RETRIES_EXCEEDED: A command could not be send in spite of several retries";
    }
};

void translate(MaxRetriesExceededException const& e)
{
    // Use the Python 'C' API to set up an exception object
    PyErr_SetString(PyExc_RuntimeError, e.what());
}


struct InvalidWaveformTooManySectionsException : std::exception
{
    char const* what() const throw()
    {
        return "DE_INVALID_WAVEFORM_TOO_MANY_SECTIONS: The passed waveform has too many sections.";
    }
};

void translate(InvalidWaveformTooManySectionsException const& e)
{
    // Use the Python 'C' API to set up an exception object
    PyErr_SetString(PyExc_RuntimeError, e.what());
}


struct InvalidWaveformRaggedException : std::exception
{
    char const* what() const throw()
    {
        return "DE_INVALID_WAVEFORM_RAGGED: The passed waveform has different number of sections for different FPUs.";
    }
};

void translate(InvalidWaveformRaggedException const& e)
{
    // Use the Python 'C' API to set up an exception object
    PyErr_SetString(PyExc_RuntimeError, e.what());
}

struct InvalidWaveformStepCountTooLargeException : std::exception
{
    char const* what() const throw()
    {
        return "DE_INVALID_WAVEFORM_STEP_COUNT_TOO_LARGE: The passed waveform has a section with too many steps.";
    }
};

void translate(InvalidWaveformStepCountTooLargeException const& e)
{
    // Use the Python 'C' API to set up an exception object
    PyErr_SetString(PyExc_RuntimeError, e.what());
}

struct InvalidWaveformChangeException : std::exception
{
    char const* what() const throw()
    {
        return "DE_INVALID_WAVEFORM_CHANGE: The passed waveform has an invalid change in step counts / speed between adjacent sections";
    }
};

void translate(InvalidWaveformChangeException const& e)
{
    // Use the Python 'C' API to set up an exception object
    PyErr_SetString(PyExc_RuntimeError, e.what());
}

struct InvalidWaveformTailException : std::exception
{
    char const* what() const throw()
    {
        return "DE_INVALID_WAVEFORM_TAIL: The passed waveform has an invalid tail section.";
    }
};

void translate(InvalidWaveformTailException const& e)
{
    // Use the Python 'C' API to set up an exception object
    PyErr_SetString(PyExc_RuntimeError, e.what());
}

struct WaveformNotReadyException : std::exception
{
    char const* what() const throw()
    {
        return "DE_WAVEFORM_NOT_READY: The FPU has no valid waveform configured for a movement.";
    }
};

void translate(WaveformNotReadyException const& e)
{
    // Use the Python 'C' API to set up an exception object
    PyErr_SetString(PyExc_RuntimeError, e.what());
}


struct FPUsNotCalibratedException : std::exception
{
    char const* what() const throw()
    {
        return "DE_FPUS_NOT_CALIBRATED: FPUs are lacking calibration by "
               "a findDatum operation. For engineering or recovery use, consider"
               " to set the 'check_protection' keyword argument to False,"
               " to disable hardware safety checks.";
    }
};

void translate(FPUsNotCalibratedException const& e)
{
    // Use the Python 'C' API to set up an exception object
    PyErr_SetString(PyExc_RuntimeError, e.what());
}


struct NoMovableFPUsException : std::exception
{
    char const* what() const throw()
    {
        return "DE_NO_MOVABLE_FPUS: No FPUs are currently movable.";
    }
};

void translate(NoMovableFPUsException const& e)
{
    // Use the Python 'C' API to set up an exception object
    PyErr_SetString(PyExc_RuntimeError, e.what());
}

struct CommandTimeOutException : std::exception
{
    char const* what() const throw()
    {
        return "DE_COMMAND_TIMEOUT: Response to a CAN command surpassed the "
               "configured maximum waiting time."
               "This can be caused by a broken connection or networking problems.";
    }
};


struct AbortedStateException : std::exception
{
    char const* what() const throw()
    {
        return "DE_ABORTED_STATE: There are FPUs in aborted state,"
               " because of an abortMotion command or a step timing error "
               "- use the resetFPUs command to reset state.";
    }
};

void translate(AbortedStateException const& e)
{
    // Use the Python 'C' API to set up an exception object
    PyErr_SetString(PyExc_RuntimeError, e.what());
}

struct FPUsLockedException : std::exception
{
    char const* what() const throw()
    {
        return "DE_FPUS_LOCKED: Some addressed FPUs are in locked state, they need to be unlocked first.";
    }
};

void translate(FPUsLockedException const& e)
{
    // Use the Python 'C' API to set up an exception object
    PyErr_SetString(PyExc_RuntimeError, e.what());
}


struct StepTimingException : std::exception
{
    char const* what() const throw()
    {
        return "DE_STEP_TIMING_ERROR: An FPU's controller"
               " generated a step timing error"
               " during movement. Possibly, reduce the microstepping level"
               " to compute the step frequency in time.";
    }
};

void translate(StepTimingException const& e)
{
    // Use the Python 'C' API to set up an exception object
    PyErr_SetString(PyExc_RuntimeError, e.what());
}


struct InvalidFPUIdException : std::exception
{
    char const* what() const throw()
    {
        return "DE_INVALID_FPU_ID: A passed FPU id is out of range.";
    }
};

void translate(InvalidFPUIdException const& e)
{
    // Use the Python 'C' API to set up an exception object
    PyErr_SetString(PyExc_RuntimeError, e.what());
}


struct InvalidParValueException : std::exception
{
    char const* what() const throw()
    {
        return "DE_INVALID_PAR_VALUE: The passed parameter value is invalid.";
    }
};

void translate(InvalidParValueException const& e)
{
    // Use the Python 'C' API to set up an exception object
    PyErr_SetString(PyExc_RuntimeError, e.what());
}

struct InvalidFPUStateException : std::exception
{
    char const* what() const throw()
    {
        return "DE_INVALID_FPU_STATE: Command not allowed for present FPU state.";
    }
};

void translate(InvalidFPUStateException const& e)
{
    // Use the Python 'C' API to set up an exception object
    PyErr_SetString(PyExc_RuntimeError, e.what());
}




struct UnimplementedException : std::exception
{
    char const* what() const throw()
    {
        return "DE_UNIMPLEMENTED: Command or operation not implemented for this protocol version";
    }
};

void translate(UnimplementedException const& e)
{
    // Use the Python 'C' API to set up an exception object
    PyErr_SetString(PyExc_RuntimeError, e.what());
}


struct AssertionFailedException : std::exception
{
    char const* what() const throw()
    {
        return "DE_ASSERTION_FAILED: The driver determined an internal logic error, "
               "should probably be terminated.";
    }
};

void translate(AssertionFailedException const& e)
{
    // Use the Python 'C' API to set up an exception object
    PyErr_SetString(PyExc_RuntimeError, e.what());
}


void translate(CommandTimeOutException const& e)
{
    // Use the Python 'C' API to set up an exception object
    PyErr_SetString(PyExc_RuntimeError, e.what());
}


void translate(TooManyGatewaysException const& e)
{
    // Use the Python 'C' API to set up an exception object
    PyErr_SetString(PyExc_RuntimeError, e.what());
}

void translate(NoGatewaysException const& e)
{
    // Use the Python 'C' API to set up an exception object
    PyErr_SetString(PyExc_RuntimeError, e.what());
}

void translate(InsufficientNumGatewaysException const& e)
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

void checkDriverError(E_DriverErrCode ecode)
{
    switch(ecode)
    {
    case DE_OK:
        break;

    case DE_DRIVER_NOT_INITIALIZED:
        throw DriverNotInitializedException();
        break;

    case DE_DRIVER_ALREADY_INITIALIZED:
        throw DriverAlreadyInitializedException();
        break;

    case DE_NO_CONNECTION :
        throw NoConnectionException();
        break;

    case DE_INSUFFICENT_NUM_GATEWAYS:
        throw InsufficientNumGatewaysException();
        break;

    case DE_STILL_BUSY:
        throw DriverStillBusyException();
        break;

    case DE_NEW_COLLISION :
        throw NewCollisionException();
        break;

    case DE_NEW_LIMIT_BREACH :
        throw NewLimitBreachException();
        break;

    case DE_UNRESOLVED_COLLISION :
        throw UnresolvedCollisionException();
        break;

    case DE_FPU_NOT_INITIALIZED:
        throw FpuNotInitializedException();
        break;

    case DE_DRIVER_ALREADY_CONNECTED :
        throw DriverAlreadyConnectedException();
        break;

    case DE_DRIVER_STILL_CONNECTED:
        throw DriverStillConnectedException();
        break;

    case DE_MAX_RETRIES_EXCEEDED :
        throw MaxRetriesExceededException();
        break;

    case DE_INVALID_WAVEFORM_TOO_MANY_SECTIONS :
        throw InvalidWaveformTooManySectionsException();
        break;

    case DE_INVALID_WAVEFORM_RAGGED :
        throw InvalidWaveformRaggedException();
        break;

    case DE_INVALID_WAVEFORM_STEPCOUNT_TOO_LARGE :
        throw InvalidWaveformStepCountTooLargeException();
        break;

    case DE_INVALID_WAVEFORM_CHANGE :
        throw InvalidWaveformChangeException();
        break;

    case DE_INVALID_WAVEFORM_TAIL :
        throw InvalidWaveformTailException();
        break;

    case DE_WAVEFORM_NOT_READY :
        throw WaveformNotReadyException();
        break;

    case DE_FPUS_NOT_CALIBRATED:
        throw FPUsNotCalibratedException();
        break;

    case DE_NO_MOVABLE_FPUS :
        throw NoMovableFPUsException();
        break;

    case DE_COMMAND_TIMEOUT :
        throw CommandTimeOutException();
        break;

    case DE_ABORTED_STATE :
        throw AbortedStateException();
        break;

    case DE_FPUS_LOCKED :
        throw FPUsLockedException();
        break;

    case DE_STEP_TIMING_ERROR:
        throw StepTimingException();
        break;


    case DE_INVALID_FPU_ID:
        throw InvalidFPUIdException();
        break;

    case DE_INVALID_FPU_STATE:
        throw InvalidFPUStateException();
        break;

    case DE_INVALID_PAR_VALUE:
        throw InvalidParValueException();
        break;

    case DE_UNIMPLEMENTED:
        throw UnimplementedException();
        break;

    case DE_ASSERTION_FAILED:
        throw AssertionFailedException();
        break;


    }
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

        E_DriverErrCode ecode = initializeDriver();
        checkDriverError(ecode);
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
            throw NoGatewaysException();
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
        E_DriverErrCode ecode = connect(actual_num_gw, address_array);
        checkDriverError(ecode);
        return ecode;

    };

    E_DriverErrCode configMotionWithDict(dict& dict_waveforms, WrapGridState& grid_state,
                                         bool check_protection)
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
        E_DriverErrCode ecode = configMotion(wtable, grid_state, check_protection);
        checkDriverError(ecode);
        return ecode;

    };

    WrapGridState wrap_getGridState()
    {
        WrapGridState grid_state;
        getGridState(grid_state);
        return grid_state;
    }

    E_DriverErrCode wrap_initializeGrid(WrapGridState& grid_state)
    {
        E_DriverErrCode ecode = initializeGrid(grid_state);
        checkDriverError(ecode);
        return ecode;
    }

    E_DriverErrCode wrap_resetFPUs(WrapGridState& grid_state)
    {
        E_DriverErrCode ecode = resetFPUs(grid_state);
        checkDriverError(ecode);
        return ecode;

    }

    E_DriverErrCode wrap_pingFPUs(WrapGridState& grid_state)
    {
        E_DriverErrCode ecode = pingFPUs(grid_state);
        checkDriverError(ecode);
        return ecode;
    }


    E_DriverErrCode wrap_getPositions(WrapGridState& grid_state)
    {
        E_DriverErrCode ecode = getPositions(grid_state);
        checkDriverError(ecode);
        return ecode;
    }

    E_DriverErrCode wrap_getCounterDeviation(WrapGridState& grid_state)
    {
        E_DriverErrCode ecode = getCounterDeviation(grid_state);
        checkDriverError(ecode);
        return ecode;
    }

    E_DriverErrCode wrap_findDatum(WrapGridState& grid_state)
    {
        E_DriverErrCode ecode = findDatum(grid_state);
        checkDriverError(ecode);
        return ecode;
    }

    E_DriverErrCode wrap_startFindDatum(WrapGridState& grid_state)
    {
        E_DriverErrCode ecode = startFindDatum(grid_state);
        checkDriverError(ecode);
        return ecode;
    }

    E_DriverErrCode wrap_waitFindDatum(WrapGridState& grid_state, double max_wait_time)
    {
        E_DriverErrCode estatus;
        bool finished = false;

        // FIXME: should return remaining wait time in tuple
        estatus =  waitFindDatum(grid_state, max_wait_time, finished);

        if (((! finished) && (estatus == DE_OK))
                || (estatus == DE_COMMAND_TIMEOUT))
        {
            estatus = DE_COMMAND_TIMEOUT;
            return estatus;
        }

        checkDriverError(estatus);
        return estatus;

    }


    E_DriverErrCode wrap_executeMotion(WrapGridState& grid_state)
    {
        E_DriverErrCode ecode =executeMotion(grid_state);
        checkDriverError(ecode);
        return ecode;
    }

    E_DriverErrCode wrap_startExecuteMotion(WrapGridState& grid_state)
    {
        E_DriverErrCode ecode =startExecuteMotion(grid_state);
        checkDriverError(ecode);
        return ecode;
    }

    E_DriverErrCode wrap_waitExecuteMotion(WrapGridState& grid_state, double max_wait_time)
    {
        E_DriverErrCode estatus;
        bool finished = false;

        // FIXME: should return remaining wait time in tuple
        estatus =  waitExecuteMotion(grid_state, max_wait_time, finished);
        if (((! finished) && (estatus == DE_OK))
                || (estatus == DE_COMMAND_TIMEOUT))
        {
            estatus = DE_COMMAND_TIMEOUT;
            return estatus;
        }

        checkDriverError(estatus);
        return estatus;
    }

    E_DriverErrCode wrap_repeatMotion(WrapGridState& grid_state)
    {
        E_DriverErrCode ecode =repeatMotion(grid_state);
        checkDriverError(ecode);
        return ecode;
    }

    E_DriverErrCode wrap_reverseMotion(WrapGridState& grid_state)
    {
        E_DriverErrCode ecode =reverseMotion(grid_state);
        checkDriverError(ecode);
        return ecode;
    }

    E_DriverErrCode wrap_abortMotion(WrapGridState& grid_state)
    {
        E_DriverErrCode ecode =abortMotion(grid_state);
        checkDriverError(ecode);
        return ecode;
    }

    E_DriverErrCode wrap_freeBetaCollision(int fpu_id, E_REQUEST_DIRECTION request_direction,
                                           WrapGridState& grid_state)
    {
        E_DriverErrCode ecode = freeBetaCollision(fpu_id, request_direction, grid_state);
        checkDriverError(ecode);
        return ecode;
    }

    E_DriverErrCode wrap_setUStepLevel(int ustep_level, WrapGridState& grid_state)
    {
        E_DriverErrCode ecode = setUStepLevel(ustep_level, grid_state);
        checkDriverError(ecode);
        return ecode;
    }

    E_DriverErrCode wrap_enableBetaCollisionProtection(WrapGridState& grid_state)
    {
        E_DriverErrCode ecode = enableBetaCollisionProtection(grid_state);
        checkDriverError(ecode);
        return ecode;
    }

    E_DriverErrCode wrap_lockFPU(WrapGridState& grid_state)
    {
        E_DriverErrCode ecode =lockFPU(grid_state);
        checkDriverError(ecode);
        return ecode;
    }

    E_DriverErrCode wrap_unlockFPU(WrapGridState& grid_state)
    {
        E_DriverErrCode ecode =unlockFPU(grid_state);
        checkDriverError(ecode);
        return ecode;
    }




};



}

BOOST_PYTHON_MODULE(fpu_driver)
{
    using namespace boost::python;

    scope().attr("__version__") = (strlen(VERSION) > 0) ?  (((const char*)VERSION) + 1) : "";

    scope().attr("CAN_PROTOCOL_VERSION") = CAN_PROTOCOL_VERSION;

    // include summary function
    def("getGridStateSummary", wrapGetGridStateSummary);


    enum_<E_FPU_STATE>("E_FPU_STATE")
    .value("FPST_UNKNOWN", FPST_UNKNOWN)
    .value("FPST_UNINITIALIZED", FPST_UNINITIALIZED)
    .value("FPST_LOCKED", FPST_LOCKED)
    .value("FPST_DATUM_SEARCH", FPST_DATUM_SEARCH)
    .value("FPST_AT_DATUM", FPST_AT_DATUM)
    .value("FPST_LOADING", FPST_LOADING)
    .value("FPST_READY_FORWARD", FPST_READY_FORWARD)
    .value("FPST_READY_BACKWARD", FPST_READY_BACKWARD)
    .value("FPST_MOVING", FPST_MOVING)
    .value("FPST_RESTING", FPST_RESTING)
    .value("FPST_ABORTED", FPST_ABORTED)
    .value("FPST_OBSTACLE_ERROR", FPST_OBSTACLE_ERROR)
    .export_values();


    enum_<E_DriverState>("E_DriverState")
    .value("DS_UNINITIALIZED", DS_UNINITIALIZED)
    .value("DS_UNCONNECTED", DS_UNCONNECTED)
    .value("DS_CONNECTED", DS_CONNECTED)
    .value("DS_ASSERTION_FAILED", DS_ASSERTION_FAILED)
    .export_values();

    /* The following codes are used in the last_status flag.  These
       values depend on the firmware protocol. It is legitimate to use
       them for engineering and troubleshooting but thy should *not*
       be used by normal driver client code.
     */
    enum_<E_MOC_ERRCODE>("E_MOC_ERRCODE")
    .value("_ER_OK",ER_OK)
    .value("_ER_COLLIDE",ER_COLLIDE)
    .value("_ER_INVALID",ER_INVALID)
    .value("_ER_WAVENRDY",ER_WAVENRDY)
    .value("_ER_WAVE2BIG",ER_WAVE2BIG)
    .value("_ER_TIMING",ER_TIMING)
    .value("_ER_M1LIMIT",ER_M1LIMIT)
    .value("_ER_PARAM",ER_PARAM)
    .value("_ER_OK_UNCONFIRMED",ER_OK_UNCONFIRMED)
    .value("_ER_TIMEDOUT",ER_TIMEDOUT)
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
    .value("CCMD_SET_USTEP_LEVEL", CCMD_SET_USTEP_LEVEL)



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
    .value("DE_NO_CONNECTION",DE_NO_CONNECTION)
    .value("DE_INSUFFICENT_NUM_GATEWAYS", DE_INSUFFICENT_NUM_GATEWAYS)
    .value("DE_STILL_BUSY",DE_STILL_BUSY)
    .value("DE_MAX_RETRIES_EXCEEDED", DE_MAX_RETRIES_EXCEEDED)
    .value("DE_UNRESOLVED_COLLISION",DE_UNRESOLVED_COLLISION)
    .value("DE_NEW_COLLISION", DE_NEW_COLLISION)
    .value("DE_NEW_LIMIT_BREACH", DE_NEW_LIMIT_BREACH)
    .value("DE_DRIVER_NOT_INITIALIZED",DE_DRIVER_NOT_INITIALIZED)
    .value("DE_FPU_NOT_INITIALIZED",DE_FPU_NOT_INITIALIZED)
    .value("DE_DRIVER_ALREADY_CONNECTED",DE_DRIVER_ALREADY_CONNECTED)
    .value("DE_DRIVER_STILL_CONNECTED",DE_DRIVER_ALREADY_CONNECTED)
    .value("DE_ASSERTION_FAILED",DE_ASSERTION_FAILED)
    .value("DE_INVALID_WAVEFORM_TOO_MANY_SECTIONS", DE_INVALID_WAVEFORM_TOO_MANY_SECTIONS)
    .value("DE_INVALID_WAVEFORM_RAGGED", DE_INVALID_WAVEFORM_RAGGED)
    .value("DE_INVALID_WAVEFORM_STEPCOUNT_TOO_LARGE", DE_INVALID_WAVEFORM_STEPCOUNT_TOO_LARGE)
    .value("DE_INVALID_WAVEFORM_CHANGE", DE_INVALID_WAVEFORM_CHANGE)
    .value("DE_INVALID_WAVEFORM_TAIL", DE_INVALID_WAVEFORM_TAIL)
    .value("DE_WAVEFORM_NOT_READY", DE_WAVEFORM_NOT_READY)
    .value("DE_NO_MOVABLE_FPUS", DE_NO_MOVABLE_FPUS)
    .value("DE_COMMAND_TIMEOUT", DE_COMMAND_TIMEOUT)
    .value("DE_ABORTED_STATE", DE_ABORTED_STATE)
    .value("DE_FPUS_LOCKED", DE_FPUS_LOCKED)
    .value("DE_STEP_TIMING_ERROR", DE_STEP_TIMING_ERROR)
    .value("DE_INVALID_FPU_ID", DE_INVALID_FPU_ID)
    .value("DE_INVALID_FPU_STATE", DE_INVALID_FPU_STATE)
    .value("DE_INVALID_PAR_VALUE", DE_INVALID_PAR_VALUE)
    .value("DE_UNIMPLEMENTED", DE_UNIMPLEMENTED)
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

    // direction of a movement request from the user
    enum_<E_REQUEST_DIRECTION>("E_REQUEST_DIRECTION")
    .value("REQD_ANTI_CLOCKWISE", REQD_ANTI_CLOCKWISE  )
    .value("REQD_CLOCKWISE", REQD_CLOCKWISE       )
    .export_values();

    // direction of the current or last actually recorded movement of each FPU
    enum_<E_MOVEMENT_DIRECTION>("E_MOVEMENT_DIRECTION")
    .value("DIRST_UNKNOWN", DIRST_UNKNOWN         )
    .value("DIRST_ANTI_CLOCKWISE", DIRST_ANTI_CLOCKWISE  )
    .value("DIRST_CLOCKWISE", DIRST_CLOCKWISE       )
    // the following two might not be needed
    .value("DIRST_RESTING_LAST_CW", DIRST_RESTING_LAST_CW )
    .value("DIRST_RESTING_LAST_ACW", DIRST_RESTING_LAST_ACW)
    .export_values();


    class_<WrapFPUState>("FPUState")
    .def_readonly("state", &WrapFPUState::state)
    .def_readonly("last_command", &WrapFPUState::last_command)
    .def_readonly("last_status", &WrapFPUState::last_status)
    .def_readonly("alpha_steps", &WrapFPUState::alpha_steps)
    .def_readonly("beta_steps", &WrapFPUState::beta_steps)
    .def_readonly("alpha_deviation", &WrapFPUState::alpha_deviation)
    .def_readonly("beta_deviation", &WrapFPUState::beta_deviation)
    .def_readonly("timeout_count", &WrapFPUState::timeout_count)
    .def_readonly("num_active_timeouts", &WrapFPUState::num_active_timeouts)
    .def_readonly("sequence_number", &WrapFPUState::sequence_number)
    .def_readonly("was_zeroed", &WrapFPUState::was_zeroed)
    .def_readonly("is_locked", &WrapFPUState::is_locked)
    .def_readonly("alpha_datum_switch_active", &WrapFPUState::alpha_datum_switch_active)
    .def_readonly("beta_datum_switch_active", &WrapFPUState::beta_datum_switch_active)
    .def_readonly("at_alpha_limit", &WrapFPUState::at_alpha_limit)
    .def_readonly("beta_collision", &WrapFPUState::beta_collision)
    .def_readonly("direction_alpha", &WrapFPUState::direction_alpha)
    .def_readonly("num_waveform_segments", &WrapFPUState::num_waveform_segments)
    .def_readonly("direction_beta", &WrapFPUState::direction_beta)
    .def_readonly("waveform_valid", &WrapFPUState::waveform_valid)
    .def_readonly("waveform_ready", &WrapFPUState::waveform_ready)
    .def_readonly("waveform_reversed", &WrapFPUState::waveform_reversed)
    .def_readonly("pending_command_set", &WrapFPUState::pending_command_set)
    .def("__repr__", &WrapFPUState::to_repr)
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
    .def("__str__", &WrapGridState::to_string)
    .def("__repr__", &WrapGridState::to_repr)
    ;

    class_<WrapGatewayAddress>("GatewayAddress", init<const char*, int>())
    .def_readwrite("ip", &WrapGatewayAddress::ip)
    .def_readwrite("port", &WrapGatewayAddress::port);

    class_<WrapGridDriver, boost::noncopyable>("GridDriver", init<int>())
    .def("getNumFPUs", &WrapGridDriver::getNumFPUs)
    .def("connect", &WrapGridDriver::connectGateways)
    .def("disconnect", &WrapGridDriver::disconnect)
    .def("deInitializeDriver", &WrapGridDriver::deInitializeDriver)
    .def("initializeGrid", &WrapGridDriver::wrap_initializeGrid)
    .def("resetFPUs", &WrapGridDriver::wrap_resetFPUs)
    .def("pingFPUs", &WrapGridDriver::wrap_pingFPUs)
    .def("getPositions", &WrapGridDriver::wrap_getPositions)
    .def("getCounterDeviation", &WrapGridDriver::wrap_getCounterDeviation)
    .def("findDatum", &WrapGridDriver::wrap_findDatum)
    .def("startFindDatum", &WrapGridDriver::wrap_startFindDatum)
    .def("waitFindDatum", &WrapGridDriver::wrap_waitFindDatum)
    .def("configMotion", &WrapGridDriver::configMotionWithDict)
    .def("executeMotion", &WrapGridDriver::wrap_executeMotion)
    .def("startExecuteMotion", &WrapGridDriver::wrap_startExecuteMotion)
    .def("waitExecuteMotion", &WrapGridDriver::wrap_waitExecuteMotion)
    .def("getGridState", &WrapGridDriver::wrap_getGridState)
    .def("repeatMotion", &WrapGridDriver::wrap_repeatMotion)
    .def("reverseMotion", &WrapGridDriver::wrap_reverseMotion)
    .def("abortMotion", &WrapGridDriver::wrap_abortMotion)
    .def("freeBetaCollision", &WrapGridDriver::wrap_freeBetaCollision)
    .def("setUStepLevel", &WrapGridDriver::wrap_setUStepLevel)
    .def("enableBetaCollisionProtection", &WrapGridDriver::wrap_enableBetaCollisionProtection)
    .def("lockFPU", &WrapGridDriver::lockFPU)
    .def("unlockFPU", &WrapGridDriver::unlockFPU)
    .def_readonly("NumFPUs", &WrapGridDriver::getNumFPUs)
    ;

}
