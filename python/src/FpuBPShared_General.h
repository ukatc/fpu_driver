// -*- mode: c++ -*-

////////////////////////////////////////////////////////////////////////////////
//
// Copyright 2020 UKRI. See file "LICENSE" for license information.
//
//
// Who       When        What
// --------  ----------  -------------------------------------------------------
// bwillemse 2020-05-13  TODO: Put comment here
//------------------------------------------------------------------------------

////////////////////////////////////////////////////////////////////////////////
// NAME FpuBPShared_General.h
//
// TODO: Put description here
//
////////////////////////////////////////////////////////////////////////////////

#ifndef FPUBPSHARED_GENERAL_H
#define FPUBPSHARED_GENERAL_H

#include <string.h>
#include <iostream>
#include <iomanip>
#include <string>
#include <vector>

#include <boost/python/class.hpp>
#include <boost/python/module.hpp>
#include <boost/python/def.hpp>
#include <boost/python/enum.hpp>
#include <boost/python/extract.hpp>
#include <boost/python/list.hpp>
#include <boost/python/dict.hpp>
#include <boost/python/tuple.hpp>
#include <boost/python/str.hpp>
#include <boost/python/suite/indexing/vector_indexing_suite.hpp>
#include <boost/python/operators.hpp>
#include <boost/python/exception_translator.hpp>
#include <boost/python/make_constructor.hpp>

#include "../../include/ethercan/E_CAN_COMMAND.h"
#include "../../include/T_GridState.h"
#include "../../include/E_GridState.h"
#include "../../include/EtherCANInterface.h"
#include "../../include/GridState.h"

using namespace mpifps;
using namespace mpifps::ethercanif;

using boost::python::object;
using boost::python::extract;
using boost::python::list;
using boost::python::dict;
using boost::python::tuple;
using boost::python::str;

using namespace boost::python;

namespace bp = boost::python;

std::ostringstream& operator<<(std::ostringstream &out, const E_FPU_STATE &s);
std::ostringstream& operator<<(std::ostringstream &out, const E_InterfaceState &s);
void checkInterfaceError(E_EtherCANErrCode ecode);


// -----------------------------------------------------------------------------
class WrapperSharedBase
{
    // TODO: Will this class need a constructor and/or (virtual) destructor
    // eventually?

protected:
    int convertGatewayAddresses(const bp::list &list_gateway_addresses,
                                t_gateway_address *address_array_to_fill);
    void getFPUSet(const bp::list &fpu_list, t_fpuset &fpuset) const;
    void getDatumFlags(bp::dict &dict_modes,
                       t_datum_search_flags &direction_flags,
                       const t_fpuset &fpuset);

    virtual const EtherCANInterfaceConfig &getConfig() const = 0;
};

// -----------------------------------------------------------------------------
class WrapFPUState : public t_fpu_state
{
public:
    bool alpha_was_referenced;
    bool beta_was_referenced;
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
    int waveform_status;
    int num_active_timeouts;
    int sequence_number;
    int movement_complete;
    int register_value;
    uint16_t register_address;
    int fw_version_major;
    int fw_version_minor;
    int fw_version_patch;
    int fw_date_year;
    int fw_date_month;
    int fw_date_day;
    int checksum_ok;
    double last_updated_sec;
    std::string serial_number;

    WrapFPUState() {}

    WrapFPUState(const t_fpu_state& fpu_state)
    {
        memcpy(cmd_timeouts,fpu_state.cmd_timeouts, sizeof(cmd_timeouts));
        pending_command_set       = fpu_state.pending_command_set;
        state                     = fpu_state.state;
        last_command              = fpu_state.last_command;
        sequence_number           = fpu_state.sequence_number;
        last_status               = fpu_state.last_status;
        alpha_steps               = fpu_state.alpha_steps;
        beta_steps                = fpu_state.beta_steps;
        alpha_deviation           = fpu_state.alpha_deviation;
        beta_deviation            = fpu_state.beta_deviation;
        timeout_count             = fpu_state.timeout_count;
        num_active_timeouts       = fpu_state.num_active_timeouts;
        step_timing_errcount      = fpu_state.step_timing_errcount;
        can_overflow_errcount     = fpu_state.can_overflow_errcount;
        direction_alpha           = fpu_state.direction_alpha;
        direction_beta            = fpu_state.direction_beta;
        num_waveform_segments     = fpu_state.num_waveform_segments;
        waveform_status           = fpu_state.waveform_status;
        sequence_number           = fpu_state.sequence_number;
        alpha_was_referenced      = fpu_state.alpha_was_referenced;
        beta_was_referenced       = fpu_state.beta_was_referenced;
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
        register_value            = fpu_state.register_value;
        register_address          = fpu_state.register_address;
        fw_version_major          = fpu_state.firmware_version[0];
        fw_version_minor          = fpu_state.firmware_version[1];
        fw_version_patch          = fpu_state.firmware_version[2];
        fw_date_year              = fpu_state.firmware_date[0];
        fw_date_month             = fpu_state.firmware_date[1];
        fw_date_day               = fpu_state.firmware_date[2];
        crc32                     = fpu_state.crc32;
        checksum_ok               = fpu_state.checksum_ok;
	last_updated_sec          = (1.0 * fpu_state.last_updated.tv_sec)
	  + (1.0e-9 * fpu_state.last_updated.tv_nsec);

        assert(strlen(fpu_state.serial_number) < LEN_SERIAL_NUMBER);
        serial_number = std::string(fpu_state.serial_number);
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

        s << "{ 'last_updated' : " << std::setprecision(10) << fpu.last_updated_sec << ", "
          << " 'pending_command_set' : " << fpu.pending_command_set << ", "
          << " 'state' : ";
        s << fpu.state << ", "
          << " 'last_command' : " << fpu.last_command << ", "
          << " 'last_status' : " << fpu.last_status << ", "
          << " 'alpha_steps' : " << fpu.alpha_steps << ", "
          << " 'beta_steps' : " << fpu.beta_steps << ", "
          << " 'alpha_deviation' : " << fpu.alpha_deviation << ", "
          << " 'beta_deviation' : " << fpu.beta_deviation << ", "
          << " 'timeout_count' : " << fpu.timeout_count << ", "
          << " 'step_timing_errcount' : " << fpu.step_timing_errcount << ", "
          << " 'can_overflow_errcount' : " << fpu.can_overflow_errcount << ", "
          << " 'direction_alpha' : " << fpu.direction_alpha << ", "
          << " 'direction_beta' : " << fpu.direction_beta << ", "
          << " 'num_waveform_segments' : " << fpu.num_waveform_segments << ", "
          << " 'waveform_status' : " << fpu.waveform_status << ", "
          << " 'num_active_timeouts' : " << fpu.num_active_timeouts << ", "
          << " 'sequence_number' : " << fpu.sequence_number << ", "
          << " 'ping_ok' : " << fpu.ping_ok << ", "
          << " 'movement_complete' : " << fpu.movement_complete << ", "
          << " 'alpha_was_referenced' : " << fpu.alpha_was_referenced << ", "
          << " 'beta_was_referenced' : " << fpu.beta_was_referenced << ", "
          << " 'is_locked' : " << fpu.is_locked << ", "
          << " 'alpha_datum_switch_active' : " << fpu.alpha_datum_switch_active << ", "
          << " 'beta_datum_switch_active' : " << fpu.beta_datum_switch_active << ", "
          << " 'at_alpha_limit' : " << fpu.at_alpha_limit << ", "
          << " 'beta_collision' : " << fpu.beta_collision << ", "
          << " 'waveform_valid' : " << fpu.waveform_valid << ", "
          << " 'waveform_ready' : " << fpu.waveform_ready << ", "
          << " 'waveform_reversed' : " << fpu.waveform_reversed << ", "
          << " 'register_address' : " << std::hex << std::showbase << fpu.register_address << ", "
          << " 'register_value' : " << fpu.register_value << std::dec << std::noshowbase << ", "
          << " 'firmware_version' : " << fpu.fw_version_major
          << "." << fpu.fw_version_minor
          << "." << fpu.fw_version_patch << ", "
          << " 'firmware_date' : '20" << std::setfill('0') << std::setw(2) << fpu.fw_date_year
          << "-" << std::setfill('0') << std::setw(2) << fpu.fw_date_month
          << "-" << std::setfill('0') << std::setw(2) << fpu.fw_date_day <<"', "
          << " 'serial_number' : \"" << fpu.serial_number << "\", "
          << " 'crc32' : " << std::hex << std::showbase << std::setfill('0') << std::setw(8) << fpu.crc32 << std::dec << ", "
          << " 'checksum_ok' : " << fpu.checksum_ok << ", "
          << " 'sequence_number' : " << fpu.sequence_number << ", "
          << " }";
        return s.str();
    }
};


// -----------------------------------------------------------------------------
class WrapGridState : public t_grid_state
{
public:
    std::vector<WrapFPUState>getStateVec()
    {
        int count_fpus = 0;
        for (int k = 0; k < NUM_FPU_STATES; k++)
        {
            count_fpus += Counts[k];
        }
        assert(count_fpus <= MAX_NUM_POSITIONERS);
        std::vector<WrapFPUState> state_vec;
        for (int i = 0; i < count_fpus; i++)
        {
            WrapFPUState fpu_state(FPU_state[i]);
            state_vec.push_back(fpu_state);
        }
        return state_vec;
    }

    std::vector<long>getCounts()
    {
        std::vector<long> count_vec;
        for (int i = 0; i < NUM_FPU_STATES; i++)
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
          << "interface_state=";
        s << gs.interface_state << ", "
          << "Counts= [ ";
        int num_fpus = 0;
        for(int i = 0; i < NUM_FPU_STATES; i++)
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
          << "'count_can_overflow' :" << gs.count_can_overflow << ", "
          << "'interface_state' :";
        s << gs.interface_state << ", "
          << "'Counts' : { ";

        int num_fpus = 0;
        for(int i = 0; i < NUM_FPU_STATES; i++)
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


// -----------------------------------------------------------------------------
class EtherCANException : public std::exception
{
public:
    EtherCANException(std::string message, mpifps::E_EtherCANErrCode errcode)
    {
        _message = message;
        _errcode = errcode;
    }

    const char *what() const throw()
    {
        return _message.c_str();
    }
    ~EtherCANException() throw()
    {
    }

    E_EtherCANErrCode getErrCode() const throw()
    {
        return _errcode;
    }

private:
    std::string _message;
    mpifps::E_EtherCANErrCode _errcode;
};


//------------------------------------------------------------------------------
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


// -----------------------------------------------------------------------------

#endif // FPUBPSHARED_GENERAL_H

