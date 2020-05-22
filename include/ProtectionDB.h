// -*- mode: c++ -*-

////////////////////////////////////////////////////////////////////////////////
//
// Copyright 2020 UKRI. See file "LICENSE" for license information.
//
//
// Who       When        What
// --------  ----------  -------------------------------------------------------
// bwillemse 2020-05-20  TODO: Put comment here
//------------------------------------------------------------------------------

////////////////////////////////////////////////////////////////////////////////
// NAME ProtectionDB.h
//
// TODO: Put description here
//
////////////////////////////////////////////////////////////////////////////////


#ifndef PROTECTIONDB_H
#define PROTECTIONDB_H

#include <vector>
#include <map>
#include <string>
#include "FPUState.h"
#include "AsyncInterface.h"
#include "lmdb.h"

using namespace mpifps;
using namespace mpifps::ethercanif;

// TODO: Decide if use C or C++ bindings for LMDB


/*
Probable data types (TODO: CHECK):
    - txn:           MDB_txn &txn?? See http://www.lmdb.tech/doc/group__internal.html#structMDB__txn
    - serial_number: const char serial_number[]  (from EtherCANInterface.h)
    - subkey:        const char subkey[]  (from FPUGridDriver.py)
    - interval:      double?
    - apos:          double (see FPU driver manual)
    - bpos:          double (see FPU driver manual)
    - offset:        double (see FPU driver manual)
    - val:           MDB_val *valPtr ?
    - fpu:           t_fpu_state
    - wentry:        std::vector<t_step_pair> steps ???? (see AsyncInterface.h -> t_waveform)
                     I defined as Wentry for now - see below

*/

// TODO: Move elsewhere, and rename to e.g. Waveform?
using Wentry = std::vector<AsyncInterface::t_step_pair>;

// TODO: The following is just a guess for now, and might not be very efficient - 
// need to check, and also see if there is already a suitable class or structure
// type elsewhere
using CounterVals = std::map<std::string, int64_t>; // Counter name, count value

// -----------------------------------------------------------------------------

class ProtectionDB
{
public:
    // TODO: For testing only
    int doStuff();


    // TODO: Check Python's @staticmethod, @classmethod (cls), other (self) - 
    // AND why does ProtectionDB Python class use these?

    // TODO: Not sure whether the following function prototypes are good 
    // correspondences to the corresponding Python functions yet - check and
    // update as go along, and also add const qualifiers wherever appropriate


    // Static functions
    static void putField(MDB_txn &txn, const char serial_number[],
                         const char subkey[], MDB_val *valPtr);
    static void putInterval(MDB_txn &txn, const char serial_number[],
                            const char subkey[], double interval,
                            double offset = 0.0);

    // Instance functions
    // TODO: Check if these could just be static functions as well - do they
    // use any private class data?

    // TODO: Should getRawField() and getField() actually return MDB_val * ?    

    MDB_val *getRawField(MDB_txn &txn, const char serial_number[],
                         const char subkey[]);
    MDB_val *getField(MDB_txn &txn, const t_fpu_state &fpu, const char subkey[]);
    void put_alpha_position(MDB_txn &txn, const t_fpu_state &fpu, double apos,
                            double aoffset);
    void put_beta_position(MDB_txn &txn, const t_fpu_state &fpu, double bpos);
    void store_reversed(MDB_txn &txn, const t_fpu_state &fpu, bool is_reversed);
    void storeWaveform(MDB_txn &txn, const t_fpu_state &fpu, const Wentry &wentry);
    void store_bretry_count(MDB_txn &txn, const t_fpu_state &fpu, bool clockwise,
                            int cnt);
    void store_aretry_count(MDB_txn &txn, const t_fpu_state &fpu, bool clockwise,
                            int cnt);

    // TODO: counter_vals: See end of _update_counters_execute_motion()?
    void put_counters(MDB_txn &txn, const t_fpu_state &fpu, CounterVals &counter_vals);
};

// -----------------------------------------------------------------------------

// TODO: Implement the following
/*
class HealthLogDB
{
public:
    void putEntry(MDB_txn &txn, const t_fpu_state &fpu, dict_counters);
    **TODO** getEntry(MDB_txn &txn, const t_fpu_state &fpu, datum_cnt, series=None);


};
*/

// -----------------------------------------------------------------------------

MDB_env *open_database_env(bool mockup = false);

// -----------------------------------------------------------------------------

#endif // PROTECTIONDB_H
