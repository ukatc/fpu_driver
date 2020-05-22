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
// NAME ProtectionDB.C
//
// TODO: Put description here
//
////////////////////////////////////////////////////////////////////////////////

#include "ProtectionDB.h"
#include "lmdb.h"

static const char *dbname = "fpu";
static const char *alpha_positions = "apos";
static const char *beta_positions = "bpos";
static const char *waveform_table = "wtab";
static const char *waveform_reversed = "wf_reversed";
static const char *alpha_limits = "alimits";
static const char *beta_limits = "blimits";
static const char *free_beta_retries = "bretries";
static const char *beta_retry_count_cw = "beta_retry_count_cw";
static const char *beta_retry_count_acw = "beta_retry_count_acw"; 

static const char *free_alpha_retries = "aretries";
static const char *alpha_retry_count_cw = "alpha_retry_count_cw";
static const char *alpha_retry_count_acw = "alpha_retry_count_acw";
static const char *counters = "counters2";
static const char *serialnumber_used = "serialnumber_used";

int ProtectionDB::doStuff()
{
    int major = 1;
    int minor = 2;
    int patch = 3;
    char *lmdb_version_str = mdb_version(&major, &minor, &patch);

    return 456;
}



