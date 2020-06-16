// -*- mode: c++ -*-

////////////////////////////////////////////////////////////////////////////////
//
// Copyright 2020 UKRI. See file "LICENSE" for license information.
//
//
// Who       When        What
// --------  ----------  -------------------------------------------------------
// bwillemse 2020-06-15  TODO: Put comment here
//------------------------------------------------------------------------------

////////////////////////////////////////////////////////////////////////////////
// NAME GridDriver.h
//
// TODO: Put description here
//
////////////////////////////////////////////////////////////////////////////////

#ifndef GRIDDRIVER_H
#define GRIDDRIVER_H

#include "UnprotectedGridDriver.h"

namespace mpifps
{


//==============================================================================

class GridDriver : public UnprotectedGridDriver
{
    //..........................................................................
public:
    GridDriver();


    //..........................................................................
private:    
    // The following hook functions override those in UnprotectedGridDriver

    void _post_connect_hook(const EtherCANInterfaceConfig &config) override;

    void _allow_find_datum_hook(t_grid_state &gs,
                    AsyncInterface::t_datum_search_flags &search_modes,
                    enum E_DATUM_SELECTION selected_arm,
                    const AsyncInterface::t_fpuset &fpuset,
                    bool support_uninitialized_auto) override;
    void _start_find_datum_hook(t_grid_state &gs,
                    const AsyncInterface::t_datum_search_flags &search_modes,
                    enum E_DATUM_SELECTION selected_arm,
                    const AsyncInterface::t_fpuset &fpuset,
                    FpuPositions &initial_positions_ret,
                    bool soft_protection) override;
    void _cancel_find_datum_hook(t_grid_state &gs,
                    const AsyncInterface::t_datum_search_flags &search_modes,
                    enum E_DATUM_SELECTION selected_arm,
                    const AsyncInterface::t_fpuset &fpuset,
                    const FpuPositions &initial_positions) override;
    void _finished_find_datum_hook(t_grid_state &prev_gs,
                    t_grid_state &datum_gs,
                    const AsyncInterface::t_datum_search_flags &search_modes,
                    const AsyncInterface::t_fpuset &fpuset,
                    bool was_cancelled,
                    const FpuPositions &initial_positions, // TODO: Not used (in Python version) - remove?
                    enum E_DATUM_SELECTION selected_arm) override;

    //..........................................................................

};

//==============================================================================

} // namespace mpifps

#endif // GRIDDRIVER_H
