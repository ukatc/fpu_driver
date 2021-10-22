#!/bin/bash
#************************************************************************
#   NAME
#      mock_collision - Simulate collision in EtherCAN gateway simulator
#
#   DESCRIPTION
#      This script sends a SIGUSR1 signal to the MOONS2 fibre positioner
#      EtherCAN gateway simulator process, which causes it to simulate a
#      collision. Used for testing and debugging.
#
#      NOTE: This script needs to be executed while the EtherCAN gateway
#      simulator is in the middle of simulating an executeMotion command,
#      so it is best to type in the command "mpietcCollision" in advance
#      and hit RETURN at the appropriate moment. 
#
#   OPTIONS
#      There are currently no options. SIGUSR2 can be used to simulate
#      a limit breach, but you can do this more realistically by
#      moving the simulated positioners to their limit. The full
#      list of signals recognised is:
#
#      * SIGHUP: triggers reset_handler()
#      * SIGUSR1: triggers collision_handler()
#      * SIGUSR2: triggers limitbreach_handler()
#      * SIGURG: triggers overflow_handler()
#      * SIGRTMIN: triggers colltest_handler() 
#
#      See JIRA ticket MOONS-902.
#
#------------------------------------------------------------------------
#
#
# Find the PID of the running process and then send the collision signal
PID=`ps -ef | grep mock_gateway | grep -v grep | awk '{print $2}'`
if [[ $PID == "" ]]; then
    echo "mock_gateway process is not running."
else
    echo "Sending SIGUSR1 to mock_gateway process ($PID)."
    kill -SIGUSR1 $PID
fi
