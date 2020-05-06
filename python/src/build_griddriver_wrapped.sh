# TODO: BW note: This file is just a quick copy/adaptation of build.sh for now
# (N.B. I don't know if the original comments below are still correct)

# This needs boost-1-65-1 - the version in Debian won't do.
# if the boost::python libraries are not installed in /usr/local,
# set the directories in which boost is installed as follows:
#
# export LIBRARY_PATH=$HOME/lib
# export CPLUS_INCLUDE_PATH=$HOME/include                     
# export LD_LIBRARY_PATH=$HOME/lib                     


# TODO: Eventually build and include the C++-side grid driver functionality as
# a library file, rather than as individual source files?

# TODO: Set version number below to what's required eventually

g++ -shared -std=c++11 -fPIC -DVERSION=\"v0.0.1\" \
    -I/usr/local/include -I/usr/include/python2.7 -I../../include \
    griddriver_wrapper.C \
    ../../src/AsyncInterface.C \
    ../../src/CommandPool.C \
    ../../src/CommandQueue.C \
    ../../src/decode_CAN_response.C \
    ../../src/EtherCANInterface.C \
    ../../src/FPUArray.C \
    ../../src/FPUGridDriver.C \
    ../../src/FPUState.C \
    ../../src/GatewayInterface.C \
    ../../src/GridState.C \
	../../src/handle_AbortMotion_response.C \
	../../src/handle_CheckIntegrity_response.C \
	../../src/handle_ConfigMotion_response.C \
	../../src/handle_EnableAlphaLimitProtection_response.C \
	../../src/handle_EnableBetaCollisionProtection_response.C \
	../../src/handle_EnableMove_response.C \
    ../../src/handle_ExecuteMotion_response.C \
	../../src/handle_FindDatum_response.C \
    ../../src/handle_FinishedDatum_message.C \
	../../src/handle_FinishedMotion_message.C \
    ../../src/handleFPUResponse.C \
	../../src/handle_FreeAlphaLimitBreach_response.C \
	../../src/handle_FreeBetaCollision_response.C \
	../../src/handle_GetFirmwareVersion_response.C \
	../../src/handle_LockUnit_response.C \
    ../../src/handle_PingFPU_response.C \
	../../src/handle_ReadRegister_response.C \
	../../src/handle_ReadSerialNumber_response.C \
	../../src/handle_RepeatMotion_response.C \
    ../../src/handle_ResetFPU_response.C \
	../../src/handle_ResetStepCounter_response.C \
	../../src/handle_ReverseMotion_response.C \
	../../src/handle_SetStepsPerSegment_response.C \
	../../src/handle_SetTicksPerSegment_response.C \
	../../src/handle_SetUStepLevel_response.C \
    ../../src/handleTimeout.C \
	../../src/handle_UnlockUnit_response.C \
    ../../src/handle_WarnCANOverflow_warning.C \
	../../src/handle_WarnCollisionBeta_warning.C \
	../../src/handle_WarnLimitAlpha_warning.C \
	../../src/handle_WriteSerialNumber_response.C \
    ../../src/SBuffer.C \
    ../../src/sync_utils.C	\
    ../../src/time_utils.C \
    ../../src/TimeOutList.C \
     -L/usr/local/lib -lboost_python27 \
     -o griddriver.so
