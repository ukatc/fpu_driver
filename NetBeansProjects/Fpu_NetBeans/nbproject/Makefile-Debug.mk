#
# Generated Makefile - do not edit!
#
# Edit the Makefile in the project folder instead (../Makefile). Each target
# has a -pre and a -post target defined where you can add customized code.
#
# This makefile implements configuration specific macros and targets.


# Environment
MKDIR=mkdir
CP=cp
GREP=grep
NM=nm
CCADMIN=CCadmin
RANLIB=ranlib
CC=gcc
CCC=g++
CXX=g++
FC=gfortran
AS=as

# Macros
CND_PLATFORM=GNU-Linux
CND_DLIB_EXT=so
CND_CONF=Debug
CND_DISTDIR=dist
CND_BUILDDIR=build

# Include project Makefile
include Makefile

# Object Directory
OBJECTDIR=${CND_BUILDDIR}/${CND_CONF}/${CND_PLATFORM}

# Object Files
OBJECTFILES= \
	${OBJECTDIR}/_ext/4211269a/mdb.o \
	${OBJECTDIR}/_ext/4211269a/midl.o \
	${OBJECTDIR}/_ext/56252444/AsyncInterface.o \
	${OBJECTDIR}/_ext/56252444/CommandPool.o \
	${OBJECTDIR}/_ext/56252444/CommandQueue.o \
	${OBJECTDIR}/_ext/56252444/DeviceLock.o \
	${OBJECTDIR}/_ext/56252444/EtherCANInterface.o \
	${OBJECTDIR}/_ext/56252444/FPUArray.o \
	${OBJECTDIR}/_ext/56252444/FPUState.o \
	${OBJECTDIR}/_ext/56252444/GatewayInterface.o \
	${OBJECTDIR}/_ext/56252444/GridDriver.o \
	${OBJECTDIR}/_ext/56252444/GridDriverTester.o \
	${OBJECTDIR}/_ext/56252444/GridState.o \
	${OBJECTDIR}/_ext/56252444/ProtectionDB.o \
	${OBJECTDIR}/_ext/56252444/SBuffer.o \
	${OBJECTDIR}/_ext/56252444/TimeOutList.o \
	${OBJECTDIR}/_ext/56252444/UnprotectedGridDriver.o \
	${OBJECTDIR}/_ext/56252444/decode_CAN_response.o \
	${OBJECTDIR}/_ext/56252444/handleFPUResponse.o \
	${OBJECTDIR}/_ext/56252444/handleTimeout.o \
	${OBJECTDIR}/_ext/56252444/handle_AbortMotion_response.o \
	${OBJECTDIR}/_ext/56252444/handle_CheckIntegrity_response.o \
	${OBJECTDIR}/_ext/56252444/handle_ConfigMotion_response.o \
	${OBJECTDIR}/_ext/56252444/handle_EnableAlphaLimitProtection_response.o \
	${OBJECTDIR}/_ext/56252444/handle_EnableBetaCollisionProtection_response.o \
	${OBJECTDIR}/_ext/56252444/handle_EnableMove_response.o \
	${OBJECTDIR}/_ext/56252444/handle_ExecuteMotion_response.o \
	${OBJECTDIR}/_ext/56252444/handle_FindDatum_response.o \
	${OBJECTDIR}/_ext/56252444/handle_FinishedDatum_message.o \
	${OBJECTDIR}/_ext/56252444/handle_FinishedMotion_message.o \
	${OBJECTDIR}/_ext/56252444/handle_FreeAlphaLimitBreach_response.o \
	${OBJECTDIR}/_ext/56252444/handle_FreeBetaCollision_response.o \
	${OBJECTDIR}/_ext/56252444/handle_GetFirmwareVersion_response.o \
	${OBJECTDIR}/_ext/56252444/handle_LockUnit_response.o \
	${OBJECTDIR}/_ext/56252444/handle_PingFPU_response.o \
	${OBJECTDIR}/_ext/56252444/handle_ReadRegister_response.o \
	${OBJECTDIR}/_ext/56252444/handle_ReadSerialNumber_response.o \
	${OBJECTDIR}/_ext/56252444/handle_RepeatMotion_response.o \
	${OBJECTDIR}/_ext/56252444/handle_ResetFPU_response.o \
	${OBJECTDIR}/_ext/56252444/handle_ResetStepCounter_response.o \
	${OBJECTDIR}/_ext/56252444/handle_ReverseMotion_response.o \
	${OBJECTDIR}/_ext/56252444/handle_SetStepsPerSegment_response.o \
	${OBJECTDIR}/_ext/56252444/handle_SetTicksPerSegment_response.o \
	${OBJECTDIR}/_ext/56252444/handle_SetUStepLevel_response.o \
	${OBJECTDIR}/_ext/56252444/handle_UnlockUnit_response.o \
	${OBJECTDIR}/_ext/56252444/handle_WarnCANOverflow_warning.o \
	${OBJECTDIR}/_ext/56252444/handle_WarnCollisionBeta_warning.o \
	${OBJECTDIR}/_ext/56252444/handle_WarnLimitAlpha_warning.o \
	${OBJECTDIR}/_ext/56252444/handle_WriteSerialNumber_response.o \
	${OBJECTDIR}/_ext/56252444/sync_utils.o \
	${OBJECTDIR}/_ext/56252444/time_utils.o \
	${OBJECTDIR}/main.o


# C Compiler Flags
CFLAGS=-pthread

# CC Compiler Flags
CCFLAGS=-pthread
CXXFLAGS=-pthread

# Fortran Compiler Flags
FFLAGS=

# Assembler Flags
ASFLAGS=

# Link Libraries and Options
LDLIBSOPTIONS=

# Build Targets
.build-conf: ${BUILD_SUBPROJECTS}
	"${MAKE}"  -f nbproject/Makefile-${CND_CONF}.mk ${CND_DISTDIR}/${CND_CONF}/${CND_PLATFORM}/fpu_netbeans

${CND_DISTDIR}/${CND_CONF}/${CND_PLATFORM}/fpu_netbeans: ${OBJECTFILES}
	${MKDIR} -p ${CND_DISTDIR}/${CND_CONF}/${CND_PLATFORM}
	${LINK.cc} -o ${CND_DISTDIR}/${CND_CONF}/${CND_PLATFORM}/fpu_netbeans ${OBJECTFILES} ${LDLIBSOPTIONS}

${OBJECTDIR}/_ext/4211269a/mdb.o: ../../lib/liblmdb/mdb.c
	${MKDIR} -p ${OBJECTDIR}/_ext/4211269a
	${RM} "$@.d"
	$(COMPILE.c) -g -I../../lib/liblmdb -I../../include -MMD -MP -MF "$@.d" -o ${OBJECTDIR}/_ext/4211269a/mdb.o ../../lib/liblmdb/mdb.c

${OBJECTDIR}/_ext/4211269a/midl.o: ../../lib/liblmdb/midl.c
	${MKDIR} -p ${OBJECTDIR}/_ext/4211269a
	${RM} "$@.d"
	$(COMPILE.c) -g -I../../lib/liblmdb -I../../include -MMD -MP -MF "$@.d" -o ${OBJECTDIR}/_ext/4211269a/midl.o ../../lib/liblmdb/midl.c

${OBJECTDIR}/_ext/56252444/AsyncInterface.o: ../../src/AsyncInterface.C
	${MKDIR} -p ${OBJECTDIR}/_ext/56252444
	${RM} "$@.d"
	$(COMPILE.cc) -g -DENABLE_PROTECTION_CODE -DVERSION=\"v0.0.1\" -I../../lib/liblmdb -I../../include -I../../include/ethercan -std=c++11 -MMD -MP -MF "$@.d" -o ${OBJECTDIR}/_ext/56252444/AsyncInterface.o ../../src/AsyncInterface.C

${OBJECTDIR}/_ext/56252444/CommandPool.o: ../../src/CommandPool.C
	${MKDIR} -p ${OBJECTDIR}/_ext/56252444
	${RM} "$@.d"
	$(COMPILE.cc) -g -DENABLE_PROTECTION_CODE -DVERSION=\"v0.0.1\" -I../../lib/liblmdb -I../../include -I../../include/ethercan -std=c++11 -MMD -MP -MF "$@.d" -o ${OBJECTDIR}/_ext/56252444/CommandPool.o ../../src/CommandPool.C

${OBJECTDIR}/_ext/56252444/CommandQueue.o: ../../src/CommandQueue.C
	${MKDIR} -p ${OBJECTDIR}/_ext/56252444
	${RM} "$@.d"
	$(COMPILE.cc) -g -DENABLE_PROTECTION_CODE -DVERSION=\"v0.0.1\" -I../../lib/liblmdb -I../../include -I../../include/ethercan -std=c++11 -MMD -MP -MF "$@.d" -o ${OBJECTDIR}/_ext/56252444/CommandQueue.o ../../src/CommandQueue.C

${OBJECTDIR}/_ext/56252444/DeviceLock.o: ../../src/DeviceLock.C
	${MKDIR} -p ${OBJECTDIR}/_ext/56252444
	${RM} "$@.d"
	$(COMPILE.cc) -g -DENABLE_PROTECTION_CODE -DVERSION=\"v0.0.1\" -I../../lib/liblmdb -I../../include -I../../include/ethercan -std=c++11 -MMD -MP -MF "$@.d" -o ${OBJECTDIR}/_ext/56252444/DeviceLock.o ../../src/DeviceLock.C

${OBJECTDIR}/_ext/56252444/EtherCANInterface.o: ../../src/EtherCANInterface.C
	${MKDIR} -p ${OBJECTDIR}/_ext/56252444
	${RM} "$@.d"
	$(COMPILE.cc) -g -DENABLE_PROTECTION_CODE -DVERSION=\"v0.0.1\" -I../../lib/liblmdb -I../../include -I../../include/ethercan -std=c++11 -MMD -MP -MF "$@.d" -o ${OBJECTDIR}/_ext/56252444/EtherCANInterface.o ../../src/EtherCANInterface.C

${OBJECTDIR}/_ext/56252444/FPUArray.o: ../../src/FPUArray.C
	${MKDIR} -p ${OBJECTDIR}/_ext/56252444
	${RM} "$@.d"
	$(COMPILE.cc) -g -DENABLE_PROTECTION_CODE -DVERSION=\"v0.0.1\" -I../../lib/liblmdb -I../../include -I../../include/ethercan -std=c++11 -MMD -MP -MF "$@.d" -o ${OBJECTDIR}/_ext/56252444/FPUArray.o ../../src/FPUArray.C

${OBJECTDIR}/_ext/56252444/FPUState.o: ../../src/FPUState.C
	${MKDIR} -p ${OBJECTDIR}/_ext/56252444
	${RM} "$@.d"
	$(COMPILE.cc) -g -DENABLE_PROTECTION_CODE -DVERSION=\"v0.0.1\" -I../../lib/liblmdb -I../../include -I../../include/ethercan -std=c++11 -MMD -MP -MF "$@.d" -o ${OBJECTDIR}/_ext/56252444/FPUState.o ../../src/FPUState.C

${OBJECTDIR}/_ext/56252444/GatewayInterface.o: ../../src/GatewayInterface.C
	${MKDIR} -p ${OBJECTDIR}/_ext/56252444
	${RM} "$@.d"
	$(COMPILE.cc) -g -DENABLE_PROTECTION_CODE -DVERSION=\"v0.0.1\" -I../../lib/liblmdb -I../../include -I../../include/ethercan -std=c++11 -MMD -MP -MF "$@.d" -o ${OBJECTDIR}/_ext/56252444/GatewayInterface.o ../../src/GatewayInterface.C

${OBJECTDIR}/_ext/56252444/GridDriver.o: ../../src/GridDriver.C
	${MKDIR} -p ${OBJECTDIR}/_ext/56252444
	${RM} "$@.d"
	$(COMPILE.cc) -g -DENABLE_PROTECTION_CODE -DVERSION=\"v0.0.1\" -I../../lib/liblmdb -I../../include -I../../include/ethercan -std=c++11 -MMD -MP -MF "$@.d" -o ${OBJECTDIR}/_ext/56252444/GridDriver.o ../../src/GridDriver.C

${OBJECTDIR}/_ext/56252444/GridDriverTester.o: ../../src/GridDriverTester.C
	${MKDIR} -p ${OBJECTDIR}/_ext/56252444
	${RM} "$@.d"
	$(COMPILE.cc) -g -DENABLE_PROTECTION_CODE -DVERSION=\"v0.0.1\" -I../../lib/liblmdb -I../../include -I../../include/ethercan -std=c++11 -MMD -MP -MF "$@.d" -o ${OBJECTDIR}/_ext/56252444/GridDriverTester.o ../../src/GridDriverTester.C

${OBJECTDIR}/_ext/56252444/GridState.o: ../../src/GridState.C
	${MKDIR} -p ${OBJECTDIR}/_ext/56252444
	${RM} "$@.d"
	$(COMPILE.cc) -g -DENABLE_PROTECTION_CODE -DVERSION=\"v0.0.1\" -I../../lib/liblmdb -I../../include -I../../include/ethercan -std=c++11 -MMD -MP -MF "$@.d" -o ${OBJECTDIR}/_ext/56252444/GridState.o ../../src/GridState.C

${OBJECTDIR}/_ext/56252444/ProtectionDB.o: ../../src/ProtectionDB.C
	${MKDIR} -p ${OBJECTDIR}/_ext/56252444
	${RM} "$@.d"
	$(COMPILE.cc) -g -DENABLE_PROTECTION_CODE -DVERSION=\"v0.0.1\" -I../../lib/liblmdb -I../../include -I../../include/ethercan -std=c++11 -MMD -MP -MF "$@.d" -o ${OBJECTDIR}/_ext/56252444/ProtectionDB.o ../../src/ProtectionDB.C

${OBJECTDIR}/_ext/56252444/SBuffer.o: ../../src/SBuffer.C
	${MKDIR} -p ${OBJECTDIR}/_ext/56252444
	${RM} "$@.d"
	$(COMPILE.cc) -g -DENABLE_PROTECTION_CODE -DVERSION=\"v0.0.1\" -I../../lib/liblmdb -I../../include -I../../include/ethercan -std=c++11 -MMD -MP -MF "$@.d" -o ${OBJECTDIR}/_ext/56252444/SBuffer.o ../../src/SBuffer.C

${OBJECTDIR}/_ext/56252444/TimeOutList.o: ../../src/TimeOutList.C
	${MKDIR} -p ${OBJECTDIR}/_ext/56252444
	${RM} "$@.d"
	$(COMPILE.cc) -g -DENABLE_PROTECTION_CODE -DVERSION=\"v0.0.1\" -I../../lib/liblmdb -I../../include -I../../include/ethercan -std=c++11 -MMD -MP -MF "$@.d" -o ${OBJECTDIR}/_ext/56252444/TimeOutList.o ../../src/TimeOutList.C

${OBJECTDIR}/_ext/56252444/UnprotectedGridDriver.o: ../../src/UnprotectedGridDriver.C
	${MKDIR} -p ${OBJECTDIR}/_ext/56252444
	${RM} "$@.d"
	$(COMPILE.cc) -g -DENABLE_PROTECTION_CODE -DVERSION=\"v0.0.1\" -I../../lib/liblmdb -I../../include -I../../include/ethercan -std=c++11 -MMD -MP -MF "$@.d" -o ${OBJECTDIR}/_ext/56252444/UnprotectedGridDriver.o ../../src/UnprotectedGridDriver.C

${OBJECTDIR}/_ext/56252444/decode_CAN_response.o: ../../src/decode_CAN_response.C
	${MKDIR} -p ${OBJECTDIR}/_ext/56252444
	${RM} "$@.d"
	$(COMPILE.cc) -g -DENABLE_PROTECTION_CODE -DVERSION=\"v0.0.1\" -I../../lib/liblmdb -I../../include -I../../include/ethercan -std=c++11 -MMD -MP -MF "$@.d" -o ${OBJECTDIR}/_ext/56252444/decode_CAN_response.o ../../src/decode_CAN_response.C

${OBJECTDIR}/_ext/56252444/handleFPUResponse.o: ../../src/handleFPUResponse.C
	${MKDIR} -p ${OBJECTDIR}/_ext/56252444
	${RM} "$@.d"
	$(COMPILE.cc) -g -DENABLE_PROTECTION_CODE -DVERSION=\"v0.0.1\" -I../../lib/liblmdb -I../../include -I../../include/ethercan -std=c++11 -MMD -MP -MF "$@.d" -o ${OBJECTDIR}/_ext/56252444/handleFPUResponse.o ../../src/handleFPUResponse.C

${OBJECTDIR}/_ext/56252444/handleTimeout.o: ../../src/handleTimeout.C
	${MKDIR} -p ${OBJECTDIR}/_ext/56252444
	${RM} "$@.d"
	$(COMPILE.cc) -g -DENABLE_PROTECTION_CODE -DVERSION=\"v0.0.1\" -I../../lib/liblmdb -I../../include -I../../include/ethercan -std=c++11 -MMD -MP -MF "$@.d" -o ${OBJECTDIR}/_ext/56252444/handleTimeout.o ../../src/handleTimeout.C

${OBJECTDIR}/_ext/56252444/handle_AbortMotion_response.o: ../../src/handle_AbortMotion_response.C
	${MKDIR} -p ${OBJECTDIR}/_ext/56252444
	${RM} "$@.d"
	$(COMPILE.cc) -g -DENABLE_PROTECTION_CODE -DVERSION=\"v0.0.1\" -I../../lib/liblmdb -I../../include -I../../include/ethercan -std=c++11 -MMD -MP -MF "$@.d" -o ${OBJECTDIR}/_ext/56252444/handle_AbortMotion_response.o ../../src/handle_AbortMotion_response.C

${OBJECTDIR}/_ext/56252444/handle_CheckIntegrity_response.o: ../../src/handle_CheckIntegrity_response.C
	${MKDIR} -p ${OBJECTDIR}/_ext/56252444
	${RM} "$@.d"
	$(COMPILE.cc) -g -DENABLE_PROTECTION_CODE -DVERSION=\"v0.0.1\" -I../../lib/liblmdb -I../../include -I../../include/ethercan -std=c++11 -MMD -MP -MF "$@.d" -o ${OBJECTDIR}/_ext/56252444/handle_CheckIntegrity_response.o ../../src/handle_CheckIntegrity_response.C

${OBJECTDIR}/_ext/56252444/handle_ConfigMotion_response.o: ../../src/handle_ConfigMotion_response.C
	${MKDIR} -p ${OBJECTDIR}/_ext/56252444
	${RM} "$@.d"
	$(COMPILE.cc) -g -DENABLE_PROTECTION_CODE -DVERSION=\"v0.0.1\" -I../../lib/liblmdb -I../../include -I../../include/ethercan -std=c++11 -MMD -MP -MF "$@.d" -o ${OBJECTDIR}/_ext/56252444/handle_ConfigMotion_response.o ../../src/handle_ConfigMotion_response.C

${OBJECTDIR}/_ext/56252444/handle_EnableAlphaLimitProtection_response.o: ../../src/handle_EnableAlphaLimitProtection_response.C
	${MKDIR} -p ${OBJECTDIR}/_ext/56252444
	${RM} "$@.d"
	$(COMPILE.cc) -g -DENABLE_PROTECTION_CODE -DVERSION=\"v0.0.1\" -I../../lib/liblmdb -I../../include -I../../include/ethercan -std=c++11 -MMD -MP -MF "$@.d" -o ${OBJECTDIR}/_ext/56252444/handle_EnableAlphaLimitProtection_response.o ../../src/handle_EnableAlphaLimitProtection_response.C

${OBJECTDIR}/_ext/56252444/handle_EnableBetaCollisionProtection_response.o: ../../src/handle_EnableBetaCollisionProtection_response.C
	${MKDIR} -p ${OBJECTDIR}/_ext/56252444
	${RM} "$@.d"
	$(COMPILE.cc) -g -DENABLE_PROTECTION_CODE -DVERSION=\"v0.0.1\" -I../../lib/liblmdb -I../../include -I../../include/ethercan -std=c++11 -MMD -MP -MF "$@.d" -o ${OBJECTDIR}/_ext/56252444/handle_EnableBetaCollisionProtection_response.o ../../src/handle_EnableBetaCollisionProtection_response.C

${OBJECTDIR}/_ext/56252444/handle_EnableMove_response.o: ../../src/handle_EnableMove_response.C
	${MKDIR} -p ${OBJECTDIR}/_ext/56252444
	${RM} "$@.d"
	$(COMPILE.cc) -g -DENABLE_PROTECTION_CODE -DVERSION=\"v0.0.1\" -I../../lib/liblmdb -I../../include -I../../include/ethercan -std=c++11 -MMD -MP -MF "$@.d" -o ${OBJECTDIR}/_ext/56252444/handle_EnableMove_response.o ../../src/handle_EnableMove_response.C

${OBJECTDIR}/_ext/56252444/handle_ExecuteMotion_response.o: ../../src/handle_ExecuteMotion_response.C
	${MKDIR} -p ${OBJECTDIR}/_ext/56252444
	${RM} "$@.d"
	$(COMPILE.cc) -g -DENABLE_PROTECTION_CODE -DVERSION=\"v0.0.1\" -I../../lib/liblmdb -I../../include -I../../include/ethercan -std=c++11 -MMD -MP -MF "$@.d" -o ${OBJECTDIR}/_ext/56252444/handle_ExecuteMotion_response.o ../../src/handle_ExecuteMotion_response.C

${OBJECTDIR}/_ext/56252444/handle_FindDatum_response.o: ../../src/handle_FindDatum_response.C
	${MKDIR} -p ${OBJECTDIR}/_ext/56252444
	${RM} "$@.d"
	$(COMPILE.cc) -g -DENABLE_PROTECTION_CODE -DVERSION=\"v0.0.1\" -I../../lib/liblmdb -I../../include -I../../include/ethercan -std=c++11 -MMD -MP -MF "$@.d" -o ${OBJECTDIR}/_ext/56252444/handle_FindDatum_response.o ../../src/handle_FindDatum_response.C

${OBJECTDIR}/_ext/56252444/handle_FinishedDatum_message.o: ../../src/handle_FinishedDatum_message.C
	${MKDIR} -p ${OBJECTDIR}/_ext/56252444
	${RM} "$@.d"
	$(COMPILE.cc) -g -DENABLE_PROTECTION_CODE -DVERSION=\"v0.0.1\" -I../../lib/liblmdb -I../../include -I../../include/ethercan -std=c++11 -MMD -MP -MF "$@.d" -o ${OBJECTDIR}/_ext/56252444/handle_FinishedDatum_message.o ../../src/handle_FinishedDatum_message.C

${OBJECTDIR}/_ext/56252444/handle_FinishedMotion_message.o: ../../src/handle_FinishedMotion_message.C
	${MKDIR} -p ${OBJECTDIR}/_ext/56252444
	${RM} "$@.d"
	$(COMPILE.cc) -g -DENABLE_PROTECTION_CODE -DVERSION=\"v0.0.1\" -I../../lib/liblmdb -I../../include -I../../include/ethercan -std=c++11 -MMD -MP -MF "$@.d" -o ${OBJECTDIR}/_ext/56252444/handle_FinishedMotion_message.o ../../src/handle_FinishedMotion_message.C

${OBJECTDIR}/_ext/56252444/handle_FreeAlphaLimitBreach_response.o: ../../src/handle_FreeAlphaLimitBreach_response.C
	${MKDIR} -p ${OBJECTDIR}/_ext/56252444
	${RM} "$@.d"
	$(COMPILE.cc) -g -DENABLE_PROTECTION_CODE -DVERSION=\"v0.0.1\" -I../../lib/liblmdb -I../../include -I../../include/ethercan -std=c++11 -MMD -MP -MF "$@.d" -o ${OBJECTDIR}/_ext/56252444/handle_FreeAlphaLimitBreach_response.o ../../src/handle_FreeAlphaLimitBreach_response.C

${OBJECTDIR}/_ext/56252444/handle_FreeBetaCollision_response.o: ../../src/handle_FreeBetaCollision_response.C
	${MKDIR} -p ${OBJECTDIR}/_ext/56252444
	${RM} "$@.d"
	$(COMPILE.cc) -g -DENABLE_PROTECTION_CODE -DVERSION=\"v0.0.1\" -I../../lib/liblmdb -I../../include -I../../include/ethercan -std=c++11 -MMD -MP -MF "$@.d" -o ${OBJECTDIR}/_ext/56252444/handle_FreeBetaCollision_response.o ../../src/handle_FreeBetaCollision_response.C

${OBJECTDIR}/_ext/56252444/handle_GetFirmwareVersion_response.o: ../../src/handle_GetFirmwareVersion_response.C
	${MKDIR} -p ${OBJECTDIR}/_ext/56252444
	${RM} "$@.d"
	$(COMPILE.cc) -g -DENABLE_PROTECTION_CODE -DVERSION=\"v0.0.1\" -I../../lib/liblmdb -I../../include -I../../include/ethercan -std=c++11 -MMD -MP -MF "$@.d" -o ${OBJECTDIR}/_ext/56252444/handle_GetFirmwareVersion_response.o ../../src/handle_GetFirmwareVersion_response.C

${OBJECTDIR}/_ext/56252444/handle_LockUnit_response.o: ../../src/handle_LockUnit_response.C
	${MKDIR} -p ${OBJECTDIR}/_ext/56252444
	${RM} "$@.d"
	$(COMPILE.cc) -g -DENABLE_PROTECTION_CODE -DVERSION=\"v0.0.1\" -I../../lib/liblmdb -I../../include -I../../include/ethercan -std=c++11 -MMD -MP -MF "$@.d" -o ${OBJECTDIR}/_ext/56252444/handle_LockUnit_response.o ../../src/handle_LockUnit_response.C

${OBJECTDIR}/_ext/56252444/handle_PingFPU_response.o: ../../src/handle_PingFPU_response.C
	${MKDIR} -p ${OBJECTDIR}/_ext/56252444
	${RM} "$@.d"
	$(COMPILE.cc) -g -DENABLE_PROTECTION_CODE -DVERSION=\"v0.0.1\" -I../../lib/liblmdb -I../../include -I../../include/ethercan -std=c++11 -MMD -MP -MF "$@.d" -o ${OBJECTDIR}/_ext/56252444/handle_PingFPU_response.o ../../src/handle_PingFPU_response.C

${OBJECTDIR}/_ext/56252444/handle_ReadRegister_response.o: ../../src/handle_ReadRegister_response.C
	${MKDIR} -p ${OBJECTDIR}/_ext/56252444
	${RM} "$@.d"
	$(COMPILE.cc) -g -DENABLE_PROTECTION_CODE -DVERSION=\"v0.0.1\" -I../../lib/liblmdb -I../../include -I../../include/ethercan -std=c++11 -MMD -MP -MF "$@.d" -o ${OBJECTDIR}/_ext/56252444/handle_ReadRegister_response.o ../../src/handle_ReadRegister_response.C

${OBJECTDIR}/_ext/56252444/handle_ReadSerialNumber_response.o: ../../src/handle_ReadSerialNumber_response.C
	${MKDIR} -p ${OBJECTDIR}/_ext/56252444
	${RM} "$@.d"
	$(COMPILE.cc) -g -DENABLE_PROTECTION_CODE -DVERSION=\"v0.0.1\" -I../../lib/liblmdb -I../../include -I../../include/ethercan -std=c++11 -MMD -MP -MF "$@.d" -o ${OBJECTDIR}/_ext/56252444/handle_ReadSerialNumber_response.o ../../src/handle_ReadSerialNumber_response.C

${OBJECTDIR}/_ext/56252444/handle_RepeatMotion_response.o: ../../src/handle_RepeatMotion_response.C
	${MKDIR} -p ${OBJECTDIR}/_ext/56252444
	${RM} "$@.d"
	$(COMPILE.cc) -g -DENABLE_PROTECTION_CODE -DVERSION=\"v0.0.1\" -I../../lib/liblmdb -I../../include -I../../include/ethercan -std=c++11 -MMD -MP -MF "$@.d" -o ${OBJECTDIR}/_ext/56252444/handle_RepeatMotion_response.o ../../src/handle_RepeatMotion_response.C

${OBJECTDIR}/_ext/56252444/handle_ResetFPU_response.o: ../../src/handle_ResetFPU_response.C
	${MKDIR} -p ${OBJECTDIR}/_ext/56252444
	${RM} "$@.d"
	$(COMPILE.cc) -g -DENABLE_PROTECTION_CODE -DVERSION=\"v0.0.1\" -I../../lib/liblmdb -I../../include -I../../include/ethercan -std=c++11 -MMD -MP -MF "$@.d" -o ${OBJECTDIR}/_ext/56252444/handle_ResetFPU_response.o ../../src/handle_ResetFPU_response.C

${OBJECTDIR}/_ext/56252444/handle_ResetStepCounter_response.o: ../../src/handle_ResetStepCounter_response.C
	${MKDIR} -p ${OBJECTDIR}/_ext/56252444
	${RM} "$@.d"
	$(COMPILE.cc) -g -DENABLE_PROTECTION_CODE -DVERSION=\"v0.0.1\" -I../../lib/liblmdb -I../../include -I../../include/ethercan -std=c++11 -MMD -MP -MF "$@.d" -o ${OBJECTDIR}/_ext/56252444/handle_ResetStepCounter_response.o ../../src/handle_ResetStepCounter_response.C

${OBJECTDIR}/_ext/56252444/handle_ReverseMotion_response.o: ../../src/handle_ReverseMotion_response.C
	${MKDIR} -p ${OBJECTDIR}/_ext/56252444
	${RM} "$@.d"
	$(COMPILE.cc) -g -DENABLE_PROTECTION_CODE -DVERSION=\"v0.0.1\" -I../../lib/liblmdb -I../../include -I../../include/ethercan -std=c++11 -MMD -MP -MF "$@.d" -o ${OBJECTDIR}/_ext/56252444/handle_ReverseMotion_response.o ../../src/handle_ReverseMotion_response.C

${OBJECTDIR}/_ext/56252444/handle_SetStepsPerSegment_response.o: ../../src/handle_SetStepsPerSegment_response.C
	${MKDIR} -p ${OBJECTDIR}/_ext/56252444
	${RM} "$@.d"
	$(COMPILE.cc) -g -DENABLE_PROTECTION_CODE -DVERSION=\"v0.0.1\" -I../../lib/liblmdb -I../../include -I../../include/ethercan -std=c++11 -MMD -MP -MF "$@.d" -o ${OBJECTDIR}/_ext/56252444/handle_SetStepsPerSegment_response.o ../../src/handle_SetStepsPerSegment_response.C

${OBJECTDIR}/_ext/56252444/handle_SetTicksPerSegment_response.o: ../../src/handle_SetTicksPerSegment_response.C
	${MKDIR} -p ${OBJECTDIR}/_ext/56252444
	${RM} "$@.d"
	$(COMPILE.cc) -g -DENABLE_PROTECTION_CODE -DVERSION=\"v0.0.1\" -I../../lib/liblmdb -I../../include -I../../include/ethercan -std=c++11 -MMD -MP -MF "$@.d" -o ${OBJECTDIR}/_ext/56252444/handle_SetTicksPerSegment_response.o ../../src/handle_SetTicksPerSegment_response.C

${OBJECTDIR}/_ext/56252444/handle_SetUStepLevel_response.o: ../../src/handle_SetUStepLevel_response.C
	${MKDIR} -p ${OBJECTDIR}/_ext/56252444
	${RM} "$@.d"
	$(COMPILE.cc) -g -DENABLE_PROTECTION_CODE -DVERSION=\"v0.0.1\" -I../../lib/liblmdb -I../../include -I../../include/ethercan -std=c++11 -MMD -MP -MF "$@.d" -o ${OBJECTDIR}/_ext/56252444/handle_SetUStepLevel_response.o ../../src/handle_SetUStepLevel_response.C

${OBJECTDIR}/_ext/56252444/handle_UnlockUnit_response.o: ../../src/handle_UnlockUnit_response.C
	${MKDIR} -p ${OBJECTDIR}/_ext/56252444
	${RM} "$@.d"
	$(COMPILE.cc) -g -DENABLE_PROTECTION_CODE -DVERSION=\"v0.0.1\" -I../../lib/liblmdb -I../../include -I../../include/ethercan -std=c++11 -MMD -MP -MF "$@.d" -o ${OBJECTDIR}/_ext/56252444/handle_UnlockUnit_response.o ../../src/handle_UnlockUnit_response.C

${OBJECTDIR}/_ext/56252444/handle_WarnCANOverflow_warning.o: ../../src/handle_WarnCANOverflow_warning.C
	${MKDIR} -p ${OBJECTDIR}/_ext/56252444
	${RM} "$@.d"
	$(COMPILE.cc) -g -DENABLE_PROTECTION_CODE -DVERSION=\"v0.0.1\" -I../../lib/liblmdb -I../../include -I../../include/ethercan -std=c++11 -MMD -MP -MF "$@.d" -o ${OBJECTDIR}/_ext/56252444/handle_WarnCANOverflow_warning.o ../../src/handle_WarnCANOverflow_warning.C

${OBJECTDIR}/_ext/56252444/handle_WarnCollisionBeta_warning.o: ../../src/handle_WarnCollisionBeta_warning.C
	${MKDIR} -p ${OBJECTDIR}/_ext/56252444
	${RM} "$@.d"
	$(COMPILE.cc) -g -DENABLE_PROTECTION_CODE -DVERSION=\"v0.0.1\" -I../../lib/liblmdb -I../../include -I../../include/ethercan -std=c++11 -MMD -MP -MF "$@.d" -o ${OBJECTDIR}/_ext/56252444/handle_WarnCollisionBeta_warning.o ../../src/handle_WarnCollisionBeta_warning.C

${OBJECTDIR}/_ext/56252444/handle_WarnLimitAlpha_warning.o: ../../src/handle_WarnLimitAlpha_warning.C
	${MKDIR} -p ${OBJECTDIR}/_ext/56252444
	${RM} "$@.d"
	$(COMPILE.cc) -g -DENABLE_PROTECTION_CODE -DVERSION=\"v0.0.1\" -I../../lib/liblmdb -I../../include -I../../include/ethercan -std=c++11 -MMD -MP -MF "$@.d" -o ${OBJECTDIR}/_ext/56252444/handle_WarnLimitAlpha_warning.o ../../src/handle_WarnLimitAlpha_warning.C

${OBJECTDIR}/_ext/56252444/handle_WriteSerialNumber_response.o: ../../src/handle_WriteSerialNumber_response.C
	${MKDIR} -p ${OBJECTDIR}/_ext/56252444
	${RM} "$@.d"
	$(COMPILE.cc) -g -DENABLE_PROTECTION_CODE -DVERSION=\"v0.0.1\" -I../../lib/liblmdb -I../../include -I../../include/ethercan -std=c++11 -MMD -MP -MF "$@.d" -o ${OBJECTDIR}/_ext/56252444/handle_WriteSerialNumber_response.o ../../src/handle_WriteSerialNumber_response.C

${OBJECTDIR}/_ext/56252444/sync_utils.o: ../../src/sync_utils.C
	${MKDIR} -p ${OBJECTDIR}/_ext/56252444
	${RM} "$@.d"
	$(COMPILE.cc) -g -DENABLE_PROTECTION_CODE -DVERSION=\"v0.0.1\" -I../../lib/liblmdb -I../../include -I../../include/ethercan -std=c++11 -MMD -MP -MF "$@.d" -o ${OBJECTDIR}/_ext/56252444/sync_utils.o ../../src/sync_utils.C

${OBJECTDIR}/_ext/56252444/time_utils.o: ../../src/time_utils.C
	${MKDIR} -p ${OBJECTDIR}/_ext/56252444
	${RM} "$@.d"
	$(COMPILE.cc) -g -DENABLE_PROTECTION_CODE -DVERSION=\"v0.0.1\" -I../../lib/liblmdb -I../../include -I../../include/ethercan -std=c++11 -MMD -MP -MF "$@.d" -o ${OBJECTDIR}/_ext/56252444/time_utils.o ../../src/time_utils.C

${OBJECTDIR}/main.o: main.cpp
	${MKDIR} -p ${OBJECTDIR}
	${RM} "$@.d"
	$(COMPILE.cc) -g -DENABLE_PROTECTION_CODE -DVERSION=\"v0.0.1\" -I../../lib/liblmdb -I../../include -I../../include/ethercan -std=c++11 -MMD -MP -MF "$@.d" -o ${OBJECTDIR}/main.o main.cpp

# Subprojects
.build-subprojects:

# Clean Targets
.clean-conf: ${CLEAN_SUBPROJECTS}
	${RM} -r ${CND_BUILDDIR}/${CND_CONF}

# Subprojects
.clean-subprojects:

# Enable dependency checking
.dep.inc: .depcheck-impl

include .dep.inc
