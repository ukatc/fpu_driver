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
	${OBJECTDIR}/_ext/56252444/ProtectionDB.o \
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

${OBJECTDIR}/_ext/56252444/ProtectionDB.o: ../../src/ProtectionDB.C
	${MKDIR} -p ${OBJECTDIR}/_ext/56252444
	${RM} "$@.d"
	$(COMPILE.cc) -g -I../../lib/liblmdb -I../../include -I../../include/ethercan -MMD -MP -MF "$@.d" -o ${OBJECTDIR}/_ext/56252444/ProtectionDB.o ../../src/ProtectionDB.C

${OBJECTDIR}/main.o: main.cpp
	${MKDIR} -p ${OBJECTDIR}
	${RM} "$@.d"
	$(COMPILE.cc) -g -I../../lib/liblmdb -I../../include -I../../include/ethercan -MMD -MP -MF "$@.d" -o ${OBJECTDIR}/main.o main.cpp

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
