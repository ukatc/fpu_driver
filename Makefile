IDIR = ./include
CC = "g++"

VERSION := v2.2.6

CXXFLAGS = -I$(IDIR) -std=c++11 -Wall -Wextra -pedantic -Werror -fPIC \
-DDEBUG -g -O3 -finline-functions -Wstrict-aliasing -march=native \
-Wshadow -Wcast-qual -Wmissing-declarations -Wundef -Wlogical-op \
-Wredundant-decls -Wfloat-equal -Wstrict-overflow=4 -Wunused-result

# flags for link time optimized build of wrapper
CXXFLAGS_LTO = -I$(IDIR) -std=c++11 -Wall -Wextra -pedantic -Werror -fPIC \
-DDEBUG -g -O3 -finline-functions -Wstrict-aliasing -march=native \
-Wshadow -Wcast-qual -Wmissing-declarations -Wundef -Wlogical-op \
-Wredundant-decls -Wfloat-equal -Wunused-result -flto

ODIR = ./object

LDIR = ./lib

SRCDIR = ./src

LIBS = -lm -lpthread

_DEPS = InterfaceState.h E_GridState.h FPUState.h EtherCANInterface.h \
	EtherCANInterfaceConfig.h E_LogLevel.h GridState.h T_GridState.h \
	ethercan/AsyncInterface.h T_GatewayAddress.h \
	ethercan/handleFPUResponse.h ethercan/handleTimeout.h \
	ethercan/CANError.h ethercan/CommandPool.h \
	ethercan/CAN_Constants.h ethercan/E_CAN_COMMAND.h \
	ethercan/CommandQueue.h InterfaceConstants.h ethercan/FPUArray.h \
	ethercan/GatewayInterface.h ethercan/CAN_Command.h \
	ethercan/I_ResponseHandler.h ethercan/SBuffer.h \
	ethercan/TimeOutList.h ethercan/RingBuffer.h \
	ethercan/cancommandsv2/AbortMotionCommand.h \
	ethercan/cancommandsv2/CheckIntegrityCommand.h \
	ethercan/cancommandsv2/ConfigureMotionCommand.h \
	ethercan/cancommandsv2/EnableAlphaLimitProtectionCommand.h \
	ethercan/cancommandsv2/EnableBetaCollisionProtectionCommand.h \
	ethercan/cancommandsv2/EnableMoveCommand.h \
	ethercan/cancommandsv2/ExecuteMotionCommand.h \
	ethercan/cancommandsv2/FindDatumCommand.h \
	ethercan/cancommandsv2/FreeAlphaLimitBreachCommand.h \
	ethercan/cancommandsv2/FreeBetaCollisionCommand.h \
	ethercan/cancommandsv2/GetFirmwareVersionCommand.h \
	ethercan/cancommandsv2/LockUnitCommand.h \
	ethercan/cancommandsv2/PingFPUCommand.h \
	ethercan/cancommandsv2/ReadFirmwareVersionCommand.h \
	ethercan/cancommandsv2/ReadRegisterCommand.h \
	ethercan/cancommandsv2/ReadSerialNumberCommand.h \
	ethercan/cancommandsv2/RepeatMotionCommand.h \
	ethercan/cancommandsv2/ResetFPUCommand.h \
	ethercan/cancommandsv2/ResetStepCounterCommand.h \
	ethercan/cancommandsv2/ReverseMotionCommand.h \
	ethercan/cancommandsv2/SetStepsPerSegmentCommand.h \
	ethercan/cancommandsv2/SetTicksPerSegmentCommand.h \
	ethercan/cancommandsv2/SetUStepLevelCommand.h \
	ethercan/cancommandsv2/SyncCommand.h \
	ethercan/cancommandsv2/UnlockUnitCommand.h \
	ethercan/cancommandsv2/WriteSerialNumberCommand.h \
	ethercan/response_handlers/handle_AbortMotion_response.h \
	ethercan/response_handlers/handle_CheckIntegrity_response.h \
	ethercan/response_handlers/handle_ConfigMotion_response.h \
	ethercan/response_handlers/handle_EnableAlphaLimitProtection_response.h \
	ethercan/response_handlers/handle_EnableBetaCollisionProtection_response.h \
	ethercan/response_handlers/handle_ExecuteMotion_response.h \
	ethercan/response_handlers/handle_FindDatum_response.h \
	ethercan/response_handlers/handle_FinishedDatum_message.h \
	ethercan/response_handlers/handle_FinishedMotion_message.h \
	ethercan/response_handlers/handle_FreeAlphaLimitBreach_response.h \
	ethercan/response_handlers/handle_FreeBetaCollision_response.h \
	ethercan/response_handlers/handle_GetFirmwareVersion_response.h \
	ethercan/response_handlers/handle_LockUnit_response.h \
	ethercan/response_handlers/handle_PingFPU_response.h \
	ethercan/response_handlers/handle_ReadRegister_response.h \
	ethercan/response_handlers/handle_ReadSerialNumber_response.h \
	ethercan/response_handlers/handle_RepeatMotion_response.h \
	ethercan/response_handlers/handle_ResetFPU_response.h \
	ethercan/response_handlers/handle_ResetStepCounter_response.h \
	ethercan/response_handlers/handle_ReverseMotion_response.h \
	ethercan/response_handlers/handle_SetStepsPerSegment_response.h \
	ethercan/response_handlers/handle_SetTicksPerSegment_response.h \
	ethercan/response_handlers/handle_SetUStepLevel_response.h \
	ethercan/response_handlers/handle_UnlockUnit_response.h \
	ethercan/response_handlers/handle_WarnCANOverflow_warning.h \
	ethercan/response_handlers/handle_WarnCollisionBeta_warning.h \
	ethercan/response_handlers/handle_WarnLimitAlpha_warning.h \
	ethercan/response_handlers/handle_WriteSerialNumber_response.h \
	ethercan/sync_utils.h ethercan/time_utils.h \
	ethercan/decode_CAN_response.h \
	../python/src/FpuBPShared_General.h

DEPS = $(patsubst %,$(IDIR)/%,$(_DEPS)) Makefile

_OBJ = EtherCANInterface.o AsyncInterface.o FPUArray.o GridState.o \
	CommandPool.o GatewayInterface.o TimeOutList.o CommandQueue.o \
	time_utils.o sync_utils.o SBuffer.o handleFPUResponse.o \
	handleTimeout.o FPUState.o decode_CAN_response.o \
	handle_AbortMotion_response.o \
	handle_CheckIntegrity_response.o \
	handle_ConfigMotion_response.o \
	handle_EnableAlphaLimitProtection_response.o \
	handle_EnableBetaCollisionProtection_response.o \
	handle_EnableMove_response.o handle_ExecuteMotion_response.o \
	handle_FindDatum_response.o handle_FinishedDatum_message.o \
	handle_FinishedMotion_message.o \
	handle_FreeAlphaLimitBreach_response.o \
	handle_FreeBetaCollision_response.o \
	handle_GetFirmwareVersion_response.o \
	handle_LockUnit_response.o handle_PingFPU_response.o \
	handle_ReadRegister_response.o \
	handle_ReadSerialNumber_response.o \
	handle_RepeatMotion_response.o handle_ResetFPU_response.o \
	handle_ResetStepCounter_response.o \
	handle_ReverseMotion_response.o \
	handle_SetStepsPerSegment_response.o \
	handle_SetTicksPerSegment_response.o \
	handle_SetUStepLevel_response.o handleTimeout.o \
	handle_UnlockUnit_response.o handle_WarnCANOverflow_warning.o \
	handle_WarnCollisionBeta_warning.o \
	handle_WarnLimitAlpha_warning.o \
	handle_WriteSerialNumber_response.o

OBJ = $(patsubst %,$(ODIR)/%,$(_OBJ))

_SRC = AsyncInterface.C CommandPool.C CommandQueue.C \
	decode_CAN_response.C EtherCANInterface.C FPUArray.C \
	FPUState.C GatewayInterface.C GridState.C \
	handle_AbortMotion_response.C \
	handle_CheckIntegrity_response.C \
	handle_ConfigMotion_response.C \
	handle_EnableAlphaLimitProtection_response.C \
	handle_EnableBetaCollisionProtection_response.C \
	handle_EnableMove_response.C handle_ExecuteMotion_response.C \
	handle_FindDatum_response.C handle_FinishedDatum_message.C \
	handle_FinishedMotion_message.C handleFPUResponse.C \
	handle_FreeAlphaLimitBreach_response.C \
	handle_FreeBetaCollision_response.C \
	handle_GetFirmwareVersion_response.C \
	handle_LockUnit_response.C handle_PingFPU_response.C \
	handle_ReadRegister_response.C \
	handle_ReadSerialNumber_response.C \
	handle_RepeatMotion_response.C handle_ResetFPU_response.C \
	handle_ResetStepCounter_response.C \
	handle_ReverseMotion_response.C \
	handle_SetStepsPerSegment_response.C \
	handle_SetTicksPerSegment_response.C \
	handle_SetUStepLevel_response.C handleTimeout.C \
	handle_UnlockUnit_response.C handle_WarnCANOverflow_warning.C \
	handle_WarnCollisionBeta_warning.C \
	handle_WarnLimitAlpha_warning.C \
	handle_WriteSerialNumber_response.C SBuffer.C sync_utils.C \
	TimeOutList.C time_utils.C

SRC = $(patsubst %,$(SRCDIR)/%,$(_SRC))

.PHONY: force clean

# This target builds the default wrapper, without link time optimization.

# ******* TODO: BW changed -lboost_python to -lboost_python27 to make it build
# on my Ubuntu Linux VM with Boost 1.72
wrapper: lib/libethercan.a python/src/ethercanif.C $(DEPS) version
	g++ -shared -std=c++11 -I/usr/local/include -I/usr/include/python2.7 -fPIC -o python/ethercanif.so \
            python/src/ethercanif.C python/src/FpuBPShared_General.C -L./lib  -lethercan -lboost_python27 $(CXXFLAGS) -DVERSION=\"$(VERSION)\"

# This target variant is using link time optimization, aka LTO,
# to build the python test module directly.
# LTO is probably useful because we have many small handler functions which
# run in performance-critical loops. (With version v2.0.2, now checked to run correctly)
# ******* TODO: BW changed -lboost_python to -lboost_python27 to make it build
# on my Ubuntu Linux VM with Boost 1.72
wrapper-lto:  python/src/ethercanif.C $(SRC) $(DEPS) version
	g++ -shared -std=c++11 -I/usr/local/include -I/usr/include/python2.7 -fPIC -o python/ethercanif.so $(CXXFLAGS_LTO)\
            python/src/ethercanif.C python/src/FpuBPShared_General.C $(SRC) -lboost_python27 -DVERSION=\"$(VERSION)\"


version: force
	echo '$(VERSION)' | cmp -s - $@ || echo '$(VERSION)' > $@

python/doc/FPU-state2.pdf : python/doc/FPU-state2.svg
	inkscape python/doc/FPU-state2.svg --export-pdf=python/doc/FPU-state2.pdf

python/doc/fpu-driver-concurrency-architecture.png: python/doc/fpu-driver-concurrency-architecture.svg
	inkscape python/doc/fpu-driver-concurrency-architecture.svg --export-png=python/doc/fpu-driver-concurrency-architecture.png

# This builds the documentation. Some extra LaTeX packages, fonts, the minted package,
# and inkscape are required for this.
manual:	python/doc/manual.tex python/doc/FPU-state2.pdf version
	cd python/doc; pdflatex --shell-escape manual.tex; makeindex manual ; pdflatex --shell-escape manual.tex;

concurrency-doc:	python/doc/fpu_driver-concurrency_architecture.tex python/doc/fpu-driver-concurrency-architecture.png version
	cd python/doc; pdflatex fpu_driver-concurrency_architecture.tex; bibtex fpu_driver-concurrency_architecture; pdflatex fpu_driver-concurrency_architecture.tex;

doc: manual concurrency-doc

cppcheck: force
	cppcheck src/*.C python/src/*.C  -I include -I include/ethercan -I include/ethercan/response_handlers \
        -I include/ethercan/cancommandsv2 --enable=all

$(ODIR)/%.o: $(SRCDIR)/%.C $(DEPS) version
	$(CC) $(CXXFLAGS) -DVERSION=\"$(VERSION)\" -c -o $@ $<

lib/libethercan.a: $(OBJ)
	ar rcs   $@ $^

libethercan: lib/libethercan.a

style:
	astyle src/*.C python/src/*.C include{,/*{,/*}}/*.h

clean:
	rm -f $(ODIR)/*.o $(ODIR)/*.d *~ core $(INCDIR)/*~ doc/*.{aux,dvi,log,out,toc,pdf} python/*.so lib/*a
