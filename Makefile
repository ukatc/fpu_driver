IDIR = ./include
CC = "g++"

VERSION := 1.3.5

CXXFLAGS = -I$(IDIR) -std=c++11 -Wall -Wextra -pedantic -Werror -fPIC -DDEBUG -g 

ODIR = ./objects

LDIR = ./lib

SRCDIR = ./src

LIBS = -lm -lpthread

_DEPS = InterfaceState.h E_GridState.h FPUState.h EtherCANInterface.h		      \
	EtherCANInterfaceConfig.h E_LogLevel.h GridState.h T_GridState.h	      \
	ethercan/AsyncInterface.h T_GatewayAddress.h			              \
	ethercan/handleFPUResponse.h ethercan/handleTimeout.h		              \
	ethercan/CANError.h ethercan/CommandPool.h			              \
	ethercan/CAN_Constants.h ethercan/E_CAN_COMMAND.h		              \
	ethercan/CommandQueue.h InterfaceConstants.h ethercan/FPUArray.h	      \
	ethercan/GatewayInterface.h ethercan/CAN_Command.h		              \
	ethercan/I_ResponseHandler.h ethercan/SBuffer.h			              \
	ethercan/TimeOutList.h                                                        \
	ethercan/cancommandsv2/AbortMotionCommand.h                                   \
	ethercan/cancommandsv2/CheckIntegrityCommand.h				      \
	ethercan/cancommandsv2/ConfigureMotionCommand.h				      \
	ethercan/cancommandsv2/EnableAlphaLimitProtectionCommand.h		      \
	ethercan/cancommandsv2/EnableBetaCollisionProtectionCommand.h		      \
	ethercan/cancommandsv2/EnableMoveCommand.h				      \
	ethercan/cancommandsv2/ExecuteMotionCommand.h				      \
	ethercan/cancommandsv2/FindDatumCommand.h				      \
	ethercan/cancommandsv2/FreeAlphaLimitBreachCommand.h			      \
	ethercan/cancommandsv2/FreeBetaCollisionCommand.h			      \
	ethercan/cancommandsv2/GetFirmwareVersionCommand.h			      \
	ethercan/cancommandsv2/LockUnitCommand.h				      \
	ethercan/cancommandsv2/PingFPUCommand.h					      \
	ethercan/cancommandsv2/ReadFirmwareVersionCommand.h			      \
	ethercan/cancommandsv2/ReadRegisterCommand.h				      \
	ethercan/cancommandsv2/ReadSerialNumberCommand.h			      \
	ethercan/cancommandsv2/RepeatMotionCommand.h				      \
	ethercan/cancommandsv2/ResetFPUCommand.h				      \
	ethercan/cancommandsv2/ResetStepCounterCommand.h			      \
	ethercan/cancommandsv2/ReverseMotionCommand.h				      \
	ethercan/cancommandsv2/SetStepsPerSegmentCommand.h			      \
	ethercan/cancommandsv2/SetTicksPerSegmentCommand.h			      \
	ethercan/cancommandsv2/SetUStepLevelCommand.h				      \
	ethercan/cancommandsv2/UnlockUnitCommand.h				      \
	ethercan/cancommandsv2/WriteSerialNumberCommand.h			      \
	ethercan/response_handlers/handle_AbortMotion_response.h		      \
	ethercan/response_handlers/handle_CheckIntegrity_response.h		      \
	ethercan/response_handlers/handle_ConfigMotion_response.h		      \
	ethercan/response_handlers/handle_EnableAlphaLimitProtection_response.h	      \
	ethercan/response_handlers/handle_EnableBetaCollisionProtection_response.h    \
	ethercan/response_handlers/handle_ExecuteMotion_response.h		      \
	ethercan/response_handlers/handle_FindDatum_response.h			      \
	ethercan/response_handlers/handle_FinishedDatum_message.h		      \
	ethercan/response_handlers/handle_FinishedMotion_message.h		      \
	ethercan/response_handlers/handle_FreeAlphaLimitBreach_response.h	      \
	ethercan/response_handlers/handle_FreeBetaCollision_response.h		      \
	ethercan/response_handlers/handle_GetFirmwareVersion_response.h		      \
	ethercan/response_handlers/handle_LockUnit_response.h			      \
	ethercan/response_handlers/handle_PingFPU_response.h			      \
	ethercan/response_handlers/handle_ReadRegister_response.h		      \
	ethercan/response_handlers/handle_ReadSerialNumber_response.h		      \
	ethercan/response_handlers/handle_RepeatMotion_response.h		      \
	ethercan/response_handlers/handle_ResetFPU_response.h			      \
	ethercan/response_handlers/handle_ResetStepCounter_response.h		      \
	ethercan/response_handlers/handle_ReverseMotion_response.h		      \
	ethercan/response_handlers/handle_SetStepsPerSegment_response.h		      \
	ethercan/response_handlers/handle_SetTicksPerSegment_response.h		      \
	ethercan/response_handlers/handle_SetUStepLevel_response.h		      \
	ethercan/response_handlers/handle_UnlockUnit_response.h			      \
	ethercan/response_handlers/handle_WarnCANOverflow_warning.h		      \
	ethercan/response_handlers/handle_WarnCollisionBeta_warning.h		      \
	ethercan/response_handlers/handle_WarnLimitAlpha_warning.h		      \
	ethercan/response_handlers/handle_WriteSerialNumber_response.h                \
	ethercan/sync_utils.h ethercan/time_utils.h                                   \
	ethercan/decode_CAN_response.h                                          

DEPS = $(patsubst %,$(IDIR)/%,$(_DEPS))

_OBJ =  EtherCANInterface.o    AsyncInterface.o FPUArray.o GridState.o \
	CommandPool.o   GatewayInterface.o  TimeOutList.o \
	CommandQueue.o  time_utils.o  sync_utils.o SBuffer.o \
	handleFPUResponse.o handleTimeout.o FPUState.o


OBJ = $(patsubst %,$(ODIR)/%,$(_OBJ))


.PHONY: force clean

version: force
	echo '$(VERSION)' | cmp -s - $@ || echo '$(VERSION)' > $@

doc/FPU-state1.pdf : doc/FPU-state1.svg
	inkscape doc/FPU-state1.svg --export-pdf=doc/FPU-state1.pdf

tutorial:	python/doc/tutorial.tex python/doc/FPU-state1.pdf version
	cd python/doc; pdflatex --shell-escape tutorial.tex; makeindex tutorial ; pdflatex --shell-escape tutorial.tex;

cppcheck: force
	cppcheck src/*.C python/src/*.cpp  -I include -I include/ethercan -I include/ethercan/cancommandsv2 --enable=all

$(ODIR)/%.o: $(SRCDIR)/%.C $(DEPS) version
	$(CC) $(CXXFLAGS) -DVERSION=\"$(VERSION)\" -c -o $@ $< 

lib/libethercan.a: $(OBJ)
	ar rcs   $@ $^ 

lib: lib/libethercan.a

pyext: lib/libethercan.a python/src/ethercanif.cpp $(DEPS) version
	g++ -shared -std=c++11 -I/usr/local/include -I/usr/include/python2.7 -fPIC -o python/ethercanif.so python/src/ethercanif.cpp -L./lib  -lethercan -lboost_python -g -DVERSION=\"$(VERSION)\"

style:
	astyle src/*.C python/src/*.cpp include{,/*{,/*}}/*.h

clean:
	rm -f $(ODIR)/*.o *~ core $(INCDIR)/*~ doc/*.{aux,dvi,log,out,toc,pdf}


