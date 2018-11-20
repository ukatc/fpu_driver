IDIR = ./include
CC = "g++"

VERSION := v1.4.0

CXXFLAGS = -I$(IDIR) -std=c++11 -Wall -Wextra -pedantic -Werror -fPIC -DDEBUG -g 

ODIR = ./objects

LDIR = ./lib

SRCDIR = ./src

LIBS = -lm -lpthread

_DEPS = InterfaceState.h E_GridState.h FPUState.h EtherCANInterface.h		\
	EtherCANInterfaceConfig.h E_LogLevel.h GridState.h T_GridState.h	\
	ethercan/AsyncInterface.h T_GatewayAddress.h			\
	ethercan/handleFPUResponse.h ethercan/handleTimeout.h		\
	ethercan/CANError.h ethercan/CommandPool.h			\
	ethercan/CAN_Constants.h ethercan/E_CAN_COMMAND.h		\
	ethercan/CommandQueue.h InterfaceConstants.h ethercan/FPUArray.h	\
	ethercan/GatewayInterface.h ethercan/I_CAN_Command.h		\
	ethercan/I_ResponseHandler.h ethercan/SBuffer.h			\
	ethercan/TimeOutList.h ethercan/cancommands/AbortMotionCommand.h	\
	ethercan/cancommands/ConfigureMotionCommand.h			\
	ethercan/cancommands/EnableBetaCollisionProtectionCommand.h	\
	ethercan/cancommands/ExecuteMotionCommand.h			\
	ethercan/cancommands/FindDatumCommand.h				\
	ethercan/cancommands/FreeBetaCollisionCommand.h			\
	ethercan/cancommands/GetErrorAlphaCommand.h			\
	ethercan/cancommands/GetErrorBetaCommand.h				\
	ethercan/cancommands/GetStepsAlphaCommand.h			\
	ethercan/cancommands/GetStepsBetaCommand.h				\
	ethercan/cancommands/PingFPUCommand.h				\
	ethercan/cancommands/ReadRegisterCommand.h				\
	ethercan/cancommands/RepeatMotionCommand.h				\
	ethercan/cancommands/ResetFPUCommand.h				\
	ethercan/cancommands/ReverseMotionCommand.h			\
	ethercan/cancommands/SetUStepLevelCommand.h			\
	ethercan/cancommands/WriteSerialNumberCommand.h			\
	ethercan/cancommands/ReadSerialNumberCommand.h			\
	ethercan/sync_utils.h ethercan/time_utils.h


DEPS = $(patsubst %,$(IDIR)/%,$(_DEPS)) Makefile

_OBJ =  EtherCANInterface.o    AsyncInterface.o FPUArray.o GridState.o \
	CommandPool.o   GatewayInterface.o  TimeOutList.o \
	CommandQueue.o  time_utils.o  sync_utils.o SBuffer.o \
	handleFPUResponse.o handleTimeout.o FPUState.o


OBJ = $(patsubst %,$(ODIR)/%,$(_OBJ))


.PHONY: force clean

version: force
	echo '$(VERSION)' | cmp -s - $@ || echo '$(VERSION)' > $@

python/doc/FPU-state1.pdf : python/doc/FPU-state1.svg
	inkscape python/doc/FPU-state1.svg --export-pdf=python/doc/FPU-state1.pdf

tutorial:	python/doc/tutorial.tex python/doc/FPU-state1.pdf version
	cd python/doc; pdflatex --shell-escape tutorial.tex; makeindex tutorial ; pdflatex --shell-escape tutorial.tex;

cppcheck: force
	cppcheck src/*.C python/src/*.cpp  -I include -I include/ethercan -I include/ethercan/cancommands --enable=all

$(ODIR)/%.o: $(SRCDIR)/%.C $(DEPS) version
	$(CC) $(CXXFLAGS) -DVERSION=\"$(VERSION)\" -c -o $@ $< 

lib/libethercan.a: $(OBJ)
	ar rcs   $@ $^ 

lib: lib/libethercan.a

pyext: lib/libethercan.a python/src/ethercanif.C $(DEPS) version
	g++ -shared -std=c++11 -I/usr/local/include -I/usr/include/python2.7 -fPIC -o python/ethercanif.so python/src/ethercanif.C -L./lib  -lethercan -lboost_python -g -DVERSION=\"$(VERSION)\"

style:
	astyle src/*cpp python/src/*cpp include/{,/*{,*/{,*/}},}/*.h

clean:
	rm -f $(ODIR)/*.o *~ core $(INCDIR)/*~ doc/*.{aux,dvi,log,out,toc,pdf} python/*.so lib/*a


