IDIR = ./include

CC = "g++"

GIT_VERSION := $(shell git describe --abbrev=4 --dirty --always --tags)

CXXFLAGS = -I$(IDIR) -std=c++11 -Wall -Wextra -pedantic -Werror -fPIC -DDEBUG -g 

ODIR = ./obj

LDIR = ./lib

SRCDIR = ./src

LIBS = -lm -lpthread

_DEPS = DriverState.h E_GridState.h FPUState.h GridDriver.h		\
	GridDriverConfig.h E_LogLevel.h GridState.h T_GridState.h	\
	canlayer/AsyncDriver.h T_GatewayAddress.h			\
	canlayer/handleFPUResponse.h canlayer/handleTimeout.h		\
	canlayer/CANError.h canlayer/CommandPool.h			\
	canlayer/CAN_Constants.h canlayer/E_CAN_COMMAND.h		\
	canlayer/CommandQueue.h DriverConstants.h canlayer/FPUArray.h	\
	canlayer/GatewayDriver.h canlayer/I_CAN_Command.h		\
	canlayer/I_ResponseHandler.h canlayer/SBuffer.h			\
	canlayer/TimeOutList.h canlayer/commands/AbortMotionCommand.h	\
	canlayer/commands/ConfigureMotionCommand.h			\
	canlayer/commands/EnableBetaCollisionProtectionCommand.h	\
	canlayer/commands/ExecuteMotionCommand.h			\
	canlayer/commands/FindDatumCommand.h				\
	canlayer/commands/FreeBetaCollisionCommand.h			\
	canlayer/commands/GetErrorAlphaCommand.h			\
	canlayer/commands/GetErrorBetaCommand.h				\
	canlayer/commands/GetStepsAlphaCommand.h			\
	canlayer/commands/GetStepsBetaCommand.h				\
	canlayer/commands/PingFPUCommand.h				\
	canlayer/commands/ReadRegisterCommand.h				\
	canlayer/commands/RepeatMotionCommand.h				\
	canlayer/commands/ResetFPUCommand.h				\
	canlayer/commands/ReverseMotionCommand.h			\
	canlayer/commands/SetUStepLevelCommand.h			\
	canlayer/sync_utils.h canlayer/time_utils.h


DEPS = $(patsubst %,$(IDIR)/%,$(_DEPS))

_OBJ =  GridDriver.o    AsyncDriver.o FPUArray.o GridState.o \
	CommandPool.o   GatewayDriver.o  TimeOutList.o \
	CommandQueue.o  time_utils.o  sync_utils.o SBuffer.o \
	handleFPUResponse.o handleTimeout.o FPUState.o


OBJ = $(patsubst %,$(ODIR)/%,$(_OBJ))


.PHONY: force clean

git_version: force
	echo '$(GIT_VERSION)' | cmp -s - $@ || echo '$(GIT_VERSION)' > $@

tutorial:	doc/tutorial.tex
	cd doc; pdflatex --shell-escape tutorial.tex; makeindex tutorial ; pdflatex --shell-escape tutorial.tex;

$(ODIR)/%.o: $(SRCDIR)/%.cpp $(DEPS) 
	$(CC) $(CXXFLAGS) -c -o $@ $< 

lib/libfpudriver.a: $(OBJ)
	ar rcs   $@ $^ 

driver: lib/libfpudriver.a

pyext: lib/libfpudriver.a python/src/fpu_driver.cpp $(DEPS) git_version
	g++ -shared -std=c++11 -I/usr/local/include -I/usr/include/python2.7 -fPIC -o python/fpu_driver.so python/src/fpu_driver.cpp -L./lib  -lfpudriver -lboost_python -g -DVERSION=\"$(GIT_VERSION)\"


clean:
	rm -f $(ODIR)/*.o *~ core $(INCDIR)/*~ doc/*.{aux,dvi,log,out,toc,pdf}


