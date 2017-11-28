IDIR = ./include

CC = "g++"

CXXFLAGS = -I$(IDIR) -std=c++11 -Wall -Wextra -pedantic -fPIC -DDEBUG

ODIR = ./obj

LDIR = ./lib

SRCDIR = ./src

LIBS = -lm -lpthread

_DEPS = DriverState.h E_GridState.h FPUState.h GridDriver.h	\
	GridState.h T_GridState.h canlayer/AsyncDriver.h	\
	T_GatewayAddress.h canlayer/handleFPUResponse.h		\
	canlayer/CANError.h canlayer/CommandPool.h		\
	canlayer/CommandQueue.h DriverConstants.h		\
	canlayer/CAN_Constants.h canlayer/E_CAN_COMMAND.h	\
	canlayer/FPUArray.h canlayer/GatewayDriver.h		\
	canlayer/I_CAN_Command.h canlayer/I_ResponseHandler.h	\
	canlayer/SBuffer.h canlayer/TimeOutList.h		\
	canlayer/sync_utils.h canlayer/time_utils.h		\
	canlayer/commands/ConfigureMotionCommand.h		\
	canlayer/commands/ExecuteMotionCommand.h		\
	canlayer/commands/GetStepsAlphaCommand.h		\
	canlayer/commands/AutoMoveDatumCommand.h		\
	canlayer/commands/PingCommand.h


DEPS = $(patsubst %,$(IDIR)/%,$(_DEPS))

_OBJ =  GridDriver.o    AsyncDriver.o FPUArray.o GridState.o \
	CommandPool.o   GatewayDriver.o  TimeOutList.o \
	CommandQueue.o  time_utils.o  sync_utils.o SBuffer.o \
	handleFPUResponse.o


OBJ = $(patsubst %,$(ODIR)/%,$(_OBJ))



$(ODIR)/%.o: $(SRCDIR)/%.cpp $(DEPS)
	$(CC) $(CXXFLAGS) -c -o $@ $< 

lib/libfpudriver.a: $(OBJ)
	ar rcs   $@ $^ 

driver: lib/libfpudriver.a

pyextension: lib/libfpudriver.a python/src/fpu_driver.cpp $(DEPS)
	g++ -shared -std=c++11 -I/usr/local/include -I/usr/include/python2.7 -fPIC -o python/fpu_driver.so python/src/fpu_driver.cpp -L./lib  -lfpudriver -lboost_python

.PHONY: clean

clean:
	rm -f $(ODIR)/*.o *~ core $(INCDIR)/*~

