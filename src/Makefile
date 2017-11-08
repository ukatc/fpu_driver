IDIR=.
CC="g++"

CXXFLAGS=-I$(IDIR) -std=c++11

ODIR=obj
LDIR =../lib

LIBS=-lm -lpthread

_DEPS = AsyncDriver.h      E_CAN_COMMAND.h  I_CAN_Command.h      \
	CANError.h         FPUArray.h       I_ResponseHandler.h  \
	CommandPool.h      FPUState.h       SBuffer.h            \
	CommandQueue.h     GatewayDriver.h  sync_utils.h         \
	DriverConstants.h  GridDriver.h     TimeOutList.h        \
	DriverState.h      time_utils.h         \
	commands/ConfigureMotionCommand.h  commands/MoveDatumOn.h \
	commands/MoveDatumOffCommand.h     commands/PingCommand.h \
	GridState.h T_GridState.h E_GridState.h



DEPS = $(patsubst %,$(IDIR)/%,$(_DEPS))

_OBJ =  GridDriver.o    AsyncDriver.o FPUArray.o GridState.o \
	CommandPool.o   GatewayDriver.o  TimeOutList.o \
	CommandQueue.o  time_utils.o  sync_utils.o

OBJ = $(patsubst %,$(ODIR)/%,$(_OBJ))


$(ODIR)/%.o: %.cpp $(DEPS)
	$(CC) $(CFLAGS) -c -o $@ $< 

driver: $(OBJ)
	$CC) -o $@ $^ $(CFLAGS) $(LIBS)

.PHONY: clean

clean:
	rm -f $(ODIR)/*.o *~ core $(INCDIR)/*~

