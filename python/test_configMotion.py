import fpu_driver

from fpu_driver import getGridStateSummary
from fpu_driver import getGridStateSummary as gGSS

NUM_FPUS = 10
gateway_adr_list = [ fpu_driver.GatewayAddress("127.0.0.1", p)
                     for p in [4700, 4701, 4702] ]


def list_positions(gs):
    return [ (gs.FPU[i].alpha_steps, gs.FPU[i].beta_steps) for i in range(NUM_FPUS)]


def list_states(gs):
    return [ gs.FPU[i].state for i in range(NUM_FPUS)]
    
gd = fpu_driver.GridDriver(NUM_FPUS)

print("initializing driver: ", gd.initializeDriver())


print("connecting grid:", gd.connect(gateway_adr_list))


print("getting grid state:")
gs = gd.getGridState()

print("issuing findDatum:")
gd.findDatum(gs)


wt = { 0: [ ( 10, 20),
           ( 15, 21),
           ( 20, 22),
           ( 30, 25),
           ( 50, 19) ],
      
      1: [ (-11, 21),
           (-11, 25),
           (-11, 30),
           (-10, 28),
           ( -9, 25) ],
      
      2: [ ( 100, 21),
           (-100, 21),
           ( 0, 22),
           ( 0, 25),
           ( 100, 20) ],
}
gd.configMotion(wt, gs)

                       






