import fpu_driver

NUM_FPUS = 1000
gateway_adr_list = [ fpu_driver.GatewayAddress("127.0.0.1", p)
                     for p in [4700, 4701, 4702] ]


def list_positions(gs):
    return [ (gs.FPU[i].alpha_steps, gs.FPU[i].beta_steps) for i in range(NUM_FPUS)]

gd = fpu_driver.GridDriver(NUM_FPUS)

print("initializing driver: ", gd.initializeDriver())


print("connecting grid:", gd.connect(gateway_adr_list))


print("getting grid state:")
gs = gd.getGridState()


print("getting positions:")
gd.getPositions(gs)

print("positions before:", list_positions(gs))

print("finding Datum")
gd.findDatum(gs)

print("set positions after:", list_positions(gs))

#print("getting positions:")
#gd.getPositions(gs)
#
#print("reported positions after:", list_positions(gs))






