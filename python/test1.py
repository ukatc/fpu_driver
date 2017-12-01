import fpu_driver

NUM_FPUS = 500
ga = fpu_driver.GatewayAddress("127.0.0.1", 4700)

gd = fpu_driver.GridDriver(NUM_FPUS)

print("initializing driver: ", gd.initializeDriver())


print("connecting grid:", gd.connect([ga]))


print("getting grid state:")
gs = gd.getGridState()

print("positions before:", [ (gs.FPU[i].alpha_steps, gs.FPU[i].beta_steps) for i in range(NUM_FPUS)])

print("getting positions:")
gd.getPositions(gs)

print("positions:", [ (gs.FPU[i].alpha_steps, gs.FPU[i].beta_steps) for i in range(NUM_FPUS)])





