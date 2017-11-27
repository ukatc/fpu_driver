import fpu_driver

NUM_FPUS = 5
ga = fpu_driver.GatewayAddress("127.0.0.1", 4700)

gd = fpu_driver.GridDriver(NUM_FPUS)

print("initializing driver: ", gd.initializeDriver())


print("connecting grid:", gd.connect([ga]))




