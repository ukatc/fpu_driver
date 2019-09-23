from __future__ import print_function
import FpuGridDriver
from FpuGridDriver import MOCK_GATEWAY_ADRESS_LIST as gw_address
from fpu_commands import list_positions, \
       list_deviations, list_states, gen_wf


if os.environ.get("MOCKUP","") != "":
    mockup = True
else:
    mockup = False


# print the driver version, just to check
print("The FPU driver version is:", FpuGridDriver.__version__,
      ", CAN PROTOCOL version:", FpuGridDriver.CAN_PROTOCOL_VERSION)

NUM_FPUS = 1 # set number of FPUs to 1
gd = FpuGridDriver.GridDriver(NUM_FPUS, mockup=mockup)
# connect to gateway with default IP 192.168.0.10
print("connecting grid:", gd.connect(address_list=gw_address))

print("getting grid state:")
grid_state = gd.getGridState()


print("issuing findDatum:")
gd.findDatum(grid_state)
print("findDatum finished")

alpha_move = 45
beta_move = 15

# Generate the required waveform for one FPU
waveform = gen_wf(alpha_move, beta_move)

# upload the waveform to the FPU
gd.configMotion(waveform, grid_state)
