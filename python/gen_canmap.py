# A utility to help generate arbitrary canmap files.
NUM_FPUS = 1068
for fpu in range(0,NUM_FPUS):
    fpu1 = fpu + 1
    strg = '\"%d\"' % fpu1
    print(' %5s : %4d,' % (strg,fpu))
 
