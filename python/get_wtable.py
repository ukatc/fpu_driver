from __future__ import print_function
from numpy import zeros
import cPickle
import sys
import time


def gwt(gd, gs, dumpfilename="wavetables-dump-{unixtime}.pck"):
    N = len(gs.FPU)
    len_wt = 128
    narms = 2
    nbytes = 2
    WT = zeros( (N, len_wt, narms, nbytes) )
    for seg in range(len_wt):
        print("\rfetching segment %i" % seg, end="")
        sys.stdout.flush()
        for arm in range(narms):
            bank = [1, 2][arm]
            for byte in range(nbytes):
                time.sleep(0.1)
                gd.readRegister(bank * 256 + seg * 2 + byte, gs)
                for i in range(N):
                    WT[i,seg,arm,byte] = gs.FPU[i].register_value

    print("\n... done")
    utime=int(time.time())
    dumpfilename=dumpfilename.format(unixtime=utime)
    f = open(dumpfilename, 'w')
    cPickle.dump(WT, f)
    f.close()
    print("wavetables data saved to ", dumpfilename)

    return WT
