from __future__ import print_function, division

import argparse
from datetime import datetime 
import os.path
from os import makedirs

from FpuGridDriver import GridDriver,GatewayAddress
from fpu_commands import gen_wf
from wflib import load_waveform
from GigE.GigECamera import GigECamera, BASLER_DEVICE_CLASS, DEVICE_CLASS, IP_ADDRESS
from vfr.conf import POS_REP_CAMERA_IP_ADDRESS as CAMERA_IP_ADDRESS



CAMERA_CONF = {
    DEVICE_CLASS: BASLER_DEVICE_CLASS,
    IP_ADDRESS: CAMERA_IP_ADDRESS,
}

IPATH = "movment%i_%i.bmp"

class MeasureGridDriver(GridDriver):
    """ Extended GridDriver Class to take images during movement.
    
    DO NOT USE FOR ANYOTHER PURPOSE
    """
    
    def executeMotion(self, gs, camera, ipath, fpuset=None, sync_command=True):
        if fpuset is None:
            fpuset = []
            
        print("USEABLE ONLY FOR MOVEMENT ANALYSIS")
        
        fpuset = self.check_fpuset(fpuset)

        if len(fpuset) == 0:
            fpuset = range(self.config.num_fpus)

        with self.lock:
            # wait a short moment to avoid spurious collision.
            initial_positions={}
            self._start_execute_motion_hook(gs, fpuset=fpuset, initial_positions=initial_positions)
            time.sleep(0.1)
            prev_gs = self._gd.getGridState() # get last FPU states and timeout counters
            try:
                time.sleep(0.1)
                rv = self._gd.startExecuteMotion(gs, fpuset, sync_command)
            except InvalidStateException as e:
                self._cancel_execute_motion_hook(gs, fpuset, initial_positions=initial_positions)
                raise
            if rv != ethercanif.E_EtherCANErrCode.DE_OK:
                raise RuntimeError("FPUs not ready to move, driver error code = %r" % rv)

            time_interval = 0.1
            is_ready = False
            was_aborted = False
            refresh_state = False
            rv = "UNDONE"
            try:
                with SignalHandler() as sh:
                    while not is_ready:
                        rv = self._gd.waitExecuteMotion(gs, time_interval, fpuset)
                        image_file = os.path.join(ipath,datetime.now().strftime('%Y-%m-%d_%H-%M-%S-%f') + '.bmp')
                        camera.saveImage(image_file)
                        if sh.interrupted:
                            print("STOPPING FPUs.")
                            self.abortMotion(gs, fpuset, sync_command)
                            was_aborted = True
                            refresh_state = True
                            break
                        is_ready = (rv != ethercanif.E_EtherCANErrCode.DE_WAIT_TIMEOUT)

            except MovementError:
                refresh_state = True
                raise
            except CommandTimeout:
                refresh_state = True
                raise

            finally:
                # This is skipped in case of a SocketFailure, for example
                if (rv == ethercanif.E_EtherCANErrCode.DE_OK) or refresh_state:
                    # execute a ping to update positions
                    # (this is only needed for protocol version 1)
                    move_gs = self._gd.getGridState()
                    try:
                        time.sleep(0.1)
                        pingset = self.need_ping(gs, fpuset)
                        if len(pingset) > 0 :
                            self._pingFPUs(gs, pingset)
                    except CommandTimeout:
                        pass
                    # the following hook will narrow down the recorded intervals of positions
                    self._post_execute_motion_hook(gs, prev_gs, move_gs, fpuset)

            if was_aborted:
                raise MovementError("executeMotion was aborted by SIGINT")

        return rv
   
def get_parser():
    parser = argparse.ArgumentParser(description='Move a singe FPU by a specified alpha, beta from datum')
    parser.add_argument('-i','--ipath',
                        help='root image location, files will stored in a datetime stamped folder.')
                        
    subparser = parser.add_subparsers(help='Chose the control mode')
    
    direct_parser = subparser.add_parser('direct', aliases='d', help='Direct control mode')
    direct_parser.add_argument('-a','--alpha',
                        type=int,
                        nargs='+',
                        help='angle to move alpha arm')
    direct_parser.add_argument('-b','--beta',
                        type=int,
                        nargs='+',
                        help='angle to move beta arm')
    direct_parser.set_defaults(directMode=True))
                        
    file_parser = subparser.add_parser('file', aliases='f', help='File control mode')
    file_parser.add_argument('-f','--file',
                        nargs='+',
                        help='file containing waveforms')
    file_parser.add_argument('-c','--cmap',
                        nargs='+',
                        help='canmap.cfg file to configure which waveforms go to which FPU')
    file_parser.set_defaults(directMode=False))
    return parser
    
if __name__ == "__main__":

    parser = get_parser()
    args = parser.parse_args()
    
    print(args)
    print("did the bad thing")
    quit()
    
    image_path = os.path.join(args.ipath,datetime.datetime.now().strftime('%Y-%m-%d_%H-%M-%S-%f'))
    
    print("Creating image folder {}".format(image_path))
    
    makedirs(image_path)
    
    camera = GigECamera(CAMERA_CONF)
    
    NUM_FPUS = 1
    
    ipath = IPATH
    
    # setup driver
    gd = MeasureGridDriver(NUM_FPUS)
    
    gwa = GatewayAddress("192.168.0.10", 4700)
    
    # connect grid driver
    gd.connect(address_list=gwa)
    #initialise grid driver
    gs = gd.getGridState()
    
    gd.findDatum(gs)
    
    #setup movement
    if args.directMode:
        waveform = gen_wf(args.alpha, args.beta)
    else:
        waveform = load_waveform(args.file,args.cmap, reverse=False)
    
    gd.configMotion(waveform, gs)
    #execute motion
    print("Starting motion")
    gd.executeMotion(gs, camera, image_path)
    print("Finished motion, images should be saved")
    
    