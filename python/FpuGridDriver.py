
#!/usr/bin/python

from __future__ import print_function, division
import os
from os import path
import errno
import time
import signal
from warnings import warn, filterwarnings
# state tracking
import lmdb
from ast import literal_eval

from fpu_constants import *

import fpu_driver

from fpu_driver import __version__, CAN_PROTOCOL_VERSION, GatewayAddress,  \
    REQD_ANTI_CLOCKWISE,  REQD_CLOCKWISE, FPUDriverException, MovementError, \
    DASEL_BOTH, DASEL_ALPHA, DASEL_BETA, \
    LOG_ERROR, LOG_INFO, LOG_GRIDSTATE, LOG_DEBUG, LOG_VERBOSE, LOG_TRACE_CAN_MESSAGES, \
    SEARCH_CLOCKWISE, SEARCH_ANTI_CLOCKWISE, SEARCH_AUTO, SKIP_FPU


import fpu_commands as cmds


MOCK_GATEWAY_ADRESS_LIST = [ GatewayAddress("127.0.0.1", p)
                                for p in [4700, 4701, 4702] ]


TEST_GATEWAY_ADRESS_LIST = [ GatewayAddress("192.168.0.10", 4700) ]

DEFAULT_GATEWAY_ADRESS_LIST = MOCK_GATEWAY_ADRESS_LIST


DEFAULT_NUM_FPUS=int(os.environ.get("NUM_FPUS","1005"))

DEFAULT_LOGDIR=os.environ.get("FPU_DRIVER_DEFAULT_LOGDIR","./_logs")
str_loglevel = os.environ.get("FPU_DRIVER_DEFAULT_LOGLEVEL","LOG_TRACE_CAN_MESSAGES")
DEFAULT_LOGLEVEL= {"LOG_ERROR":    LOG_ERROR,    
                   "LOG_INFO":     LOG_INFO,     
                   "LOG_GRIDSTATE":LOG_GRIDSTATE,
                   "LOG_VERBOSE":LOG_VERBOSE,
                   "LOG_DEBUG":    LOG_DEBUG,    
                   "LOG_TRACE_CAN_MESSAGES": LOG_TRACE_CAN_MESSAGES}[str_loglevel]


class SignalHandler(object):
    """Context manager for handling a signal
    while waiting for command completion.
    """

    def __init__(self, sig=signal.SIGINT):
        self.sig = sig

    def __enter__(self):

        self.interrupted = False
        self.released = False

        self.original_handler = signal.getsignal(self.sig)

        def handler(signum, frame):
            self.release()
            self.interrupted = True

        signal.signal(self.sig, handler)

        return self

    def __exit__(self, type, value, tb):
        self.release()

    def release(self):

        if self.released:
            return False

        signal.signal(self.sig, self.original_handler)

        self.released = True

        return True

def get_logname(basename,
                log_dir="",
                timestamp=None):
    
    if timestamp=="ISO8601":
        timestamp= time.strftime("%Y-%m-%dT%H:%M:%S", time.localtime())

    filename = basename.format(start_timestamp=timestamp)
    return path.join(log_dir, path.expandvars(path.expanduser(filename)))


def make_logdir(log_dir):
    log_path = path.abspath(path.expandvars(path.expanduser(log_dir)))
    try:
        os.makedirs(log_path, 0o00744)
    except OSError as e:
        if e.errno == errno.EEXIST:
            pass
        else:
            raise
        
    if log_dir != DEFAULT_LOGDIR:
        print("logging to ", log_path)
        
    return log_path

filterwarnings("default", "keyword check_protection", DeprecationWarning)

class UnprotectedGridDriver:
    def __init__(self, nfpus=DEFAULT_NUM_FPUS,
                 SocketTimeOutSeconds=20.0,
                 alpha_datum_offset=ALPHA_DATUM_OFFSET,
                 logLevel=DEFAULT_LOGLEVEL,
                 log_dir=DEFAULT_LOGDIR,
                 control_logfile="_{start_timestamp}-fpu_control.log",
                 tx_logfile = "_{start_timestamp}-fpu_tx.log",
                 rx_logfile = "_{start_timestamp}-fpu_rx.log",
                 start_timestamp="ISO8601"):


        config = fpu_driver.GridDriverConfig()
        config.num_fpus = nfpus
        config.SocketTimeOutSeconds = SocketTimeOutSeconds
        config.alpha_datum_offset = alpha_datum_offset
        
        flags = os.O_CREAT | os.O_APPEND | os.O_WRONLY
        mode = 0o00644

        

        config.logLevel = logLevel

        log_path = make_logdir(log_dir)
        
        config.fd_controllog = os.open(get_logname(control_logfile, 
                 log_dir=log_path, timestamp=start_timestamp), flags, mode)
        config.fd_txlog = os.open(get_logname(tx_logfile, 
                 log_dir=log_path, timestamp=start_timestamp), flags, mode)
        config.fd_rxlog = os.open(get_logname(rx_logfile, 
                 log_dir=log_path, timestamp=start_timestamp), flags, mode)
        
        self.config = config
        self.last_wavetable = {}

        self._gd = fpu_driver.GridDriver(config)

    def __del__(self):
        # the following delete is necessary to run destructors!!
        del self._gd
        os.close(self.config.fd_controllog)
        os.close(self.config.fd_txlog)
        os.close(self.config.fd_rxlog)

    def post_connect_hook(self):
        pass

    def connect(self, address_list=DEFAULT_GATEWAY_ADRESS_LIST):
        rv = self._gd.connect(address_list)
        self.post_connect_hook()
        return rv

    def setUStepLevel(self, ustep_level,  gs, fpuset=[]):
        return self._gd.setUStepLevel(ustep_level, gs, fpuset)
    
    def getGridState(self):
        return self._gd.getGridState()

    def findDatumB(self, gs, search_modes={}, selected_arm=DASEL_BOTH, soft_protection=True,
                   check_protection=None, fpuset=[]):
        """Moves all FPUs to datum position. 

        This is a blocking variant of the findDatum command,
        it is not interruptible by Control-C."""
        if check_protection != None:
            warn("keyword check_protection is deprecated; use 'soft_protection=False' instead!",
                 DeprecationWarning, 2)
            soft_protection=check_protection
        
        return self._gd.findDatum(gs, search_modes, fpuset, selected_arm, soft_protection)

    def findDatum(self, gs, search_modes={}, selected_arm=DASEL_BOTH, soft_protection=True,
                  check_protection=None, fpuset=[]):
        """Moves all FPUs to datum position. 

        If the program receives a SIGNINT, or Control-C is pressed, an
        abortMotion command is automatically sended, aborting the search.

        The parameter selected_arm (DASEL_BOTH, DASEL_ALPHA, DASEL_BETA) 
        controls which arms are moved.

        The dictionary fpu_modes has integer FPU IDs as keys, and 
        each value is one of SEARCH_CLOCKWISE, SEARCH_ANTI_CLOCKWISE, SEARCH_AUTO, SKIP_FPU,
        which controls whether the datum search moves clockwise (decreasing step count),
        anti-clockwise (increasing step count), automatically, or skips the FPU.
        The default mode is automatically.

        If an beta arm is not datumed, automatic datum search will be refused.
        If a beta arm position appears not to be safe to be moved into
        the requested position, the manual datum search will be refused
        unless soft_protection is set to False.

        """
        if check_protection != None:
            warn("keyword check_protection is deprecated; use 'soft_protection=False' instead!",
                 DeprecationWarning, 2)
            soft_protection=check_protection
            
        rv = self._gd.startFindDatum(gs, search_modes, selected_arm, fpuset, soft_protection)
        if rv != fpu_driver.E_DriverErrCode.DE_OK:
            raise RuntimeError("can't search Datum, driver error code = %r" % rv)
        time.sleep(0.1)
        time_interval = 0.1
        is_ready = False
        was_aborted = False
        with SignalHandler() as sh:
            while not is_ready:
                rv = self._gd.waitFindDatum(gs, time_interval, fpuset)
                if sh.interrupted:
                    print("STOPPING FPUs.")
                    self.abortMotion(gs)
                    was_aborted = True
                    break
                is_ready = (rv != fpu_driver.E_DriverErrCode.DE_WAIT_TIMEOUT)

        if was_aborted:
            self.pingFPUs(gs)
            print("findDatumw as aborted by SIGINT, movement stopped")
            raise MovementError("findDatum was aborted by SIGINT")

        return rv

    def pingFPUs(self, gs, fpuset=[]):
        return self._gd.pingFPUs(gs, fpuset)

    def reset_hook(self, gs. fpuset=[]):
        pass
    
    def resetFPUs(self, gs, fpuset=[]):
        old_state = copy.deepcopy(gs)
        rval = self._gd.resetFPUs(gs, fpuset)
        self.reset_hook(self, old_state, gs, fpuset=fpuset)
        return rval

    def getPositions(self, gs, fpuset=[]):
        return self._gd.getPositions(gs, fpuset)

    def readRegister(self, address, gs, fpuset=[]):
        return self._gd.readRegister(address, gs, fpuset)
    
    def getFirmwareVersion(self, gs, fpuset=[]):
        return self._gd.getFirmwareVersion(gs, fpuset)

    def printFirmwareVersion(self, gs, fpuset=[]):
        self.getFirmwareVersion(gs, fpuset)
        for fpu_id, fpu in enumerate(gs.FPU):
            if (fpuset == []) or (fpu_id in fpuset):
                print("FPU %i firmware version: (%i,%i,%i) created %02i-%02i-%02i" % (
                    fpu_id,
                    fpu.fw_version_major, fpu.fw_version_minor, fpu.fw_version_patch,
                    fpu.fw_date_year, fpu.fw_date_month, fpu.fw_date_day))

    def minFirmwareVersion(self, gs, fpuset=[]):
        self.getFirmwareVersion(gs, fpuset)
        min_version = (255,255,255)
        for fpu_id, fpu in enumerate(gs.FPU):
            if (fpuset == []) or (fpu_id in fpuset):
                version = (fpu.fw_version_major, fpu.fw_version_minor, fpu.fw_version_patch)
                if version < min_version:
                    min_version = version
                    
        return min_version
                             
    def getCounterDeviation(self, gs, fpuset=[]):
        return self._gd.getCounterDeviation(gs, fpuset)

    def readSerialNumbers(self, gs, fpuset=[]):
        return self._gd.readSerialNumbers(gs, fpuset)

    def printSerialNumbers(self, gs, fpuset=[]):
        self.readSerialNumbers(gs, fpuset=fpuset)
        for i in range(self.config.num_fpus):
            if (len(fpuset) == 0) or i in fpuset:
                print("FPU %i : SN = %s" % (i, gs.FPU[i].serial_number))
        
    
    def writeSerialNumber(self, fpu_id, serial_number,  gs):
        return self._gd.writeSerialNumber(fpu_id, serial_number, gs)

    def pre_config_motion_hook(self, wtable, gs, fpuset):
        pass
    
    def post_config_motion_hook(self, wtable, gs, fpuset):
        pass
    
    def configMotion(self, wavetable, gs, fpuset=[], soft_protection=True, check_protection=None):
        """ 
        Configures movement by sending a waveform table to a group of FPUs.
        Call signature is configMotion({ fpuid0 : {(asteps,bsteps), (asteps, bsteps), ...], fpuid1 : { ... }, ...}})

        When the 'protected' flag is set to False, bypass all 
        hardware protection checks, which will allow to move a
        collided or uncalibrated FPU (even if the movement might damage
        the hardware).

        """
        if check_protection != None:
            warn("keyword check_protection is deprecated; use 'soft_protection=False' instead!",
                 DeprecationWarning, 2)
            soft_protection=check_protection

        wtable = wavetable.copy()
        if len(fpuset) > 0:
            kys = wtable.keys()
            for k in kys:
                if not k in fpuset:
                    del wtable[k]
                    
        # check if wavetable is safe    
        self.pre_config_motion_hook(wtable, gs, fpuset)
        rval = self._gd.configMotion(wtable, gs, fpuset, soft_protection)
        # remember wavetable
        self.last_wavetable.update(wtable)
        
        self.post_config_motion_hook(wtable, gs, fpuset)
        return rval

    def getCurrentWaveTables(self):
        return self.last_wavetable
    
    def executeMotionB(self, gs, fpuset=[]):
        return self._gd.executeMotion(gs, fpuset)
    

    def executeMotion(self, gs, fpuset=[]):
        # wait a short moment to avoid spurious collision.
        time.sleep(2.5)
        rv = self._gd.startExecuteMotion(gs, fpuset)
        if rv != fpu_driver.E_DriverErrCode.DE_OK:
            print("rv=",rv)
            raise RuntimeError("FPUs not ready to move, driver error code = %r" % rv)
        time.sleep(0.1)
        time_interval = 0.1
        is_ready = False
        was_aborted = False
        refresh_state = False
        try:
            with SignalHandler() as sh:
                while not is_ready:
                    rv = self._gd.waitExecuteMotion(gs, time_interval, fpuset)
                    if sh.interrupted:
                        print("STOPPING FPUs.")
                        self.abortMotion(gs, fpuset)
                        was_aborted = True
                        break
                    is_ready = (rv != fpu_driver.E_DriverErrCode.DE_WAIT_TIMEOUT)
        except MovementError:
            refresh_state = True
            raise
            

        if (rv == fpu_driver.E_DriverErrCode.DE_OK) or was_aborted or refresh_state:
            # execute a ping to update positions
            # (this is only needed for protocol version 1)
            self.pingFPUs(gs, fpuset)
                            
        if was_aborted:
            raise MovementError("executeMotion was aborted by SIGINT")
        
        return rv

    def abortMotion(self, gs, fpuset=[]):
        return self._gd.abortMotion(gs, fpuset)

    def freeBetaCollision(self, fpu_id, direction,  gs):
        return self._gd.freeBetaCollision(fpu_id, direction, gs)
    
    def enableBetaCollisionProtection(self, gs):
        return self._gd.enableBetaCollisionProtection(gs)

    def reverseMotion(self, gs, fpuset=[]):
        return self._gd.reverseMotion(gs, fpuset)

    def repeatMotion(self, gs, fpuset=[]):
        return self._gd.repeatMotion(gs, fpuset)



DATABASE_FILE_NAME = os.environ.get("FPU_DATABASE")

env = lmdb.open(DATABASE_FILE_NAME, max_dbs=10)

fpudb = env.open_db("fpu")

class GridDriver(UnprotectedGridDriver):
    
    def post_connect_hook(self):
        grid_state = self.getGridState()
        self.readSerialNumbers(grid_state)
        apositions = {}
        bpositions = {}
        wtabs = {}
        limits = {}
        in_dicts = { 'apos' : apositions, 'bpos' : bpositions, 'wtab' : wtabs, 'limits' : limits }
        print("reading serial numbers from DB....")
        a_min_offsets = []
        a_max_offsets = []
        b_min_offsets = []
        b_max_offsets = []
        for fpu_id, fpu in enumerate(grid_state.FPU):
            serial_number = fpu.serial_number
            a_min_offsets.append(0.0)
            a_max_offsets.append(0.0)
            b_min_offsets.append(0.0)
            b_max_offsets.append(0.0)
            
            with env.begin(db=fpudb) as txn:
                for subkey in ["apos", "bpos", "wtab", "limits"]:
                    key = str((serial_number, subkey))
                    val = txn.get(key)
                          
                    print(key,":", val)
                    if val == None: 
                        raise fpu_driver.ProtectionError("serial number {0!r} not found in position database"
                                                         " - run fpu-admin.py to create entry".format(serial_number))
                    in_dicts[subkey][fpu_id] = literal_eval(val)
        print("state data: ", in_dicts)
        self.apositions = apositions
        self.bpositions = bpositions
        self.wtabs = wtabs
        self.limits = limits
        self.a_min_offsets = a_min_offsets
        self.a_max_offsets = a_max_offsets
        self.b_min_offsets = b_min_offsets
        self.b_max_offsets = b_max_offsets
        

        # query positions and compute offsets, if FPUs have been resetted.
        # This assumes that the stored positions are correct.
        super(GridDriver,self).pingFPUs(grid_state, record_positions=False)
        self.reset_hook(grid_state)
        self.refreshPositions(grid_state, store=False)


    def reset_hook(self, former_grid_state, new_state, fpuset=[]):
        """Updates the offset between the stored FPU positions and positions
        reported by ping. 

        If the FPUs have been switched off and powered on again, or if
        it has been resetted for another reason, the ping values
        differ from the recorded valid values, until a successful
        datum command has been run.  This defines an offset which must
        be added every time in order to yield the correct position.

        This method needs to be run:
        - every time the driver is initialising, after the initial ping
        - after every resetFPU command

        """
        for fpu_id, fpu in enumerate(former_grid_state.FPU):
            if fpuset!= None and (not (fpu_id in fpuset)):
                continue

            if ( (new_state.FPU[fpu_id].state != FPST_UNINITIALISED)
                 or (new_state.FPU[fpu_id].alpha_steps != 0)
                 or (new_state.FPU[fpu_id].beta_steps != 0)):
                # fpu wasn't resetted successfully
                continue
            
            apos = fpu.alpha_steps / StepsPerDegreeAlpha
            bpos = fpu.beta_steps / StepsPerDegreeBeta
            a_min_offsets[fpu_id] += apos - self.apositions[fpu_id][0]
            a_max_offsets[fpu_id] += apos - self.apositions[fpu_id][1]
                


            b_min_offsets[fpu_id] += bpos - self.bpositions[fpu_id][0]
            b_max_offsets[fpu_id] += bpos - self.bpositions[fpu_id][1]
                

                
    def refresh_positions(self, grid_state, store=True, fpuset=[]):
        """Computes new current positions from step count
        and offsets (if set), and stores them to database.
        """
        for fpu_id, fpu in enumerate(former_grid_state.FPU):
            if fpuset!= None and (not (fpu_id in fpuset)):
                continue
            
            apos = fpu.alpha_steps / StepsPerDegreeAlpha
            bpos = fpu.beta_steps / StepsPerDegreeBeta
            serial_number = fpu.serial_number
            
            self.apositions[fpu_id] = (self.a_min_offsets[fpu_id] + apos,
                                       self.a_max_offsets[fpu_id] + apos)
            self.bpositions[fpu_id] = (self.b_min_offsets[fpu_id] + bpos,
                                       self.b_max_offsets[fpu_id] + bpos)
            if store:
                with env.begin(db=fpudb) as txn:
                    key = str( (serial_number, "apos"))
                    val = str([alpha_pos, alpha_pos])
                    txn.put(key, val)
                    key = str( (serial_number, "bpos"))
                    val = str([beta_pos, beta_pos])

    def pingFPUs(self, grid_state, fpuset=[]):
        print("""super call to UnprotectedGridDriver.pingFPUs(). Make sure it
        does now has weird results.""")
        
        super(GridDriver, self).pingFPUs(grid_state, fpuset=fpuset)
        self.refresh_positions(grid_state, fpuset=fpuset)

            
    def __del__(self):
        # if connection is live, gets and stores
        # the positions before exiting
        grid_state = self.getGridState()
        if grid_state.driver_state == DS_CONNECTED:
            # fetch current positions
            super(GridDriver,self).pingFPUs(grid_state, record_positions=False)

        self.refresh_positions(grid_state)
        super(GridDriver, self).__del()
        
        
        
    def pre_config_motion_hook(self, wtable, gs, fpuset):
        # compute movement range for each FPU
        # add to current known min / max position
        # compare to allowed range
        # if not in range, throw exception, or print warning,
        # depending on protection state
        pass
    
    def post_config_motion_hook(self, wtable, gs, fpuset):
        pass
