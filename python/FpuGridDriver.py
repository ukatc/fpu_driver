
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
from interval import Interval

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

    def getSwitchStates(self, gs, fpuset=[]):
        ADRESS_SWITCH=0x0060
        
        if len(fpuset) == 0:
            fpuset = range(self.config.num_fpus)
            
        fv = self.minFirmwareVersion(gs, fpuset=fpuset)
        if fv < (1,3,2):
            raise  FPUDriverException("Not all addressed FPU's firmware implements reading the switch states")

        self.readRegister(ADRESS_SWITCH, gs, fpuset=fpuset)
        def getState(fpu):
            return {'alpha_limit_active' : ((fpu.register_value & 1) == 1 ),
                    'beta_datum_active' : (((fpu.register_value >> 1) & 1) == 1)  }
        return dict([ (fpu_id, getState(gs.FPU[fpu_id]) )for fpu_id in fpuset])
    
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

        if soft_protection:
            self.allow_find_datum_hook(gs, search_modes, selected_arm=selected_arm,
                                       fpuset=fpuset)
            
        self.start_find_datum_hook(gs, fpuset)
        try:
            rv =  self._gd.findDatum(gs, search_modes, fpuset, selected_arm, soft_protection)
        finally:
            was_cancelled = (rv != fpu_driver.E_DriverErrCode.DE_OK)
            if was_cancelled:
                try:
                    self.pingFPUs(gs)
                except:
                    print("ignoring error in pingFPUs()")
                    pass
            self.finished_find_datum_hook(gs, fpuset, was_cancelled=was_cancelled)
        
        return rv
    
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

        if soft_protection:
            self.allow_find_datum_hook(gs, search_modes, selected_arm=selected_arm,
                                       fpuset=fpuset)
        rv = self._gd.startFindDatum(gs, search_modes, selected_arm, fpuset, soft_protection)
        if rv != fpu_driver.E_DriverErrCode.DE_OK:
            raise RuntimeError("can't search Datum, driver error code = %r" % rv)

        self.start_find_datum_hook(gs,fpuset)
        time.sleep(0.1)
        time_interval = 0.1
        is_ready = False
        was_aborted = False
        finished_ok = False
        
        try:
            with SignalHandler() as sh:
                while not is_ready:
                    rv = self._gd.waitFindDatum(gs, time_interval, fpuset)
                    if sh.interrupted:
                        print("STOPPING FPUs.")
                        self.abortMotion(gs)
                        was_aborted = True
                        break
                    is_ready = (rv != fpu_driver.E_DriverErrCode.DE_WAIT_TIMEOUT)
                    finished_ok = (rv == fpu_driver.E_DriverErrCode.DE_OK)
        finally:
            if not finished_ok:
                self.pingFPUs(gs)

            self.finished_find_datum_hook(gs, fpuset, was_cancelled=(not finished_ok))
 
        if was_aborted:
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
        self.last_wavetable = {}
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

                        
    def pre_config_motion_hook(self, wtable, gs, fpuset, wmode=Range.Error):
        pass
        
        
    def post_config_motion_hook(self, wtable, gs, fpuset):
        pass

        
    def pre_repeat_motion_hook(self, wtable, gs, fpuset, wmode=Range.Error):
        pass
        
        
    def post_repeat_motion_hook(self, wtable, gs, fpuset):
        pass

    def pre_reverse_motion_hook(self, wtable, gs, fpuset, wmode=Range.Error):
        pass
        
        
    def post_reverse_motion_hook(self, wtable, gs, fpuset):
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
                    
        # check if wavetable is safe, and if so, register it    
        self.pre_config_motion_hook(wtable, gs, fpuset)
        rval = self._gd.configMotion(wtable, gs, fpuset, soft_protection)
        # remember wavetable
        self.last_wavetable.update(wtable)
        
        self.post_config_motion_hook(wtable, gs, fpuset)
        return rval

    def getCurrentWaveTables(self):
        return self.last_wavetable
    
    def executeMotionB(self, gs, fpuset=[]):
        self.started_execute_motion_hook(gs, fpuset=fpuset)
        rv = self._gd.executeMotion(gs, fpuset)
        self.pingFPUs(gs, fpuset)
        self.post_execute_motion_hook(gs, fpuset)
        return rv

    def executeMotion(self, gs, fpuset=[]):
        # wait a short moment to avoid spurious collision.
        time.sleep(2.5)
        rv = self._gd.startExecuteMotion(gs, fpuset)
        if rv != fpu_driver.E_DriverErrCode.DE_OK:
            print("rv=",rv)
            raise RuntimeError("FPUs not ready to move, driver error code = %r" % rv)
        self.started_execute_motion_hook(gs, fpuset=fpuset)
        time.sleep(0.1)
        time_interval = 0.1
        is_ready = False
        was_aborted = False
        refresh_state = False
        try:
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
            except CommandTimeout:
                refresh_state = True
                raise
            
        finally:
            # This is skipped in case of a SocketFailure, for example
            if (rv == fpu_driver.E_DriverErrCode.DE_OK) or was_aborted or refresh_state:
                # execute a ping to update positions
                # (this is only needed for protocol version 1)
                self.pingFPUs(gs, fpuset)
                # the following hook will narrow down the recorded intervals of positions
                self.post_execute_motion_hook(gs, fpuset)
                            
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
        wtable = self.last_wavetable.copy()
        if len(fpuset) > 0:
            kys = wtable.keys()
            for k in kys:
                if not k in fpuset:
                    del wtable[k]
                    
        self.pre_reverse_motion_hook(wtable, gs, fpuset)
        rv = self._gd.reverseMotion(gs, fpuset)
        self.post_reverse_motion_hook(wtable, gs, fpuset)
        return rv

    def repeatMotion(self, gs, fpuset=[]):
        wtable = self.last_wavetable.copy()
        if len(fpuset) > 0:
            kys = wtable.keys()
            for k in kys:
                if not k in fpuset:
                    del wtable[k]
                    
        self.pre_repeat_motion_hook(wtable, gs, fpuset)
        rv = self._gd.repeatMotion(gs, fpuset)
        self.post_repeat_motion_hook(wtable, gs, fpuset)
        return rv



DATABASE_FILE_NAME = os.environ.get("FPU_DATABASE")

env = lmdb.open(DATABASE_FILE_NAME, max_dbs=10)

# enumeration which defines the strictness of checks
class Range:
    Error = "Error - path rejected"
    Warn = "Warning - path unsafe"
    Ignore = "Ignore - path unchecked"

    
class GridDriver(UnprotectedGridDriver):
    def __init__(*args, **kwargs):
        super(GridDriver,self).__init__(*args, **kwargs)
        
        self.alpha_configured_range = {}
        self.beta_configured_range = {}

        def post_connect_hook(self):
        self.fpudb = env.open_db("fpu")
        
        grid_state = self.getGridState()
        self.readSerialNumbers(grid_state)
        apositions = {}
        bpositions = {}
        wtabs = {}
        wf_reversed = {}
        alimits = {}
        blimits = {}
        bretries = {}
        in_dicts = { 'apos' : apositions, 'bpos' : bpositions,
                     'wtab' : wtabs, 'wf_reversed' : wf_reversed,
                     'alimits' : alimits, 'blimits' : blimits, 'bretries' : bretries }
        
        print("reading serial numbers from DB....")
        print("here is a wart: the alpha zero point should be stored, or the step counts independent")
        a_offsets = []
        b_offsets = []
        for fpu_id, fpu in enumerate(grid_state.FPU):
            serial_number = fpu.serial_number
            a_offsets.append(Interval(0.0))
            b_offsets.append(Interval(0.0))
            
            with env.begin(db=self.fpudb) as txn:
                for subkey in ["apos", "bpos", "wtab", "alimits", "blimits", "wf_reversed", "bretries"]:
                    key = str((serial_number, subkey))
                    val = txn.get(key)
                          
                    print(key,":", val)
                    if val == None: 
                        raise fpu_driver.ProtectionError("serial number {0!r} not found in position database"
                                                         " - run fpu-admin.py to create entry".format(serial_number))
                    if subkey in ["wf_reversed", "bretries"]:
                        in_dicts[subkey][fpu_id] = literal_eval(val)
                    else:
                        # assign interval object
                        in_dicts[subkey][fpu_id] = Interval(val)
                        
        print("state data: ", in_dicts)
        self.apositions = apositions
        self.bpositions = bpositions
        self.wtabs = wtabs
        self.wf_reversed = wf_reversed
        self.alimits = alimits
        self.blimits = blimits
        self.a_offsets = a_offsets
        self.b_offsets = b_offsets
        self.wf_reversed = wf_reversed
        self.bretries = bretries
        

        # query positions and compute offsets, if FPUs have been resetted.
        # This assumes that the stored positions are correct.
        super(GridDriver,self).pingFPUs(grid_state, record_positions=False)
        self.reset_hook(grid_state)
        self.refreshPositions(grid_state, store=False)


    def alpha_angle(self, fpu):
        print("here is a wart: the alpha zero point should be stored, or the step counts independent")
        return fpu.alpha_steps / StepsPerDegreeAlpha + self.config.alpha_datum_offset

    def beta_angle(self, fpu):
        return fpu.beta_steps / StepsPerDegreeBeta

    def reset_hook(self, former_grid_state, new_state, fpuset=[]):
        """This is to be run after a reset or hardware power-on-reset.
        Updates the offset between the stored FPU positions and positions
        reported by ping. 

        If the FPUs have been switched off and powered on again, or if
        they have been resetted for another reason, the ping values
        will usually differ from the recorded valid values, until a
        successful datum command has been run.  This difference
        defines an offset which must be added every time in order to
        yield the correct position.

        This method needs to be run:
        - every time the driver is initialising, after the initial ping
        - after every resetFPU command

        """
        for fpu_id, fpu in enumerate(former_grid_state.FPU):
            if fpuset!= None and (not (fpu_id in fpuset)):
                continue

            if ( (new_state.FPU[fpu_id].state != FPST_UNINITIALISED)
                 or (not new_state.FPU[fpu_id].ping_ok)
                 or (new_state.FPU[fpu_id].alpha_steps != 0)
                 or (new_state.FPU[fpu_id].beta_steps != 0)):
                # fpu wasn't resetted successfully
                # We do not set the offset to zero - the FPU 
                # has been moved from a former reset
                continue
                        
                        
            a_offsets[fpu_id] += self.alpha_angle(fpu) - self.apositions[fpu_id]

            b_offsets[fpu_id] += self.beta_angle(fpu) - self.bpositions[fpu_id]
                        


                
    def refresh_positions(self, grid_state, store=True, fpuset=[]):
        """This is to be run after any movement.
        
        Computes new current positions from step count
        and offsets (if set), and stores them to database.
        """
        for fpu_id, fpu in enumerate(former_grid_state.FPU):
            if fpuset!= None and (not (fpu_id in fpuset)):
                continue

            if not fpu.ping_ok:
                # position is not known
                continue
            
            # compute alpha and beta position intervals,
            # and store both to DB
            self.apositions[fpu_id] = self.a_offsets[fpu_id] + self.alpha_angle(fpu)
            self.bpositions[fpu_id] = self.b_offset[fpu_id] + self.beta_angle(fpu)
            
            if store:
                serial_number = fpu.serial_number
                with env.begin(db=self.fpudb) as txn:
                    key = str( (serial_number, "apos"))
                    val = str(self.apositions[fpu_id])
                    txn.put(key, val)
                    key = str( (serial_number, "bpos"))
                    val = str(self.bpositions[fpu_id])
                    txn.put(key, val)

    def pingFPUs(self, grid_state, fpuset=[]):
        print("""Uses super call to UnprotectedGridDriver.pingFPUs(). Make sure this
        hasn't any weird results.""")
        
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
        super(GridDriver, self).__del__()
        
    def check_allowed_range(self, fpu_id, stepnum, arm_name,
                            xlimits, x, 
                            new_range, wmode):
        """Checks whether waveform step is in allowed interval for this FPU.
        Parameters:
        xlimits - list with the manually configured limits of the
                 individual FPU, in degrees.

        new_range - current minimum / maximum interval of real position, in degrees
        x         - new value of variable
        new_range - new minimum / maximum range if movement is executed
        
        """
        if wmode == Range.Ignore:
            return
        

        # if the movement extends the range of the FPU location,
        # this is updated in the movement range
        new_range.combine(x)
        
        if not xlimits.contain(x):
            if wmode == Range.Error:
                throw ProtectionException("For FPU %i, at step %i, arm %s, the wavetable steps outside the tracked safe limits" %(
                    fpu_id, stepnum, arm_name))
                
            elif wmode == Range.Warn:
                print("Warning: wavetable defines unsafe path")

    """The complexity of the wave table data flow which follows merits a bit of 
    explanation.
    Generally, the protection wrapper tries to track the position of each 
    FPU as it is moved. When a ping returns, the new position can
    generally be updated.
    
    However, when a movement is started, there is no way to know the
    exact position before the movement finished regularly, or the next
    successful ping returns. When an abortMotion message is sent, or a
    collision occurs, the position is not known.
    
    The solution I try here is to track not the position, but the
    worst-case minimum and maximum position. In other words, for each
    of the alpha and beta coordinate, an /interval/ of positions is
    tracked.  When a deterministic movement is added to the current
    interval, the minimum and maximum extend of that movement become
    added to the current minimum and maximum. In other words,
    movements expand the region of unterctainty, and regular
    terminations, pings and datum responses collaps it to an interval
    of length zero.

    If a configMotion command is issued, this defines a tentative
    requested interval, which becomes valid once the corresponding
    movement is started.  If a reverseMotion is issued, this generates
    an interval with the opposite sign of movements.

    Tracking the region of uncertainty is not always required, as ping
    commands can be used to narrow the possible range of possitions
    down. But doing this has, first, the disadvantage
    thatspecial-casing is needed for handling an abortMotion or
    collision situation. Yet these are situations which need to be
    handled robustly. And second, tracking the known state of
    positions during start-up also requires special-casing, which
    appears to require a lot of fiddly state handling. This could be
    error-prone, and the solution to track intervals is much more
    general and clean.

    """
        
    def check_and_register_wtable(self, wtable, gs, fpuset, wmode=Range.Error, sign=1):
        # compute movement range for each FPU
        # add to current known min / max position
        # compare to allowed range
        # if not in range, throw exception, or print warning,
        # depending on protection setting
        alpha_configured_range = {}
        beta_configured_range = {}
        for fpu_id, wt_row in wtable.items():
            fpu = gs.FPU[fpu_id]
 
            alimits= self.alimits[fpu_id]
            blimits= self.blimits[fpu_id]

            alpha0 = self.apositions[fpu_id]
            beta0 = self.bpositions[fpu_id]

            wf_arange = Interval(alpha0)
            wf_brange = Interval(beta0)
                        
            self.check_allowed_range(fpu_id, -1, "alpha", alimits, alpha0,
                                     wf_arange,  wmode)

            self.check_allowed_range(fpu_id, -1, "beta",  blimits, beta0, 
                                     wf_brange, wmode)
            
            for step_num, entry in enumerate(wt_row):
                a, b = entry
                asteps += a *sign
                bsteps += b * sign
                alpha_sect = alpha0 + asteps / StepsPerDegreeAlpha
                beta_sect = beta0 + bsteps / StepsPerDegreeBeta
                
                self.check_allowed_range(fpu_id, step_num, "alpha",
                                         alimits, alpha_sect,
                                         wf_arange,  wmode)
                self.check_allowed_range(fpu_id, step_num, "beta",
                                         blimits, beta_sect,
                                         wf_brange, wmode)
                
            alpha_configured_range[fpu_id] = wf_arange
            beta_configured_range[fpu_id] = wf_brange

        # this is the list of alpha/beta position intervals that
        # will become valid if and after an executeMotion is started,
        # and before the new positions can be retrieved via ping
        self.alpha_configured_range.update(alpha_configured_range)
        self.beta_configured_range.update(beta_configured_range)

    def pre_config_motion_hook(self, wtable, gs, fpuset, wmode=Range.Error):
        self.check_and_register_wtable(wtable, gs, fpuset, wmode, sign=1)

    # this can possibly be deleted
    # but do we want to store the full wavetable ?
    def save_wtable_direction(self, fpu_id_list, is_reversed=False):
        for fpu_id in fpu_id_list:
            serial_number = fpu.serial_number
            self.wv_reversed[fpu_id] = is_reversed
            with env.begin(db=self.fpudb) as txn:
                key = str( (serial_number, "wf_refersed"))
                val = str(is_reversed)
                txn.put(key, val)
        
        
    def post_config_motion_hook(self, wtable, gs, fpuset):
        # updateranges that become valid once executeMotion is started
        self.configured_arange.update(self.alpha_configured_range)
        self.configured_brange.update(self.beta_configured_range)
        self.save_wtable_direction(wtable.keys(), is_reversed=False)

        
    def pre_repeat_motion_hook(self, wtable, gs, fpuset, wmode=Range.Error):
        self.check_and_register_wtable(wtable, gs, fpuset, wmode, sign=1)
        
        
    def post_repeat_motion_hook(self, wtable, gs, fpuset):
        # updateranges that become valid once executeMotion is started
        self.configured_arange.update(self.alpha_configured_range)
        self.configured_brange.update(self.beta_configured_range)
        self.save_wtable_direction(wtable.keys(), is_reversed=False)

    def pre_reverse_motion_hook(self, wtable, gs, fpuset, wmode=Range.Error):
        self.check_and_register_wtable(wtable, gs, fpuset, wmode, sign=-1)
        
        
    def post_reverse_motion_hook(self, wtable, gs, fpuset):
        # updateranges that become valid once executeMotion is started
        self.configured_arange.update(self.alpha_configured_range)
        self.configured_brange.update(self.beta_configured_range)
        self.save_wtable_direction(wtable.keys(), is_reversed=True)
        

    def started_execute_motion_hook(self, gs, fpuset):
        """This runs after an executeMotion command has successfully
        started. At that point, the FPU in fpuset should be moving
        within the ranges set by the last config_motion, repeat_motion
        or reverse_motion command. These ranges are set as the 
        intervals which define the possible positions
        until the command has finished.

        """
        for fpu_id, fpu in enumerate(gs.FPU):
            if fpuset!= None and (not (fpu_id in fpuset)):
                continue
            
            # copy configured alpha and beta position intervals, and
            # store both to DB
            if self.configured_arange.has_key(fpu_id):
                self.apositions[fpu_id] = self.configured_arange[fpu_id]
            if self.configured_brange.has_key(fpu_id):
                self.bpositions[fpu_id] = self.configured_brange[fpu_id]
            
            serial_number = fpu.serial_number
            with env.begin(db=self.fpudb) as txn:
                key = str( (serial_number, "apos"))
                val = str(self.apositions[fpu_id])
                txn.put(key, val)
                key = str( (serial_number, "bpos"))
                val = str(self.bpositions[fpu_id])
                txn.put(key, val)

        
    
    def post_execute_motion_hook(self, gs, fpuset):
        """This runs after both an executeMotion has run, and *also*
        a ping has returned successfully."""

        # What do we here if FPUs are still moving?
        # Solution for now: wait.
        while True:
            if (gs.Counts[FPST_MOVING] > 0)or (gs.Counts[FPST_DATUM_SEARCH] > 0):
                print("waiting for movement to finish in order to retrieve final positions..")
                sleep(0.5)
                super(GridDriver, self).pingFPUs(gs, fpuset=fpuset)
            else:
                break
        # The assumption here is that the offsets did not change,
        # even if the waveform was aborted. This might not be correct
        # in a sense valid for science measurements, because collisions
        # would degrade precision of step counts, but for protection purposes
        # this should be OK.
        #
        # Thus, use the step counter positions to update the location
        self.refresh_positions(gs, fpuset=fpuset)
        # reset wavetable spans for the addressed FPUs - they are not longer valid
        for k in self.configured_arange.keys():
            if k in fpuset:
                del self.configured_arange[k]
                
        for k in self.configured_brange.keys():
            if k in fpuset:
                del self.configured_brange[k]

    def allow_find_datum_hook(self,gs, search_modes, selected_arm=None, fpuset=None):
        """This function checks whether a datum search is safe, and 
        throws an exception otherwise. It does that based on the stored position."""
        # for example: 
        # - alpha in positive range
        # - beta initialised, or in positive range
        # if manual search, check the direction 
        pass

    def finished_find_datum_hook(self, gs, search_modes, fpuset=None, was_cancelled=False):
        print("driver: always clear ping_ok when movement commands are started")
        for fpu_id, fpu in enumerate(gs.FPU):
            if fpuset!= None and (not (fpu_id in fpuset)):
                continue

            if (len(search_modes) != 0) and (not search_modes.kas_key(fpu_id)):
                continue
            
                                                                                
            serial_number = fpu.serial_number
            # set position intervals to zero, and store in DB
            with env.begin(db=self.fpudb) as txn:
                if fpu.alpha_was_zeroed:
                    self.a_offsets = Interval(0)
                    if fpu.alpha_steps == 0:                        
                        self.apositions[fpu_id] = Interval(0.0)
                    key = str( (serial_number, "apos"))
                    val = str(self.apositions[fpu_id])
                    txn.put(key, val)
                else:
                    ##TODO: check if step_count is valid (pingok?),
                    ## and store result
                    ## if ping_ok is set, we assume that the step counter
                    ## is valid, and the offset is unchanged
                    if fpu.ping_ok:
                        self.apositions[fpu_id] = Interval(self.alpha_angle(fpu)) + self.a_offsets[fpu_id]
                        key = str( (serial_number, "apos"))
                        val = str(self.apositions[fpu_id])
                        txn.put(key, val)

                
                if fpu.beta_was_zeroed:
                    self.b_offsets = Interval(0)
                    if fpu.beta_steps == 0:                        
                        self.bpositions[fpu_id] = Interval(0.0)
                    key = str( (serial_number, "bpos"))
                    val = str(self.bpositions[fpu_id])
                    txn.put(key, val)
                else:
                    ##TODO: check for valid step count, store state
                    if fpu.ping_ok:
                        self.bpositions[fpu_id] = Interval(self.beta_angle(fpu)) + self.b_offsets[fpu_id]
                        key = str( (serial_number, "bpos"))
                        val = str(self.bpositions[fpu_id])
                        txn.put(key, val)
        

    def start_find_datum_hook(self, gs, search_modes,  selected_arm=None,fpuset):
        """This is run when na findDatum command is actually started.
        It updates the new range of possible positions to include the zero point of each arm."""
        for fpu_id, fpu in enumerate(gs.FPU):
            if fpuset!= None and (not (fpu_id in fpuset)):
                continue

            if (len(search_modes) != 0) and (not search_modes.kas_key(fpu_id)):
                continue
            
                                                                                
            serial_number = fpu.serial_number
            # update stored intervals to include zero, and store in DB
            with env.begin(db=self.fpudb) as txn:
                if selected_arm in [DASEL_ALPHA, DASEL_BOTH]:
                    self.apositions[fpu_id].extend(0.0)
                    key = str( (serial_number, "apos"))
                    val = str(self.apositions[fpu_id])
                    txn.put(key, val)
                if selected_arm in [DASEL_BETA, DASEL_BOTH]:
                    self.bpositions[fpu_id].extend(0.0)
                    key = str( (serial_number, "bpos"))
                    val = str(self.bpositions[fpu_id])
                    txn.put(key, val)

                

