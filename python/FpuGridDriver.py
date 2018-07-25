#!/usr/bin/python

from __future__ import print_function, division
import pdb
import os
from os import path
import errno
import time
import signal
from warnings import warn, filterwarnings
# state tracking
import lmdb
from interval import Interval, Inf, nan

from fpu_constants import *
from protectiondb import ProtectionDB

import fpu_driver
import fpu_commands

from fpu_driver import (__version__, CAN_PROTOCOL_VERSION, GatewayAddress,  
                        REQD_ANTI_CLOCKWISE,  REQD_CLOCKWISE, 
                        FPUDriverException, MovementError, CollisionError, LimitBreachError,
                        AbortMotionError, StepTimingError, InvalidStateException, SystemFailure,
                        InvalidParameterError, SetupError, InvalidWaveformException, ConnectionFailure,
                        SocketFailure, CommandTimeout, ProtectionError,                        
                        DASEL_BOTH, DASEL_ALPHA, DASEL_BETA, 
                        LOG_ERROR, LOG_INFO, LOG_GRIDSTATE, LOG_DEBUG, LOG_VERBOSE, LOG_TRACE_CAN_MESSAGES, 
                        SEARCH_CLOCKWISE, SEARCH_ANTI_CLOCKWISE, SEARCH_AUTO, SKIP_FPU, FPST_UNINITIALIZED,
                        FPST_LOCKED, FPST_DATUM_SEARCH, FPST_AT_DATUM, FPST_LOADING, FPST_READY_FORWARD,
                        FPST_READY_REVERSE, FPST_MOVING, FPST_RESTING, FPST_ABORTED, FPST_OBSTACLE_ERROR, DS_CONNECTED )


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

# enumeration which defines the strictness of checks
class Range:
    Error = "Error - path rejected"
    Warn = "Warning - path unsafe"
    Ignore = "Ignore - path unchecked"


filterwarnings("default", "keyword check_protection", DeprecationWarning)

class UnprotectedGridDriver (object):
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
        self.wf_reversed = {}

        self.wavetables_incomplete = False

        self._gd = fpu_driver.GridDriver(config)

    def __del__(self):
        # the following delete is necessary to run destructors!!
        del self._gd
        os.close(self.config.fd_controllog)
        os.close(self.config.fd_txlog)
        os.close(self.config.fd_rxlog)

    def _post_connect_hook(self, config):
        pass

    def connect(self, address_list=DEFAULT_GATEWAY_ADRESS_LIST):
        rv = self._gd.connect(address_list)
        self._post_connect_hook(self.config)
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


    def _start_find_datum_hook(self, gs, search_modes,  selected_arm=None,
                               fpuset=[], initial_positions={}):
        pass

    def _cancel_find_datum_hook(self, gs, search_modes,  selected_arm=None,
                               fpuset=[], initial_positions={}):
        pass

    def _finished_find_datum_hook(self, gs, search_modes,  selected_arm=None,
                               fpuset=[], initial_positions={}):
        pass
    
    def findDatumB(self, gs, search_modes={}, selected_arm=DASEL_BOTH, soft_protection=True,
                   check_protection=None, fpuset=[], support_uninitialized_auto=True):
        """Moves all FPUs to datum position. 

        This is a blocking variant of the findDatum command,
        it is not interruptible by Control-C."""
        if check_protection != None:
            warn("keyword check_protection is deprecated; use 'soft_protection=False' instead!",
                 DeprecationWarning, 2)
            soft_protection=check_protection

        count_protection = soft_protection
        if soft_protection:
            search_modes = search_modes.copy()
            # check whether datum search is safe, adjusting search_modes
            self._allowfind_datum_hook(gs, search_modes, selected_arm=selected_arm,
                                       fpuset=fpuset, support_uninitialized_auto=support_uninitialized_auto)
            if (SEARCH_CLOCKWISE in search_modes) or (SEARCH_ANTI_CLOCKWISE in search_modes):
                count_protection = False

        initial_positions = {}
        self._start_find_datum_hook(gs, fpuset, initial_positions=initial_positions)
        try:
            try:
                rv =  self._gd.findDatum(gs, search_modes, fpuset, selected_arm, count_protection)
            except (RuntimeError,
                    InvalidParameterException,
                    SetupErrorException,
                    InvalidStateException,
                    ProtectionErrorException) as e:
                # we cancel the datum search altogether, so we can reset
                # positions to old value
                self._cancel_find_datum_hook(gs, fpuset, initial_positions=initial_positions)
                was_cancelled = True
        finally:
            if was_cancelled or (rv != fpu_driver.E_DriverErrCode.DE_OK):
                self.pingFPUs(gs, fpuset=fpuset)
                
            self._finished_find_datum_hook(gs, fpuset, was_cancelled=was_cancelled,
                                          initial_positions=initial_positions)
        
        return rv
    
    def findDatum(self, gs, search_modes={}, selected_arm=DASEL_BOTH, fpuset=[],
                  soft_protection=True, count_protection=True, check_protection=None,
                  support_uninitialized_auto=True):
        """Moves all FPUs to datum position. 

        If the program receives a SIGNINT, or Control-C is pressed, an
        abortMotion command is automatically sended, aborting the search.

        The parameter selected_arm (DASEL_BOTH, DASEL_ALPHA, DASEL_BETA) 
        controls which arms are moved.

        The dictionary fpu_modes has integer FPU IDs as keys, and each
        value is one of SEARCH_CLOCKWISE, SEARCH_ANTI_CLOCKWISE,
        SEARCH_AUTO, SKIP_FPU, which controls whether the datum search
        moves clockwise (decreasing step count), anti-clockwise
        (increasing step count), automatically, or skips the FPU.  The
        default mode is SEARCH_AUTO.

        It is critical that the search direction for the beta arm is
        always set correctly, otherwise a beta arm collision will
        happen which could degrade the FPU (or even destroy the FPU,
        if the collision detection does not work).  If a beta arm was
        datumed, the FPU will move in the direction corresponding to
        its internal step count.  If an beta arm is not datumed,
        automatic datum search will use the position database value,
        unless support_uninitialized_auto is set to False.  In
        addition, manually set search directions will be checked using
        the step count value for the beta arm, unless count_protection
        is set to False.
        
        If a beta arm position appears not to be safe to be moved into
        the requested position, the manual datum search will be refused
        unless soft_protection is set to False.

        """

        if check_protection != None:
            warn("keyword check_protection is deprecated; use 'soft_protection=False' instead!",
                 DeprecationWarning, 2)
            soft_protection=check_protection

        if soft_protection:
            orig_search_modes = search_modes
            search_modes = orig_search_modes.copy()
            # check whether datum search is safe, adjusting search_modes
            self._allowfind_datum_hook(gs, search_modes, selected_arm=selected_arm,
                                       fpuset=fpuset, support_uninitialized_auto=support_uninitialized_auto)
            # We might need to disable the stepcount-based check if
            # (and only if) the step counter does not agree with the
            # database Check whether a movement against the step count
            # is needed.
            for k, m in search_modes.items():
                fpu = gs.FPU[k]
                if ( (m in [SEARCH_CLOCKWISE, SEARCH_ANTI_CLOCKWISE ])
                     and (orig_search_modes.get(k, SEARCH_AUTO)  == SEARCH_AUTO)):
                    print("beta arm %i not initialized: overriding count_protection flag" % k)
                    count_protection = False
                
        initial_positions = {}
        self._start_find_datum_hook(gs,fpuset, initial_positions=initial_positions)
        try:
            rv = self._gd.startFindDatum(gs, search_modes, selected_arm, fpuset, count_protection)
            if rv != fpu_driver.E_DriverErrCode.DE_OK:
                raise RuntimeError("can't search Datum, driver error code = %r" % rv)
            
        except (RuntimeError,
                InvalidParameterError,
                SetupError,
                InvalidStateException,
                ProtectionError) as e:
            # we cancel the datum search altogether, so we can reset
            # positions to old value
            self._cancel_find_datum_hook(gs, fpuset,initial_positions=initial_positions)
            raise
            

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
                self.pingFPUs(gs, fpuset=fpuset)

            self._finished_find_datum_hook(gs, fpuset, was_cancelled=(not finished_ok),
                                          initial_positions=initial_positions)
 
        if was_aborted:
            print("findDatumw as aborted by SIGINT, movement stopped")
            raise MovementError("findDatum was aborted by SIGINT")
        
        
        return rv

    def pingFPUs(self, gs, fpuset=[]):
        return self._gd.pingFPUs(gs, fpuset)

    def _reset_hook(self, old_state, gs, fpuset=[]):
        pass
    
    def resetFPUs(self, gs, fpuset=[]):
        old_state = self.getGridState()
        rval = self._gd.resetFPUs(gs, fpuset)
        self.last_wavetable = {}
        self._reset_hook(old_state, gs, fpuset=fpuset)
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
                print("FPU %i : SN = %r" % (i, gs.FPU[i].serial_number))
        
    
    def writeSerialNumber(self, fpu_id, serial_number,  gs):
        return self._gd.writeSerialNumber(fpu_id, serial_number, gs)

                        
    def _pre_config_motion_hook(self, wtable, gs, fpuset, wmode=Range.Error):
        pass
        
        
    def _post_config_motion_hook(self, wtable, gs, fpuset):
        self.set_wtable_reversed(fpuset, False)

        
    def _pre_repeat_motion_hook(self, wtable, gs, fpuset, wmode=Range.Error):
        pass        
        
    def _post_repeat_motion_hook(self, wtable, gs, fpuset):
        self.set_wtable_reversed(fpuset, False)

    def _pre_reverse_motion_hook(self, wtable, gs, fpuset, wmode=Range.Error):
        pass
        
        
    def _post_reverse_motion_hook(self, wtable, gs, fpuset):
        self.set_wtable_reversed(fpuset, True)

    
    def configMotion(self, wavetable, gs, fpuset=[], soft_protection=True, check_protection=None,  allow_uninitialized=False):
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

        # we make a copy of the wavetable to make sure
        # no side effects are visible to the caller
        wtable = wavetable.copy()
        if len(fpuset) > 0:
            kys = wtable.keys()
            for k in kys:
                if k not in fpuset:
                    del wtable[k]
                    
        # check whether wavetable is safe, and if so, register it
        if soft_protection:
            wmode=Range.Error
        else:
            wmode=Range.Warn
        self._pre_config_motion_hook(wtable, gs, fpuset, wmode=wmode)
        self.wavetables_incomplete = False
        update_config = False
        try:
            try:
                rval = self._gd.configMotion(wtable, gs, fpuset, soft_protection, allow_uninitialized)
                update_config = True
                
            except (SocketFailure, CommandTimeout) as e:
                # Transmission failure. Here, it is possible that some
                # FPUs have finished loading valid data, but the
                # process was not finished for all FPUs.
                self.wavetables_incomplete=True
                update_config = True
                for fpu_id, fpu in enumerate(gs.FPU):
                    # We accept entries which appear to have been
                    # configured successfuly. This is not 100%
                    # failure-proof - an FPU could hold onto a
                    # wavetable which has been configured earlier, and
                    # which happens to have the same length.
                    if wtable.has_key(fpu_id):
                        if (fpu.state==FPST_READY_FORWARD) and (
                                fpu.fpu.num_waveform_segments == len(wtable[fpu_id])):
                            self.last_wavetable[fpu_id] = wtable[fpu_id]
                raise
            # remember wavetable
            self.last_wavetable.update(wtable)
            
        finally:
            if update_config:
                self._post_config_motion_hook(wtable, gs, fpuset,
                                             partial_config=self.wavetables_incomplete)
            
        return rval

    def getCurrentWaveTables(self):
        if self.wavetables_incomplete:
            print("Warning: waveform upload failed or incomplete, "
                  "waveforms displayed may not be valid.")
        return self.last_wavetable.copy()
    
    def getReversed(self):
        return self.wf_reversed.copy()
    
    def set_wtable_reversed(self, fpu_id_list, is_reversed=False):
        if len(fpu_id_list) == 0:
            fpu_id_list = range(self.config.num_fpus)
        for fpu_id in fpu_id_list:
            self.wf_reversed[fpu_id] = is_reversed
                
    def _start_execute_motion_hook(self, gs, fpuset, initial_positions={}):
        pass
    
    def _cancel_execute_motion_hook(self, gs, fpuset,initial_positions=None):
        pass
    
    def _post_execute_motion_hook(self, gs, fpuset):
        pass
    

    def executeMotionB(self, gs, fpuset=[]):
        initial_positions = {}
        self._start_execute_motion_hook(gs, fpuset=fpuset, initial_positions=initial_positions)
        try:
            try:
                rv = self._gd.executeMotion(gs, fpuset)
            except InvalidState as e:
                self_cancel_execute_motion_hook(gs, fpuset, initial_positions=initial_positions)
                raise
        finally:
            self.pingFPUs(gs, fpuset)
            self._post_execute_motion_hook(gs, fpuset)
            
        return rv

    def executeMotion(self, gs, fpuset=[]):
        # wait a short moment to avoid spurious collision.
        time.sleep(2.5)
        initial_positions={}
        self._start_execute_motion_hook(gs, fpuset=fpuset, initial_positions=initial_positions)
        time.sleep(0.1)
        try:
            rv = self._gd.startExecuteMotion(gs, fpuset)
        except InvalidStateException as e:
            self_cancel_execute_motion_hook(gs, fpuset, initial_positions=initial_positions)
            raise
        if rv != fpu_driver.E_DriverErrCode.DE_OK:
            raise RuntimeError("FPUs not ready to move, driver error code = %r" % rv)
        
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
                self._post_execute_motion_hook(gs, fpuset)
                            
        if was_aborted:
            raise MovementError("executeMotion was aborted by SIGINT")
        
        return rv

    def abortMotion(self, gs, fpuset=[]):
        return self._gd.abortMotion(gs, fpuset)

    def _pre_free_beta_collision_hook(self, fpu_id,direction, gs):
        pass

    def _post_free_beta_collision_hook(self, fpu_id,direction, gs):
        pass
    
    def freeBetaCollision(self, fpu_id, direction,  gs, soft_protection=True):

        if soft_protection:
            self._pre_free_beta_collision_hook(fpu_id,direction, gs)
            
        rv = self._gd.freeBetaCollision(fpu_id, direction, gs)
        
        self._post_free_beta_collision_hook(fpu_id, direction, gs)
        return rv
    
    def enableBetaCollisionProtection(self, gs):
        return self._gd.enableBetaCollisionProtection(gs)

    def reverseMotion(self, gs, fpuset=[], soft_protection=True):
        wtable = self.last_wavetable.copy()
        if len(fpuset) > 0:
            kys = wtable.keys()
            for k in kys:
                if not k in fpuset:
                    del wtable[k]
                    
        if soft_protection:
            wmode=Range.Error
        else:
            wmode=Range.Warn
                    
        self._pre_reverse_motion_hook(wtable, gs, fpuset, wmode=wmode)
        rv = self._gd.reverseMotion(gs, fpuset)
        self._post_reverse_motion_hook(wtable, gs, fpuset)
        return rv

    def repeatMotion(self, gs, fpuset=[], soft_protection=True):
        wtable = self.last_wavetable.copy()
        if len(fpuset) > 0:
            kys = wtable.keys()
            for k in kys:
                if not k in fpuset:
                    del wtable[k]
                    
        if soft_protection:
            wmode=Range.Error
        else:
            wmode=Range.Warn
            
        self._pre_repeat_motion_hook(wtable, gs, fpuset, wmode=wmode)
        rv = self._gd.repeatMotion(gs, fpuset)
        self._post_repeat_motion_hook(wtable, gs, fpuset)
        return rv

    def countedAngles(self, gs, fpuset=[], show_uninitialized=False):
        if len(fpuset) == 0:
            fpuset = range(self.config.num_fpus)
            
        self.pingFPUs(gs, fpuset=fpuset)
        
        angles = fpu_commands.list_angles(gs, show_uninitialized=show_uninitialized,
                                          alpha_datum_offset=self.config.alpha_datum_offset)
        return [angles[k] for k in fpuset ]


DATABASE_FILE_NAME = os.environ.get("FPU_DATABASE")

env = lmdb.open(DATABASE_FILE_NAME, max_dbs=10)

def get_duplicates(idlist):
    duplicates = {} 
    for n, id in enumerate(idlist):
        if idlist.count(id) > 1:
            duplicates[n] = id
    return duplicates
    
class GridDriver(UnprotectedGridDriver):
    def __init__(self, *args, **kwargs):
        super(GridDriver,self).__init__(*args, **kwargs)

        # position intervals which are being configured by configMotion
        self.alpha_configuring_range = {}
        self.beta_configuring_range = {}
        # position intervals which have successfully been configured
        # and will become valid with next executeMotion
        self.configured_arange = {}
        self.configured_brange = {}

    def _post_connect_hook(self, config):
        self.fpudb = env.open_db(ProtectionDB.dbname)
        
        grid_state = self.getGridState()
        self.readSerialNumbers(grid_state)
        # check for uniqueness
        snumbers = [fpu.serial_number for fpu in grid_state.FPU]
        if len(snumbers) > len(set(snumbers)):
            duplicates = get_duplicates(snumbers)
            raise ProtectionError("FPU serial numbers are not unique: " + repr(duplicates))
        
        apositions = {}
        bpositions = {}
        wtabs = {}
        wf_reversed = {}
        alimits = {}
        blimits = {}
        maxbretries = {}
        bretries_cw = {}
        bretries_acw = {}
        in_dicts = { ProtectionDB.alpha_positions : apositions,
                     ProtectionDB.beta_positions : bpositions,
                     ProtectionDB.waveform_table : wtabs, ProtectionDB.waveform_reversed : wf_reversed,
                     ProtectionDB.alpha_limits : alimits, ProtectionDB.beta_limits : blimits,
                     ProtectionDB.free_beta_retries : maxbretries,
                     ProtectionDB.beta_retry_count_cw : bretries_cw,
                     ProtectionDB.beta_retry_count_acw : bretries_acw }
        
        a_caloffsets = []
        b_caloffsets = []
        
        alpha_datum_offset=config.alpha_datum_offset
        
        for fpu_id, fpu in enumerate(grid_state.FPU):
            a_caloffsets.append(Interval(0.0))
            b_caloffsets.append(Interval(0.0))
            
            with env.begin(db=self.fpudb) as txn:
                for subkey in [ ProtectionDB.alpha_positions,
                                ProtectionDB.beta_positions,
                                ProtectionDB.waveform_table,
                                ProtectionDB.waveform_reversed,
                                ProtectionDB.alpha_limits,
                                ProtectionDB.beta_limits,
                                ProtectionDB.free_beta_retries,
                                ProtectionDB.beta_retry_count_cw,
                                ProtectionDB.beta_retry_count_acw]:
                    
                    val = ProtectionDB.getField(txn, fpu, subkey)
                          
                    if val == None: 
                        raise fpu_driver.ProtectionError("serial number {0!r} not found in position database"
                                                         " - run fpu-admin.py to create entry".format(fpu.serial_number))
                    else:
                        if subkey in [ProtectionDB.alpha_positions, ProtectionDB.alpha_limits]:
                            in_dicts[subkey][fpu_id] = val + alpha_datum_offset
                        else:
                            in_dicts[subkey][fpu_id] = val
                        
        self.apositions = apositions
        self.bpositions = bpositions
        self.last_wavetable = wtabs
        self.wf_reversed = wf_reversed
        self.alimits = alimits
        self.blimits = blimits
        self.a_caloffsets = a_caloffsets
        self.b_caloffsets = b_caloffsets
        self.maxbretries = maxbretries
        self.bretries_cw = bretries_cw
        self.bretries_acw = bretries_acw
        

        # query positions and compute offsets, if FPUs have been resetted.
        # This assumes that the stored positions are correct.
        super(GridDriver,self).pingFPUs(grid_state)
        self._reset_hook(grid_state, grid_state)
        self._refresh_positions(grid_state, store=False)


    def _alpha_angle(self, fpu):
        return fpu.alpha_steps / StepsPerDegreeAlpha + self.config.alpha_datum_offset

    def _beta_angle(self, fpu):
        return fpu.beta_steps / StepsPerDegreeBeta

    def _reset_hook(self, old_state, new_state, fpuset=[]):
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
        self.readSerialNumbers(new_state, fpuset=fpuset)
        for fpu_id, fpu in enumerate(new_state.FPU):
            if (len(fpuset) > 0) and (fpu_id not in fpuset):
                continue
                        
            # these offsets are the difference between the calibrated
            # angle and the uncalibrated angle - after a findDatum,
            # they are set to zero
            self.a_caloffsets[fpu_id] = self.apositions[fpu_id] - self._alpha_angle(fpu)

            self.b_caloffsets[fpu_id] = self.bpositions[fpu_id] - self._beta_angle(fpu)
                        
    def trackedAngles(self, gs, fpuset=[]):
        """lists tracked angles, offset, and waveform span
        for configured waveforms, for each FPU"""
        
        if len(fpuset) == 0:
            fpuset = range(self.config.num_fpus)
            
        warange_dict = self.alpha_configuring_range
        wbrange_dict = self.beta_configuring_range
        for fi in fpuset:
            fpu = gs.FPU[fi]
            aangle = self._alpha_angle(fpu)
            bangle = self._beta_angle(fpu)
            wf_arange = warange_dict.get(fi,Interval())
            wf_brange = wbrange_dict.get(fi,Interval())
            
            print("FPU #{}: angle = ({!s}, {!s}), offsets = ({!s}, {!s}),"
                  " stepcount angle= ({!s}, {!s}), last_wform_range=({!s},{!s})".
                  format(fi, self.apositions[fi],self.bpositions[fi],
                         self.a_caloffsets[fi], self.b_caloffsets[fi],
                         aangle, bangle,
                         wf_arange, wf_brange))

    def _update_apos(self, txn, fpu, fpu_id, new_apos, store=True):
        self.apositions[fpu_id] = new_apos
        if store:
            ProtectionDB.put_alpha_position(txn, fpu, new_apos, self.config.alpha_datum_offset)
        
    def _update_bpos(self, txn, fpu, fpu_id, new_bpos, store=True):
        self.bpositions[fpu_id] = new_bpos
        if store:
            ProtectionDB.put_beta_position(txn, fpu, new_bpos)
        
        
                
    def _refresh_positions(self, grid_state, store=True, fpuset=[]):
        """This is to be run after any movement.
        
        Computes new current positions from step count
        and offsets (if set), and stores them to database.

        Note: We do not try to recognize a reset behind the back of
        the driver as there is no reliable indicator. Protocol 1 does
        not support to recognize that. Protocol 2 allows to recognize
        it, so this might change.

        """
        
        with env.begin(db=self.fpudb, write=True) as txn:
            for fpu_id, fpu in enumerate(grid_state.FPU):
                if len(fpuset) > 0 and (fpu_id not in fpuset):
                    continue

                if not fpu.ping_ok:
                    # position is not known. This flag is set by a
                    # successful ping or datum response, and cleared by
                    # every movement as well as all movement time-outs
                    continue

                new_alpha = self.a_caloffsets[fpu_id] + self._alpha_angle(fpu)
                new_beta = self.b_caloffsets[fpu_id] + self._beta_angle(fpu)
                if ((not self.apositions[fpu_id].contains(new_alpha, tolerance=0.05))
                    or (not self.bpositions[fpu_id].contains(new_beta, tolerance=0.05))) :
                    
                    print("""FATAL ERROR: RECEIVED FPU POSITION OUTSIDE OF TRACKED RANGE. 
FPU was likely moved or power-cycled circumventing the running driver. 
Emergency exit: Position database needs to be re-initialized.""")
                    os.abort()
            
                # compute alpha and beta position intervals,
                # and store both to DB
                self._update_apos(txn, fpu, fpu_id, new_alpha, store=store)
                self._update_bpos(txn, fpu, fpu_id, new_beta, store=store)
                
        env.sync()
 
                    

    def pingFPUs(self, grid_state, fpuset=[]):
        super(GridDriver, self).pingFPUs(grid_state, fpuset=fpuset)
        self._refresh_positions(grid_state, fpuset=fpuset)

            
    def __del__(self):
        # if connection is live, gets and stores
        # the positions before exiting
        grid_state = self.getGridState()
        if grid_state.driver_state == DS_CONNECTED:
            # fetch current positions
            super(GridDriver,self).pingFPUs(grid_state)

        self._refresh_positions(grid_state)
        super(GridDriver, self).__del__()
        
    def _check_allowed_range(self, fpu_id, stepnum, arm_name,
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
        
        if not xlimits.contains(x):
            if wmode == Range.Error:
                raise ProtectionError("For FPU %i, at step %i, arm %s, the wavetable"
                                      " steps outside the tracked safe limits (angle=%r, limits=%r)" %(
                    fpu_id, stepnum, arm_name, x, xlimits))
                
            elif wmode == Range.Warn:
                print("Warning: wavetable defines unsafe path for FPU %i, at step %i, arm %s  (angle=%r, limits=%r)" %(
                    fpu_id, stepnum, arm_name, x, xlimits))

    """The complexity of the wave table data flow which follows merits a bit of 
    explanation.
    Generally, the protection wrapper tries to track the position of each 
    FPU as it is moved. When a ping returns, the new position can
    generally be updated to that point.
    
    However, when a movement is started, there is no way to know the
    exact position before the movement finished regularly, or the next
    successful ping returns. When an abortMotion message is sent, or a
    collision occurs, the position is not known.
    
    The solution used here is to track not the position, but the
    worst-case minimum and maximum position. In other words, for each
    of the alpha and beta coordinate, an /interval/ of positions is
    tracked.  When a deterministic movement is added to the current
    interval, the minimum and maximum extend of that movement become
    added to the current minimum and maximum. In other words,
    movements expand the region of unterctainty, and regular
    terminations, pings and datum responses collaps it to an interval
    of length zero.

    If a configMotion command is issued, this defines a tentative
    future interval, which becomes valid once the corresponding
    movement is started.  If a reverseMotion is issued, this generates
    an interval with the opposite sign of movements.

    Tracking the region of uncertainty is not always required, as ping
    commands can often be used to narrow the possible range of
    possitions down. But doing this has, first, the disadvantage that
    special-casing is needed for handling an abortMotion or collision
    situation. Yet exactle these are situations which need to be
    handled very robustly. And second, tracking the known state of
    positions during start-up also requires special-casing, which
    would require a lot of fiddly state handling. This could be
    error-prone, and the solution to track intervals is therefore much
    more general and clean.

    """
        
    def _check_and_register_wtable(self, wtable, gs, fpuset, wmode=Range.Error, sign=1):
        # compute movement range for each FPU
        # add to current known min / max position
        # compare to allowed range
        # if not in range, throw exception, or print warning,
        # depending on protection setting
        alpha_configuring_range = {}
        beta_configuring_range = {}
        for fpu_id, wt_row in wtable.items():
            fpu = gs.FPU[fpu_id]
 
            alimits= self.alimits[fpu_id]
            blimits= self.blimits[fpu_id]

            alpha0 = self.apositions[fpu_id] 
            beta0 = self.bpositions[fpu_id]

            wf_arange = Interval(alpha0)
            wf_brange = Interval(beta0)
                        
            self._check_allowed_range(fpu_id, -1, "alpha", alimits, alpha0,
                                     wf_arange,  wmode)

            self._check_allowed_range(fpu_id, -1, "beta",  blimits, beta0, 
                                     wf_brange, wmode)

            asteps = 0
            bsteps = 0
            for step_num, entry in enumerate(wt_row):
                a, b = entry
                asteps += a *sign
                bsteps += b * sign
                alpha_sect = alpha0 + asteps / StepsPerDegreeAlpha
                beta_sect = beta0 + bsteps / StepsPerDegreeBeta
                
                self._check_allowed_range(fpu_id, step_num, "alpha",
                                         alimits, alpha_sect,
                                         wf_arange,  wmode)
                self._check_allowed_range(fpu_id, step_num, "beta",
                                         blimits, beta_sect,
                                         wf_brange, wmode)
                
            alpha_configuring_range[fpu_id] = wf_arange
            beta_configuring_range[fpu_id] = wf_brange

        # this is the list of alpha/beta position intervals that
        # will become valid if and after an executeMotion is started,
        # and before the new positions can be retrieved via ping
        self.alpha_configuring_range.update(alpha_configuring_range)
        self.beta_configuring_range.update(beta_configuring_range)

    def _pre_config_motion_hook(self, wtable, gs, fpuset, wmode=Range.Error):
        self._check_and_register_wtable(wtable, gs, fpuset, wmode, sign=1)

    # this can possibly be deleted
    # but do we want to store the full wavetable ?
    def _save_wtable_direction(self, fpu_id_list, is_reversed=False, grid_state=None):
        with env.begin(db=self.fpudb, write=True) as txn:
            for fpu_id in fpu_id_list:
                fpu = grid_state.FPU[fpu_id]
                
                self.wf_reversed[fpu_id] = is_reversed
                ProtectionDB.store_reversed(txn, fpu, is_reversed)
        
        
    def _post_config_motion_hook(self, wtable, gs, fpuset, partial_config=False):
        # update ranges that will become valid once executeMotion is started
        if not partial_config:
            self.configured_arange.update(self.alpha_configuring_range)
            self.configured_brange.update(self.beta_configuring_range)
        else:
            # In this case, not all waveforms were sent successfully.
            #
            # Because we do not know which waveforms have been
            # changed, and which not, we simply extend the worst-case
            # range of the affected waveforms. This covers both
            # the situation that the old wave table is used, and
            # the case that the new one is used.
            for fpu_id, rentry in self.alpha_configuring_range.items():
                if gd.FPU[fpu_id].state == FPST_READY_FORWARD:
                    self.configured_arange[fpu_id].extend(rentry)
                else:
                    self.configured_arange[fpu_id] = Interval(0)
                    del wtable[fpu_id]
                    
                
            for fpu_id, rentry in self.beta_configuring_range.items():
                if gd.FPU[fpu_id].state == FPST_READY_FORWARD:
                    self.configured_brange[fpu_id].extend(rentry)
                else:
                    self.configured_brange[fpu_id] = Interval(0)
                    
                    if wtable.has_key(fpu_id):
                        del wtable[fpu_id]

        self._save_wtable_direction(wtable.keys(), is_reversed=False, grid_state=gs)
        # save the changed waveform tables
        with env.begin(db=self.fpudb, write=True) as txn:
            for fpu_id, wentry in wtable.items():
                fpu = gs.FPU[fpu_id]
                ProtectionDB.storeWaveform(txn, fpu, wentry)

        
    def _pre_repeat_motion_hook(self, wtable, gs, fpuset, wmode=Range.Error):
        self._check_and_register_wtable(wtable, gs, fpuset, wmode, sign=1)
        
        
    def _post_repeat_motion_hook(self, wtable, gs, fpuset):
        # updateranges that become valid once executeMotion is started
        self.configured_arange.update(self.alpha_configuring_range)
        self.configured_brange.update(self.beta_configuring_range)
        self._save_wtable_direction(wtable.keys(), is_reversed=False, grid_state=gs)

    def _pre_reverse_motion_hook(self, wtable, gs, fpuset, wmode=Range.Error):
        self._check_and_register_wtable(wtable, gs, fpuset, wmode, sign=-1)
        
        
    def _post_reverse_motion_hook(self, wtable, gs, fpuset):
        # updateranges that become valid once executeMotion is started
        self.configured_arange.update(self.alpha_configuring_range)
        self.configured_brange.update(self.beta_configuring_range)
        self._save_wtable_direction(wtable.keys(), is_reversed=True, grid_state=gs)
        

    def _start_execute_motion_hook(self, gs, fpuset, initial_positions={}):
        """This runs before executeMotion command is started. After that
        point, the FPU in fpuset should be moving within the ranges
        set by the last config_motion, repeat_motion or reverse_motion
        command. These ranges are set as the intervals which define
        the possible positions until the command has finished.
        
        initial_positions is a dictionary which is filled with the
        positions as (alpha,beta) tuples at the time the command was
        called.

        """
        self.pingFPUs(gs, fpuset=fpuset)
        
        with env.begin(db=self.fpudb, write=True) as txn:
            for fpu_id, fpu in enumerate(gs.FPU):
                if len(fpuset) != 0 and (fpu_id not in fpuset):
                    continue
                
                initial_positions[fpu_id] = (self.apositions[fpu_id],
                                             self.bpositions[fpu_id])
                # copy configured alpha and beta position intervals, and
                # store both to DB
                if self.configured_arange.has_key(fpu_id):
                    self._update_apos(txn, fpu, fpu_id, self.configured_arange[fpu_id])
                    self._update_bpos(txn, fpu, fpu_id,  self.configured_brange[fpu_id])
                    
        env.sync()
     

    def _cancel_execute_motion_hook(self, gs, fpuset, initial_positions=None):
        """This hook cancel registering an executeMotion command which was
        rejected by the driver, before an actual movement was started.

        """
        
        with env.begin(db=self.fpudb, write=True) as txn:
            for fpu_id in initial_positions.keys():
                if (len(fpuset) != 0) and (fpu_id not in fpuset):
                    continue
                apos, bpos = initial_positions[fpu_id]
                fpu = gs.FPU[fpu_id]
                self._update_apos(txn, fpu, fpu_id, apos)
                self._update_bpos(txn, fpu, fpu_id,  bpos)
                
        env.sync()
                                
    
    def _post_execute_motion_hook(self, gs, fpuset):
        """This runs after both an executeMotion has run, and *also*
        a ping has returned successfully."""

        # What do we here if FPUs are still moving
        # (this would happen in case of an error)?
        # Solution for now: wait.
        while True:
            if (gs.Counts[FPST_MOVING] > 0)or (gs.Counts[FPST_DATUM_SEARCH] > 0):
                print("waiting for movement to finish in order to retrieve reached positions..")
                time.sleep(0.5)
                super(GridDriver, self).pingFPUs(gs, fpuset=fpuset)
            else:
                break
        # The assumption here is that the offsets did not change, even
        # if the waveform execution was aborted. This might not be
        # correct in a sense valid for science measurements, because
        # collisions would compromise the precision of step counts,
        # but for protection purposes this should be OK.
        #
        # Thus, use the step counter positions to update the location
        self._refresh_positions(gs, fpuset=fpuset)
        
        # clear wavetable spans for the addressed FPUs - they are not longer valid
        for k in self.configured_arange.keys():
            if (len(fpuset) == 0) or (k in fpuset):
                del self.configured_arange[k]
                
        for k in self.configured_brange.keys():
            if (len(fpuset) == 0) or (k in fpuset):
                del self.configured_brange[k]

    def _allowfind_datum_hook(self,gs, search_modes, selected_arm=None, fpuset=[],
                              support_uninitialized_auto=True):
        """This function checks whether a datum search is safe, and throws an
        exception if not. It does that based on the stored
        position.

        """

        # get fresh ping data
        self.pingFPUs(gs, fpuset=fpuset)
        
        # super(GridDriver, self).pingFPUs(gs, fpuset=fpuset)

        acw_range = Interval(0, Inf)
        cw_range = Interval(-Inf, 0)
        for fpu_id, fpu in enumerate(gs.FPU):
            if len(fpuset) != 0 and (fpu_id not in fpuset):
                continue
            # set default value of SEARCH_AUTO
            if not search_modes.has_key(fpu_id):
                search_modes[fpu_id] = SEARCH_AUTO

            if (selected_arm in [DASEL_BETA, DASEL_BOTH]):
                blim = self.blimits[fpu_id]
                bpos = self.bpositions[fpu_id]
                search_beta_clockwise_range = blim.intersects(acw_range)
                search_beta_anti_clockwise_range = blim.intersects(cw_range)
                
                if search_modes[fpu_id] == SEARCH_AUTO :

                    if not fpu.beta_was_zeroed :
                        if not blim.contains(bpos):
                            # arm is completely out of range, probably needs manual move
                            raise ProtectionError("Beta arm of FPU %i is not in safe range"
                                                  " for datum search (angle=%r, range=%r)" % (fpu_id, bpos, blim))

                        if support_uninitialized_auto:
                            # operator wants auto search but hardware is not
                            # initialized. If possible, we use the database value
                            # to set the correct direction.


                            if search_beta_clockwise_range.contains(bpos) :
                                search_modes[fpu_id] = SEARCH_CLOCKWISE
                            elif search_beta_anti_clockwise_range.contains(bpos):
                                search_modes[fpu_id] = SEARCH_ANTI_CLOCKWISE
                            else:
                                raise ProtectionError("No directed datum search possible - "
                                                      " position for FPU %i ambiguous (consider to move the FPU"
                                                      " into an unambigous range)."% fpu_id)
                        else:
                            raise ProtectionError(("FPU %i not initialized, support_uninitialized_auto not"
                                                   + " set, cannot do protected automatic search") % fpu_id)
                    else:
                        # beta was zeroed, check automatic search is safe
                        if not blim.contains(bpos):
                            # arm is completely out of range, probably needs manual move
                            raise ProtectionError("Beta arm of FPU %i is not in safe range"
                                                  " for datum search (angle=%r, range=%r)" % (fpu_id, bpos, blim))
                        
                elif search_modes[fpu_id] == SEARCH_CLOCKWISE:
                    if not search_beta_clockwise_range.contains(bpos):
                        raise ProtectionError("Beta arm of FPU %i is outside of"
                                              " safe clockwise search range (angle=%r, range=%r)" % (fpu_id, bpos,
                                                                                                     search_beta_clockwise_range))
                    
                elif search_modes[fpu_id] == SEARCH_ANTI_CLOCKWISE:
                    if not search_beta_anti_clockwise_range.contains(bpos):
                        raise ProtectionError("Beta arm of FPU %i is outside of"
                                              " safe anti-clockwise search range"
                                              " (angle=%r, range=%r)" % (fpu_id, bpos,
                                                                         search_beta_anti_clockwise_range))
                    
            # check alpha arm
            if (selected_arm in [DASEL_ALPHA, DASEL_BOTH]):
                alim = self.alimits[fpu_id]
                apos = self.apositions[fpu_id]
                if not alim.contains(apos):
                    raise ProtectionError("Alpha arm of FPU %i is not in safe range"
                                          " for datum search (angle=%r, range=%r)" % (fpu_id, apos, alim))
            
    def _finished_find_datum_hook(self, gs, search_modes, fpuset=[], was_cancelled=False, initial_positions={}):
        with env.begin(db=self.fpudb, write=True) as txn:
            for fpu_id, fpu in enumerate(gs.FPU):
                if len(fpuset) > 0 and (not (fpu_id in fpuset)):
                    continue
            
                if (len(search_modes) != 0) and (not search_modes.kas_key(fpu_id)):
                    continue
                
                                                                                    
                # set position intervals to zero, and store in DB
                if fpu.alpha_was_zeroed:
                    self.a_caloffsets[fpu_id] = Interval(0)
                    if fpu.alpha_steps == 0:
                        self._update_apos(txn, fpu, fpu_id, Interval(0.0) + self.config.alpha_datum_offset)
                else:
                    ## If ping_ok is set, we assume that even if the
                    ## datum operation itself did not succeed, the
                    ## step counter was successfully retrieved by a
                    ## subsequent ping, and the offset is unchanged.
                    if fpu.ping_ok:
                        self._update_apos(txn, fpu, fpu_id, Interval(self._alpha_angle(fpu)) + self.a_caloffsets[fpu_id])

                
                if fpu.beta_was_zeroed:
                    self.b_caloffsets[fpu_id] = Interval(0)
                    if fpu.beta_steps == 0:                        
                        self._update_bpos(txn, fpu, fpu_id,  Interval(0.0))

                        if self.bretries_acw[fpu_id] > 0:
                            clockwise = False
                            ProtectionDB.store_bretry_count(txn, fpu, clockwise, 0)
                        if self.bretries_cw[fpu_id] > 0:
                            clockwise = True
                            ProtectionDB.store_bretry_count(txn, fpu, clockwise, 0)
                else:
                    if fpu.ping_ok:
                        self._update_bpos(txn, fpu, fpu_id,  Interval(self._beta_angle(fpu)) + self.b_caloffsets[fpu_id])

        env.sync()
 
        

    def _start_find_datum_hook(self, gs, search_modes,  selected_arm=None,fpuset=[], initial_positions={}):
        """This is run when an findDatum command is actually started.
        It updates the new range of possible positions to include the zero point of each arm."""
        with env.begin(db=self.fpudb, write=True) as txn:
            for fpu_id, fpu in enumerate(gs.FPU):
                if len(fpuset) > 0 and (not (fpu_id in fpuset)):
                    continue
            
                if (len(search_modes) != 0) and (not search_modes.kas_key(fpu_id)):
                    continue
            
                # record initial position intervals so that the
                # known range can be restored if the datum search is
                # rejected
                initial_positions[fpu_id] = (self.apositions[fpu_id],
                                             self.bpositions[fpu_id])
                
                # update stored intervals to include zero, and store in DB
                if selected_arm in [DASEL_ALPHA, DASEL_BOTH]:
                    self._update_apos(txn, fpu, fpu_id, self.apositions[fpu_id].extend(0.0 + self.config.alpha_datum_offset))
                    
                if selected_arm in [DASEL_BETA, DASEL_BOTH]:
                    self._update_bpos(txn, fpu, fpu_id,  self.bpositions[fpu_id].extend(0.0))
                    
        env.sync()
 
                

    def _cancel_find_datum_hook(self, gs, search_modes,  selected_arm=None,
                               fpuset=[], initial_positions={}):

        with env.begin(db=self.fpudb, write=True) as txn:
            for fpu_id in initial_positions.keys():
                # get last stored positions
                apos, bpos = initial_positions[fpu_id] 
            
                fpu = gs.FPU[fpu_id]
                # revert stored intervals to old values
                self._update_apos(txn, fpu, fpu_id, apos)
                self._update_bpos(txn, fpu, fpu_id, bpos)
                
        env.sync()
 

    def _pre_free_beta_collision_hook(self, fpu_id, direction, grid_state):
        
        if direction == REQD_CLOCKWISE:
            brcnt = self.bretries_cw[fpu_id]
        else:
            brcnt = self.bretries_acw[fpu_id]
            
        if brcnt >= self.maxbretries[fpu_id]:
            raise ProtectionError("Retry count for FPU %i is already %i, exceeds safe maximum" % (fpu_id, brcnt))
        
        
    def _post_free_beta_collision_hook(self, fpu_id, direction, grid_state):
        
        clockwise = (direction == REQD_CLOCKWISE)
        
        if clockwise:
            self.bretries_cw[fpu_id] += 1
            cnt = self.bretries_cw[fpu_id]
        else:
            self.bretries_acw[fpu_id] += 1
            cnt = self.bretries_acw[fpu_id]


        fpu = grid_state.FPU[fpu_id]
        
        with env.begin(db=self.fpudb, write=True) as txn:
            ProtectionDB.store_bretry_count(txn, fpu, clockwise, cnt)

        fpuset = [fpu_id]
        super(GridDriver, self).pingFPUs(grid_state, fpuset=fpuset)
        self._refresh_positions(grid_state, fpuset=fpuset)
