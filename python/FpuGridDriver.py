#!/usr/bin/python

from __future__ import print_function, division
import sys
import platform
import threading
import warnings

import pdb
import traceback
import os
from os import path
import errno
import time
import signal
from warnings import warn, filterwarnings
import textwrap
# state tracking
import lmdb
import devicelock
from interval import Interval, Inf, nan

from fpu_constants import *
from protectiondb import ProtectionDB, HealthLogDB, open_database_env

import fpu_commands

import ethercanif
assert(ethercanif.CAN_PROTOCOL_VERSION == 2), "requires ethercanif module for CAN protocol version 2!"

from ethercanif import (__version__, CAN_PROTOCOL_VERSION, GatewayAddress,  EtherCANInterfaceConfig,
                        REQD_ANTI_CLOCKWISE,  REQD_CLOCKWISE, DIRST_CLOCKWISE, DIRST_RESTING_LAST_CW,
                        DIRST_ANTI_CLOCKWISE, DIRST_RESTING_LAST_ACW,
                        DEFAULT_WAVEFORM_RULESET_VERSION,
                        EtherCANException, MovementError, FirmwareTimeoutError, CollisionError, LimitBreachError,
                        AbortMotionError, StepTimingError, InvalidStateException, SystemFailure,
                        InvalidParameterError, SetupError, InvalidWaveformException, ConnectionFailure,
                        SocketFailure, CommandTimeout, ProtectionError, HardwareProtectionError,
                        DASEL_BOTH, DASEL_ALPHA, DASEL_BETA,
                        DATUM_TIMEOUT_ENABLE, DATUM_TIMEOUT_DISABLE,
                        LOG_ERROR, LOG_INFO, LOG_GRIDSTATE, LOG_DEBUG, LOG_VERBOSE, LOG_TRACE_CAN_MESSAGES,
                        SEARCH_CLOCKWISE, SEARCH_ANTI_CLOCKWISE, SEARCH_AUTO, SKIP_FPU, FPST_UNINITIALIZED,
                        FPST_LOCKED, FPST_DATUM_SEARCH, FPST_AT_DATUM, FPST_LOADING, FPST_READY_FORWARD,
                        FPST_READY_REVERSE, FPST_MOVING, FPST_RESTING, FPST_ABORTED, FPST_OBSTACLE_ERROR, DS_CONNECTED,
                        CCMD_EXECUTE_MOTION,
                        DE_OK,
                        MCE_FPU_OK, MCE_WARN_COLLISION_DETECTED,
                        MCE_WARN_LIMIT_SWITCH_BREACH, MCE_ERR_INVALID_COMMAND, MCE_NOTIFY_COMMAND_IGNORED,
                        MCE_ERR_WAVEFORM_NOT_READY, WAVEFORM_TOO_BIG, WAVEFORM_SEQUENCE, WAVEFORM_BADVALUE,
                        MCE_WARN_STEP_TIMING_ERROR, MCE_ERR_INVALID_PARAMETER, MCE_ERR_DATUM_TIME_OUT, MCE_NOTIFY_DATUM_ALPHA_ONLY,
                        MCE_NOTIFY_DATUM_BETA_ONLY, MCE_ERR_AUTO_DATUM_UNINITIALIZED, MCE_ERR_DATUM_ON_LIMIT_SWITCH,
                        MCE_ERR_CAN_OVERFLOW_HW, MCE_ERR_CAN_OVERFLOW_SW, MCE_NO_CONFIRMATION_EXPECTED, MCE_COMMAND_TIMEDOUT)

# Aliases for the ANTI_CLOCKWISE and CLOCKWISE directions.
REQD_POSITIVE = REQD_ANTI_CLOCKWISE
REQD_NEGATIVE = REQD_CLOCKWISE


MOCK_GATEWAY_ADDRESS_LIST = [ GatewayAddress("127.0.0.1", p)
                                for p in [4700, 4701, 4702] ]


TEST_GATEWAY_ADDRESS_LIST = [ GatewayAddress("192.168.0.10", 4700) ]

DEFAULT_GATEWAY_ADDRESS_LIST = MOCK_GATEWAY_ADDRESS_LIST

# provide constants with typo, for backwards compatibility
MOCK_GATEWAY_ADRESS_LIST = MOCK_GATEWAY_ADDRESS_LIST
TEST_GATEWAY_ADRESS_LIST = TEST_GATEWAY_ADDRESS_LIST
DEFAULT_GATEWAY_ADRESS_LIST = DEFAULT_GATEWAY_ADDRESS_LIST


DEFAULT_NUM_FPUS=int(os.environ.get("NUM_FPUS","1005"))

DEFAULT_LOGDIR=os.environ.get("FPU_DRIVER_DEFAULT_LOGDIR","./_logs")
str_loglevel = os.environ.get("FPU_DRIVER_DEFAULT_LOGLEVEL","LOG_TRACE_CAN_MESSAGES")
DEFAULT_LOGLEVEL= {"LOG_ERROR":    LOG_ERROR,
                   "LOG_INFO":     LOG_INFO,
                   "LOG_GRIDSTATE":LOG_GRIDSTATE,
                   "LOG_VERBOSE":LOG_VERBOSE,
                   "LOG_DEBUG":    LOG_DEBUG,
                   "LOG_TRACE_CAN_MESSAGES": LOG_TRACE_CAN_MESSAGES}[str_loglevel]

# ===========================================================================
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

# ---------------------------------------------------------------------------
def get_logname(basename,
                log_dir="",
                timestamp=None):

    if timestamp=="ISO8601":
        timestamp= time.strftime("%Y-%m-%dT%H:%M:%S", time.localtime())

    filename = basename.format(start_timestamp=timestamp)
    return path.join(log_dir, path.expandvars(path.expanduser(filename)))


# ---------------------------------------------------------------------------
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
        print("Logging to ", log_path)

    return log_path

# ===========================================================================
# enumeration which defines the strictness of checks
class Range:
    Error = "Error - path rejected"
    Warn = "Warning - path unsafe"
    Ignore = "Ignore - path unchecked"


filterwarnings("default", "keyword check_protection", DeprecationWarning)

# ---------------------------------------------------------------------------
def sign(x):
    if x == 0:
        return 0
    elif x < 0:
        return -1
    elif x > 0:
        return 1
    else:
        raise AssertionError("sign(x): a NaN value was probably passed here")

# ---------------------------------------------------------------------------
def fpu_in_set(fpu_id, fpuset):
    if len(fpuset) == 0:
        return True
    else:
        return (fpu_id in fpuset)

# ---------------------------------------------------------------------------
def countMovableFPUs(gs):
    num_forward = 0
    num_reversed = 0
    for fpu in gs.FPU:
        state = fpu.state
        if state == FPST_READY_FORWARD:
            num_forward += 1
        elif state == FPST_READY_REVERSE:
            num_reversed += 1

    return num_forward, num_reversed, num_forward + num_reversed



# ===========================================================================

class UnprotectedGridDriver (object):
    """
    
    The UnprotectedGridDriver class implements a Python interface to
    the EtherCAN communication software.
    
    """
    def __init__(self, nfpus=DEFAULT_NUM_FPUS,
                 SocketTimeOutSeconds=20.0,
                 confirm_each_step=False,
                 waveform_upload_pause_us=0,
                 configmotion_max_retry_count=5,
                 configmotion_max_resend_count=10,
                 min_bus_repeat_delay_ms = 0,
	             min_fpu_repeat_delay_ms = 1,
                 alpha_datum_offset=ALPHA_DATUM_OFFSET,
                 logLevel=DEFAULT_LOGLEVEL,
                 log_dir=DEFAULT_LOGDIR,
                 motor_minimum_frequency=MOTOR_MIN_STEP_FREQUENCY,
                 motor_maximum_frequency=MOTOR_MAX_STEP_FREQUENCY,
                 motor_max_start_frequency=MOTOR_MAX_START_FREQUENCY,
                 motor_max_rel_increase=MAX_ACCELERATION_FACTOR,
                 motor_max_step_difference=MAX_STEP_DIFFERENCE,
                 firmware_version_address_offset=0x61,
                 protection_logfile="_{start_timestamp}-fpu_protection.log",
                 control_logfile="_{start_timestamp}-fpu_control.log",
                 tx_logfile = "_{start_timestamp}-fpu_tx.log",
                 rx_logfile = "_{start_timestamp}-fpu_rx.log",
                 start_timestamp="ISO8601"):

        self.lock = threading.RLock()

        if confirm_each_step:
            warn("confirm_each_steps set to True, which requires extra"
                 " confirmation requests of waveform step upload, and reduces performance")

        if min_bus_repeat_delay_ms > 0:
            warn("min_bus_repeat_delay_ms is set to value above 0."
                 " Decrease if message rate is too low.")

        # Create an EtherCAN configuration object and load it with parameters.
        config = EtherCANInterfaceConfig()
        config.num_fpus = nfpus
        config.SocketTimeOutSeconds = SocketTimeOutSeconds
        config.alpha_datum_offset = alpha_datum_offset
        config.motor_minimum_frequency = motor_minimum_frequency
        config.motor_maximum_frequency = motor_maximum_frequency
        config.motor_max_start_frequency= motor_max_start_frequency
        config.motor_max_rel_increase = motor_max_rel_increase
        config.confirm_each_step = confirm_each_step
        config.configmotion_max_retry_count = configmotion_max_retry_count
        config.configmotion_max_resend_count = configmotion_max_resend_count
        config.waveform_upload_pause_us = waveform_upload_pause_us
       	config.min_bus_repeat_delay_ms = min_bus_repeat_delay_ms
        config.min_fpu_repeat_delay_ms = min_fpu_repeat_delay_ms
        config.configmotion_max_resend_count = configmotion_max_resend_count
        config.configmotion_max_retry_count = configmotion_max_retry_count
        config.firmware_version_address_offset = firmware_version_address_offset

        flags = os.O_CREAT | os.O_APPEND | os.O_WRONLY
        mode = 0o00644

        config.logLevel = logLevel

        log_path = make_logdir(log_dir)

        self.protectionlog = open(get_logname(protection_logfile,
                                              log_dir=log_path, timestamp=start_timestamp), "wt")

        print("%f: Starting communication interface version %s" % (time.time(), ethercanif.__version__),
              file=self.protectionlog)

        # Open file descriptors to the CONTROL, TX and RX log files.
        control_filename = get_logname(control_logfile,
                                  log_dir=log_path,
                                  timestamp=start_timestamp)
        tx_filename = get_logname(tx_logfile,
                                  log_dir=log_path,
                                  timestamp=start_timestamp)
        rx_filename = get_logname(rx_logfile,
                                  log_dir=log_path,
                                  timestamp=start_timestamp)
        config.fd_controllog = os.open(control_filename, flags, mode)
        config.fd_txlog = os.open(tx_filename, flags, mode)
        config.fd_rxlog = os.open(rx_filename, flags, mode)

        # Check the log files have actually been opened successfully
        if config.fd_controllog > 0:
           print("CONTROL log file: %s" % control_filename)
        else:
           print("Error: Failed to open CONTROL log file (%s)" % control_filename)

        if config.fd_txlog > 0:
           print("     TX log file: %s" % tx_filename)
        else:
           print("Error: Failed to open TX log file (%s)" % tx_filename)

        if config.fd_controllog > 0:
           print("     RX log file: %s" % rx_filename)
        else:
           print("Error: Failed to open RX log file (%s)" % rx_filename)

        self.config = config
        self.last_wavetable = {}
        self.wf_reversed = {}

        self.wavetables_incomplete = False

        self._gd = ethercanif.EtherCANInterface(config)
        self.locked_gateways = []

    def __del__(self):
        # the following delete is necessary to run destructors!!
        del self._gd
        del self.locked_gateways
        self.protectionlog.close()
        os.close(self.config.fd_controllog)
        os.close(self.config.fd_txlog)
        os.close(self.config.fd_rxlog)

    def _post_connect_hook(self, config):
        pass

    # ........................................................................
    def connect(self, address_list=DEFAULT_GATEWAY_ADRESS_LIST):
        with self.lock:
            self.locked_gateways = [] # this del's and releases any previously acquired locks
            for gw in address_list:
                groupname = os.environ.get("MOONS_GATEWAY_ACCESS_GROUP","moons")
                # acquire a unique inter-process lock for each gateway IP
                dl = devicelock.DeviceLock('ethercan-gateway@%s:%i' % (gw.ip, gw.port), groupname)
                self.locked_gateways.append(dl)
            rv = self._gd.connect(address_list)
            self._post_connect_hook(self.config)
            return rv

    def check_fpuset(self, fpuset):
        if len(fpuset) == 0:
            return fpuset

        if min(fpuset) < 0:
            raise InvalidParameterError("DE_INVALID_FPU_ID: a passed FPU ID is smaller than zero")

        if max(fpuset) >= self.config.num_fpus:
            raise InvalidParameterError("DE_INVALID_FPU_ID: a passed FPU ID is too large"
                                        " for the configured number of FPUs")
        return fpuset

    def need_ping(self, grid_state, fpuset):
        if len(fpuset) == 0:
            fpuset = range(self.config.num_fpus)
        return [ fpu_id for fpu_id in fpuset if (not grid_state.FPU[fpu_id].ping_ok) ]



    # ........................................................................
    def setUStepLevel(self, ustep_level,  gs, fpuset=None):
        if fpuset is None:
            fpuset = []
        fpuset = self.check_fpuset(fpuset)
        return self._gd.setUStepLevel(ustep_level, gs, fpuset)

    def setTicksPerSegment(self, nticks,  gs, fpuset=None):
        if fpuset is None:
            fpuset = []
        fpuset = self.check_fpuset(fpuset)
        return self._gd.setTicksPerSegment(nticks, gs, fpuset)

    def setStepsPerSegment(self, min_steps, max_steps,  gs, fpuset=None):
        if fpuset is None:
            fpuset = []
        fpuset = self.check_fpuset(fpuset)
        return self._gd.setStepsPerSegment(min_steps, max_steps, gs, fpuset)

    def getSwitchStates(self, gs, fpuset=None):
        if fpuset is None:
            fpuset = []
        if len(fpuset) == 0:
            fpuset = range(self.config.num_fpus)

        def getState(fpu):
            return {'alpha_limit_active' : fpu.alpha_datum_switch_active,
                    'beta_datum_active' : fpu.beta_datum_switch_active  }
        return dict([ (fpu_id, getState(gs.FPU[fpu_id]) ) for fpu_id in fpuset])

    # ........................................................................
    def getGridState(self):
        return self._gd.getGridState()


    def _allow_find_datum_hook(self, gs, search_modes, selected_arm=None,
                               fpuset=None, support_uninitialized_auto=None):
        pass

    def _start_find_datum_hook(self, gs, search_modes=None,  selected_arm=None, fpuset=None, initial_positions=None, soft_protection=None):
        pass

    def _cancel_find_datum_hook(self, gs, search_modes,  selected_arm=None,
                               fpuset=None, initial_positions=None):
        pass

    def _finished_find_datum_hook(self, prev_gs, datum_gs, search_modes=None,
                                  fpuset=None, was_cancelled=False, initial_positions=None,
                                  selected_arm=None):
        pass

    def findDatumB(self, gs, search_modes=None,
                   selected_arm=DASEL_BOTH,
                   soft_protection=True,
                   check_protection=None,
                   fpuset=None,
                   support_uninitialized_auto=True,
                   timeout=DATUM_TIMEOUT_ENABLE):
        """Moves all FPUs to datum position.

        This is a blocking variant of the findDatum command,
        it is not interruptible by Control-C."""

        if search_modes is None:
            search_modes = {}
            
        if fpuset is None:
            fpuset = []
        if len(fpuset) == 0:
            fpuset = range(self.config.num_fpus)

        fpuset = self.check_fpuset(fpuset)


        with self.lock:

            if check_protection != None:
                warn("keyword check_protection is deprecated; use 'soft_protection=False' instead!",
                     DeprecationWarning, 2)
                soft_protection=check_protection

            count_protection = soft_protection
            if soft_protection:
                search_modes = search_modes.copy()
                # check whether datum search is safe, adjusting search_modes
                self._allow_find_datum_hook(gs, search_modes, selected_arm=selected_arm,
                                           fpuset=fpuset, support_uninitialized_auto=support_uninitialized_auto)
                if (SEARCH_CLOCKWISE in search_modes) or (SEARCH_ANTI_CLOCKWISE in search_modes):
                    count_protection = False

            initial_positions = {}

            self._start_find_datum_hook(gs, search_modes=search_modes, selected_arm=selected_arm,
                                        fpuset=fpuset, initial_positions=initial_positions, soft_protection=soft_protection)
            prev_gs = self._gd.getGridState()
            was_cancelled = False
            try:
                try:
                    rv =  self._gd.findDatum(gs, search_modes, fpuset, selected_arm, timeout, count_protection)
                except (RuntimeError,
                        InvalidParameterError,
                        SetupError,
                        InvalidStateException,
                        HardwareProtectionError,
                        ProtectionError) as e:
                    # we cancel the datum search altogether, so we can reset
                    # positions to old value
                    self._cancel_find_datum_hook(gs, fpuset, initial_positions=initial_positions)
                    was_cancelled = True
                    print("Operation canceled, error = ", str(e))
                    raise
            finally:
                if was_cancelled or (rv != ethercanif.E_EtherCANErrCode.DE_OK):
                    try:
                        pingset = self.need_ping(gs, fpuset)
                        if len(pingset) > 0 :
                            self._pingFPUs(gs, pingset)
                    except CommandTimeout:
                        pass


                self._finished_find_datum_hook(prev_gs, gs, search_modes=search_modes, fpuset=fpuset,
                                               was_cancelled=was_cancelled,
                                               initial_positions=initial_positions,
                                               selected_arm=selected_arm)

            return rv

    # ........................................................................
    def findDatum(self, gs, search_modes=None, selected_arm=DASEL_BOTH, fpuset=None,
                  soft_protection=True, count_protection=True, check_protection=None,
                  support_uninitialized_auto=True, timeout=DATUM_TIMEOUT_ENABLE):
        """Moves all FPUs to datum position.

        If the program receives a SIGNINT, or Control-C is pressed, an
        abortMotion command is automatically sent, aborting the search.

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
        if search_modes is None:
            search_modes = {}

        if fpuset is None:
            fpuset = []
        if len(fpuset) == 0:
            fpuset = range(self.config.num_fpus)

        fpuset = self.check_fpuset(fpuset)


        with self.lock:

            if check_protection != None:
                warn("keyword check_protection is deprecated; use 'soft_protection=False' instead!",
                     DeprecationWarning, 2)
                soft_protection=check_protection

            if soft_protection:
                orig_search_modes = search_modes
                search_modes = orig_search_modes.copy()
                # check whether datum search is safe, adjusting search_modes
                self._allow_find_datum_hook(gs, search_modes, selected_arm=selected_arm,
                                           fpuset=fpuset, support_uninitialized_auto=support_uninitialized_auto)
                # We might need to disable the stepcount-based check if
                # (and only if) the step counter does not agree with the
                # database Check whether a movement against the step count
                # is needed.
                for k, m in search_modes.items():
                    fpu = gs.FPU[k]
                    if ( (m in [SEARCH_CLOCKWISE, SEARCH_ANTI_CLOCKWISE ])
                         and (orig_search_modes.get(k, SEARCH_AUTO)  == SEARCH_AUTO)):
                        count_protection = False

            initial_positions = {}
            self._start_find_datum_hook(gs, search_modes=search_modes, selected_arm=selected_arm, fpuset=fpuset,
                                        initial_positions=initial_positions, soft_protection=soft_protection)

            prev_gs = self._gd.getGridState()

            try:
                rv = self._gd.startFindDatum(gs, search_modes, fpuset, selected_arm, timeout, count_protection)
                if rv != ethercanif.E_EtherCANErrCode.DE_OK:
                    raise RuntimeError("can't search Datum, driver error code = %r" % rv)

            except (RuntimeError,
                    InvalidParameterError,
                    SetupError,
                    InvalidStateException,
                    HardwareProtectionError,
                    ProtectionError) as e:
                # we cancel the datum search altogether, so we can reset
                # positions to old value
                self._cancel_find_datum_hook(gs, fpuset,initial_positions=initial_positions)
                raise

            time_interval = 0.1
            time.sleep(time_interval)
            is_ready = False
            was_aborted = False
            finished_ok = False

            try:
                with SignalHandler() as sh:
                    while not is_ready:
                        rv = self._gd.waitFindDatum(gs, time_interval, fpuset)
                        if sh.interrupted:
                            print("STOPPING FPUs.")
                            self.abortMotion(gs, fpuset=fpuset)
                            was_aborted = True
                            break
                        is_ready = (rv != ethercanif.E_EtherCANErrCode.DE_WAIT_TIMEOUT)
                        finished_ok = (rv == ethercanif.E_EtherCANErrCode.DE_OK)
            finally:
                if not finished_ok:
                    time.sleep(time_interval)
                    try:
                        pingset = self.need_ping(gs, fpuset)
                        if len(pingset) > 0 :
                            self._pingFPUs(gs, pingset)
                    except CommandTimeout:
                        pass

                self._finished_find_datum_hook(prev_gs, gs, search_modes=search_modes,
                                               fpuset=fpuset, was_cancelled=(not finished_ok),
                                               initial_positions=initial_positions,
                                               selected_arm=selected_arm)

            if was_aborted:
                print("findDatum was aborted by SIGINT, movement stopped")
                raise MovementError("findDatum was aborted by SIGINT")


        return rv

    def _pingFPUs(self, gs, fpuset=None):

        if fpuset is None:
            fpuset = []
        fpuset = self.check_fpuset(fpuset)

        return self._gd.pingFPUs(gs, fpuset)

    # ........................................................................
    def pingFPUs(self, gs, fpuset=None):
        """Communicate with all FPUs and query their status
        """
        if fpuset is None:
            fpuset = []
        return self._pingFPUs(gs, fpuset=fpuset)


    def _update_error_counters(counters, prev_state, fpu):
        pass


    def _reset_hook(self, old_state, gs, fpuset=None):
        pass

    # ........................................................................
    def resetFPUs(self, gs, fpuset=None, verbose=True):
        if fpuset is None:
            fpuset = []
        fpuset = self.check_fpuset(fpuset)

        with self.lock:
            old_state = self.getGridState()
            try:
                rval = self._gd.resetFPUs(gs, fpuset)
            finally:
                if self.__dict__.has_key("counters"):
                    for fpu_id, fpu in enumerate(gs.FPU):
                        self._update_error_counters(self.counters[fpu_id], old_state.FPU[fpu_id], fpu)

            self.last_wavetable = {}
            msg = "resetFPUs(): waiting for FPUs to become active.... %s"
            w = "|/-\\"
            for k in range(24):
                time.sleep(0.1)
                if verbose:
                    print(msg % w[k%4], end="\r")
                sys.stdout.flush()
            if verbose:
                print(msg % "OK")
            self._reset_hook(old_state, gs, fpuset=fpuset)

        return rval

    def _reset_counter_hook(self, old_state, gs, fpuset=None):
        pass

    # ........................................................................
    def resetStepCounters(self, new_alpha_steps, new_beta_steps, gs, fpuset=None):
        if fpuset is None:
            fpuset = []
        fpuset = self.check_fpuset(fpuset)

        with self.lock:
            old_state = self.getGridState()
            try:
                rval = self._gd.resetStepCounters(new_alpha_steps, new_beta_steps, gs, fpuset)
            finally:
                if self.__dict__.has_key("counters"):
                    for fpu_id, fpu in enumerate(gs.FPU):
                        self._update_error_counters(self.counters[fpu_id], old_state.FPU[fpu_id], fpu)

            alpha_target = new_alpha_steps / StepsPerDegreeAlpha + self.config.alpha_datum_offset
            beta_target = new_beta_steps / StepsPerDegreeBeta

            self._reset_counter_hook(alpha_target, beta_target, old_state, gs, fpuset=fpuset)

        return rval


    # ........................................................................
    def getPositions(self, gs, **kwargs):
        warnings.warn(textwrap.dedent("""This command is obsolete, use pingFPUs().
        Note: In protocol v2, it is not any more needed to perform a ping as long as movement
        commands finished without errors."""))

        return self.pingFPUs(gs, **kwargs)

    ##  def getPositions(self, gs, fpuset=None):
    ##    if fpuset is None:
    ##        fpuset = []
    ##      fpuset = self.check_fpuset(fpuset)
    ##
    ##      time.sleep(0.1)
    ##      try:
    ##          prev_gs = self._gd.getGridState()
    ##          rval = self._gd.getPositions(gs, fpuset)
    ##      finally:
    ##          for fpu_id, fpu in enumerate(gs.FPU):
    ##              self._update_error_counters(self.counters[fpu_id], prev_gs.FPU[fpu_id], fpu)
    ##      return rval

    # ........................................................................
    def readRegister(self, address, gs, fpuset=None):
        if fpuset is None:
            fpuset = []
        fpuset = self.check_fpuset(fpuset)

        try:
            prev_gs = self._gd.getGridState()
            rval= self._gd.readRegister(address, gs, fpuset)
        finally:
            if self.__dict__.has_key("counters"):
                for fpu_id, fpu in enumerate(gs.FPU):
                    self._update_error_counters(self.counters[fpu_id], prev_gs.FPU[fpu_id], fpu)

        return rval

    def getDiagnostics(self, gs, fpuset=None):
        if fpuset is None:
            fpuset = []
        fpuset = self.check_fpuset(fpuset)

        names     = ["sstatus2", "sstatus3", "sstatus4", "sstatus5", "intflags", "stateflags"]
        addresses = [0x00001E,   0x00001F,   0x000020,   0x000021,   0x000022,   0x000023]
        numreg    = min(len(names), len(addresses))

        if fpuset:
           fpulist = fpuset
        else:
           fpulist = range(0, len(gs.FPU))
        strg = "  RegName    RegAddress:"
        for fpu_id in fpulist:
           strg += "  FPU[%d] " % fpu_id
        strg += "\n"

        strg += "  --------   -----------"
        for fpu_id in fpulist:
           strg += "  ------ "
        strg += "\n"

        for ii in range(0, numreg):
           self.readRegister(addresses[ii], gs, fpuset=fpuset)
           strg += "%10s        %.6x" % (names[ii], addresses[ii])
           for fpu_id in fpulist:
              strg += "    %4x " % gs.FPU[fpu_id].register_value
           strg += "\n"
        return strg

    def logDiagnostics(self, gs, fpuset=None):
        if fpuset is None:
            fpuset = []
        print(self.getDiagnostics(gs, fpuset=fpuset))

    # ........................................................................
    def getFirmwareVersion(self, gs, fpuset=None):
        if fpuset is None:
            fpuset = []
        fpuset = self.check_fpuset(fpuset)

        try:
            prev_gs = self._gd.getGridState()
            rval = self._gd.getFirmwareVersion(gs, fpuset)
        finally:
            if self.__dict__.has_key("counters"):
                for fpu_id, fpu in enumerate(gs.FPU):
                    self._update_error_counters(self.counters[fpu_id], prev_gs.FPU[fpu_id], fpu)

        return rval

    # ........................................................................
    def printFirmwareVersion(self, gs, fpuset=None):
        if fpuset is None:
            fpuset = []
        fpuset = self.check_fpuset(fpuset)

        self.getFirmwareVersion(gs, fpuset)
        for fpu_id, fpu in enumerate(gs.FPU):
            if fpu_in_set(fpu_id, fpuset):
                print("FPU %i firmware version: (%i,%i,%i) created %02i-%02i-%02i" % (
                    fpu_id,
                    fpu.fw_version_major, fpu.fw_version_minor, fpu.fw_version_patch,
                    fpu.fw_date_year, fpu.fw_date_month, fpu.fw_date_day))

    def minFirmwareVersion(self, fpuset=None):
        """Note: This command does not retrieve the firmware version via
        the bus, in order to reduce unnecessary bus communication
        (it is intended to be suitable for very frequent use).
        """
        if fpuset is None:
            fpuset = []
        fpuset = self.check_fpuset(fpuset)
        gs = self._gd.getGridState()
        return self._gd.getMinFirmwareVersion(gs, fpuset)

    # ........................................................................
    def getCounterDeviation(self, gs, **kwargs):
        warnings.warn(textwrap.dedent("""This command is obsolete, use gs.FPU[i].alpha_deviation and
        gs.FPU[i].beta_deviation after findDatum()."""))

        return self.pingFPUs(gs, **kwargs)

    ##  ## this method is gone, use the grid state
    ##  def getCounterDeviation(self, gs, fpuset=None):
    ##    if fpuset is None:
    ##        fpuset = []
    ##     fpuset = self.check_fpuset(fpuset)
    ##
    ##     time.sleep(0.1)
    ##     try:
    ##         prev_gs = self._gd.getGridState()
    ##         rval = self._gd.getCounterDeviation(gs, fpuset)
    ##     finally:
    ##         for fpu_id, fpu in enumerate(gs.FPU):
    ##             self._update_error_counters(self.counters[fpu_id], prev_gs.FPU[fpu_id], fpu)
    ##
    ##     return rval

    # ........................................................................
    def readSerialNumbers(self, gs, fpuset=None):
        if fpuset is None:
            fpuset = []
        fpuset = self.check_fpuset(fpuset)

        try:
            prev_gs = self._gd.getGridState()
            rval = self._gd.readSerialNumbers(gs, fpuset)
        finally:
            # needs to check for presence of counters because serial numbers
            # are also used during start-up before the counter data is loaded
            if self.__dict__.has_key("counters"):
                for fpu_id, fpu in enumerate(gs.FPU):
                    self._update_error_counters(self.counters[fpu_id], prev_gs.FPU[fpu_id], fpu)

        return rval

    # ........................................................................
    def printSerialNumbers(self, gs, fpuset=None):
        if fpuset is None:
            fpuset = []
        fpuset = self.check_fpuset(fpuset)

        with self.lock:
            self.readSerialNumbers(gs, fpuset=fpuset)
            for i in range(self.config.num_fpus):
                if fpu_in_set(i, fpuset):
                    print("FPU %i : SN = %r" % (i, gs.FPU[i].serial_number))


    # ........................................................................
    def writeSerialNumber(self, fpu_id, serial_number,  gs):
        with self.lock:
            try:
                prev_gs = self._gd.getGridState()
                rval = self._gd.writeSerialNumber(fpu_id, serial_number, gs)
                if rval == DE_OK:
                    rval = self._gd.readSerialNumbers(gs, [fpu_id])
            finally:
                if self.__dict__.has_key("counters"):
                    for fpu_id, fpu in enumerate(gs.FPU):
                        self._update_error_counters(self.counters[fpu_id], prev_gs.FPU[fpu_id], fpu)

            return rval


    def _pre_config_motion_hook(self, wtable, gs, fpuset, wmode=Range.Error):
        pass


    def _post_config_motion_hook(self, wtable, gs, fpuset):
        with self.lock:
            self.set_wtable_reversed(fpuset, False)


    def _pre_repeat_motion_hook(self, wtable, gs, fpuset, wmode=Range.Error):
        pass

    def _post_repeat_motion_hook(self, wtable, gs, fpuset):
        with self.lock:
            self.set_wtable_reversed(fpuset, False)

    def _pre_reverse_motion_hook(self, wtable, gs, fpuset, wmode=Range.Error):
        pass


    def _post_reverse_motion_hook(self, wtable, gs, fpuset):
        with self.lock:
            self.set_wtable_reversed(fpuset, True)


    @staticmethod
    def wavetable_was_received(wtable, gs, fpu_id, fpu,
                               allow_unconfirmed=False,
                               target_state=FPST_READY_FORWARD):

        return ( wtable.has_key(fpu_id)
                 and (fpu.state==target_state)
                 and ((fpu.last_status == 0) or (allow_unconfirmed and (fpu.last_status == MCE_NO_CONFIRMATION_EXPECTED))))


    # ........................................................................
    def configMotion(self, wavetable, gs, fpuset=None, soft_protection=True, check_protection=None,
                     allow_uninitialized=False, ruleset_version=DEFAULT_WAVEFORM_RULESET_VERSION,
                     warn_unsafe=True, verbosity=3):
        """
        Configures movement by sending a waveform table to a group of FPUs.
        Call signature is configMotion({ fpuid0 : {(asteps,bsteps), (asteps, bsteps), ...], fpuid1 : { ... }, ...}})

        When the 'protected' flag is set to False, bypass all
        hardware protection checks, which will allow to move a
        collided or uncalibrated FPU (even if the movement might damage
        the hardware).

        """
        assert isinstance(wavetable,dict), "wavetable must be a dictionary. Was it generated using gen_wf()?"
        if fpuset is None:
            fpuset = []
        fpuset = self.check_fpuset(fpuset)

        with self.lock:

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
                if warn_unsafe:
                    wmode=Range.Warn
                else:
                    wmode = None

            self._pre_config_motion_hook(wtable, gs, fpuset, wmode=wmode)
            update_config = False
            prev_gs = self._gd.getGridState()
            try:
                try:
                    try:
                        rval = self._gd.configMotion(wtable, gs, fpuset, allow_uninitialized,
                                                     ruleset_version)

                    except InvalidWaveformException as e:
                        print("%f: Error %s for wtable=%r" % (
                            time.time(), e, wtable), file=self.protectionlog)
                        print("Error %s for wtable=%r" % (
                            e, wtable))
                        raise

                    except InvalidStateException:
                        raise

                    update_config = True

                except (SocketFailure, CommandTimeout) as e:
                    # Transmission failure. Here, it is possible that some
                    # FPUs have finished loading valid data, but the
                    # process was not finished for all FPUs.
                    update_config = True
                    raise

            finally:
                if update_config:
                    # accept configured wavetable entries
                    for fpu_id, fpu in enumerate(gs.FPU):
                        if not wtable.has_key(fpu_id):
                            continue

                        if self.wavetable_was_received(wtable, gs, fpu_id, fpu):
                                self.last_wavetable[fpu_id] = wtable[fpu_id]
                        else:
                            print("Warning: waveform table for FPU %i was not confirmed" % fpu_id)
                            del wtable[fpu_id]

                        if self.__dict__.has_key("counters"):
                            self._update_error_counters(self.counters[fpu_id], prev_gs.FPU[fpu_id], fpu)

                    self._post_config_motion_hook(wtable, gs, fpuset)

        if (len(wtable.keys()) < self.config.num_fpus) and (verbosity > 0):
            num_forward,  num_reversed, num_movable = countMovableFPUs(gs)
            if num_reversed > 0:
                print("%i wave tables added: now %i out of %i FPUs configured to move, %i of them reversed." % (
                    len(wtable), num_movable, self.config.num_fpus, num_reversed))
            else:
                print("%i wave tables added: now %i out of %i FPUs configured to move." % (
                    len(wtable), num_movable, self.config.num_fpus))


        return rval

    def getCurrentWaveTables(self):
        with self.lock:
            if self.wavetables_incomplete:
                print("Warning: waveform upload failed or incomplete, "
                      "waveforms displayed may not be valid.")
            return self.last_wavetable.copy()

    def getReversed(self):
        with self.lock:
            return self.wf_reversed.copy()

    def set_wtable_reversed(self, fpu_id_list, is_reversed=False):
        with self.lock:
            if len(fpu_id_list) == 0:
                fpu_id_list = range(self.config.num_fpus)
            for fpu_id in fpu_id_list:
                self.wf_reversed[fpu_id] = is_reversed

    def _start_execute_motion_hook(self, gs, fpuset, initial_positions=None):
        pass

    def _cancel_execute_motion_hook(self, gs, fpuset,initial_positions=None):
        pass

    def _post_execute_motion_hook(self, gs, old_gs, move_gs, fpuset):
        pass


    def executeMotionB(self, gs, fpuset=None, sync_command=True):
        if fpuset is None:
            fpuset = []
        fpuset = self.check_fpuset(fpuset)

        if len(fpuset) == 0:
            fpuset = range(self.config.num_fpus)

        with self.lock:
            initial_positions = {}
            self._start_execute_motion_hook(gs, fpuset=fpuset, initial_positions=initial_positions)
            prev_gs = self._gd.getGridState() # get last FPU states and timeout counters
            try:
                try:
                    rv = self._gd.executeMotion(gs, fpuset, sync_command)
                except InvalidStateException as e:
                    self_cancel_execute_motion_hook(gs, fpuset, initial_positions=initial_positions)
                    raise
            finally:
                move_gs = self._gd.getGridState()
                try:
                    pingset = self.need_ping(gs, fpuset)
                    if len(pingset) > 0 :
                        self._pingFPUs(gs, pingset)
                except CommandTimeout:
                    pass
                self._post_execute_motion_hook(gs, prev_gs, move_gs, fpuset)

        return rv

    # ........................................................................
    def executeMotion(self, gs, fpuset=None, sync_command=True):
        if fpuset is None:
            fpuset = []
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
                try:
                    with SignalHandler() as sh:
                        while not is_ready:
                            rv = self._gd.waitExecuteMotion(gs, time_interval, fpuset)
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

    # ........................................................................
    def abortMotion(self, gs, fpuset=None, sync_command=True):
        if fpuset is None:
            fpuset = []
        fpuset = self.check_fpuset(fpuset)

        if not sync_command:
            warnings.warn("not using SYNC command")

        # this must not use locking - it can be sent from any thread by design
        try:
            prev_gs = self._gd.getGridState()
            rval = self._gd.abortMotion(gs, fpuset, sync_command)
        finally:
            if self.__dict__.has_key("counters"):
                for fpu_id, fpu in enumerate(gs.FPU):
                    self._update_error_counters(self.counters[fpu_id], prev_gs.FPU[fpu_id], fpu)

        return rval

    def _pre_free_beta_collision_hook(self, fpu_id,direction, gs, soft_protection=True):
        pass

    def _post_free_beta_collision_hook(self, fpu_id,direction, gs):
        pass

    # ........................................................................
    def freeBetaCollision(self, fpu_id, direction, gs, soft_protection=True):

        with self.lock:
            self._pre_free_beta_collision_hook(fpu_id,direction, gs,
                                               soft_protection=soft_protection)
            try:
                prev_gs = self._gd.getGridState()

                rv = self._gd.freeBetaCollision(fpu_id, direction, gs)
                status = gs.FPU[fpu_id].last_status
            finally:
                if self.__dict__.has_key("counters"):
                    for fpu_id, fpu in enumerate(gs.FPU):
                        self._update_error_counters(self.counters[fpu_id], prev_gs.FPU[fpu_id], fpu)


            self._post_free_beta_collision_hook(fpu_id, direction, gs)

        return rv, status

    # ........................................................................
    def enableBetaCollisionProtection(self, gs):
        try:
            prev_gs = self._gd.getGridState()
            rval = self._gd.enableBetaCollisionProtection(gs)
        finally:
            if self.__dict__.has_key("counters"):
                for fpu_id, fpu in enumerate(gs.FPU):
                    self._update_error_counters(self.counters[fpu_id], prev_gs.FPU[fpu_id], fpu)

        return rval


    def _pre_free_alpha_limit_breach_hook(self, fpu_id,direction, gs, soft_protection=True):
        pass

    def _post_free_alpha_limit_breach_hook(self, fpu_id,direction, gs):
        pass

    # ........................................................................
    def freeAlphaLimitBreach(self, fpu_id, direction, gs, soft_protection=True):

        with self.lock:
            self._pre_free_alpha_limit_breach_hook(fpu_id,direction, gs,
                                                   soft_protection=soft_protection)
            try:
                prev_gs = self._gd.getGridState()

                rv = self._gd.freeAlphaLimitBreach(fpu_id, direction, gs)
                status = gs.FPU[fpu_id].last_status
            finally:
                if self.__dict__.has_key("counters"):
                    for fpu_id, fpu in enumerate(gs.FPU):
                        self._update_error_counters(self.counters[fpu_id], prev_gs.FPU[fpu_id], fpu)


            self._post_free_alpha_limit_breach_hook(fpu_id, direction, gs)

        return rv, status

    # ........................................................................
    def enableAlphaLimitProtection(self, gs):
        try:
            prev_gs = self._gd.getGridState()
            rval = self._gd.enableAlphaLimitProtection(gs)
        finally:
            if self.__dict__.has_key("counters"):
                for fpu_id, fpu in enumerate(gs.FPU):
                    self._update_error_counters(self.counters[fpu_id], prev_gs.FPU[fpu_id], fpu)

        return rval

    # ........................................................................
    def reverseMotion(self, gs, fpuset=None, soft_protection=True, verbosity=3):
        if fpuset is None:
            fpuset = []
        fpuset = self.check_fpuset(fpuset)

        with self.lock:
            wtable = self.last_wavetable

            if soft_protection:
                wmode=Range.Error
            else:
                wmode=Range.Warn

            self._pre_reverse_motion_hook(wtable, gs, fpuset, wmode=wmode)
            try:
                prev_gs = self._gd.getGridState()
                rv = self._gd.reverseMotion(gs, fpuset)
            finally:
                if self.__dict__.has_key("counters"):
                    for fpu_id, fpu in enumerate(gs.FPU):
                        self._update_error_counters(self.counters[fpu_id], prev_gs.FPU[fpu_id], fpu)

            self._post_reverse_motion_hook(wtable, gs, fpuset)

        num_configured = len(fpuset) if (len(fpuset) != 0) else self.config.num_fpus
        if (num_configured < self.config.num_fpus) and (verbosity > 0):
            num_forward,  num_reversed, num_movable = countMovableFPUs(gs)
            if num_forward > 0:
                print("%i wave tables reversed: now %i out of %i FPUs configured to move, %i of them forward." % (
                    num_configured, num_movable, self.config.num_fpus, num_forward))
            else:
                print("%i wave tables reversed: now %i out of %i FPUs configured to move." % (
                    num_configured, num_movable, self.config.num_fpus))

        return rv

    def repeatMotion(self, gs, fpuset=None, soft_protection=True):
        if fpuset is None:
            fpuset = []
        fpuset = self.check_fpuset(fpuset)

        with self.lock:
            wtable = self.last_wavetable

            if soft_protection:
                wmode=Range.Error
            else:
                wmode=Range.Warn

            self._pre_repeat_motion_hook(wtable, gs, fpuset, wmode=wmode)
            try:
                prev_gs = self._gd.getGridState()
                rv = self._gd.repeatMotion(gs, fpuset)
            finally:
                if self.__dict__.has_key("counters"):
                    for fpu_id, fpu in enumerate(gs.FPU):
                        self._update_error_counters(self.counters[fpu_id], prev_gs.FPU[fpu_id], fpu)

            self._post_repeat_motion_hook(wtable, gs, fpuset)

        num_configured = len(fpuset) if (len(fpuset) != 0) else self.config.num_fpus
        if num_configured < self.config.num_fpus:
            num_forward,  num_reversed, num_movable = countMovableFPUs(gs)
            if num_reversed > 0:
                print("%i wave tables activated: now %i out of %i FPUs configured to move, %i of them reversed." % (
                    num_configured, num_movable, self.config.num_fpus, num_reversed))
            else:
                print("%i wave tables activated: now %i out of %i FPUs configured to move." % (
                    num_configured, num_movable, self.config.num_fpus))

        return rv

    # ........................................................................
    def countedAngles(self, gs, fpuset=None, show_uninitialized=False):
        """ Return alpha and beta angles (in degrees) for the given set of FPUs
            based on the motor step count relative to datum. The angles are
            only valid if the FPU has been referenced. Use show_uninitialized=True
            to return the step count angles regardless of the state of the
            referenced flag (it is up to the caller to decide if the counts can
            still be trusted).
        """
        if fpuset is None:
            fpuset = []
        fpuset = self.check_fpuset(fpuset)

        if len(fpuset) == 0:
            fpuset = range(self.config.num_fpus)

        try:
            prev_gs = self._gd.getGridState()

            self._pingFPUs(gs, fpuset=fpuset)
        finally:
            if self.__dict__.has_key("counters"):
                for fpu_id, fpu in enumerate(gs.FPU):
                    self._update_error_counters(self.counters[fpu_id], prev_gs.FPU[fpu_id], fpu)



        angles = fpu_commands.list_angles(gs, show_uninitialized=show_uninitialized,
                                          alpha_datum_offset=self.config.alpha_datum_offset)
        return [angles[k] for k in fpuset ]


    # ........................................................................
    def lockFPU(self, fpu_id, gs):

        with self.lock:
            try:
                prev_gs = self._gd.getGridState()

                rv = self._gd.lockFPU(fpu_id, gs)
                status = gs.FPU[fpu_id].last_status
            finally:
                if self.__dict__.has_key("counters"):
                    for fpu_id, fpu in enumerate(gs.FPU):
                        self._update_error_counters(self.counters[fpu_id], prev_gs.FPU[fpu_id], fpu)

        return rv, status

    # ........................................................................
    def unlockFPU(self, fpu_id, gs):

        with self.lock:
            try:
                prev_gs = self._gd.getGridState()

                rv = self._gd.unlockFPU(fpu_id, gs)
                status = gs.FPU[fpu_id].last_status
            finally:
                if self.__dict__.has_key("counters"):
                    for fpu_id, fpu in enumerate(gs.FPU):
                        self._update_error_counters(self.counters[fpu_id], prev_gs.FPU[fpu_id], fpu)

        return rv, status

    # ........................................................................
    def enableMove(self, fpu_id, gs):

        with self.lock:
            try:
                prev_gs = self._gd.getGridState()

                rv = self._gd.enableMove(fpu_id, gs)

                status = gs.FPU[fpu_id].last_status
            finally:
                if self.__dict__.has_key("counters"):
                    for fpu_id, fpu in enumerate(gs.FPU):
                        self._update_error_counters(self.counters[fpu_id], prev_gs.FPU[fpu_id], fpu)

        return rv, status

    # ........................................................................
    def checkIntegrity(self, gs, fpuset=None, verbose=True):
        if fpuset is None:
            fpuset = []
        fpuset = self.check_fpuset(fpuset)

        try:
            prev_gs = self._gd.getGridState()
            rval= self._gd.checkIntegrity(gs, fpuset)
        finally:
            if self.__dict__.has_key("counters"):
                for fpu_id, fpu in enumerate(gs.FPU):
                    self._update_error_counters(self.counters[fpu_id], prev_gs.FPU[fpu_id], fpu)

        if verbose:
            for fpu_id, fpu in enumerate(gs.FPU):
                if fpu_in_set(fpu_id, fpuset):
                    print("FPU %i firmware checksum: 0x%0x" % (
                        fpu_id,
                        fpu.crc32))

        return rval


# ---------------------------------------------------------------------------
def get_duplicates(idlist):
    duplicates = {}
    for n, id in enumerate(idlist):
        if idlist.count(id) > 1:
            duplicates[n] = id
    return duplicates



# ===========================================================================

class GridDriver(UnprotectedGridDriver):
    """
    
    The GridDriver class adds a software protection layer on top
    of the basic UnprotectedGridDriver class.
    
    """
    def __init__(self, NFPUS, env=None, mockup=False, *args, **kwargs):
        if env is None:
            env = open_database_env(mockup=mockup)

        if env is None:
            raise ValueError("The environment variable FPU_DATABASE needs to"
                             " be set to the directory path of the LMDB position database!")

        self.env = env
        #super(GridDriver,self).__init__(*args, **kwargs)
        UnprotectedGridDriver.__init__(self, NFPUS, *args, **kwargs)


        with self.lock:
            # position intervals which are being configured by configMotion
            self.configuring_ranges = {}
            # position intervals which have successfully been configured
            # and will become valid with next executeMotion
            self.configured_ranges = {}

    def _post_connect_hook(self, config):
        self.fpudb = self.env.open_db(ProtectionDB.dbname)
        self.healthlog = self.env.open_db(HealthLogDB.dbname)

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
        maxaretries = {}
        aretries_cw = {}
        aretries_acw = {}
        counters = {}
        in_dicts = { ProtectionDB.alpha_positions : apositions,
                     ProtectionDB.beta_positions : bpositions,
                     ProtectionDB.waveform_table : wtabs, ProtectionDB.waveform_reversed : wf_reversed,
                     ProtectionDB.alpha_limits : alimits, ProtectionDB.beta_limits : blimits,
                     ProtectionDB.free_alpha_retries : maxaretries,
                     ProtectionDB.alpha_retry_count_cw : aretries_cw,
                     ProtectionDB.alpha_retry_count_acw : aretries_acw,
                     ProtectionDB.free_beta_retries : maxbretries,
                     ProtectionDB.beta_retry_count_cw : bretries_cw,
                     ProtectionDB.beta_retry_count_acw : bretries_acw,
                     ProtectionDB.counters : counters}

        a_caloffsets = []
        b_caloffsets = []

        alpha_datum_offset=config.alpha_datum_offset

        for fpu_id, fpu in enumerate(grid_state.FPU):
            a_caloffsets.append(Interval(0.0))
            b_caloffsets.append(Interval(0.0))

            with self.env.begin(db=self.fpudb) as txn:
                for subkey in [ ProtectionDB.alpha_positions,
                                ProtectionDB.beta_positions,
                                ProtectionDB.waveform_table,
                                ProtectionDB.waveform_reversed,
                                ProtectionDB.alpha_limits,
                                ProtectionDB.beta_limits,
                                ProtectionDB.free_alpha_retries,
                                ProtectionDB.alpha_retry_count_cw,
                                ProtectionDB.alpha_retry_count_acw,
                                ProtectionDB.free_beta_retries,
                                ProtectionDB.beta_retry_count_cw,
                                ProtectionDB.beta_retry_count_acw,
                                ProtectionDB.counters ]:

                    val = ProtectionDB.getField(txn, fpu, subkey)

                    if val == None:
                        raise ethercanif.ProtectionError("serial number {0!r} not found in position database"
                                                         " - run fpu-admin.py to create entry".format(fpu.serial_number))
                    else:
                        if subkey in [ProtectionDB.alpha_positions, ProtectionDB.alpha_limits]:
                            in_dicts[subkey][fpu_id] = val + alpha_datum_offset
                        else:
                            in_dicts[subkey][fpu_id] = val

        target_positions = {}
        for fpu_id, fpu in enumerate(grid_state.FPU):
            target_positions[fpu_id] = (apositions[fpu_id].copy(),bpositions[fpu_id].copy())

        self.apositions = apositions
        self.bpositions = bpositions
        self.last_wavetable = wtabs
        self.wf_reversed = wf_reversed
        self.alimits = alimits
        self.blimits = blimits
        self.a_caloffsets = a_caloffsets
        self.b_caloffsets = b_caloffsets
        self.maxaretries = maxaretries
        self.aretries_cw = aretries_cw
        self.aretries_acw = aretries_acw
        self.maxbretries = maxbretries
        self.bretries_cw = bretries_cw
        self.bretries_acw = bretries_acw
        self.counters = counters
        self._last_counters = counters.copy()
        self.target_positions = target_positions
        self.configuring_targets = {}
        self.configured_targets = {}


        # query positions and compute offsets, if FPUs have been resetted.
        # This assumes that the stored positions are correct.
        self._pingFPUs(grid_state)
        self._reset_hook(grid_state, grid_state)
        self._refresh_positions(grid_state, store=False)


    def _alpha_angle(self, fpu):
        alpha_underflow = (fpu.alpha_steps == ALPHA_UNDERFLOW_COUNT)
        alpha_overflow = (fpu.alpha_steps == ALPHA_OVERFLOW_COUNT)
        return (fpu.alpha_steps / StepsPerDegreeAlpha + self.config.alpha_datum_offset,
                alpha_underflow,
                alpha_overflow)

    def _beta_angle(self, fpu):
        beta_underflow = (fpu.beta_steps == BETA_UNDERFLOW_COUNT)
        beta_overflow = (fpu.beta_steps == BETA_OVERFLOW_COUNT)
        return (fpu.beta_steps / StepsPerDegreeBeta,
                beta_underflow,
                beta_overflow)

    def _reset_hook(self, old_state, new_state, fpuset=None):
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
        if fpuset is None:
            fpuset = []
        self.readSerialNumbers(new_state, fpuset=fpuset)
        print("%f: Resetting fpu set %r" % (time.time(), fpuset),  file=self.protectionlog)

        for fpu_id, fpu in enumerate(new_state.FPU):
            if not fpu_in_set(fpu_id, fpuset):
                continue

            # these offsets are the difference between the calibrated
            # angle and the uncalibrated angle - after a findDatum,
            # they are set to zero
            alpha_angle, a_underflow, a_overflow = self._alpha_angle(fpu)
            if a_underflow or a_overflow:
                print("_reset_hook(): Warning: alpha step counter is at underflow / overflow value")
            self.a_caloffsets[fpu_id] = self.apositions[fpu_id] - alpha_angle

            beta_angle, b_underflow, b_overflow = self._beta_angle(fpu)
            if b_underflow or b_overflow:
                print("Beta step counter is at underflow / overflow value")
            self.b_caloffsets[fpu_id] = self.bpositions[fpu_id] - beta_angle

            if self.configured_ranges.has_key(fpu_id):
                del self.configured_ranges[fpu_id]

            if self.last_wavetable.has_key(fpu_id):
                del self.last_wavetable[fpu_id]

        print("%f: _reset_hook(): New a_caloffsets = %r, b_cal_offsets=%r" % (
            time.time(), self.a_caloffsets, self.b_caloffsets), file=self.protectionlog)


    def _reset_counter_hook(self, alpha_target, beta_target, old_state, new_state, fpuset=None):
        """similar to reset_hook, but run after resetStepCounter and
        only updating the caloffsets.
        """
        if fpuset is None:
            fpuset = []

        for fpu_id, fpu in enumerate(new_state.FPU):
            if not fpu_in_set(fpu_id, fpuset):
                continue

            # these offsets are the difference between the calibrated
            # angle and the uncalibrated angle - after a findDatum,
            # they are set to zero
            alpha_angle, a_underflow, a_overflow = self._alpha_angle(fpu)
            if a_underflow or a_overflow:
                print("_reset_counter_hook(): Warning: reported alpha step counter is at underflow / overflow value")
                self.a_caloffsets[fpu_id] = self.apositions[fpu_id] - alpha_target
            else:
                self.a_caloffsets[fpu_id] = self.apositions[fpu_id] - alpha_angle

            beta_angle, b_underflow, b_overflow = self._beta_angle(fpu)
            if b_underflow or b_overflow:
                print("_reset_counter_hook(): Warning: reported beta step counter is at underflow / overflow value")
                self.b_caloffsets[fpu_id] = self.bpositions[fpu_id] - beta_target
            else:
                self.b_caloffsets[fpu_id] = self.bpositions[fpu_id] - beta_angle

            print("reset_counter_hook(): FPU %i: alpha counted angle = %f, offset = %r" % (fpu_id, alpha_angle,
                                                                                             self.a_caloffsets[fpu_id]))
            print("reset_counter_hook(): FPU %i: beta counted angle = %f, offset = %r" % (fpu_id, beta_angle,
                                                                                            self.b_caloffsets[fpu_id]))

        print("%f: _reset_counter_hook(): New a_caloffsets = %r, b_cal_offsets=%r" % (
            time.time(), self.a_caloffsets, self.b_caloffsets), file=self.protectionlog)

    # ........................................................................
    def trackedAngles(self, gs=None, fpuset=None, show_offsets=False, active=False, retrieve=False, display=True):
        """lists tracked angles, offset, and waveform span
        for configured waveforms, for each FPU"""

        if fpuset is None:
            fpuset = []
        fpuset = self.check_fpuset(fpuset)

        with self.lock:
            if len(fpuset) == 0:
                fpuset = range(self.config.num_fpus)

            if gs is None:
                gs = self.getGridState()
            if retrieve:
                return [ (self.apositions[fi],self.bpositions[fi]) for fi in fpuset ]
            else:
                s = ""
                for fi in fpuset:
                    fpu = gs.FPU[fi]
                    aangle, a_underflow, a_overflow = self._alpha_angle(fpu)
                    bangle, b_underflow, b_overflow = self._beta_angle(fpu)
                    if active:
                        wf_arange, wf_brange = self.configured_ranges.get(fi, (Interval(), Interval()))
                        prefix="active"
                    else:
                        wf_arange, wf_brange = self.configuring_ranges.get(fi, (Interval(), Interval()))
                        prefix="last"

                    if show_offsets and (gs != None):
                        if a_underflow:
                            aflag = "!u"
                        elif a_overflow:
                            aflag = "!o"
                        else:
                            aflag = ""

                        if b_underflow:
                            bflag = "!u"
                        elif a_overflow:
                            bflag = "!o"
                        else:
                            bflag = ""

                        s += ("FPU #{}: angle = ({!s}, {!s}), offsets = ({!s}, {!s}),"
                              " stepcount angle= ({!s}{!s}, {!s}{!s}), {!s}_wform_range=({!s},{!s})\n".
                              format(fi, self.apositions[fi],self.bpositions[fi],
                                     self.a_caloffsets[fi], self.b_caloffsets[fi],
                                     aangle, aflag, bangle, bflag,
                                     prefix, wf_arange, wf_brange))
                    else:
                        s += ("FPU #{}: angle = ({!s}, {!s}), {!s}_wform_range=({!s},{!s})\n".
                              format(fi, self.apositions[fi],self.bpositions[fi],
                                     prefix, wf_arange, wf_brange))

                print("%f : " % time.time(), file=self.protectionlog, end='')
                print(s, file=self.protectionlog)
                if display:
                    print(s, file=sys.stdout)
                else:
                    return s

    def _update_apos(self, txn, fpu, fpu_id, new_apos, store=True):
        self.apositions[fpu_id] = new_apos
        if store:
            ProtectionDB.put_alpha_position(txn, fpu, new_apos, self.config.alpha_datum_offset)

    def _update_bpos(self, txn, fpu, fpu_id, new_bpos, store=True):
        self.bpositions[fpu_id] = new_bpos
        if store:
            ProtectionDB.put_beta_position(txn, fpu, new_bpos)



    def _refresh_positions(self, grid_state, store=True, fpuset=None):
        """This is to be run after any movement.

        Computes new current positions from step count
        and offsets (if set), and stores them to database.

        Note: We do not try to recognize a reset behind the back of
        the driver as there is no reliable indicator. Protocol 1 does
        not support to recognize that. Protocol 2 allows to recognize
        it, so this might change.

        """
        if fpuset is None:
            fpuset = []

        inconsistency_abort = False
        with self.env.begin(db=self.fpudb, write=True) as txn:
            for fpu_id, fpu in enumerate(grid_state.FPU):
                if not fpu_in_set(fpu_id, fpuset):
                    continue

                if not fpu.ping_ok:
                    # position is not known. This flag is set by a
                    # successful ping or datum response, and cleared by
                    # every movement as well as all movement time-outs
                    continue

                counted_alpha_angle, a_underflow, a_overflow  = self._alpha_angle(fpu)
                new_alpha = self.a_caloffsets[fpu_id] + counted_alpha_angle

                counted_beta_angle, b_underflow, b_overflow  = self._beta_angle(fpu)
                new_beta = self.b_caloffsets[fpu_id] + counted_beta_angle
                # compute alpha and beta position intervals,
                # and store both to DB
                a_target, b_target = self.target_positions[fpu_id]
                if a_underflow or a_overflow:
                    print("Warning: _refresh_positions(): FPU id %i: using stored alpha target value"
                          " to bypass counter underflow/overflow" % fpu_id)
                    new_alpha = a_target
                if b_underflow or b_overflow :
                    print("Warning: _refresh_positions(): FPU id %i: using stored beta target value"
                          " to bypass counter underflow/overflow" % fpu_id)
                    new_beta = b_target
                self.target_positions[fpu_id] = (new_alpha, new_beta)

                if ((not self.apositions[fpu_id].contains(new_alpha, tolerance=0.25))
                    or (not self.bpositions[fpu_id].contains(new_beta, tolerance=0.25))) :

                    print("\a\a\aERROR: RECEIVED POSITION = (%r, %r) FOR FPU %i OUTSIDE OF TRACKED RANGE = (%r, %r)."
                          " step counters = (%r,%r), offsets = (%r,%r)"  % (
                              new_alpha, new_beta, fpu_id, self.apositions[fpu_id], self.bpositions[fpu_id],
                              fpu.alpha_steps, fpu.beta_steps, self.a_caloffsets[fpu_id], self.b_caloffsets[fpu_id]))
                    inconsistency_abort = True


                self._update_apos(txn, fpu, fpu_id, new_alpha, store=store)
                self._update_bpos(txn, fpu, fpu_id, new_beta, store=store)

        self.env.sync()

        print("%f: refresh_positions(): new apositions = %r, bpositions = %r" % (
            time.time(), self.apositions, self.bpositions),
              file=self.protectionlog)

        if inconsistency_abort:
            raise HardwareProtectionError("Invalid step counter. FPU was possibly moved or "
                                          "power-cycled circumventing the running driver.\n"
                                          "WARNING: Position database needs to be re-initialized by measurement.")





    # ........................................................................
    def pingFPUs(self, grid_state, fpuset=None):
        if fpuset is None:
            fpuset = []
        fpuset = self.check_fpuset(fpuset)

        with self.lock:
            prev_gs = self._gd.getGridState()
            try:
                self._pingFPUs(grid_state, fpuset=fpuset)
                self._refresh_positions(grid_state, fpuset=fpuset)
            finally:
                for fpu_id, fpu in enumerate(grid_state.FPU):
                    self._update_error_counters(self.counters[fpu_id], prev_gs.FPU[fpu_id], fpu)


    def __del__(self):
        # if connection is live, gets and stores
        # the positions before exiting
        if self.__dict__.has_key("lock"):
            with self.lock:
                grid_state = self.getGridState()
                if grid_state.interface_state == DS_CONNECTED:
                    # fetch current positions
                    self._pingFPUs(grid_state)

                if self.__dict__.has_key("a_caloffsets") and self.__dict__.has_key("b_caloffsets"):
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
        new_range.assignCombine(x)

        if not xlimits.contains(x):

            if wmode == Range.Error:
                print("%f: Error: wavetable defines unsafe path for FPU %i, at step %i, arm %s  (angle=%r, limits=%r)" %(
                    time.time(), fpu_id, stepnum, arm_name, x, xlimits), file=self.protectionlog)
                raise ProtectionError("For FPU %i, at step %i, arm %s, the wavetable"
                                      " steps outside the tracked safe limits (angle=%r, limits=%r)" %(
                    fpu_id, stepnum, arm_name, x, xlimits))

            elif wmode == Range.Warn:
                print("%f: Warning: wavetable defines unsafe path for FPU %i, at step %i, arm %s  (angle=%r, limits=%r)" %(
                    time.time(),
                    fpu_id, stepnum, arm_name, x, xlimits), file=self.protectionlog)

                print("Warning: wavetable defines unsafe path for FPU %i, at step %i, arm %s  (angle=%r, limits=%r)" %(
                    fpu_id, stepnum, arm_name, x, xlimits))

    # ........................................................................

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
    movements expand the region of uncertainty, and regular
    terminations, pings and datum responses collapse it to an interval
    of length zero.

    If a configMotion command is issued, this defines a tentative
    future interval, which becomes valid once the corresponding
    movement is started.  If a reverseMotion is issued, this generates
    an interval with the opposite sign of movements.

    Tracking the region of uncertainty is not always required, as ping
    commands can often be used to narrow the possible range of
    positions down. But doing this has, first, the disadvantage that
    special-casing is needed for handling an abortMotion or collision
    situation. Yet exactly these are situations which need to be
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
        configuring_ranges = {}
        configuring_targets = {}
        for fpu_id, wt_row in wtable.items():
            if not fpu_in_set(fpu_id, fpuset):
                continue

            fpu = gs.FPU[fpu_id]

            alimits= self.alimits[fpu_id]
            blimits= self.blimits[fpu_id]

            alpha0 = self.apositions[fpu_id]
            beta0 = self.bpositions[fpu_id]

            wf_arange = Interval(alpha0)
            wf_brange = Interval(beta0)

            try:

                self._check_allowed_range(fpu_id, -1, "alpha", alimits, alpha0,
                                         wf_arange,  wmode)

                self._check_allowed_range(fpu_id, -1, "beta",  blimits, beta0,
                                         wf_brange, wmode)

                asteps = 0
                bsteps = 0
                # if the waveform is reversed, we need to
                # go backwards with the check!
                if sign == 1:
                    step_sequence = range(len(wt_row))
                elif sign == -1:
                    step_sequence = range(len(wt_row) -1, -1, -1)
                else:
                    assert(0)


                for step_num in step_sequence:
                    entry = wt_row[step_num]
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

                configuring_ranges[fpu_id] = (wf_arange, wf_brange)
                configuring_targets[fpu_id] = (alpha_sect, beta_sect)
            except ProtectionError as e:
                print("%f: Error %s for fpu %i, wtable=%r" % (
                    time.time(), e, fpu_id, wt_row), file=self.protectionlog)
                print("Error %s for fpu %i, wtable=%r" % (
                    e, fpu_id, wt_row))
                raise

        # this is the list of alpha/beta position intervals that
        # will become valid if and after an executeMotion is started,
        # and before the new positions can be retrieved via ping
        self.configuring_ranges.update(configuring_ranges)
        self.configuring_targets.update(configuring_targets)

    def _pre_config_motion_hook(self, wtable, gs, fpuset, wmode=Range.Error):
        self._check_and_register_wtable(wtable, gs, fpuset, wmode, sign=1)

    # this can possibly be deleted
    # but do we want to store the full wavetable ?
    def _save_wtable_direction(self, fpu_id_list, is_reversed=False, grid_state=None):
        with self.env.begin(db=self.fpudb, write=True) as txn:
            for fpu_id in fpu_id_list:
                fpu = grid_state.FPU[fpu_id]

                self.wf_reversed[fpu_id] = is_reversed
                ProtectionDB.store_reversed(txn, fpu, is_reversed)


    def _post_config_motion_hook(self, wtable, gs, fpuset, partial_config=False):
        # update ranges that will become valid once executeMotion is started
        for fpu_id, rentry in self.configuring_ranges.items():

            if not fpu_in_set(fpu_id, fpuset):
                continue

            fpu = gs.FPU[fpu_id]
            if self.wavetable_was_received(wtable, gs, fpu_id, fpu):
                self.configured_ranges[fpu_id] = rentry
                # store actual target position
                self.configured_targets[fpu_id] = self.configuring_targets[fpu_id]


        self._save_wtable_direction(wtable.keys(), is_reversed=False, grid_state=gs)
        # save the changed waveform tables
        with self.env.begin(db=self.fpudb, write=True) as txn:
            for fpu_id, wentry in wtable.items():
                fpu = gs.FPU[fpu_id]
                ProtectionDB.storeWaveform(txn, fpu, wentry)

        print("%f: _post_config_motion_hook(): configured_targets =%r" % (time.time(), self.configured_targets),
              file=self.protectionlog)



    def _pre_repeat_motion_hook(self, wtable, gs, fpuset, wmode=Range.Error):
        self._check_and_register_wtable(wtable, gs, fpuset, wmode, sign=1)


    def _post_repeat_motion_hook(self, wtable, gs, fpuset):
        # update ranges that become valid once executeMotion is started
        idlist = []
        for fpu_id, rentry in self.configuring_ranges.items():
            if not fpu_in_set(fpu_id, fpuset):
                continue
            fpu = gs.FPU[fpu_id]
            if self.wavetable_was_received(wtable, gs, fpu_id, fpu, allow_unconfirmed=True):
                self.configured_ranges[fpu_id] = rentry
                idlist.append(fpu_id)
                self.configured_targets[fpu_id] = self.configuring_targets[fpu_id]
        self._save_wtable_direction(idlist, is_reversed=False, grid_state=gs)
        print("%f: _post_repeat_motion_hook(): configured_targets =%r" % (
            time.time(), self.configured_targets),
              file=self.protectionlog)

    def _pre_reverse_motion_hook(self, wtable, gs, fpuset, wmode=Range.Error):
        self._check_and_register_wtable(wtable, gs, fpuset, wmode, sign=-1)


    def _post_reverse_motion_hook(self, wtable, gs, fpuset):
        # update ranges that become valid once executeMotion is started
        idlist = []
        for fpu_id, rentry in self.configuring_ranges.items():
            if not fpu_in_set(fpu_id, fpuset):
                continue
            fpu = gs.FPU[fpu_id]
            if self.wavetable_was_received(wtable, gs, fpu_id, fpu,
                                           allow_unconfirmed=True,
                                           target_state=FPST_READY_REVERSE):
                self.configured_ranges[fpu_id] = rentry
                self.configured_targets[fpu_id] = self.configuring_targets[fpu_id]
                idlist.append(fpu_id)

        self._save_wtable_direction(idlist, is_reversed=True, grid_state=gs)
        print("%f: _post_reverse_motion_hook(): configured_targets =%r" % (
            time.time(), self.configured_targets),
              file=self.protectionlog)

    def _update_counters_execute_motion(self, fpu_id, fpu_counters, wtable, is_reversed, cancel=False):
        sum_beta_steps = 0
        sum_alpha_steps = 0
        alpha_reversals = 0
        beta_reversals = 0
        alpha_starts = 0
        beta_starts = 0
        alpha_sign = 0
        beta_sign = 0
        last_asteps = 0
        last_bsteps = 0

        if is_reversed:
            rsign = -1
        else:
            rsign = 1

        if cancel:
            fpu_counters.update(self._last_counters[fpu_id])

        alpha_lsign = fpu_counters["sign_alpha_last_direction"]
        beta_lsign = fpu_counters["sign_beta_last_direction"]

        for asteps, bsteps in wtable:
            asteps *= rsign
            bsteps *= rsign

            sum_alpha_steps += abs(asteps)
            sum_beta_steps += abs(bsteps)

            alpha_sign = sign(asteps)
            if alpha_sign != 0:
                alpha_nzsign = alpha_sign

                if alpha_lsign != alpha_nzsign:
                    if alpha_lsign != 0:
                        alpha_reversals += 1
                    alpha_lsign = alpha_nzsign

            beta_sign = sign(bsteps)
            if beta_sign != 0:
                beta_nzsign = beta_sign

                if beta_lsign != beta_nzsign:
                    if beta_lsign != 0:
                        beta_reversals += 1
                    beta_lsign = beta_nzsign

            if (last_asteps == 0) and (asteps != 0):
                alpha_starts += 1

            if (last_bsteps == 0) and (bsteps != 0):
                beta_starts += 1

            last_asteps = asteps
            last_bsteps = bsteps

        # update sums for that FPU

        # store values for case of subsequent cancellation
        self._last_counters[fpu_id] = fpu_counters

        fpu_counters["executed_waveforms" ] += 1
        fpu_counters["total_beta_steps" ] += sum_beta_steps
        fpu_counters["total_alpha_steps" ] += sum_alpha_steps
        fpu_counters["alpha_direction_reversals" ] += alpha_reversals
        fpu_counters["beta_direction_reversals" ] += beta_reversals
        fpu_counters["sign_alpha_last_direction" ] = alpha_lsign
        fpu_counters["sign_beta_last_direction" ] =  beta_lsign
        fpu_counters["alpha_starts" ] += alpha_starts
        fpu_counters["beta_starts" ] += beta_starts



    def _start_execute_motion_hook(self, gs, fpuset, initial_positions=None):
        """This runs before executeMotion command is started. After that
        point, the FPU in fpuset should be moving within the ranges
        set by the last config_motion, repeat_motion or reverse_motion
        command. These ranges are set as the intervals which define
        the possible positions until the command has finished.

        initial_positions is a dictionary which is filled with the
        positions as (alpha,beta) tuples at the time the command was
        called.

        """

        # initial_positions has to be a dict
        initial_positions.clear()

        self._pingFPUs(gs, fpuset=fpuset)

        with self.env.begin(db=self.fpudb, write=True) as txn:
            for fpu_id, fpu in enumerate(gs.FPU):
                if not fpu_in_set(fpu_id, fpuset):
                    continue

                # Needs to make a copy here because otherwise the
                # referenced objects are mutated later. Duh.
                initial_positions[fpu_id] = (self.apositions[fpu_id].copy(),
                                             self.bpositions[fpu_id].copy())
                # copy configured alpha and beta position intervals, and
                # store both to DB
                if self.configured_ranges.has_key(fpu_id):
                    (arange, brange) = self.configured_ranges[fpu_id]
                    self._update_apos(txn, fpu, fpu_id, arange)
                    self._update_bpos(txn, fpu, fpu_id,  brange)
                    # update target position which is used in case of step counter overflow
                    self.target_positions[fpu_id] = self.configured_targets[fpu_id]

                    self._update_counters_execute_motion(fpu_id, self.counters[fpu_id],
                                                         self.last_wavetable[fpu_id],
                                                         self.wf_reversed[fpu_id])

                    ProtectionDB.put_counters(txn, fpu, self.counters[fpu_id])

        self.env.sync()


    def _cancel_execute_motion_hook(self, gs, fpuset, initial_positions=None):
        """This hook cancel registering an executeMotion command which was
        rejected by the driver, before an actual movement was started.

        """

        with self.env.begin(db=self.fpudb, write=True) as txn:
            for fpu_id in initial_positions.keys():
                if not fpu_in_set(fpu_id, fpuset):
                    continue
                apos, bpos = initial_positions[fpu_id]
                fpu = gs.FPU[fpu_id]
                self._update_apos(txn, fpu, fpu_id, apos)
                self._update_bpos(txn, fpu, fpu_id,  bpos)
                self.target_positions[fpu_id] = (apos, bpos)

                if self.configured_ranges.has_key(fpu_id):
                    self._update_counters_execute_motion(fpu_id, self.counters[fpu_id],
                                                         self.last_wavetable[fpu_id],
                                                         self.wf_reversed[fpu_id],
                                                         cancel=True)
                ProtectionDB.put_counters(txn, fpu, self.counters[fpu_id])

        self.env.sync()
        print("%f: _cancel_execute_motion_hook(): Movement cancelled" % time.time(),
              file=self.protectionlog)



    def _update_error_counters(self, fpu_counters, prev_fpu, moved_fpu, datum_cmd=False):

        if moved_fpu.beta_collision:
            fpu_counters["collisions"]  += 1
        if (moved_fpu.state == FPST_OBSTACLE_ERROR) and moved_fpu.at_alpha_limit:
             fpu_counters["limit_breaches"] += 1

        if moved_fpu.timeout_count != prev_fpu.timeout_count:
            diff = moved_fpu.timeout_count - prev_fpu.timeout_count
            if diff < 0:
                # the underlying unsigned 16-bit value has wrapped around and needs to be corrected
                diff += 1 << 16
            fpu_counters["can_timeout"] += diff

        if (moved_fpu.last_status == MCE_ERR_DATUM_TIME_OUT) or ((moved_fpu.last_status == MCE_COMMAND_TIMEDOUT) and datum_cmd):
            fpu_counters["datum_timeout"] += 1

        if (moved_fpu.last_status == MCE_COMMAND_TIMEDOUT) and (moved_fpu.last_command == CCMD_EXECUTE_MOTION):
            fpu_counters["movement_timeout"] += 1



    def _post_execute_motion_hook(self, gs, old_gs, move_gs, fpuset):
        """This runs after both an executeMotion has run, and *also*
        a ping has returned successfully."""

        # What do we here if FPUs are still moving
        # (this would happen in case of an error)?
        # Solution for now: wait.
        for k in range(50):
            if (gs.Counts[FPST_MOVING] > 0) or (gs.Counts[FPST_DATUM_SEARCH] > 0):
                print("_post_execute_motion_hook(): Waiting for movement to finish in order to retrieve reached positions..")
                try:
                    fpuset_refresh = []
                    for fpu_id, fpu in enumerate(gs.FPU):
                        if fpu.state in [FPST_MOVING, FPST_DATUM_SEARCH]:
                            fpuset_refresh.append(fpu_id)
                    if len(fpuset_refresh) == 0:
                        break
                    fpuset_ping_notok = self.need_ping(gs, fpuset)
                    fpuset_refresh.extend(fpuset_ping_notok)
                    time.sleep(0.2)
                    self._pingFPUs(gs, fpuset=fpuset_refresh)
                except CommandTimeout:
                    break
            else:
                break
        # The assumption here is that the offsets did not change, even
        # if the waveform execution was aborted. This might not be
        # correct in a sense valid for science measurements, because
        # collisions would compromise the precision of step counts,
        # but for protection purposes this should be OK.
        #
        # Thus, use the step counter positions to update the location.
        # If the FPU step counters are in underflow / overflow, use
        # the registered target positions, to continue tracking.
        # But, roll back the latter to the movement range for an FPU, if
        # that FPU is not yet at target.
        for fpu_id in fpuset:
            if gs.FPU[fpu_id].state != FPST_RESTING:
                # print("_post_execute_motion_hook(): Retaining interval positions for FPU id %i" % fpu_id)
                self.target_positions[fpu_id] = (self.apositions[fpu_id].copy(),
                                                 self.bpositions[fpu_id].copy())

        self._refresh_positions(gs, fpuset=fpuset)

        with self.env.begin(db=self.fpudb, write=True) as txn:
            for fpu_id in fpuset:

                moved_fpu = move_gs.FPU[fpu_id]
                prev_fpu = old_gs.FPU[fpu_id]
                self._update_error_counters(self.counters[fpu_id], prev_fpu, moved_fpu)
                ProtectionDB.put_counters(txn, moved_fpu, self.counters[fpu_id])


        # clear wavetable spans for the addressed FPUs - they are not longer valid
        for k in self.configured_ranges.keys():
            if fpu_in_set(k, fpuset):
                del self.configured_ranges[k]


    def _allow_find_datum_hook(self,gs, search_modes, selected_arm=None, fpuset=None,
                              support_uninitialized_auto=True):
        """This function checks whether a datum search is safe, and throws an
        exception if not. It does that based on the stored
        position.

        """
        if fpuset is None:
            fpuset = []

        # get fresh ping data
        self._pingFPUs(gs, fpuset=fpuset)

        acw_range = Interval(0, Inf)
        cw_range = Interval(-Inf, 0)
        for fpu_id, fpu in enumerate(gs.FPU):
            if not fpu_in_set(fpu_id, fpuset):
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

                    if (not fpu.beta_was_referenced) or (fpu.state == FPST_UNINITIALIZED) :
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

    def _finished_find_datum_hook(self, prev_gs, datum_gs, search_modes=None, fpuset=None,
                                  was_cancelled=False, initial_positions=None, selected_arm=None):
        if fpuset is None:
            fpuset = []
            
        if initial_positions is None:
            initial_positions = {}

        # FIXME: check if the next block is still needed
        fpuset_refresh = []
        for fpu_id, fpu in enumerate(datum_gs.FPU):
            if fpu.state in [FPST_MOVING, FPST_DATUM_SEARCH] or (not fpu.ping_ok):
                fpuset_refresh.append(fpu_id)

        if len(fpuset_refresh) > 0:
            time.sleep(0.1)
            self._pingFPUs(datum_gs, fpuset=fpuset_refresh)


        with self.env.begin(db=self.fpudb, write=True) as txn:

            for fpu_id, datum_fpu in enumerate(datum_gs.FPU):
                if not fpu_in_set(fpu_id, fpuset):
                    continue

                if (len(search_modes) != 0) and (not search_modes.has_key(fpu_id)):
                    continue


                # set position intervals to zero, and store in DB
                if (datum_fpu.alpha_was_referenced) and (datum_fpu.alpha_steps == 0):
                    self.a_caloffsets[fpu_id] = Interval(0)
                    self._update_apos(txn, datum_fpu, fpu_id, Interval(0.0) + self.config.alpha_datum_offset)

                    # reset retry count for freeAlphaLimitBreach, because
                    # we have reached a safe position.

                    if self.aretries_acw[fpu_id] > 0:
                        #print("Resetting retry count for anti-clockwise freeAlphaLimitBreach movements.")
                        clockwise = False
                        self.aretries_acw[fpu_id] = 0
                        ProtectionDB.store_aretry_count(txn, datum_fpu, clockwise, 0)
                    if self.aretries_cw[fpu_id] > 0:
                        #print("Resetting retry count for clockwise freeAlphaLimitBreach movements.")
                        clockwise = True
                        self.aretries_cw[fpu_id] = 0
                        ProtectionDB.store_aretry_count(txn, datum_fpu, clockwise, 0)
                else:
                    ## If ping_ok is set, we assume that even if the
                    ## datum operation itself did not succeed, the
                    ## step counter was successfully retrieved by a
                    ## subsequent ping, and the offset is unchanged.
                    if datum_fpu.ping_ok and (selected_arm in [DASEL_ALPHA, DASEL_BOTH]):
                        alpha_angle, a_underflow, a_overflow = self._alpha_angle(datum_fpu)
                        if a_underflow or a_overflow:
                            # we know only we are in the interval between the
                            # overflow / underflow value and the datum position
                            a_int = (Interval(alpha_angle) + self.a_caloffsets[fpu_id]
                            ).extend(0.0 + self.config.alpha_datum_offset)

                        else:
                            a_int = Interval(alpha_angle) + self.a_caloffsets[fpu_id]

                        self._update_apos(txn, datum_fpu, fpu_id, a_int)

                if (datum_fpu.beta_was_referenced) and (datum_fpu.beta_steps == 0):
                    self.b_caloffsets[fpu_id] = Interval(0)
                    self._update_bpos(txn, datum_fpu, fpu_id,  Interval(0.0))

                    if self.bretries_acw[fpu_id] > 0:
                        #print("Resetting retry count for anti-clockwise freeBetaCollision movements.")
                        clockwise = False
                        self.bretries_acw[fpu_id] = 0
                        ProtectionDB.store_bretry_count(txn, datum_fpu, clockwise, 0)
                    if self.bretries_cw[fpu_id] > 0:
                        #print("Resetting retry count for clockwise freeBetaCollision movements.")
                        clockwise = True
                        self.bretries_cw[fpu_id] = 0
                        ProtectionDB.store_bretry_count(txn, datum_fpu, clockwise, 0)
                else:
                    if datum_fpu.ping_ok and (selected_arm in [DASEL_BETA, DASEL_BOTH]):
                        beta_angle, b_underflow, b_overflow = self._beta_angle(datum_fpu)
                        if b_underflow or b_overflow:
                            # we know only we are in the interval between the
                            # overflow / underflow value and the datum position
                            b_int = (Interval(beta_angle) + self.b_caloffsets[fpu_id]).extend(0.0)
                        else:
                            b_int = Interval(beta_angle) + self.b_caloffsets[fpu_id]

                        self._update_bpos(txn, datum_fpu, fpu_id,  b_int)


                prev_fpu = prev_gs.FPU[fpu_id]

                # this passes prev_fpu and datum_fpu, to deduce the time out counts
                self._update_error_counters(self.counters[fpu_id], prev_fpu, datum_fpu, datum_cmd=True)
                # this passes prev_fpu and datum_fpu, to get the counter deviations

                self._update_counters_find_datum(fpu_id, self.counters[fpu_id], prev_fpu, datum_fpu)
                ProtectionDB.put_counters(txn, datum_fpu, self.counters[fpu_id])



        with self.env.begin(db=self.healthlog, write=True) as txn:
            for fpu_id, datum_fpu in enumerate(datum_gs.FPU):
                cnt = self.counters[fpu_id].copy()
                cnt['unixtime'] = time.time()
                HealthLogDB.putEntry(txn, datum_fpu, cnt)
        self.env.sync()


    def _update_counters_find_datum(self, fpu_id, fpu_counters, prev_fpu, datum_fpu):

        fpu_counters["datum_count" ] += 1
        # discard error states, and states which were uninitialised before
        # fpu is none if datum command didn't succeed. Storing
        # abberrations is skipped in that case

        if datum_fpu.timeout_count != prev_fpu.timeout_count:
            fpu_counters["datum_timeout"] += 1


        if datum_fpu.last_status == 0:
            if prev_fpu.alpha_was_referenced and datum_fpu.alpha_was_referenced:
                fpu_counters["alpha_aberration_count" ] += 1
                fpu_counters["datum_sum_alpha_aberration"] += datum_fpu.alpha_deviation
                fpu_counters["datum_sqsum_alpha_aberration"] += (datum_fpu.alpha_deviation ** 2)

            if prev_fpu.beta_was_referenced and datum_fpu.beta_was_referenced:
                fpu_counters["beta_aberration_count" ] += 1
                fpu_counters["datum_sum_beta_aberration"] += datum_fpu.beta_deviation
                fpu_counters["datum_sqsum_beta_aberration"] += (datum_fpu.beta_deviation ** 2)





    def _start_find_datum_hook(self, gs, search_modes=None,  selected_arm=None, fpuset=None, initial_positions=None, soft_protection=None):
        """This is run when an findDatum command is actually started.
        It updates the new range of possible positions to include the zero point of each arm."""
        if fpuset is None:
            fpuset = []
            
        # initial_positions needs to be a dict

        initial_positions.clear()
        # we allow 0.5 degree of imprecision. This is mainly for the
        # FPU simulator which steps at discrete intervals

        with self.env.begin(db=self.fpudb, write=True) as txn:
            for fpu_id, fpu in enumerate(gs.FPU):
                if not fpu_in_set(fpu_id, fpuset):
                    continue

                if (len(search_modes) != 0) and (not search_modes.has_key(fpu_id)):
                    continue

                # record initial position intervals so that the
                # known range can be restored if the datum search is
                # rejected
                initial_positions[fpu_id] = (self.apositions[fpu_id].copy(),
                                             self.bpositions[fpu_id].copy())

                (atarget, btarget) = initial_positions[fpu_id]

                # update stored intervals to include zero, and store in DB
                if selected_arm in [DASEL_ALPHA, DASEL_BOTH]:
                    if soft_protection:
                        new_apos = self.apositions[fpu_id].extend(0.0 + self.config.alpha_datum_offset)
                        self._update_apos(txn, fpu, fpu_id, new_apos)
                    else:
                        protection_interval = Interval(ALPHA_MIN_HARDSTOP_DEGREE, ALPHA_MAX_HARDSTOP_DEGREE)
                        apos = self.apositions[fpu_id]
                        new_range = apos.combine(protection_interval)
                        self._update_apos(txn, fpu, fpu_id, new_range)
                    atarget = 0.0


                if selected_arm in [DASEL_BETA, DASEL_BOTH]:
                    bpos = self.bpositions[fpu_id]
                    if soft_protection:
                        new_bpos = bpos.extend(0.0)
                        self._update_bpos(txn, fpu, fpu_id,  new_bpos)
                    else:

                        m = search_modes.get(fpu_id, SEARCH_AUTO)
                        if m == SEARCH_CLOCKWISE:
                            new_range = bpos.extend(BETA_MIN_HWPROT_DEGREE)
                        elif m == SEARCH_ANTI_CLOCKWISE:
                            new_range = bpos.extend(BETA_MAX_HWPROT_DEGREE)
                        else:
                            protection_interval = Interval(BETA_MIN_HWPROT_DEGREE, BETA_MAX_HWPROT_DEGREE)
                            new_range = bpos.combine(protection_interval)


                        self._update_bpos(txn, fpu, fpu_id,  new_range)
                    btarget = 0.0

                # update target positions. The target positions are
                # used as a computed tracked value if the driver
                # cannot retrieve the exact counted position of the
                # FPU because the firmware counter is in underflow /
                # overflow. (That should only happen if the datum
                # search does not move this arm.)

                self.target_positions[fpu_id] = (atarget, btarget)

        self.env.sync()



    def _cancel_find_datum_hook(self, gs, search_modes,  selected_arm=None,
                               fpuset=None, initial_positions=None):
        if fpuset is None:
            fpuset = []
            
        if initial_positions is None:
            initial_positions = {}

        with self.env.begin(db=self.fpudb, write=True) as txn:
            for fpu_id in initial_positions.keys():
                # get last stored positions
                apos, bpos = initial_positions[fpu_id]

                fpu = gs.FPU[fpu_id]
                # revert stored intervals to old values
                self._update_apos(txn, fpu, fpu_id, apos)
                self._update_bpos(txn, fpu, fpu_id, bpos)

                self.target_positions[fpu_id] = (apos, bpos)

        self.env.sync()
        print("%f: _cancel_find_datum_hook(): Movement cancelled" % time.time(),
              file=self.protectionlog)


    def _pre_free_beta_collision_hook(self, fpu_id, direction, grid_state, soft_protection=True):

        if direction == REQD_CLOCKWISE:
            brcnt = self.bretries_cw[fpu_id]
        else:
            brcnt = self.bretries_acw[fpu_id]

        # NOTE: Maximum number of beta retries is now taken from the constants rather than from the database.
        #print("Beta retry count for FPU %i is %i, with limit at %i" % (fpu_id, brcnt, DEFAULT_FREE_BETA_RETRIES))
        #if soft_protection and (brcnt >= self.maxbretries[fpu_id]): # Old Johannes code.
        if soft_protection and (brcnt >= DEFAULT_FREE_BETA_RETRIES):
            raise ProtectionError("Retry count for FPU %i is already %i, exceeds safe maximum" % (fpu_id, brcnt))

        fpu = grid_state.FPU[fpu_id]

        if direction == REQD_CLOCKWISE:
            diff = - FREE_BETA_STEPCOUNT
        else:
            diff = FREE_BETA_STEPCOUNT

        bpos = self.bpositions[fpu_id]
        new_bpos = bpos + diff / StepsPerDegreeBeta

        self.target_positions[fpu_id] = ( self.apositions[fpu_id], new_bpos)

        with self.env.begin(db=self.fpudb, write=True) as txn:
            self._update_bpos(txn, fpu, fpu_id,  bpos.combine(new_bpos))

    def _post_free_beta_collision_hook(self, fpu_id, direction, grid_state):

        clockwise = (direction == REQD_CLOCKWISE)

        fpu = grid_state.FPU[fpu_id]

        if clockwise:
            self.bretries_cw[fpu_id] += 1
            cnt = self.bretries_cw[fpu_id]
            #print("Incrementing beta clockwise retry count to make %i" % cnt)
        else:
            self.bretries_acw[fpu_id] += 1
            cnt = self.bretries_acw[fpu_id]
            #print("Incrementing beta anti-clockwise retry count to make %i" % cnt)


        with self.env.begin(db=self.fpudb, write=True) as txn:
            ProtectionDB.store_bretry_count(txn, fpu, clockwise, cnt)

        fpuset = [fpu_id]
        #self._pingFPUs(grid_state, fpuset=fpuset)
        self._refresh_positions(grid_state, fpuset=fpuset)


    def _pre_free_alpha_limit_breach_hook(self, fpu_id, direction, grid_state, soft_protection=True):

        if direction == REQD_CLOCKWISE:
            pass
            #print("Clockwise")
        else:
            brcnt = self.aretries_acw[fpu_id]

        # NOTE: Maximum retry count is now taken from the constants rather than from the database.
        # print("Alpha retry count for FPU %i is %i, with limit at %i" % (fpu_id, brcnt, DEFAULT_FREE_ALPHA_RETRIES))
        #if soft_protection and (brcnt >= self.maxaretries[fpu_id]): # Old Johannes code.
        if soft_protection and (brcnt >= DEFAULT_FREE_ALPHA_RETRIES):
            raise ProtectionError("Retry count for FPU %i is already %i, exceeds safe maximum" % (fpu_id, brcnt))

        fpu = grid_state.FPU[fpu_id]

        if direction == REQD_CLOCKWISE:
            diff = - FREE_ALPHA_STEPCOUNT
        else:
            diff = FREE_ALPHA_STEPCOUNT

        apos = self.apositions[fpu_id]
        new_apos = apos + diff / StepsPerDegreeAlpha

        self.target_positions[fpu_id] = ( new_apos, self.bpositions[fpu_id])

        with self.env.begin(db=self.fpudb, write=True) as txn:
            self._update_apos(txn, fpu, fpu_id,  apos.combine(new_apos))

    def _post_free_alpha_limit_breach_hook(self, fpu_id, direction, grid_state):

        clockwise = (direction == REQD_CLOCKWISE)

        fpu = grid_state.FPU[fpu_id]

        if clockwise:
            self.aretries_cw[fpu_id] += 1
            cnt = self.aretries_cw[fpu_id]
            #print("Incrementing alpha clockwise retry count to make %i" % cnt)
        else:
            self.aretries_acw[fpu_id] += 1
            cnt = self.aretries_acw[fpu_id]
            #print("Incrementing alpha anti-clockwise retry count to make %i" % cnt)

        with self.env.begin(db=self.fpudb, write=True) as txn:
            ProtectionDB.store_aretry_count(txn, fpu, clockwise, cnt)

        fpuset = [fpu_id]
        #self._pingFPUs(grid_state, fpuset=fpuset)
        self._refresh_positions(grid_state, fpuset=fpuset)

    # ........................................................................
    def configPaths(self, paths, grid_state, fpuset=None, soft_protection=True, check_protection=None,
                    allow_uninitialized=False, ruleset_version=DEFAULT_WAVEFORM_RULESET_VERSION, reverse=False):
        """This methods takes a path which was loaded using wflib.load_path, and
        passes it to the configMotion method. All normal checks apply.
        In addition, the start point of the passed path must match the
        current position of the FPUs exactly.
        """
        if fpuset is None:
            fpuset = []

        waveform = {}
        for fpu_id, (alpha_path, beta_path)  in paths.items():
            assert(len(alpha_path) == len(beta_path)), "fpu_id=%d. Alpha and beta paths must be the same length" % fpu_id
            assert(len(alpha_path) > 0), "fpu_id=%d. Paths must contain at least one element" % fpu_id

            if not fpu_in_set(fpu_id, fpuset):
                continue


            alpha_steps, alpha_sum = fpu_commands.path_to_steps(alpha_path, StepsPerDegreeAlpha,
                                        origin=ALPHA_DATUM_OFFSET)
            beta_steps, beta_sum = fpu_commands.path_to_steps(beta_path, StepsPerDegreeBeta,
                                       origin=BETA_DATUM_OFFSET)

            if soft_protection:
                sidx = 0
                if reverse:
                    sidx = -1
                if alpha_sum[sidx] != grid_state.FPU[fpu_id].alpha_steps:
                    raise ProtectionError("for FPU %i, start point of passed path (%i) does not match "
                                          "current alpha arm position (%i) " % (fpu_id, alpha_sum[sidx],
                                                                                grid_state.FPU[fpu_id].alpha_steps))
                if beta_sum[sidx] != grid_state.FPU[fpu_id].beta_steps:
                    raise ProtectionError("for FPU %i, start point of passed path (%i) does not match "
                                          "current beta arm position (%i) " % (fpu_id, beta_sum[sidx],
                                                                                grid_state.FPU[fpu_id].beta_steps))


            tseries = zip(alpha_steps, beta_steps)
            if reverse:
                tseries.reverse()
            waveform[fpu_id] = tseries



        self.configMotion(waveform, grid_state,
                          fpuset=fpuset,
                          soft_protection=soft_protection,
                          check_protection=check_protection,
                          allow_uninitialized=allow_uninitialized,
                          ruleset_version=ruleset_version)

    # ........................................................................
    def configZero(self, grid_state, soft_protection=True, check_protection=None,
                   allow_uninitialized=False, ruleset_version=DEFAULT_WAVEFORM_RULESET_VERSION):
        """This method examines the current location of the FPUs and configures
        them with a waveform that will move them all to the zero position (where
        the path planning software assumes they will start from.
        """

        # All FPUs must be referenced.
        all_referenced = True
        for fpu_id, fpu in enumerate(grid_state.FPU):
            if not fpu.alpha_was_referenced or not fpu.beta_was_referenced:
                print("ERROR: FPU %d is not referenced." % fpu_id )
                all_referenced = False
        if not all_referenced:
            raise ProtectionError("All FPUs must be referenced before moving. "
                                  "Please datum the FPUs and try again.")

        current_angles = self.countedAngles( grid_state )
        alpha_list = []
        beta_list = []
        for alpha, beta in current_angles:
            if abs(alpha) > 0.0:
                alpha_list.append( alpha * -1.0 )
            else:
                alpha_list.append( 0.0 )
            if abs(beta) > 0.0:
                beta_list.append( beta * -1.0 )
            else:
                beta_list.append( 0.0 )

        #print("FPUs will be moved by alpha=%s and beta=%s" % (alpha_list, beta_list))
        waveform = fpu_commands.gen_wf( alpha_list, beta_list )
        self.configMotion( waveform, grid_state,
                           soft_protection=soft_protection,
                           check_protection=check_protection,
                           allow_uninitialized=allow_uninitialized,
                           ruleset_version=ruleset_version)


    # ........................................................................
    def configDatum(self, grid_state, alpha_safe=-179.0, beta_safe=1.0,
                    soft_protection=True, check_protection=None,
                    allow_uninitialized=False, use_step_counts=False,
                    ruleset_version=DEFAULT_WAVEFORM_RULESET_VERSION):
        """This method examines the current location of the FPUs and configures
        them with a waveform that will move them all to a location close
        enough to datum that a findDatum command can be executed.


        # TODO: Check that all FPUs are in a SAFE state unless collision_protection=False


        """

        # All FPUs must be referenced.
        all_referenced = True
        for fpu_id, fpu in enumerate(grid_state.FPU):
            if not fpu.alpha_was_referenced or not fpu.beta_was_referenced:
                all_referenced = False
                if allow_uninitialized:
                    print("NOTE: FPU %d is not referenced. Enabling movement" % fpu_id )
                    self.enableMove( fpu_id, grid_state )
                else:
                    print("ERROR: FPU %d is not referenced." % fpu_id )
        if not allow_uninitialized and not all_referenced:
            raise ProtectionError("All FPUs must be referenced before moving. "
                                  "Use allow_uninitialized=True to override.")

        # The counted angles function can only be used when all FPUs have been
        # referenced, otherwise extract a conservative estimate from the
        # tracked angles.
        if all_referenced:
            current_angles = self.countedAngles( grid_state )
        elif use_step_counts:
            # Assume the step counts are still valid.
            current_angles = self.countedAngles( grid_state, show_uninitialized=True )
        else:
            # Assume the step counts are no longer valid.
            # Use the last known positions recorded in the database.
            current_angles = []
            tracked_angles = self.trackedAngles( grid_state, retrieve=True )
            for alpha_interval, beta_interval in tracked_angles:
               # The most conservative estimate is the minimum alpha angle
               # and the middle of the known beta range.
               beta_estimate = (beta_interval.min() + beta_interval.max())/2.0
               current_angles.append( [alpha_interval.min(), beta_estimate] )

        alpha_list = []
        beta_list = []
        for alpha, beta in current_angles:
            alpha_motion = alpha_safe - alpha
            if abs(alpha_motion) > 0.0:
                alpha_list.append( alpha_motion )
            else:
                alpha_list.append( 0.0 )
            beta_motion = beta_safe - beta
            if abs(beta_motion) > 0.0:
                beta_list.append( beta_motion )
            else:
                beta_list.append( 0.0 )

        #print("FPUs will be moved by alpha=%s and beta=%s" % (alpha_list, beta_list))
        waveform = fpu_commands.gen_wf( alpha_list, beta_list )
        self.configMotion( waveform, grid_state,
                           soft_protection=soft_protection,
                           check_protection=check_protection,
                           allow_uninitialized=allow_uninitialized,
                           ruleset_version=ruleset_version)


    # ........................................................................
    def recoverFaults(self, grid_state, fpuset=None, verbose=False):
        """

        This method attempts to automatically recover a set of FPUs from a
        failure (alpha limit breach, beta limit breach, beta collision or
        aborted motion).

        NOTE: Sometimes a limit breach is not fully recovered and
        this method may need to be repeated while moving the FPUs
        closer to their safe zone.

        """
        import copy
        if fpuset is None:
            fpuset = []
        if len(fpuset) == 0:
            fpuset = range(self.config.num_fpus)
        fpuset = self.check_fpuset(fpuset)

        # First locate all the FPUs which have a fault and make a record of
        # the last position and the movement direction needed to free each FPU.
        fpus_with_fault = []
        last_position = {}    # Last position before recovery dictionary
        direction_needed = {} # Recovery direction dictionary
        self.pingFPUs(grid_state)
        for fpu_id in fpuset:
            # Obtain the status of this FPU
            #self.pingFPUs(grid_state, [fpu_id])
            fpu = grid_state.FPU[fpu_id]

            if verbose:
                print("\nPass 1: Checking FPU %i" % fpu_id)
                print("\t%s" % str(fpu))

            # Is there an alpha limit breach or beta collision detection?
            # FIXME: fpu.beta_collision is coming back True even for FPUs which have not collided!
            if fpu.state == FPST_OBSTACLE_ERROR or fpu.state == FPST_ABORTED:
                #   or fpu.at_alpha_limit or fpu.beta_collision: # <-- beta_collision flag incorrectly set
                print("FPU %d has a fault condition. state=%s, alpha_limit=%s, beta_collision=%s" % \
                   (fpu_id, str(fpu.state), str(fpu.at_alpha_limit), str(fpu.beta_collision)) )
                fpus_with_fault.append(fpu_id)

            # Last known direction of movement
            if fpu.direction_alpha == DIRST_CLOCKWISE or fpu.direction_alpha == DIRST_RESTING_LAST_CW:
                free_alpha_dir = REQD_ANTI_CLOCKWISE
            elif fpu.direction_alpha == DIRST_ANTI_CLOCKWISE or fpu.direction_alpha == DIRST_RESTING_LAST_ACW:
                free_alpha_dir = REQD_CLOCKWISE
            else:
                free_alpha_dir = None

            if fpu.direction_beta == DIRST_CLOCKWISE or fpu.direction_beta == DIRST_RESTING_LAST_CW:
                free_beta_dir = REQD_ANTI_CLOCKWISE
            elif fpu.direction_beta == DIRST_ANTI_CLOCKWISE or fpu.direction_beta == DIRST_RESTING_LAST_ACW:
                free_beta_dir = REQD_CLOCKWISE
            else:
                free_beta_dir = None
        
            # Create a dictionary entry for this FPU.
            direction_needed[fpu_id] = (free_alpha_dir, free_beta_dir)
            # Last known position
            current_angles = self.countedAngles( grid_state, fpuset=[fpu_id], show_uninitialized=True )
            alpha_angle = current_angles[0][0]
            beta_angle = current_angles[0][1]
            last_position[fpu_id] = (alpha_angle, beta_angle)
            if verbose:
                print("\tFPU %d: Direction needed to reverse (%s,%s)." % (fpu_id, str(free_alpha_dir), str(free_beta_dir)))
                print("\tLast position: (%.3f, %.3f) (deg)" % (alpha_angle, beta_angle) )

        if len(fpus_with_fault) == 0:
            print("No faults detected.")
            return
        else:
            print("%d FPUs have a fault condition: %s" % (len(fpus_with_fault), fpus_with_fault))
 
        # Now pass through the FPUs with a fault and correct all the limit
        # breaches.
        self.pingFPUs(grid_state)
        for fpu_id in copy.copy(fpus_with_fault):
            # Obtain the status of this FPU
            #self.pingFPUs(grid_state, [fpu_id])
            fpu = grid_state.FPU[fpu_id]

            if verbose:
                print("\nPass 2: Checking FPU %i" % fpu_id)
                print("\t%s" % str(fpu))

            # Count the number of faults on this FPU (both its motors might
            # have reached a limit)
            nfaults = 0
            nrecovered = 0
            if fpu.at_alpha_limit:
                nfaults += 1
            if fpu.beta_collision:
                nfaults += 1
            if fpu.state == FPST_ABORTED:
                nfaults += 1
            
            # Recall the recovery direction
            (free_alpha_dir, free_beta_dir) = direction_needed[fpu_id]

            # Recall the last known position before recovery started
            (alpha_angle, beta_angle) = last_position[fpu_id]
            if verbose:
                print("\tLast position recalled: (%.3f, %.3f) (deg)" % (alpha_angle, beta_angle) )

            # Decide if a beta collision is caused by a genuine collision or a beta limit breach.
            if fpu.beta_collision and (beta_angle < BETA_MIN_DEGREE or beta_angle > BETA_MAX_DEGREE):
                fpu_beta_limit = True
            else:
                fpu_beta_limit = False

            # Free an obstacle error.
            # NOTE: Functions enableBetaCollisionProtection and enableAlphaLimitProtection
            # can change the state of the whole grid. Just check the state of the individual flags.
            #if fpu.state == FPST_OBSTACLE_ERROR:
            if True:
                if fpu.at_alpha_limit:
                    strg = "FPU %i: Alpha limit breach." % fpu_id
                    if free_alpha_dir == REQD_CLOCKWISE:
                        strg += " Clockwise (negative) movement needed."
                    elif free_alpha_dir == REQD_ANTI_CLOCKWISE:
                        strg += " Anti-clockwise (positive) movement needed."
                    else:
                        print("Last alpha direction for FPU %i unknown. Cannot free alpha limit breach!" % fpu_id)
                        free_alpha_dir = None
                    print(strg)

                    if free_alpha_dir is not None:
                        self.freeAlphaLimitBreach( fpu_id, free_alpha_dir, grid_state, soft_protection=False )
                        for k in range(1, DEFAULT_FREE_ALPHA_RETRIES):
                            self.freeAlphaLimitBreach( fpu_id, free_alpha_dir, grid_state, soft_protection=False )
                            self.enableAlphaLimitProtection( grid_state )
                            #self.pingFPUs(grid_state, [fpu_id])
                            fpu = grid_state.FPU[fpu_id]
                            if not fpu.at_alpha_limit:
                                nrecovered += 1
                                print("FPU %i: Alpha limit breach recovered." % fpu_id)
                                break
                        # Move a little bit further from the limit
                        for k in range(0, 2):
                            self.freeAlphaLimitBreach( fpu_id, free_alpha_dir, grid_state, soft_protection=False )
                        self.enableAlphaLimitProtection( grid_state )
                    
                elif fpu.beta_collision and not fpu_beta_limit:
                    # A genuine collision will be corrected on the next pass.
                    pass

                elif fpu.beta_collision:
                    # A beta limit breach/
                    strg = "FPU %i: Beta limit breach." % fpu_id
                    if free_beta_dir == REQD_CLOCKWISE:
                        strg += " Clockwise (negative) movement needed."
                    elif free_beta_dir == REQD_ANTI_CLOCKWISE:
                        strg += " Anti-clockwise (positive) movement needed."
                    else:
                        print("Last beta direction for FPU %i unknown. Cannot free beta limit breach!" % fpu_id)
                        free_beta_dir = None
                    print(strg)

                    if free_beta_dir is not None:
                        self.freeBetaCollision( fpu_id, free_beta_dir, grid_state, soft_protection=False )
                        for k in range(1, DEFAULT_FREE_BETA_RETRIES):
                            self.freeBetaCollision( fpu_id, free_beta_dir, grid_state, soft_protection=False )
                            self.enableBetaCollisionProtection( grid_state )
                            #self.pingFPUs(grid_state, [fpu_id])
                            fpu = grid_state.FPU[fpu_id]
                            if not fpu.beta_collision:
                                nrecovered += 1
                                print("FPU %i: Beta limit breach recovered." % fpu_id)
                                break
                        # Move a little bit further from the limit
                        for k in range(0, 2):
                            self.freeBetaCollision( fpu_id, free_beta_dir, grid_state, soft_protection=False )
                        self.enableBetaCollisionProtection( grid_state )
                else:
                    print("FPU %i: Limit breaches have spontaneously recovered!" % fpu_id)
                    nrecovered += 1

            if fpu.state == FPST_ABORTED:
                nfaults += 1
                print("FPU %i was aborted. Enabling." % fpu_id )
                self.enableMove( fpu_id, grid_state )
                nrecovered += 1

            # If all faults have been recovered, remove the FPU from the
            # fault list
            if verbose:
                print("FPU %d: Number of faults %d of which %d were recovered." % (fpu_id,nfaults, nrecovered))
            if nrecovered >= nfaults:
                if verbose:
                    print("Removing FPU %d" % fpu_id)
                fpus_with_fault.remove(fpu_id)

        # Next FPU

        if len(fpus_with_fault) == 0:
            print("All faults cleared.")
            return
        else:
            print("After pass 2, %d FPUs still have a fault condition: %s" % (len(fpus_with_fault), fpus_with_fault))

        # Make a third pass through the FPUs with a fault and correct the
        # beta collisions. This time small corrections are made to each
        # FPU in turn until the collisions are freed.
        for t in range(0, DEFAULT_FREE_BETA_RETRIES):
            self.pingFPUs(grid_state)
            for fpu_id in copy.copy(fpus_with_fault):
                # Obtain the status of this FPU
                #self.pingFPUs(grid_state, [fpu_id])
                fpu = grid_state.FPU[fpu_id]

                if verbose:
                    print("\nPass 3 step %s: Checking FPU %i" % (t, fpu_id))
                    print("\t%s" % str(fpu))

                # Recall the recovery direction
                (free_alpha_dir, free_beta_dir) = direction_needed[fpu_id]
    
                # Recall the last known position before recovery started
                (alpha_angle, beta_angle) = last_position[fpu_id]
                if verbose:
                    print("\tLast position recalled: (%.3f, %.3f) (deg)" % (alpha_angle, beta_angle) )
    
                # Free the beta collision.
                # NOTE: Functions enableBetaCollisionProtection and enableAlphaLimitProtection
                # can change the state of the whole grid. Just check the state of the individual flags.
                #if fpu.state == FPST_OBSTACLE_ERROR:
                if True:
                    if fpu.beta_collision:
                        # A beta collision. Reverse both the alpha and beta motors.
                        strg = "FPU %i: Beta collision." % fpu_id
                        if free_beta_dir == REQD_CLOCKWISE:
                            strg += " Clockwise (negative) beta movement needed."
                        elif free_beta_dir == REQD_ANTI_CLOCKWISE:
                            strg += " Anti-clockwise (positive) beta movement needed."
                        else:
                            print("Last beta direction for FPU %i unknown. Cannot free beta collision!" % fpu_id)
                            break
                        if free_alpha_dir == REQD_CLOCKWISE:
                            strg += " Clockwise (negative) alpha movement needed."
                        elif free_alpha_dir == REQD_ANTI_CLOCKWISE:
                            strg += " Anti-clockwise (positive) alpha movement needed."
                        else:
                            strg += " No alpha movement needed."
                        print(strg)
    
                        # NOTE: The FPU might be moving tangentially to another FPU
                        # but by reversing all the FPUs by a small amount in rotation
                        # the collision will eventually be freed.
                        self.freeBetaCollision( fpu_id, free_beta_dir, grid_state, soft_protection=False )
                        self.freeBetaCollision( fpu_id, free_beta_dir, grid_state, soft_protection=False )
                        if free_alpha_dir is not None:
                            self.freeAlphaLimitBreach( fpu_id, free_alpha_dir, grid_state, soft_protection=False )
                        self.enableBetaCollisionProtection( grid_state )
                        #self.pingFPUs(grid_state, [fpu_id])
                        fpu = grid_state.FPU[fpu_id]
                        if not fpu.beta_collision:
                            print("FPU %i: Beta collision recovered." % fpu_id)
                            # Move a little bit further from the collision
                            for k in range(0, 3):
                                self.freeBetaCollision( fpu_id, free_beta_dir, grid_state, soft_protection=False )
                            self.enableBetaCollisionProtection( grid_state )
                            # The fault has been recovered
                            print("FPU %d: Collision recovered - removing" % fpu_id)
                            fpus_with_fault.remove(fpu_id)
                    else:
                        print("FPU %i: Beta collision spontaneously recovered!" % fpu_id)
                        fpus_with_fault.remove(fpu_id)

            # Next FPU
        # Next iteration

        if len(fpus_with_fault) > 0:
            print("Not all faults could be recovered. Manual intervention needed.")
            print("These FPUs still have a fault: %s" % str(fpus_with_fault))
        else:
            print("All faults recovered. Now move the FPUs to a safe location.")
