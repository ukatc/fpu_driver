#!/usr/bin/python
from ast import literal_eval
from interval import Interval, Inf, nan


INIT_COUNTERS = {
    # updated on executeMotion
    # aborted movements are not subtracted
    "total_beta_steps" : 0,           # total step count for beta arm
    "total_alpha_steps" : 0,          # total step count for alpha arm
    "executed_waveforms" : 0,         # number of waveform tables executed
    "alpha_direction_reversals" : 0,  # number of times alpha arm movement was reversed
    "beta_direction_reversals" : 0,   # number of times alpha arm movement was reversed
    "sign_alpha_last_direction" : 0,  # sign of last alpha arm movement
    "sign_beta_last_direction" : 0,   # sign of last alpha arm movement
    "alpha_starts" : 0,               # number of times alpha arm started to move
    "beta_starts" : 0,                # number of times alpha arm started to move

    # updated on finish of executeMotion / findDatum:
    "collisions" : 0,
    "limit_breaches" : 0,
    "can_timeout" : 0,
    "datum_timeout" : 0,
    "movement_timeout" : 0,

    # updated on finish of findDatum:
    "datum_count" : 0,
    "alpha_aberration_count" : 0,
    "beta_aberration_count" : 0,
    "datum_sum_alpha_aberration" : 0,   # sum of residual count on alpha datum
    "datum_sum_beta_aberration" : 0,    # sum of residual count on beta datum
    "datum_sqsum_alpha_aberration" : 0, # square sum of above
    "datum_sqsum_beta_aberration" : 0,  # square sum of above
}


class ProtectionDB:
    dbname = "fpu"
    alpha_positions = 'apos'
    beta_positions = 'bpos'
    waveform_table = 'wtab'
    waveform_reversed = 'wf_reversed'
    alpha_limits = 'alimits'
    beta_limits = 'blimits'
    free_beta_retries = 'bretries'
    beta_retry_count_cw = 'beta_retry_count_cw'
    beta_retry_count_acw = 'beta_retry_count_acw'
    counters = 'counters'
    
    @staticmethod
    def putField(txn, serial_number, subkey, val):
        assert(serial_number != "@@@@@")
        key = str( (serial_number, subkey))
        txn.put(key, repr(val))
        
    @staticmethod
    def putInterval(txn, serial_number, subkey, interval, offset=0):
        """Stores a position interval.
        
        In theory, it is cleaner to only store
        the relative values. But we want the DB content
        to be human-readable, and easy to interpret,
        and a uniform angle interpretation, 
        so it is better to store positional values always 
        along with the offset they refer to."""
        
        val = [interval, offset]
        ProtectionDB.putField(txn, serial_number, subkey, val)
        
    @staticmethod
    def getRawField(txn, serial_number, subkey):
        key = str((serial_number, subkey))
        data = txn.get(key)
        if data == None:
            return None
        val = literal_eval(data)
        return val

    
    @classmethod
    def getField(cls, txn, fpu, subkey):
        val = cls.getRawField(txn, fpu.serial_number, subkey)
        
        if subkey == cls.counters:
            rval = INIT_COUNTERS
            if val != None:
                rval.update(val)
            return rval
        
        if val == None:
            return None

        if subkey in [cls.waveform_table,
                      cls.waveform_reversed,
                      cls.free_beta_retries,
                      cls.beta_retry_count_cw,
                      cls.beta_retry_count_acw]:
            return val
        else:
            # return position span as interval object
            
            # The code below transforms an absolute position and
            # offset into a relative position. This allows to have
            # configurable offsets for the alpha datum position.  The
            # reason for that absolute position data is stored along
            # with the offset is the goal to store data both
            # offset-independent and human-readable.
            ivlist, stored_offset = val
            return Interval(ivlist) - stored_offset


    @classmethod
    def put_alpha_position(cls, txn, fpu, apos, aoffset):
        
        # store the datum offsets along with each position
        # (this allows to reconfigure the zero point later)
        cls.putInterval(txn, fpu.serial_number, cls.alpha_positions, apos, aoffset)
        
    @classmethod
    def put_beta_position(cls, txn, fpu, bpos):
        # store the datum offsets along with each position
        # (this allows to reconfigure the zero point later)

        cls.putInterval(txn, fpu.serial_number, cls.beta_positions, bpos, 0)

    @classmethod
    def store_reversed(cls, txn, fpu, is_reversed):
        cls.putField(txn, fpu.serial_number, cls.waveform_reversed, is_reversed)
        
    @classmethod
    def storeWaveform(cls, txn, fpu, wentry):
        cls.putField(txn, fpu.serial_number, cls.waveform_table, wentry)

    @classmethod
    def store_bretry_count(cls, txn, fpu, clockwise, cnt):

        if clockwise :
            subkey = cls.beta_retry_count_cw
        else:
            subkey = cls.beta_retry_count_acw

        cls.putField(txn, fpu.serial_number, subkey, cnt)

        
            
    @classmethod
    def put_counters(cls, txn, fpu, counter_vals):
        # store the datum offsets along with each position
        # (this allows to reconfigure the zero point later)

        cls.putField(txn, fpu.serial_number, cls.counters, counter_vals)
            
    
