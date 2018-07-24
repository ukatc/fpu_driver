#!/usr/bin/python
from ast import literal_eval
from interval import Interval, Inf, nan


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

        
            
            
    
