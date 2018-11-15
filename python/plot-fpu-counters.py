#!/usr/bin/python
from __future__ import print_function, division

import os
from math import sqrt
from ast import literal_eval
import warnings
import sys 
import argparse

import datetime
import lmdb

from numpy import array, zeros
import matplotlib
import pylab as pl


from protectiondb import ProtectionDB
from protectiondb import HealthLogDB




def parse_args():
    parser = argparse.ArgumentParser(description='plot time series for FPU with a given serial number')

    parser.add_argument('serial_numbers', type=str, nargs='+',
                        help="""serial numbers of FPUs to plot""")
    
    parser.add_argument('-v', '--verbosity', metavar='VERBOSITY', type=int,
                        default=1,
                        help='verbosity level of progress messages (default: %(default)s)')

    parser.add_argument('-1', '--legacy_pre_v1.3.7', dest="pre137",  action='store_true',
                        help='plot data from driver version before 1.3.7  (which is likely not valid) (default: %(default)s)')

    parser.add_argument('-*', '--show_all', dest="show_all",  action='store_true',
                        help='plot all available charts (default: %(default)s)')

    parser.add_argument('-a', '--show_alpha_aberrations', dest="show_alpha_aberrations",  action='store_true',
                        help='plot alpha datum aberrations (default: %(default)s)')

    parser.add_argument('-b', '--show_beta_aberrations', dest="show_beta_aberrations",  action='store_true',
                        help='plot beta datum aberrations (default: %(default)s)')

    parser.add_argument('-A', '--show_alpha_aberration_stats', dest="show_alpha_aberration_stats",  action='store_true',
                        help='plot statistics of alpha datum aberrations (default: %(default)s)')

    parser.add_argument('-B', '--show_beta_aberration_stats', dest="show_beta_aberration_stats",  action='store_true',
                        help='plot statistics of beta datum aberrations (default: %(default)s)')
    
    parser.add_argument('-k', '--binlen', metavar='BINLEN', type=int,
                        default=25,
                        help='number of samples combined into the mean and stdev values (default: %(default)s)')
    
    parser.add_argument('-w', '--show_number_waveforms', dest="show_number_waveforms",  action='store_true',
                        help='plot number of waveforms over time (default: %(default)s)')

    parser.add_argument('-t', '--show_timeouts', dest="show_timeouts",  action='store_true',
                        help='plot number of time-outs over time (default: %(default)s)')

     
    args = parser.parse_args()
#    args.stop_time = time.strptime(args.end_time, '%Y-%m-%dT%H:%M:%S%Z')

    if args.show_all or (not (args.show_alpha_aberrations
                              or args.show_alpha_aberration_stats
                              or args.show_beta_aberrations
                              or args.show_beta_aberration_stats
                              or args.show_number_waveforms
                              or args.show_timeouts)):
        
        args.show_alpha_aberrations = True
        args.show_beta_aberrations = True
        args.show_alpha_aberration_stats = True
        args.show_beta_aberration_stats = True
        args.show_number_waveforms = True
        args.show_timeouts = True
          
    return args


class StatCount:
    def __init__(self, max_samples):
        self.count_series = zeros((max_samples,), dtype=int)
        self.sum_series = zeros((max_samples,), dtype=int)
        self.sqsum_series = zeros((max_samples,), dtype=int)
        self.idx = 0


    def addcount(self, count, sum, sqsum):        
        if count > 0:
            idx = self.idx
            self.count_series[idx] = count
            self.sum_series[idx] = sum
            self.sqsum_series[idx] = sqsum
            self.idx += 1

    def get_vals(self):
        vals = self.sum_series[0:self.idx-1] - self.sum_series[1:self.idx]
        return (self.count_series[:self.idx-1], vals)
        
            
    def get_stats(self, binlen):
        means_series = zeros((self.idx,), dtype=float)
        stddev_series = zeros((self.idx,), dtype=float)

        for k in range(self.idx):
            if k == 0:
                dcount = self.count_series[k]
                dsum = self.sum_series[k]
                dsqsum = self.sqsum_series[k]
            else:
                i = max(0, k - binlen)
                dcount = self.count_series[k] - self.count_series[i]
                dsum = self.sum_series[k] - self.sum_series[i]
                dsqsum = self.sqsum_series[k] - self.sqsum_series[i]
            
            mean = dsum / dcount
            stdev = sqrt(dsqsum / dcount - mean ** 2)
            
            if dcount > 1:
                # compute unbiased sample variance
                stdev *= float(dcount) / float(dcount-1)
            
            means_series[k] = mean
            stddev_series[k] = stdev
            
        return (self.count_series[:self.idx], means_series, stddev_series)
    
                 

def plot_fpu_data(args, serial_number):

    with env.begin(db=fpudb) as txn:
        counters = ProtectionDB.getField(txn,serial_number, ProtectionDB.counters)
        
        num_datum_ops = counters.get(HealthLogDB.datum_count, 0)

    if num_datum_ops == 0:
        print("no data found for FPU %s" % serial_number)
        sys.exit(1)

    series=None
    if args.pre137:
        # use old series table (which is buggy because of aliasing problem)
        series='counters'
    
    healthlog = env.open_db("healthlog")

    alpha_counts = StatCount(num_datum_ops)
    
    beta_counts = StatCount(num_datum_ops)
    
    beta_count_series = []
    beta_mean_series = []
    beta_stdev_series = []

    datum_timeouts_series = []
    can_timeouts_series = []
    movement_timeouts_series = []
    
    
    executed_waveforms_series = []
    timestamp_series = []
    datum_count_series = []
    with env.begin(db=healthlog) as txn:
        for cnt in range(1, num_datum_ops+1):
            key, val = HealthLogDB.getEntry(txn,serial_number, cnt, series=series)
            if val is not None:
                if args.verbosity > 1:
                    print(key,":", val)
                    
                datum_count = val["datum_count"]
                
                alpha_counts.addcount(val["alpha_aberration_count"],
                                      val["datum_sum_alpha_aberration"],
                                      val["datum_sqsum_alpha_aberration"])
                
                beta_counts.addcount(val["beta_aberration_count"],
                                      val["datum_sum_beta_aberration"],
                                      val["datum_sqsum_beta_aberration"])

                datum_timeouts_series.append(val["datum_timeout"])
                can_timeouts_series.append(val["can_timeout"])
                movement_timeouts_series.append(val["movement_timeout"])
                # convert unitx time stamps (seconds since epoch) to datetime values
                print("val=", val)
                if val.has_key('unixtime') and (val['unixtime'] != 0):
                    timestamp_series.append(datetime.datetime.fromtimestamp(val['unixtime']))
                    # get number of executed waveforms
                    executed_waveforms_series.append(val['executed_waveforms'])
                    datum_count_series.append(datum_count)


    # the lines below convert Python lists to numpy arrays which
    # makes it easy to handle them as vectors which can be
    # added and subtracted
    handles2 = []
    if args.show_alpha_aberrations:
        alpha_count_ab, alpha_aberration = alpha_counts.get_vals()
        
        ha4, = pl.plot(alpha_count_ab, alpha_aberration, 'r.',
                       label='alpha datum aberration (steps)')

        handles2.append(ha4)

    
    if args.show_beta_aberrations:
        beta_count_ab, beta_aberration = beta_counts.get_vals()

        hb4, = pl.plot(beta_count_ab, beta_aberration, 'b.',
                       label='beta datum aberration (steps)')

        handles2.append(hb4)

    if len(handles2) > 0:
        pl.legend(handles=handles2)
        pl.xlabel("count of datum ops [1]")
        pl.ylabel("datum aberration steps [1]")
        pl.title("Datum aberrations for FPU %s" % serial_number)
        pl.grid()
        pl.show()


    handles = []
    if args.show_alpha_aberration_stats:
        alpha_count, alpha_mean, alpha_stdev = alpha_counts.get_stats(args.binlen)
    
        ha1, = pl.plot(alpha_count, alpha_mean, 'r-',
                       label='mean of alpha datum aberration (averaged over %i bins)' % args.binlen)
    
        ha2, = pl.plot(alpha_count, alpha_mean - alpha_stdev,  'r:',
                       label='standard deviation of alpha datum aberration (averaged over %i bins)' % args.binlen)

        ha3, = pl.plot(alpha_count, alpha_mean + alpha_stdev,  'r:')
        handles.extend([ha1, ha2, ha3])


    
    if args.show_beta_aberration_stats:
        beta_count, beta_mean, beta_stdev = beta_counts.get_stats(args.binlen)
    
        hb1, = pl.plot(beta_count, beta_mean, 'b-',
                       label='mean of beta datum aberration (averaged over %i bins)' % args.binlen)
        hb2, = pl.plot(beta_count, beta_mean - beta_stdev,  'b:',
                       label='standard deviation of beta datum aberration (averaged over %i bins)' % args.binlen)
    
        hb3, = pl.plot(beta_count, beta_mean + beta_stdev,  'b:')
        handles.extend([hb1, hb2, hb3])


    if len(handles) > 0:
        with warnings.catch_warnings():
            # suppress warning about empty label
            warnings.filterwarnings('ignore', """The handle .* has a label of .* which cannot be automatically added to the legend.""",
                                    UserWarning)
            pl.legend(handles=handles)
        pl.xlabel("count of datum ops [1]")
        pl.ylabel("steps [1]")
        pl.title("statistics of datum aberrations for FPU %s" % serial_number)
        print("close plot window to continue")
        pl.grid()
        pl.show()


    if args.show_number_waveforms:
        
        hw, = pl.plot(timestamp_series, executed_waveforms_series, 'g.',
                      label='number of executed waveforms over time')
        
        hd, = pl.plot(timestamp_series, datum_count_series, 'b-',
                      label='number of datum operations over time')
                          
        pl.legend(handles=[hw, hd])
        pl.xlabel("timestamp")
        pl.ylabel("count executed operations [1]")
        pl.title("Count of operations over time for FPU %s" % serial_number)
        pl.grid()
        pl.show()      

    if args.show_timeouts:
        
        td, = pl.plot(timestamp_series, datum_timeouts_series, 'm-',
                      label='number of datum time-outs over time')
        
        tm, = pl.plot(timestamp_series, movement_timeouts_series, 'r-',
                      label='number of movement time-outs over time')
        
        tc, = pl.plot(timestamp_series, can_timeouts_series, 'b-',
                      label='number of CAN time-outs over time')                          
                          
        pl.legend(handles=[td, tm, tc])
        pl.xlabel("timestamp")
        pl.ylabel("count [1]")
        pl.title("Count of time-outs over time for FPU %s" % serial_number)
        pl.grid()
        pl.show()      


if __name__ == '__main__' :
    DATABASE_FILE_NAME = os.environ.get("FPU_DATABASE")

    env = lmdb.open(DATABASE_FILE_NAME, max_dbs=10, map_size=(5*1024*1024*1024))
    
    fpudb = env.open_db("fpu")


    args = parse_args()

    for serial_number in args.serial_numbers:
        plot_fpu_data(args, serial_number)


