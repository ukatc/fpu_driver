#!/usr/bin/python
from __future__ import print_function, division

import os
from math import sqrt
from ast import literal_eval

from sys import argv, exit
import argparse

import datetime
import lmdb

from numpy import array
import matplotlib
import pylab as pl


from protectiondb import ProtectionDB
from protectiondb import HealthLogDB




def parse_args():
    parser = argparse.ArgumentParser(description='plot time series for FPU with a given serial number')

    parser.add_argument('serial_number', type=str, nargs='?',
                        help="""serial number of FPU to plot""")
    
    parser.add_argument('-v', '--verbosity', metavar='VERBOSITY', type=int,
                        default=1,
                        help='verbosity level of progress messages (default: %(default)s)')
        


    parser.add_argument('-w', '--show_number_waveforms', dest="show_number_waveforms",  action='store_true',
                        help='plot number of waveforms over time (default: %(default)s)')


     
    args = parser.parse_args()
#    args.stop_time = time.strptime(args.end_time, '%Y-%m-%dT%H:%M:%S%Z')

          
    return args




if __name__ == '__main__' :
    DATABASE_FILE_NAME = os.environ.get("FPU_DATABASE")

    env = lmdb.open(DATABASE_FILE_NAME, max_dbs=10, map_size=(5*1024*1024*1024))
    
    fpudb = env.open_db("fpu")


    args = parse_args()

    serial_number = args.serial_number

    with env.begin(db=fpudb) as txn:
        counters = ProtectionDB.getField(txn,serial_number, ProtectionDB.counters)
        
        num_datum_ops = counters.get(HealthLogDB.datum_count, 0)

    if num_datum_ops == 0:
        print("no data found for FPU %s" % serial_number)
        sys.exit(1)
    
    healthlog = env.open_db("healthlog")

    alpha_count_series = []
    beta_count_series = []
    alpha_mean_series = []
    beta_mean_series = []
    alpha_stdev_series = []
    beta_stdev_series = []

    executed_waveforms_series = []
    timestamp_series = []
    with env.begin(db=healthlog) as txn:
        for cnt in range(1, num_datum_ops+1):
            key, val = HealthLogDB.getEntry(txn,serial_number, cnt)
            if val is not None:
                if args.verbosity > 1:
                    print(key,":", val)
                    
                count = val["datum_count"]

                alpha_count = val["alpha_aberration_count"]
                if alpha_count > 0:
                    alpha_count_series.append(alpha_count)
                    alpha_sum = val["datum_sum_alpha_aberration"]
                    alpha_sqsum = val["datum_sqsum_alpha_aberration"]
                    alpha_mean = alpha_sum / alpha_count
                    alpha_mean_series.append(alpha_mean)
                    alpha_stdev = sqrt(alpha_sqsum / alpha_count - alpha_mean ** 2)
                    alpha_stdev_series.append(alpha_stdev)

                beta_count = val["beta_aberration_count"]
                if beta_count > 0:
                    beta_count_series.append(beta_count)
                    beta_sum = val["datum_sum_beta_aberration"]
                    beta_sqsum = val["datum_sqsum_beta_aberration"]
                    beta_mean = beta_sum / beta_count
                    beta_mean_series.append(beta_mean)
                    beta_stdev = sqrt(beta_sqsum / beta_count - beta_mean ** 2)
                    beta_stdev_series.append(beta_stdev)

                # convert unitx time stamps (seconds since epoch) to datetime values
                timestamp_series.append(datetime.datetime.fromtimestamp(val['unixtime']))
                # get number of executed waveforms
                executed_waveforms_series.append(val['executed_waveforms'])


    # the lines below convert Python lists to numpy arrays which
    # makes it easy to handle them as vectors which can be
    # added and subtracted
    alpha_count = array(alpha_count_series)
    alpha_mean = array(alpha_mean_series)
    alpha_stdev= array(alpha_stdev_series)
    
    ha1, = pl.plot(alpha_count_series, alpha_mean, 'r-',
                  label='alpha datum aberration mean over time')
    
    ha2, = pl.plot(alpha_count_series, alpha_mean - alpha_stdev,  'r:',
                  label='alpha datum aberration standard deviation over time')
    
    ha3, = pl.plot(alpha_count_series, alpha_mean + alpha_stdev,  'r:')

    
    beta_count = array(beta_count_series)
    beta_mean = array(beta_mean_series)
    beta_stdev= array(beta_stdev_series)
    
    hb1, = pl.plot(beta_count_series, beta_mean, 'b-',
                  label='beta datum aberration mean over time')
    hb2, = pl.plot(beta_count_series, beta_mean - beta_stdev,  'b:',
                  label='beta datum aberration standard deviation over time')
    
    hb3, = pl.plot(beta_count_series, beta_mean + beta_stdev,  'b:')
        
    pl.legend(handles=[ha1, ha2, ha3, hb1, hb2, hb3])
    pl.xlabel("count of datum ops [1]")
    pl.ylabel("steps [1]")

    print("close plot window to continue")
    pl.show()
                

    if args.show_number_waveforms:
        
        hw, = pl.plot(timestamp_series, executed_waveforms_series, 'g-',
                      label='number of executed waveforms over time')
                          
        pl.legend(handles=[hw])
        pl.xlabel("count executed waveforms [1]")
        pl.ylabel("timestamp")
        pl.show()      

