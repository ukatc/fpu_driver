# TODO: BW note: This file is just a quick copy/adaptation of build.sh for now
# (N.B. I don't know if the original comments below are still correct)

# This needs boost-1-65-1 - the version in Debian won't do.
# if the boost::python libraries are not installed in /usr/local,
# set the directories in which boost is installed as follows:
#
# export LIBRARY_PATH=$HOME/lib
# export CPLUS_INCLUDE_PATH=$HOME/include                     
# export LD_LIBRARY_PATH=$HOME/lib                     


# TODO: Eventually build and include the C++-side grid driver functionality as
# a library file, rather than as individual source files?

g++ -shared -std=c++11 -fPIC \
    -I/usr/local/include -I/usr/include/python2.7 -I../../include \
     griddriver_wrapper.C ../../src/FPUGridDriver.C \
     -L/usr/local/lib -lboost_python27 \
     -o griddriver.so
