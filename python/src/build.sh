# This needs boost-1-65-1 - the version in Debian won't do.
# if the boost::python libraries are not installed in /usr/local,
# set the directories in which boost is installed as follows:
#
# export LIBRARY_PATH=$HOME/lib
# export CPLUS_INCLUDE_PATH=$HOME/include                     
# export LD_LIBRARY_PATH=$HOME/lib                     

g++ -shared -std=c++11 -I/usr/local/include -I/usr/include/python2.7 -fPIC -o fpu_driver.so fpu_driver.cpp -L../../lib  -lfpudriver -lboost_python
