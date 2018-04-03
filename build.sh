#!/bin/bash
make -C ~/linux/ M=`pwd` modules
g++ read_event.cpp -o read_event
exit
