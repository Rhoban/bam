#!/bin/bash

if [ $# -le 2 ]; then
    echo "Usage: plot.sh [testbench] [log1 [log2 ...]]"
    exit 1
fi

testbench=$1

params=""
if [ $testbench == "mx" ]
then
    for i in {1..6}
    do
        params+="params/mx106/m$i.json,params/mx64/m$i.json "
    done
fi
if [ $testbench == "erob" ]
then
    for i in {1..6}
    do
        params+="params/erob80_100/m$i.json,params/erob80_50/m$i.json "
    done
fi

python 2R/sim.py --log "${@:2}" --params $params --testbench $testbench --mae