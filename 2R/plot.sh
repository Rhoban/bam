#!/bin/bash

# Copyright 2025 Marc Duclusaud & Grégoire Passault

# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at:

#     http://www.apache.org/licenses/LICENSE-2.0

if [ $# -lt 2 ]; then
    echo "Usage: plot.sh [testbench] [log1 [log2 ...]]"
    exit 1
fi

testbench=$1

params=""
if [ $testbench == "mx" ]
then
    for i in {1,4}
    do
        params+="params/mx106/m$i.json,params/mx64/m$i.json "
    done
fi
if [ $testbench == "erob" ]
then
    for i in {1,6}
    do
        params+="params/erob80_100/m$i.json,params/erob80_50/m$i.json "
    done
fi

python 2R/sim.py --log "${@:2}" --params $params --testbench $testbench --plot