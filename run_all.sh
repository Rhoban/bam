#!/bin/bash

MODELS="m1 m2 m3 m4 m5 m6"
RUNS=3

for MODEL in $MODELS; do
    for RUN in `seq 1 $RUNS`; do
        echo "Running $MODEL $RUN"
        nohup bash ./run.sh $MODEL &
        sleep 5
    done
done
