#!/bin/bash

ACTUATOR="mx106"
DATA="data_106"
VAL_KP=8

if [ $# -eq 0 ]; then
    echo "Usage: $0 <model>"
    exit 1
fi

N=`ls output/*.json|wc -w`
N=$[$N + 1]

python fit.py --wandb --logdir $DATA \
	--model $1 \
    --control \
	--workers 1 --trials 10000 \
	--validation_kp $VAL_KP \
	--actuator $ACTUATOR \
	--output $N \
	> output/fit_$N.log 2>&1


