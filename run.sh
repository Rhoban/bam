#!/bin/bash

ACTUATOR="erob80_100"
VAL_KP=25

#ACTUATOR="mx64"
#ACTUATOR="mx106"
#VAL_KP=8

DATA="data/$ACTUATOR"

if [ $# -eq 0 ]; then
    echo "Usage: $0 <model>"
    exit 1
fi

N=`ls output/fit_*.log|wc -w`
N=$[$N + 1]
mkdir -p output/

python -m rham.fit --wandb --logdir $DATA \
	--model $1 \
    	--control \
	--workers 1 --trials 10000 \
	--validation_kp $VAL_KP \
	--actuator $ACTUATOR \
	--output $N \
	> output/fit_$N.log 2>&1


