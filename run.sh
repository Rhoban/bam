
MODEL=m1
DATA="data_106"
ARGS="--control"
VAL_KP=8

N=`ls output/*.json|wc -w`
N=$[$N + 1]

python fit.py --wandb --logdir $DATA \
	$ARGS --model $MODEL \
	--workers 1 --trials 10000 \
	--validation_kp $VAL_KP \
	--actuator mx \
	--output $N \
	> output/fit_$N.log 2>&1


