
#MODEL="m1"
DATA="data_106"
ARGS="--control"

for MODEL in m1
do
	N=`ls output/*.json|wc -w`
	N=$[$N + 1]
	
	python fit.py --wandb --logdir $DATA \
		$ARGS --model $MODEL \
		--workers 1 --trials 10000 \
		--output $N \
		> output/fit_$N.log 2>&1
	echo $k
done


