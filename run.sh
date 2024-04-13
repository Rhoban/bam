
MODEL="stribeck_viscous"
DATA="data_106"

for k in `seq 1 1`
do
	N=`ls output/*.json|wc -w`
	N=$[$N + 1]
	JSON="output/params_$N.json"
	touch $JSON
	python fit.py --wandb --logdir $DATA \
		--control --model $MODEL \
		--workers 1 --trials 50000 \
		--output $JSON \
		> output/fit_$N.log 2>&1
	echo $k
done


