
#MODEL="m1"
DATA="data_106"

for MODEL in m1 m2 m3 m4:
do
	N=`ls output/*.json|wc -w`
	N=$[$N + 1]
	
	python fit.py --wandb --logdir $DATA \
		--control --model $MODEL \
		--workers 1 --trials 50000 \
		--output $N \
		> output/fit_$N.log 2>&1
	echo $k
done


