
for k in `seq 1 10`
do
	N=`ls output/*.json|wc -w`
	N=$[$N + 1]
	python fit.py --wandb --logdir data_106 \
		--control --model stribeck_viscous \
		--workers 8 --trials 1500 \
		--output output/params_$N.json \
		> output/fit_$N.log 2>&1
	echo $k
done


