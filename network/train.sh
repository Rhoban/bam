wandb=1
epochs=250
losses=("smooth_l1_loss" "mse")
nodes=(128)
windows=(1)
activations=("LeakyReLU" "Tanh" "Softsign" "ReLU")
last_activation=("Softplus" "Square" "Custom" "None")

for loss in "${losses[@]}"; do
    for a in "${activations[@]}"; do
        for la in "${last_activation[@]}"; do
            for n in "${nodes[@]}"; do
                for w in "${windows[@]}"; do
                    python learn.py --window $w --wandb $wandb -a $a -e $epochs -n $n --last $la --loss $loss
                done
            done
        done
    done
done
