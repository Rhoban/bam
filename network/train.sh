wandb=1
epochs=150
losses=("smooth_l1_loss" "mse_loss" "l1_loss")
nodes=(128)
windows=(1 4 16)
activations=("LeakyReLU" "Tanh" "ReLU")
last_activation=("Softplus" "Square" "Custom" "Abs")

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