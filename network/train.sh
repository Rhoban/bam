wandb=1
epochs=150
losses=("l1_loss" "mse_loss" "smooth_l1_loss")
nodes=(128)
windows=(1 4 16)
activations=("LeakyReLU" "Tanh" "Softsign" "ReLU")

for loss in "${losses[@]}"; do
    for a in "${activations[@]}"; do
        for n in "${nodes[@]}"; do
            for w in "${windows[@]}"; do
                python learn.py --window $w --wandb $wandb -a $a -e $epochs -n $n --loss $loss
            done
        done
    done
done
