wandb=0
epochs=150
losses=("l1_loss") #"smooth_l1_loss")
nodes=(128)
windows=(1 4 16)
activations=("LeakyReLU" "Tanh" "ReLU") #"Softsign")
lasts=("Abs") # "Square" "Custom")

for loss in "${losses[@]}"; do
    for a in "${activations[@]}"; do
        for n in "${nodes[@]}"; do
            for w in "${windows[@]}"; do
                for last in "${lasts[@]}"; do
                    python learn.py --max --window $w --wandb $wandb --activation $a --epochs $epochs --nodes $n --loss $loss --last $last
                done
            done
        done
    done
done