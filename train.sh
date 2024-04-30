flags="--wandb --max --simplify_tau_m"

# Parameters
epochs=150
losses=("l1_loss") #"mse" "smooth_l1_loss")
activations=("LeakyReLU") #"Tanh" "ReLU") #"Softsign")
nodes=(128)
windows=(1 4 16)
lasts=("Square" "Custom" "Abs")

# Cleaning nohup.out
rm nohup.out

for loss in "${losses[@]}"; do
    for n in "${nodes[@]}"; do
        for a in "${activations[@]}"; do
            for w in "${windows[@]}"; do
                for last in "${lasts[@]}"; do
                    nohup python learn.py $flags --window $w --wandb $wandb --activation $a --epochs $epochs --nodes $n --loss $loss --last $last &
                done
            done
        done
    done
done