flags="--wandb --max --simplify_tau_m"

# Parameters
epochs=150
nodes=(128)
losses=("l1_loss") #"smooth_l1_loss")
windows=(1 4 16)
activations=("LeakyReLU" "Tanh" "ReLU") #"Softsign")
lasts=("Abs") # "Square" "Custom")

# Cleaning nohup.out
rm nohup.out

for loss in "${losses[@]}"; do
    for a in "${activations[@]}"; do
        for n in "${nodes[@]}"; do
            for w in "${windows[@]}"; do
                for last in "${lasts[@]}"; do
                    nohup python learn.py $flags --window $w --wandb $wandb --activation $a --epochs $epochs --nodes $n --loss $loss --last $last &
                done
            done
        done
    done
donegit starting