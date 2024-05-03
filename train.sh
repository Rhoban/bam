flags="" #--wandb --save" # --max" # Better without --simplify_tau_m, NaNs with --soft_min

# Parameters
epochs=150
lasts=("Abs" "Square" "Custom") # (No significant difference) 
losses=("l1_loss") #"mse" "smooth_l1_loss"
activations=("LeakyReLU") #"Tanh" "ReLU" "Softsign" 
nodes=(128)
windows=(1 4 16)

# Cleaning nohup.out
rm nohup.out

for loss in "${losses[@]}"; do
    for n in "${nodes[@]}"; do
        for a in "${activations[@]}"; do
            for w in "${windows[@]}"; do
                for last in "${lasts[@]}"; do
                    nohup python learn.py $flags --window $w --activation $a --epochs $epochs --nodes $n --loss $loss --last $last &
                done
            done
        done
    done
done