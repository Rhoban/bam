flags="--wandb --save" # Better without --simplify_tau_m, NaNs with --soft_min

# Output file
host=$(hostname -f)
name=$(cut -d. -f1 <<< $host)
out="nohup-$name.out"
rm $out

# Parameters
epochs=150
lasts=("Abs") # "Square" "Custom") # (No significant difference) 
losses=("l1_loss") #"mse" "smooth_l1_loss"
activations=("LeakyReLU" "Tanh" "ReLU") # "Softsign" 
nodes=(128)
windows=(1 4 16)

for loss in "${losses[@]}"; do
    for n in "${nodes[@]}"; do
        for a in "${activations[@]}"; do
            for w in "${windows[@]}"; do
                for last in "${lasts[@]}"; do
                    nohup python learn.py $flags --window $w --activation $a --epochs $epochs --nodes $n --loss $loss --last $last > $out &
                done
            done
        done
    done
done