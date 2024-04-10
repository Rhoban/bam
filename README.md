# Dynamixel Identification

## Setup

**TODO**: Picture of the setup

When $\alpha = 0$, the pendulum is downward.

## Usage

### Recording the data

You can use `record.py` to execute a trajectory and record it, here is an example of usage:

```
python record.py \
    --port /dev/ttyUSB0 \
    --mass 0.567 \
    --length 0.105 \
    --logdir data_raw \
    --trajectory sinus_time_square \
    --kp 8
```

Where the arguments are:

* `port`: The port where the Dynamixel is connected
* `mass`: The mass of the object attached to the pendulum
* `length`: The length of the pendulum
* `logdir`: The directory where the data will be saved
* `trajectory`: The trajectory to be executed (see below)
* `kp`: The proportional gain of the controller

Available trajectories are:

* `sinus_time_square`: A sinusoidal trajectory, where the time is squared. This results in a progressively augmenting
  frequency.
* `lift_and_drop`: The mass is lifted upward and then the torque is released, leaving it falling
* `nothing`: The torque is purely released, mostly for test purpose

### Post-processing

To post-process, you can use:

```
python process.py --raw data_raw --logdir data_processed --dt 0.005
```

This will process the data with linear interpolation to enforce a constant given timestep.

### Model fitting

The model fitting can be done with:

```
python fit.py \
    --logdir data_processed \
    --method cmaes \
    --output params.json \
    --trials 1000 \
    --control
```

The argument meaning is:
* `logdir`: The directory where the processed data is stored
* `method`: The method to be used for optimization. Available methods are `cmaes`, `random`, `nsgaii` (default: `cmaes`)
* `output`: The file where the parameters will be saved (default: `params.json`)
* `trials`: The number of trials to be executed (default: `100_000`)
* `control`: If present, the voltage is computed given the current goal and error instead of used from the logs

### Plotting

You can then use:

```
python plot.py \
    --logdir data_processed \
    --params params.json \
    --control
```

To plot simulated vs real data.