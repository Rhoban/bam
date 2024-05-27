# Dynamixel Identification

## Setup

**TODO**: Picture of the setup

When $\alpha = 0$, the pendulum is downward.

## Recording raw data

### Dynamixel

You can use `record.py` to execute a trajectory and record it, here is an example of usage:

```
python dynamixel/record.py \
    --port /dev/ttyUSB0 \
    --offset 1.23 \
    --mass 0.567 \
    --length 0.105 \
    --logdir data_raw \
    --trajectory sin_time_square \
    --motor some_name \
    --kp 8 \
    --vin 15.0
```

Where the arguments are:

* `port`: The port where the Dynamixel is connected
* `offset`: The angular offset to be used for the zero position
* `mass`: The mass of the object attached to the pendulum
* `length`: The length of the pendulum
* `logdir`: The directory where the data will be saved
* `trajectory`: The trajectory to be executed (see below)
* `motor`: The name of the motor
* `kp`: The proportional gain of the controller
* `vin`: The input voltage (default: `15.0`)

### Erob (with etherban)

First, you need to have the Etherban server running. You also need to compile the `proto` files, by running:

```
cd erob/
bash generate_protobuf.sh
```

You can monitor the devices by running `python erob/etherban.py`. Notably, this will give you the angular offset
to use for the zero position.

You can then use the `record.py` script as following:

```
python erob/record.py \
    --host 127.0.0.1 \
    --mass 2.0 \
    --arm_mass 1.0 \
    --length 0.105 \
    --logdir data_raw \
    --trajectory sin_time_square \
    --motor some_name \
    --kp 8
```

Where:

* `host`: The host where the Etherban server is running (by default `localhost`)
* `mass`: The mass of the object attached to the pendulum
* `arm_mass`: The mass of the arm
* `length`: The length of the pendulum
* `logdir`: The directory where the data will be saved
* `trajectory`: The trajectory to be executed (see below)
* `motor`: The name of the motor
* `kp`: The proportional gain of the controller


## Trajectories list

Available trajectories are:

* `sin_time_square`: A sinusoidal trajectory, where the time is squared. This results in a progressively augmenting
  frequency.
* `sin_sin`: Two sinus summed together, with different frequencies
* `lift_and_drop`: The mass is lifted upward and then the torque is released, leaving it falling
* `up_and_down`: The mass is lifted upward and then took down slowly
* `nothing`: The torque is purely released, mostly for test purpose

## Post-processing

To post-process, you can use:

```
python process.py --raw data_raw --logdir data_processed --dt 0.005
```

This will process the data with linear interpolation to enforce a constant given timestep.

## Model fitting

The model fitting can be done with:

```
python fit.py \
    --actuator mx \
    --logdir data_processed \
    --method cmaes \
    --output params.json \
    --trials 1000 \
    --control
```

The argument meaning is:
* `actuator`: The actuator to be used
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