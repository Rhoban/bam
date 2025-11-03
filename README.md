# BAM: Better Actuator Models

<img align="left" src="https://github.com/user-attachments/assets/be9176e3-2aa7-4476-9d2b-88ffca177eb1" height=410>

Accurate models of servo actuators are essential for the simulation of robotic systems. It is particularly important 
while performing Reinforcement Learning (RL) on real robots, as the precision of the model impacts directly the 
transferability of the learned policy.

However, the friction model generally implemented in widely used simulators like MuJoCo or IsaacGym is the Coulomb-Viscous 
friction model (M1). This model is too simplistic to accurately represent the behavior of servo actuators, which are subject 
to more complex friction phenomena like Stribeck, load-dependence or quadratic effects.

In this repository, we propose a method to identify extended friction models (M2 to M6) for servo actuators. 
The detail of the method is presented in this [article](https://arxiv.org/pdf/2410.08650v1), and this [video](https://youtu.be/5XPEEKDnQEM) presents the 
motivation, the protocol, and the results of such identification. The improvement allowed by these models has been 
demonstrated on tests on 2R arms, where the simulation error has been reduced by more than 50% compared to the Coulomb-Viscous 
model (cf. figure on the left).

<br>

<img align="right" src="https://github.com/user-attachments/assets/ef76a5d0-31dd-436a-b8a3-96d4dc61fe98" width=450>

The method has been applied to the identification of the Dynamixel MX-64, Dynamixel MX-106, eRob80:50 and eRob80:100 actuators. The data collected on these actuators is available [here](https://drive.google.com/drive/folders/1SwVCcpJko7ZBsmSTuu3G_ZipVQFGZ11N?usp=drive_link). The MAE obtained by each proposed model during the identification is presented on the figure 
on the right, showing that the proposed models outperform the Coulomb-Viscous model.

<br>

## Friction model

The friction models used in this repository are:
* M1: Coulomb-Viscous model
* M2: Stribeck model
* M3: Load-dependent model
* M4: Stribeck load-dependent model
* M5: Directional model
* M6: Quadratic model

For a detailled description of these models, please refer to the [article](https://arxiv.org/pdf/2410.08650v1) or the [video](https://youtu.be/5XPEEKDnQEM).

# Identification

## Requirements

To install the requirements for the identification part, you can use the following command:

```bash
pip install -r requirements_bam.txt

```

## Setup

<img align="right" src=https://github.com/user-attachments/assets/2317aa80-5274-454c-b209-09f7759ff554 height=300>


To identify the model, you need to record logs on a pendulum test bench. 
While building such a bench, you need to ensure that the pendulum is free 
to move up to the horizontal position on both sides.
Note that the pendulum is considered at an angle of 0 when it is downward.

An exemple of such a bench for a Dynamixel MX-106 is presented on the right.

To augment the variety of logs, several pendulum parameters can be changed, such as:
* load mass
* pendulum length 
* control law
  
<br>

## Recording raw data

Files to record trajectories with Dynamixel or eRob servo actuators are available in the `dynamixel` and `erob` directories. To identify other actuators, you can use `bam/dynamixel/record.py` as a template.

### Trajectories list

Available trajectories are:

* `sin_time_square`: A sinusoidal trajectory, where the time is squared. This results in a progressively augmenting frequency.
* `sin_sin`: Two sinus summed together, with different frequencies
* `lift_and_drop`: The mass is lifted upward and then the torque is released, leaving it falling
* `up_and_down`: The mass is lifted upward and then took down slowly
* `nothing`: The torque is purely released, mostly for test purpose
  
<img width="49%" alt="Identif_AccSin2" src="https://github.com/user-attachments/assets/173d02be-f9bc-4562-bd24-8f06a8f1286f">
<img width="49%" alt="Identif_SinSin2" src="https://github.com/user-attachments/assets/84901c28-8345-4c37-a248-5beb7cf3038c">
<img width="49%" alt="Identif_LiftDrop2" src="https://github.com/user-attachments/assets/0a1ce588-9a40-4b9e-8b45-ff2a8363e120">
<img width="49%" alt="Identif_UpDown2" src="https://github.com/user-attachments/assets/4cb3c4c0-3824-4b32-bb17-83528cfb77b0">

### Dynamixel

You can use `bam/dynamixel/record.py` to execute a trajectory and record it, here is an example of usage:

```
python -m bam.dynamixel.record \
    --port /dev/ttyUSB0 \
    --mass 0.567 \
    --length 0.105 \
    --logdir data_raw \
    --trajectory sin_time_square \
    --motor mx106 \
    --kp 8 \
    --vin 15.0
```

Where the arguments are:

* `port`: The port where the Dynamixel is connected
* `mass`: The mass of the load
* `length`: The length of the pendulum
* `logdir`: The directory where the data will be saved
* `trajectory`: The trajectory to be executed
* `motor`: The name of the motor
* `kp`: The proportional gain of the controller
* `vin`: The input voltage (default: `15.0`)

To record the data for a set of different kp and trajectories, you can modify and use `bam/dynamixel/all_record.py`. Here is an example of usage:

```
python -m bam.dynamixel.all_record \
    --port /dev/ttyUSB0 \
    --mass 0.567 \
    --length 0.105 \
    --logdir data_raw \
    --motor mx106 \
    --speak
```

Where the arguments are the same as above, with the addition of `speak` which allows the trajectory and kp to be spoken before execution.

### ERob (with etherban)

First, you need to have the Etherban server running. You also need to compile the `proto` files, by running:

```
cd bam/erob/
bash generate_protobuf.sh
```

You can monitor the devices by running `python erob/etherban.py`. Notably, this will give you the angular offset
to use for the zero position.

You can then use the `record.py` script as following:

```
python -m bam.erob.record \
    --host 127.0.0.1 \
    --offset 1.57 \
    --mass 2.0 \
    --arm_mass 1.0 \
    --length 0.105 \
    --logdir data_raw \
    --trajectory sin_time_square \
    --motor erob100 \
    --kp 8 \
    --damping 0.1
```

Where the arguments are:

* `host`: The host where the Etherban server is running (by default `localhost`)
* `offset`: The angular offset to be used for the zero position
* `mass`: The mass of the load
* `arm_mass`: The mass of the arm
* `length`: The length of the pendulum
* `logdir`: The directory where the data will be saved
* `trajectory`: The trajectory to be executed
* `motor`: The name of the motor
* `kp`: The proportional gain of the controller
* `damping`: The damping of the controller

To record the data for a set of different kp and trajectories, you can modify and use `bam/erob/all_record.py`. Here is an example of usage:

```
python -m bam.erob.all_record \
    --host 127.0.0.1 \
    --offset 1.57 \
    --mass 2.0 \
    --arm_mass 1.0 \
    --length 0.105 \
    --logdir data_raw \
    --motor erob100 \
    --damping 0.1 \
    --speak
```

Where the arguments are the same as above, with the addition of `speak` which allows the trajectory and kp to be spoken before execution.

## Post-processing

To post-process, you can use:

```
python -m bam.process \
    --raw data_raw \
    --logdir data_processed \
    --dt 0.005
```

This will process the data with linear interpolation to enforce a constant given timestep.

## Model fitting

<img align="right" width="50%" src="https://github.com/user-attachments/assets/b127f39c-48d3-4242-80c8-59166b4d4dcd">

The model fitting can be done with:

```
python -m bam.fit \
    --actuator mx106 \
    --model m6 \
    --logdir data_processed \
    --method cmaes \
    --output params/mx106/m1.json \
    --trials 1000 
```

Where the arguments are:
* `actuator`: The actuator to be used
* `model`: The model to be used
* `logdir`: The directory where the processed data is stored
* `method`: The method to be used for optimization. Available methods are `cmaes`, `random`, `nsgaii` (default: `cmaes`)
* `output`: The file where the parameters will be saved (default: `params.json`)
* `trials`: The number of trials to be executed (default: `100_000`)

## Plotting

<img align="right" width="50%" src="https://github.com/user-attachments/assets/dc9ec4d6-47e7-49fd-92bc-a27c0026279f">

To plot simulated vs real data, you can use:

```
python -m bam.plot \
    --actuator mx106 \
    --logdir data_processed \
    --sim \
    --params params/mx106/m6.json
```

Where the arguments are:
* `actuator`: The actuator to be used
* `logdir`: The directory where the processed data is stored
* `sim`: If present, the simulated data will be plotted
* `params`: The file where the model parameters are stored (is necessary if `sim` is present)


If you want to check your logs at each step of the data collection and processing, you can use the same command without the `--sim` flag.

## Drive/Backdrive diagram

<img align="left" height="150px" src="https://github.com/user-attachments/assets/01f10f9a-e2c3-47d7-8d37-dfe51aea17e9">

To draw some drive/backdrive diagrams, you can use for example:

```
python -m bam.drive_backdrive \
    --params params/erob80_100/m6.json \
    --max_torque 120
```

<br>

# Validation on 2R arms

To validate the models, 2R arms composed of Dynamixel and eRob actuators are used. MuJoCo is used 
to simulate these arms and the MuJoCo URDFs of these 2 arms are available in the `2R` directory. 
If you want to use another 2R arm, the conversion process from a classic URDF to a MuJoCo 
URDF is detailed in the `2R/README.md`.

## Requirements

To install the requirements for the validation part, you can use the following command:

```bash
pip install -r requirements_2R.txt

```

## Setup

The arms used for the validation are composed of 2 segments with a load at the end. The Dynamixel arm uses Dynamixel MX-64 and MX-106, while the eRob arm uses eRob80:50 and eRob80:100:

<p align="center">
  <img src="https://github.com/user-attachments/assets/976cbd93-14e0-4a3f-91ca-43dd6f755e31" height="400px">
  <img src="https://github.com/user-attachments/assets/7cda5406-560e-4cd0-8b63-f78b1467fe5f" height="400px">
</p>
  
## Recording raw data

### Trajectories list

<img align="right" src="https://github.com/user-attachments/assets/7b38212d-ae6e-43f3-86ae-624c702796af" height="160px">
  
4 trajectories are used for the validation:
*  `circle` 
*  `square` 
* `square_wave` 
* `triangular_wave`


### Dynamixel

You can use `record_2R.py` to execute a trajectory and record it, here is an example of usage:

```
python -m bam.dynamixel.record_2R \
    --port /dev/ttyUSB0 \
    --mass 0.567 \
    --logdir data_2R_dyn \
    --trajectory circle \
    --kp 8 \
    --speed 1.0
```

Where the arguments are:

* `port`: The port where the Dynamixel is connected
* `mass`: The mass of the load
* `logdir`: The directory where the data will be saved
* `trajectory`: The trajectory to be executed
* `kp`: The proportional gain of the controller
* `speed`: The speed at which the trajectory is executed
  
### ERob (with etherban)

You can use `record_2R.py` to execute a trajectory and record it, here is an example of usage:

```
python -m bam.erob.record_2R \
    --host 127.0.0.1 \
    --r1_offset 1.57 \
    --r2_offset -0.72 \
    --mass 2.0 \
    --logdir data_2R_erob \
    --trajectory circle \
    --kp 8
```

Where the arguments are:

* `host`: The host where the Etherban server is running (by default `localhost`)
* `r1_offset`: The angular offset to be used for the zero position of the first motor
* `r2_offset`: The angular offset to be used for the zero position of the second motor
* `mass`: The mass of the load
* `logdir`: The directory where the data will be saved
* `trajectory`: The trajectory to be executed
* `kp`: The proportional gain of the controller

## Simulating 2R arms

<img align="right" src="https://github.com/user-attachments/assets/0854f51d-97f3-4189-9d58-bd5f57c59c8a" width="50%">
    
To simulate the 2R arms, you can use:

```
python -m 2R.sim \
    --log log.json \
    --params params/mx106/m4.json,params/mx64/m4.json \
    --testbench mx \
    --render \
    --plot \
    --vertical \
    --mae
```

Where the arguments are:
* `log`: The log file or several log files to be used. If a whole directory should be used, you can use `--log log_dir/*`.
* `params`: Model parameters for the actuators in the format `params_m1,params_m2`. Several parameters can be used, separated by spaces.
* `testbench`: The testbench to be used. Available testbenches are `mx` and `erob`.
* `render`: If present, the simulation (MuJoCo) will be rendered.
* `plot`: If present, the measured and simulated positions will be plotted.
* `vertical`: If present, the plot will be vertical.
* `mae`: If present, the Mean Absolute Error will be computed for each trajectory and couple of model parameters.

## Plotting

To quickly plot logs for a given testbench, you can use the command:

```
./2R/plot.sh testbench log_dir/*
```

If you want to obtain the MAE for each model for a set of logs and a given testbench, you can use:

```
./2R/mae.sh testbench log_dir/*
```

<p align="center">
  <img src="https://github.com/user-attachments/assets/3effdce3-7d70-48a5-835a-5b92af254910" height="260">
  <img src="https://github.com/user-attachments/assets/5bfffcfa-c4ea-4644-aac4-6966c4254482" height="260">
</p>


# Citation

If you use **BAM** in your scientific works, please cite the software as follows:

```bibtex
@software{BAM,
  title = {{BAM: Better Actuator Models}},
  author = {Duclusaud, Marc and Passault, Grégoire},
  license = {Apache-2.0},
  url = {https://github.com/Rhoban/bam},
  version = {0.0.1},
  year = {2024}
}
```

If you build upon the friction models presented in our paper, please cite the publication:

```bibtex
@inproceedings{duclusaud2025extended,
  title={Extended Friction Models for the Physics Simulation of Servo Actuators},
  author={Duclusaud, Marc and Passault, Gr{\'e}goire and Padois, Vincent and Ly, Olivier},
  booktitle={2025 IEEE International Conference on Robotics and Automation (ICRA)},
  pages={12091--12097},
  year={2025},
  organization={IEEE}
}
```