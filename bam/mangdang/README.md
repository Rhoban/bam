# MangDang MD-300-150

The MD-300-150 is MangDang's three-output leg module for the Mini Pupper v4:
three coreless brushed DC motors with gearboxes in one housing. An AT32
microcontroller runs a position loop for each output.

id_1: side output (abduction on Mini Pupper v4); id_2: outer coaxial output
(hip); id_3: inner coaxial output, through a rod (knee).

This package holds the actuator model and the tools that record BAM logs from
the module. The identified parameters are in `bam/params/mangdang_md300_id_1`,
`mangdang_md300_id_2` and `mangdang_md300_id_3` (M1 to M6), all for
`MD300Actuator`.

## Hardware

    host (spi.py) --USB serial--> ESP32-S3 bridge --SPI--> AT32 (one per module) --> motors + potentiometers

- AT32 firmware 0.0.9, voltage (PWM) position control (`control_mode` 1).
- Rig (printed parts, BOM) and the ESP32-S3 SPI bridge firmware:
  https://github.com/mangdang-bam/md300-bam-rig
- Bus ids 1 to 12: the module in slot `k` (0 to 3) answers as `3k + 1` (id_1),
  `3k + 2` (id_2) and `3k + 3` (id_3).
- The AT32 answers each frame with the state it had at the previous frame; the
  recorder stamps each sample accordingly. It does not measure the supply
  voltage; the current it reports is logged as `load` and is not used by the model.

## Model

`MD300Actuator` models the firmware's position loop:

    duty = clip(pwm_scale * kp * (error - dead band) - D, +/-max_pwm)

with the error in degrees, a 0.1 deg dead band and a D term proportional to the
measured velocity. `pwm_scale`, `kp`, `kd`, `max_pwm` and the supply voltage are
read from each log. The plant is the DC motor of `VoltageControlledActuator`
with a back-EMF constant `kt_emf` separate from `kt`, and the H-bridge braking
the motor while torque is off.

Without a log (MuJoCo, mjlab), the class uses the identified configuration:
12 V, `pwm_scale` 0.001, `kd` 400, `max_pwm` 0.99. Set `kp` as on the robot.
With `bam.mujoco.load_config`, use `error_gain` 0.0572958 (`pwm_scale * 180 / pi`)
and `max_pwm` 0.99.

## Recording

With the module held fixed and a pendulum arm on the output under test (here
id_2 of the module in slot 2, bus id 8):

```bash
python -m bam.mangdang.all_record --id 8 --mass 0.16078 --arm-mass 0.01872 --length 0.145 \
    --arm-length 0.125 --payload-offset 0.02 --payload-inertia 7.68e-05 \
    --kps 40,80,120,160 --kd 400 --shuffle --logdir raw \
    --trajectories lift_and_drop,sin_sin,sin_time_square,up_and_down,gait_hip \
    --extra "--zero-dd 1556 --control-mode 1 --pwm-scale 0.001 --max-pwm-duty-cycle 0.99 \
             --ki-position 0 --supply-v 12"
python -m bam.process --raw raw --logdir processed --dt 0.005
python -m bam.mangdang.pendulum --logdir processed
python -m bam.fit --actuator mangdang_md300_id_2 --model m6 --logdir processed --validation_kp 120
```

`--mass` and `--arm-mass` are the payload and arm masses. `--payload-offset`
is the distance from the arm's end (`--arm-length`, default `--length`) to the
payload's centre and `--payload-inertia` the payload's inertia about its centre;
the recorder logs them as `geometry_*` keys. `bam.mangdang.pendulum` rewrites
each processed log's `mass`, `arm_mass` and `length` from those keys into BAM's
single-length pendulum with the same inertia and gravity torque.

`--zero-dd` is the raw reading (deci-degrees) of the pendulum's downward pose.
Measure it with the payload mounted: `python -m bam.mangdang.zero --id 8
--logdir raw` lifts the arm to each side, releases it and takes the midpoint
of the rest positions, which cancels static friction.
The recorder writes the firmware configuration to RAM before each run (nothing
is saved to flash) and logs what the AT32 reads back (`at32_*` keys).
`--dry-run` exercises the tools without hardware.

`trajectories.py` adds to BAM's set: `abd_steps`, `abd_sts`, `abd_sweep` and
`abd_drop`, small-signal trajectories within +/-0.35 rad for id_1; and
`gait_hip` and `gait_knee`, the hip and knee setpoints of a walking gait
(`streams/`, 50 Hz), played about +0.5 rad.

## Identification conditions

- 12 V bench supply.
- Firmware gains: `kp` 40 / 80 / 120 / 160, `kd` 400, `pwm_scale` 0.001,
  `max_pwm` 0.99. `kp` 120 is held out for validation.
- Trajectories: `lift_and_drop`, `sin_sin`, `sin_time_square`, `up_and_down`,
  plus `abd_sweep` (id_1), `gait_hip` (id_2) or `gait_knee` (id_3).
- Logs of 6 s (7 s for the gait streams), processed at 5 ms.
- Pendulum: the payload's centre is 2 cm beyond the arm's end. The processed
  logs' `mass`, `arm_mass` and `length` come from `bam.mangdang.pendulum`, so
  they carry the inertia (payload's own included) and gravity torque of the arm
  and payload.
- Setups (arm length x payload): none; 10 cm bare; 10 cm x 61 g; 10 cm x 120 g;
  12.5 cm x 161 g; 15 cm x 80 g; 15 cm x 239 g; 15 cm x 257 g. No `kp` 160
  logs for id_1's 257 g setup and id_3's 239 g and 257 g setups.
- id_1: all eight setups, one module, 163 logs. id_2: seven (no 257 g), two
  modules (15 cm x 80 g recorded on both), 168 logs. id_3: all eight, one
  module, 158 logs.
- Raw data: https://github.com/mangdangroboticsclub/bam/releases/download/md300-data-v1/md300_logs.zip
  (raw and processed, per output).

## Results

Mean absolute position error in mrad, training logs / held-out `kp` 120 logs
(`bam.mae`: reference simulator, all samples, mean over logs):

| Model | id_1 | id_2 | id_3 |
|---|---|---|---|
| m1 | 25.0 / 22.8 | 18.1 / 17.2 | 15.6 / 20.1 |
| m2 | 25.0 / 22.8 | 18.1 / 17.2 | 15.6 / 20.1 |
| m3 | 14.6 / 17.0 | 12.1 / 11.2 | 10.8 / 14.9 |
| m4 | 13.7 / 16.6 | 11.5 / 12.2 | 10.6 / 15.9 |
| m5 | 13.1 / 16.3 | 11.8 / 11.1 | 10.5 / 15.9 |
| m6 | 13.1 / 17.1 | 11.5 / 12.1 | 10.5 / 15.7 |

m1 and m2 fit the D term at zero, a delay under 1 ms and a back-EMF constant
of 0.25 to 0.40 V.s/rad, against about 3 ms and 0.73 to 0.88 V.s/rad for m3 to
m6.

Position data constrain only combinations of the plant constants (the stiffness
`kt * error_gain_ratio / R`, the damping `kt * kt_emf / R` and the D term at the
single `kd` recorded), so the individual `kt`, `R`, `kt_emf` and `kd_ratio` vary
between models and are effective values. m1 and m2 rest partly on their
parameter bounds; use m3 to m6.

## Limits

- At stall the current falls by about a third (1.28 A to 0.87 A) over 30 s as
  the winding heats. The models do not depend on temperature, so sustained
  near-stall loads are over-estimated.
- Setups below 0.3 N.m static load are tracked to 2.3 to 13.5 mrad (m6,
  `kp` 120, per setup); the heaviest setups, near the torque limit, fit worse.
- `error_gain_ratio` and `kt_emf` are effective values: they also absorb loop
  lag and viscous damping.
- `bam.mujoco` and `bam.mjlab` apply no torque while torque is off, so the
  H-bridge braking is only simulated by `bam.simulate` (`bam.fit`, `bam.mae`).
