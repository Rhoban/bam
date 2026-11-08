# MangDang MD-300-150

The MD-300-150 is MangDang's three-output leg module for the Mini Pupper v4:
three coreless brushed DC motors with gearboxes in one housing (abduction, hip,
and a coaxial knee driven through a rod). An AT32 microcontroller runs a
position loop for each output.

This package holds the actuator model and the tools that record BAM logs from
the module. The identified parameters are in `bam/params/mangdang_md300_abd`,
`mangdang_md300_hip` and `mangdang_md300_knee` (M1 to M6), all for
`MD300Actuator`.

## Hardware

    host (spi.py) --USB serial--> ESP32-S3 bridge --SPI--> AT32 (one per module) --> motors + potentiometers

- AT32 firmware 0.0.9, voltage (PWM) position control (`control_mode` 1).
- Bridge firmware: https://github.com/SiIverHusky/bam-firmware
- Bus ids 1 to 12: the module in slot `k` (0 to 3) answers as `3k + 1` (abduction),
  `3k + 2` (hip) and `3k + 3` (knee).
- The AT32 answers each frame with the state it had at the previous frame; the
  recorder stamps each sample accordingly. It reports the supply-side current
  (`I_motor * |duty|`) and does not measure the supply voltage.

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
the hip of the module in slot 2, bus id 8):

```bash
python -m bam.mangdang.all_record --id 8 --mass 0.1605 --arm-mass 0.0196 --length 0.15 \
    --kps 40,80,120,160 --kd 400 --shuffle --logdir raw \
    --trajectories lift_and_drop,sin_sin,sin_time_square,up_and_down,gait_hip \
    --extra "--zero-dd 1556 --control-mode 1 --pwm-scale 0.001 --max-pwm-duty-cycle 0.99 \
             --ki-position 0 --supply-v 12"
python -m bam.process --raw raw --logdir processed --dt 0.005
python -m bam.fit --actuator mangdang_md300_hip --model m6 --logdir processed --validation_kp 120
```

`--zero-dd` is the raw reading (deci-degrees) of the pendulum's downward pose.
The recorder writes the firmware configuration to RAM before each run (nothing
is saved to flash) and logs what the AT32 reads back (`at32_*` keys).
`--dry-run` exercises the tools without hardware.

`trajectories.py` adds to BAM's set: `abd_steps`, `abd_sts`, `abd_sweep` and
`abd_drop`, small-signal trajectories within +/-0.35 rad for the abduction; and
`gait_hip` and `gait_knee`, the hip and knee setpoints of a walking gait
(`streams/`, 50 Hz), played about +0.5 rad.

## Identification conditions

- 12 V bench supply.
- Firmware gains: `kp` 40 / 80 / 120 / 160, `kd` 400, `pwm_scale` 0.001,
  `max_pwm` 0.99. `kp` 120 is held out for validation.
- Trajectories: `lift_and_drop`, `sin_sin`, `sin_time_square`, `up_and_down`,
  plus `abd_sweep` (abduction) or `gait_hip` / `gait_knee`.
- Logs of 6 s, processed at 5 ms.
- Pendulum: the payload's centre is 2 cm beyond the arm's end. Its own inertia
  is folded into `mass`, `arm_mass` and `length`, so the logged pendulum has
  exactly the inertia and gravity torque of the arm and payload.
- Setups: {{arm length x payload per output}}.

## Results

Mean absolute position error on the held-out `kp` 120 logs (reference
simulator, `bam.fit` scoring):

| Model | Abduction | Hip | Knee |
|---|---|---|---|
| m1 | 22.80 mrad | 31.70 mrad | 20.07 mrad |
| m2 | {{...}} | {{...}} | {{...}} |
| m3 | {{...}} | {{...}} | {{...}} |
| m4 | {{...}} | {{...}} | {{...}} |
| m5 | {{...}} | {{...}} | {{...}} |
| m6 | 17.13 mrad | 20.63 mrad | 15.70 mrad |

## Limits

- At stall the current falls by about 30 % over 30 s as the winding heats. The
  models do not depend on temperature, so sustained near-stall loads are
  over-estimated.
- The heaviest setups, near the torque limit, fit worse than light and medium
  ones.
- `error_gain_ratio` and `kt_emf` are effective values: they also absorb loop
  lag and viscous damping.
- `bam.mujoco` and `bam.mjlab` apply no torque while torque is off, so the
  H-bridge braking is only simulated by `bam.simulate` (`bam.fit`, `bam.mae`).
