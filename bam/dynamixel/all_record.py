# Copyright 2025 Marc Duclusaud & Grégoire Passault

# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at:

#     http://www.apache.org/licenses/LICENSE-2.0

import argparse
import os
import time


arg_parser = argparse.ArgumentParser()
arg_parser.add_argument("--mass",        type=float, required=True)
arg_parser.add_argument("--length",      type=float, required=True)
arg_parser.add_argument("--motor",       type=str,   required=True)
arg_parser.add_argument("--port",        type=str,   default="/dev/ttyUSB0")
arg_parser.add_argument("--logdir",      type=str,   required=True)
arg_parser.add_argument("--vin",         type=float, default=None,
                        help="Input voltage. Defaults: mx64/mx106=15.0, xl320=7.5, xm430=12.0")
arg_parser.add_argument("--id",          type=int,   default=1)
arg_parser.add_argument("--baudrate",    type=int,   default=None,
                        help="Serial baud rate passed to record.py. "
                             "Defaults: mx/xl320=1000000, xm430=57600")
arg_parser.add_argument("--kps",         type=str,   default=None,
                        help="Comma-separated kp values, e.g. '400,800,1200,1600'. "
                             "Overrides motor-specific defaults.")
arg_parser.add_argument("--trajectories", type=str,  default=None,
                        help="Comma-separated trajectory names. "
                             "Default: sin_sin,lift_and_drop,up_and_down,sin_time_square")
arg_parser.add_argument("--speak",       action="store_true")
args = arg_parser.parse_args()

motor_key = args.motor.lower()

# Motor-specific vin default (mirrors record.py logic)
if args.vin is not None:
    vin = args.vin
elif motor_key.startswith("xm430"):
    vin = 12.0
elif motor_key == "xl320":
    vin = 7.5
else:
    vin = 15.0

# kp sweep: XM430 Position P Gain lives in 0–16383, needs a different scale
if args.kps is not None:
    kps = [int(x.strip()) for x in args.kps.split(",") if x.strip()]
elif motor_key.startswith("xm430"):
    kps = [400, 800, 1200, 1600]
else:
    kps = [4, 8, 16, 32]

if args.trajectories is not None:
    trajectories = [x.strip() for x in args.trajectories.split(",") if x.strip()]
else:
    trajectories = ["sin_sin", "lift_and_drop", "up_and_down", "sin_time_square"]

command_base = (
    f"python3 -m bam.dynamixel.record"
    f" --mass {args.mass} --length {args.length}"
    f" --port {args.port} --logdir {args.logdir}"
    f" --motor {args.motor} --vin {vin} --id {args.id}"
)
if args.baudrate is not None:
    command_base += f" --baudrate {args.baudrate}"

for kp in kps:
    for trajectory in trajectories:
        sentence = f"Kp {kp}, trajectory {trajectory.replace('_', ' ')}"
        print(sentence)

        if args.speak:
            from gtts import gTTS
            myobj = gTTS(text=sentence, lang="en", slow=False)
            myobj.save("/tmp/message.mp3")
            os.system("mpg321 /tmp/message.mp3")

        command = f"{command_base} --kp {kp} --trajectory {trajectory}"
        os.system(command)

        if trajectory == "sin_time_square":
            time.sleep(3)
