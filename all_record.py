import argparse
import os


arg_parser = argparse.ArgumentParser()
arg_parser.add_argument("--mass", type=float, required=True)
arg_parser.add_argument("--length", type=float, required=True)
arg_parser.add_argument("--motor", type=str, required=True)
arg_parser.add_argument("--port", type=str, default="/dev/ttyUSB0")
arg_parser.add_argument("--logdir", type=str, required=True)
args = arg_parser.parse_args()

kps = [4, 8, 16, 32]
trajectories = ["lift_and_drop", "sinus_time_square", "up_and_down", "sin_sin"]

command_base = f"python3 record.py --mass {args.mass} --length {args.length}"
command_base += f" --port {args.port} --logdir {args.logdir} --motor {args.motor}"


for kp in kps:
    for trajectory in trajectories:
        print(f"Recording with kp={kp} and trajectory={trajectory}")
        command = f"{command_base} --kp {kp} --trajectory {trajectory}"
        os.system(command)
