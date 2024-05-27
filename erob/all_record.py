import argparse
import os
import time


arg_parser = argparse.ArgumentParser()
arg_parser.add_argument("--host", type=str, default="127.0.0.1")
arg_parser.add_argument("--offset", type=float, required=True)
arg_parser.add_argument("--mass", type=float, required=True)
arg_parser.add_argument("--arm_mass", type=float, required=True)
arg_parser.add_argument("--length", type=float, required=True)
arg_parser.add_argument("--motor", type=str, required=True)
arg_parser.add_argument("--logdir", type=str, required=True)
arg_parser.add_argument("--speak", action="store_true")
args = arg_parser.parse_args()

kps = [10, 25, 50, 100]
trajectories = ["sin_sin", "lift_and_drop", "up_and_down", "sin_time_square"]

command_base = f"python3 erob/record.py --offset {args.offset} --mass {args.mass} --arm_mass {args.mass} --length {args.length}"
command_base += f" --host {args.host} --logdir {args.logdir} --motor {args.motor}"


for kp in kps:
    for trajectory in trajectories:
        sentence = f"Kp {kp}, trajectory {trajectory.replace('_', ' ')}"
        print(sentence)

        if args.speak:
            from gtts import gTTS 
            myobj = gTTS(text=sentence, lang='en', slow=False) 
            myobj.save("/tmp/message.mp3")
            os.system("mpg321 /tmp/message.mp3")

        command = f"{command_base} --kp {kp} --trajectory {trajectory}"
        os.system(command)

        if trajectory == "sin_time_square":
            time.sleep(3)
